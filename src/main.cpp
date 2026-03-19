/**
 * Control de Extractor – ESP8266 (NodeMCU v2)
 * ============================================
 * I2C (Wire por defecto en ESP8266):
 *   D1 → SCL (GPIO5)
 *   D2 → SDA (GPIO4)
 *
 * Módulo ENS160+AHT21 (mismo bus I2C):
 *   ENS160 @ 0x52 → AQI (1–5) / TVOC (ppb) / eCO₂ (ppm)
 *   AHT21  @ 0x38 → Temperatura (°C) / Humedad relativa (%)
 *
 * D5 → Relay IN (GPIO14) – NC active-low
 *   PIN HIGH → bobina en reposo → NC cerrado → extractor ON
 *   PIN LOW  → bobina activada  → NC abierto → extractor OFF
 *
 * Lógica de disparo (modo AUTO):
 *   ENCENDER: AQI >= thresh1  O  Humedad >= thresh2
 *   APAGAR:   AQI <  thresh1  Y  Humedad < (thresh2 - HYST_HUM)
 *   Si ENS160 aún está calentando, solo la humedad actúa como disparador.
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ScioSense_ENS16x.h>
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>

/* ─── Pins ───────────────────────────────────────────────────
 * I2C usa D1(SCL/GPIO5) y D2(SDA/GPIO4) — pines Wire por defecto.
 * El relay SE MOVIÓ de D1 a D5 para liberar el bus I2C.          */
#define PIN_RELAY D5            // GPIO14

/* ─── Relay NC active-low ─────────────────────────────────── */
#define RELAY_ON   HIGH         // bobina en reposo → NC cerrado → ON
#define RELAY_OFF  LOW          // bobina activada  → NC abierto → OFF

/* ─── Histéresis ──────────────────────────────────────────── */
// AQI es entero 1–5; el ENS160 filtra internamente, sin histéresis extra.
// La humedad sí usa ±5 % para evitar chatter.
#define HYST_HUM  5.0f

/* ─── Intervalos ──────────────────────────────────────────── */
#define INTERVAL_SENSOR      10000UL  // leer sensores cada 10 s
#define INTERVAL_MQTT        15000UL  // publicar MQTT cada 15 s
#define INTERVAL_RECONNECT    5000UL  // reintentar conexión MQTT cada 5 s
#define INTERVAL_WIFI_WDT   120000UL  // reiniciar si WiFi caído > 2 min

/* ─── MQTT topics ─────────────────────────────────────────── */
#define BASE "extractor"
// ← Publicar
#define T_AQI          BASE "/s1/aqi"            // 1–5
#define T_TVOC         BASE "/s1/tvoc"           // ppb
#define T_ECO2         BASE "/s1/eco2"           // ppm
#define T_HUMIDITY     BASE "/s1/humidity"       // %
#define T_TEMP         BASE "/s1/temperature"    // °C
#define T_STATE        BASE "/relay/state"       // ON|OFF|OFFLINE
#define T_MODE         BASE "/mode/state"        // AUTO|MANUAL
#define T_THRESH1      BASE "/cfg/thresh1/state" // AQI umbral (1–5)
#define T_THRESH2      BASE "/cfg/thresh2/state" // Humedad umbral (%)
#define T_TIMER_ON     BASE "/cfg/timer_on/state"
#define T_TIMER_OFF    BASE "/cfg/timer_off/state"
#define T_TIMER_STATUS BASE "/timer/status"      // IDLE|ON_TIMER|COOLDOWN
// → Comandos (suscribir)
#define T_SET_RELAY     BASE "/relay/set"
#define T_SET_MODE      BASE "/mode/set"
#define T_SET_THRESH1   BASE "/cfg/thresh1/set"
#define T_SET_THRESH2   BASE "/cfg/thresh2/set"
#define T_SET_TIMER_ON  BASE "/cfg/timer_on/set"
#define T_SET_TIMER_OFF BASE "/cfg/timer_off/set"


#define T_STATUS BASE "/status"   

#define ENS160_I2C_ADDRESS 0x52

/* ─── Objetos ─────────────────────────────────────────────── */
ENS160 ens160;

Adafruit_AHTX0 aht;
WiFiClient        wifiClient;
PubSubClient      mqtt(wifiClient);
ESP8266WebServer  webServer(80);

/* ─── Estado sensores ─────────────────────────────────────── */
uint8_t  gAQI        = 1;
uint16_t gTVOC       = 0;
uint16_t gECO2       = 400;
bool     gENSValid   = false;   // true solo cuando ENS160 reporta NORMAL
float    gHumidity   = 0.0f;
float    gTemp       = 0.0f;

/* ─── Control extractor ───────────────────────────────────── */
bool     gExtractor  = false;
bool     gAutoMode   = true;
float    gThresh1    = 3.0f;   // AQI umbral: encender si AQI >= este valor
float    gThresh2    = 80.0f;  // Humedad %: encender si humedad >= este valor

/* ─── Temporizador ────────────────────────────────────────── */
uint16_t      gTimerOnMin    = 0;   // 0 = desactivado
uint16_t      gTimerOffMin   = 0;   // 0 = desactivado
unsigned long tExtractorOn   = 0;
unsigned long tCooldownStart = 0;
bool          gInCooldown    = false;

/* ─── Config MQTT ─────────────────────────────────────────── */
char cfgHost[64] = "";
char cfgPort[6]  = "1883";
char cfgUser[32] = "";
char cfgPass[32] = "";
bool needSave = false;

/* ─── Timers globales ─────────────────────────────────────── */
unsigned long tSensor    = 0;
unsigned long tMqtt      = 0;
unsigned long tRecon     = 0;
unsigned long tWifiLost  = 0;
bool          wifiWasLost = false;

/* ══════════════════════════════════════════════════════════
   LittleFS – carga y guardado de configuración
   ══════════════════════════════════════════════════════════ */
void loadConfig() {
    if (!LittleFS.begin()) { Serial.println("LittleFS error"); return; }
    File f = LittleFS.open("/cfg.json", "r");
    if (!f) return;
    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        strlcpy(cfgHost, doc["h"] | "",     sizeof(cfgHost));
        strlcpy(cfgPort, doc["p"] | "1883", sizeof(cfgPort));
        strlcpy(cfgUser, doc["u"] | "",     sizeof(cfgUser));
        strlcpy(cfgPass, doc["w"] | "",     sizeof(cfgPass));
        gThresh1     = doc["t1"]  | 3.0f;
        gThresh2     = doc["t2"]  | 80.0f;
        gTimerOnMin  = doc["ton"] | 0;
        gTimerOffMin = doc["tof"] | 0;
    }
    f.close();
    Serial.printf("Config OK. MQTT: %s:%s\n", cfgHost, cfgPort);
}

void saveConfig() {
    if (!LittleFS.begin()) return;
    File f = LittleFS.open("/cfg.json", "w");
    if (!f) return;
    JsonDocument doc;
    doc["h"]   = cfgHost;     doc["p"]   = cfgPort;
    doc["u"]   = cfgUser;     doc["w"]   = cfgPass;
    doc["t1"]  = gThresh1;    doc["t2"]  = gThresh2;
    doc["ton"] = gTimerOnMin; doc["tof"] = gTimerOffMin;
    serializeJson(doc, f);
    f.close();
    Serial.println("Config guardado.");
}

/* ══════════════════════════════════════════════════════════
   Relay
   ══════════════════════════════════════════════════════════ */
void setExtractor(bool on) {
    if (on == gExtractor) return;
    gExtractor = on;
    digitalWrite(PIN_RELAY, on ? RELAY_ON : RELAY_OFF);
    if (on) {
        tExtractorOn = millis();
        gInCooldown  = false;
    }
    Serial.printf("Extractor: %s\n", on ? "ON" : "OFF");
}

/* ══════════════════════════════════════════════════════════
   Lógica automática: histéresis + temporizador
   ══════════════════════════════════════════════════════════ */
void evaluateAuto() {
    if (!gAutoMode) return;
    unsigned long now = millis();

    /* ── Timer: apagado por tiempo máximo encendido ── */
    if (gExtractor && gTimerOnMin > 0) {
        if (now - tExtractorOn >= (unsigned long)gTimerOnMin * 60000UL) {
            Serial.println("Timer ON expirado → apagando.");
            setExtractor(false);
            if (gTimerOffMin > 0) {
                gInCooldown    = true;
                tCooldownStart = now;
                Serial.printf("Cooldown: %u min.\n", gTimerOffMin);
            }
            return;
        }
    }

    /* ── Cooldown: espera antes de volver a encender ── */
    if (gInCooldown) {
        if (now - tCooldownStart < (unsigned long)gTimerOffMin * 60000UL)
            return;
        gInCooldown = false;
        Serial.println("Cooldown terminado.");
    }

    /* ── Disparadores ────────────────────────────────────────
     *  ENCENDER: AQI >= thresh1  O  Humedad >= thresh2
     *  APAGAR:   AQI <  thresh1  Y  Humedad < (thresh2 - HYST_HUM)
     *
     *  gENSValid = false durante WARMUP/INITIAL (~3 min, primer arranque).
     *  En ese período el AQI se ignora; solo la humedad actúa como
     *  disparador para evitar falsos positivos con datos no fiables.
     * ──────────────────────────────────────────────────────── */
    bool trigAQI = gENSValid && ((float)gAQI >= gThresh1);
    bool trigHum = (gHumidity >= gThresh2);

    if (!gExtractor) {
        if (trigAQI || trigHum) setExtractor(true);
    } else {
        bool okAQI = !gENSValid || ((float)gAQI < gThresh1);
        bool okHum = (gHumidity < gThresh2 - HYST_HUM);
        if (okAQI && okHum) setExtractor(false);
    }
}

/* ══════════════════════════════════════════════════════════
   Sensores
   ══════════════════════════════════════════════════════════ */
void readSensors() {
    /* 1. AHT21 primero – sus datos se usan para compensar el ENS160 */
    sensors_event_t evtHum, evtTemp;
    if (aht.getEvent(&evtHum, &evtTemp)) {
        if (!isnan(evtTemp.temperature))      gTemp     = evtTemp.temperature;
        if (!isnan(evtHum.relative_humidity)) gHumidity = evtHum.relative_humidity;
        /* Compensación de T y HR mejora precisión de TVOC/eCO₂ */
        ens160.writeCompensation(gTemp, gHumidity);
    } else {
        Serial.println("[AHT21] error de lectura.");
    }

    

    /* 2. ENS160 */
    ens160.wait();
    if (ens160.update() == RESULT_OK)
     {
        if (ens160.hasNewData())
        {
            gAQI = ens160.getAirQualityIndex_UBA();
            gTVOC = ens160.getTvoc();
            gECO2 = ens160.getEco2();
            gENSValid = true;
        }
    }

    else
    {
        // Opcional: Solo si la comunicación I2C falla catastróficamente
        // marcamos que el dato ya no es válido.
        Serial.println("[ENS160] error de lectura.");
        gENSValid = false;
    }


    Serial.printf(
        "ENS160 AQI:%u  TVOC:%u ppb  eCO2:%u ppm | "
        "AHT21  HR:%.1f%%  T:%.1f°C\n",
         gAQI, gTVOC, gECO2, gHumidity, gTemp);
}

/* ══════════════════════════════════════════════════════════
   MQTT – publicar todo el estado
   ══════════════════════════════════════════════════════════ */
void publishAll() {
    if (!mqtt.connected()) return;
    char buf[12];

    auto pubF = [&](const char* t, float v, uint8_t dec = 1) {
        dtostrf(v, 1, dec, buf);
        mqtt.publish(t, buf, true);
    };
    auto pubU = [&](const char* t, uint32_t v) {
        snprintf(buf, sizeof(buf), "%u", v);
        mqtt.publish(t, buf, true);
    };

    pubU(T_AQI,  gAQI);
    pubU(T_TVOC, gTVOC);
    pubU(T_ECO2, gECO2);

    pubF(T_HUMIDITY, gHumidity);
    pubF(T_TEMP,     gTemp);

    pubF(T_THRESH1, gThresh1, 0);   // AQI sin decimales
    pubF(T_THRESH2, gThresh2);

    pubU(T_TIMER_ON,  gTimerOnMin);
    pubU(T_TIMER_OFF, gTimerOffMin);

    mqtt.publish(T_STATE, gExtractor ? "ON"   : "OFF",    true);
    mqtt.publish(T_MODE,  gAutoMode  ? "AUTO" : "MANUAL", true);

    const char* ts = gInCooldown                     ? "COOLDOWN"
                   : (gExtractor && gTimerOnMin > 0) ? "ON_TIMER"
                   : "IDLE";
    mqtt.publish(T_TIMER_STATUS, ts, true);
}

/* ══════════════════════════════════════════════════════════
   MQTT – callback de comandos entrantes
   ══════════════════════════════════════════════════════════ */
void mqttCallback(char* topic, byte* payload, unsigned int len) {
    char msg[64] = {0};
    memcpy(msg, payload, min(len, (unsigned int)63));
    Serial.printf("MQTT ← [%s]: %s\n", topic, msg);

    if (strcmp(topic, T_SET_RELAY) == 0) {
        gAutoMode = false; gInCooldown = false;
        setExtractor(strcmp(msg, "ON") == 0);

    } else if (strcmp(topic, T_SET_MODE) == 0) {
        gAutoMode = (strcmp(msg, "AUTO") == 0);
        if (gAutoMode) evaluateAuto();

    } else if (strcmp(topic, T_SET_THRESH1) == 0) {
        gThresh1 = constrain(atof(msg), 1.0f, 5.0f);
        saveConfig(); evaluateAuto();

    } else if (strcmp(topic, T_SET_THRESH2) == 0) {
        gThresh2 = constrain(atof(msg), 0.0f, 100.0f);
        saveConfig(); evaluateAuto();

    } else if (strcmp(topic, T_SET_TIMER_ON) == 0) {
        gTimerOnMin = (uint16_t)constrain(atoi(msg), 0, 1440);
        saveConfig();

    } else if (strcmp(topic, T_SET_TIMER_OFF) == 0) {
        gTimerOffMin = (uint16_t)constrain(atoi(msg), 0, 1440);
        saveConfig();
    }
    publishAll();
}

/* ══════════════════════════════════════════════════════════
   MQTT – reconexión (no bloqueante)
   ══════════════════════════════════════════════════════════ */
/* ══════════════════════════════════════════════════════════
   MQTT – reconexión (no bloqueante)
   ══════════════════════════════════════════════════════════ */
bool mqttReconnect()
{
    if (mqtt.connected())
        return true;
    unsigned long now = millis();
    if (now - tRecon < INTERVAL_RECONNECT)
        return false;
    tRecon = now;
    if (cfgHost[0] == '\0')
        return false;

    Serial.printf("MQTT → %s:%s  (heap: %u B)…\n", cfgHost, cfgPort, ESP.getFreeHeap());
    String cid = "esp-ext-" + String(ESP.getChipId(), HEX);

    // Aquí está el cambio del LWT:
    bool ok = mqtt.connect(
        cid.c_str(),
        cfgUser[0] ? cfgUser : nullptr,
        cfgPass[0] ? cfgPass : nullptr,
        T_STATUS, 0, true, "offline", // LWT: si desconecta → "offline"
        true
    );

    if (ok)
    {
        Serial.println("MQTT OK.");
        // Publicamos que estamos vivos
        mqtt.publish(T_STATUS, "online", true);

        mqtt.subscribe(T_SET_RELAY);
        mqtt.subscribe(T_SET_MODE);
        mqtt.subscribe(T_SET_THRESH1);
        mqtt.subscribe(T_SET_THRESH2);
        mqtt.subscribe(T_SET_TIMER_ON);
        mqtt.subscribe(T_SET_TIMER_OFF);
        publishAll();
    }
    else
    {
        Serial.printf("MQTT error %d\n", mqtt.state());
    }
    return ok;
}

/* ══════════════════════════════════════════════════════════
   Web UI
   ══════════════════════════════════════════════════════════ */
const char HTML[] PROGMEM = R"html(
<!DOCTYPE html><html lang="es"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Extractor</title>
<style>
  *{box-sizing:border-box}
  body{font-family:sans-serif;max-width:440px;margin:0 auto;padding:16px;background:#f0f2f5}
  h2{margin:0 0 14px;color:#222}
  .card{background:#fff;border-radius:12px;padding:14px 16px;margin:10px 0;box-shadow:0 1px 4px #0001}
  .card h3{margin:0 0 10px;font-size:.95em;color:#555}
  .row{display:flex;justify-content:space-between;align-items:center;margin:6px 0}
  .val{font-size:1.2em;font-weight:700;color:#1565c0}
  .aqi1{color:#2e7d32}.aqi2{color:#558b2f}.aqi3{color:#f57f17}.aqi4{color:#e65100}.aqi5{color:#c62828}
  .on{color:#2e7d32;font-weight:700}.off{color:#aaa;font-weight:700}
  .badge{display:inline-block;padding:2px 10px;border-radius:12px;font-size:.8em;font-weight:700}
  .ba{background:#e3f2fd;color:#1565c0}.bm{background:#fff3e0;color:#e65100}
  .bc{background:#fce4ec;color:#c62828}.bw{background:#f3e5f5;color:#6a1b9a}
  button{padding:8px 14px;border:none;border-radius:6px;cursor:pointer;font-size:.9em;margin:3px 2px}
  .b-on{background:#2e7d32;color:#fff}.b-off{background:#c62828;color:#fff}.b-auto{background:#1565c0;color:#fff}
  input[type=number]{width:68px;padding:5px 7px;border:1px solid #ccc;border-radius:5px;font-size:.95em}
  .lbl{color:#666;font-size:.88em}
  .tbox{background:#f5f5f5;border-radius:8px;padding:8px 10px;margin-top:8px}
  .note{font-size:.8em;color:#999;margin:4px 0 8px}
</style></head><body>
<h2>Extractor</h2>

<div class="card">
  <h3>Calidad del aire &nbsp;</h3>
  <div class="row"><span class="lbl">🏭 AQI (1–5)</span><span id="vaqi" class="val">–</span></div>
  <div class="row"><span class="lbl">💨 TVOC</span><span class="val" id="vtvoc">–</span></div>
  <div class="row"><span class="lbl">☁️ eCO₂</span><span class="val" id="veco2">–</span></div>
</div>

<div class="card">
  <h3>Ambiente</h3>
  <div class="row"><span class="lbl">💦 Humedad</span><span class="val" id="vhum">–</span></div>
  <div class="row"><span class="lbl">🌡️ Temperatura</span><span class="val" id="vtemp">–</span></div>
</div>

<div class="card">
  <h3>Extractor &nbsp;<span id="mode-b" class="badge">–</span></h3>
  <div class="row">
    <span>Estado: <span id="ext-s">–</span></span>
    <span id="timer-info" style="font-size:.82em;color:#888"></span>
  </div>
  <div style="margin-top:8px">
    <button class="b-on"   onclick="cmd('/set?relay=ON')">▶ Encender</button>
    <button class="b-off"  onclick="cmd('/set?relay=OFF')">■ Apagar</button>
    <button class="b-auto" onclick="cmd('/set?mode=AUTO')">⟳ Auto</button>
  </div>
</div>

<div class="card">
  <h3>Umbrales <span style="font-weight:400;color:#aaa;font-size:.85em">(Humedad ±5% histéresis)</span></h3>
  <div class="row">
    <span class="lbl">AQI mín. (1–5)</span>
    <span><input type="number" id="t1" min="1" max="5" step="1">
          <button onclick="setV('t1')">✓</button></span>
  </div>
  <div class="row">
    <span class="lbl">Humedad mín.</span>
    <span><input type="number" id="t2" min="0" max="100">%
          <button onclick="setV('t2')">✓</button></span>
  </div>
</div>

<div class="card">
  <h3>⏱ Temporizador</h3>
  <p class="note">0 minutos = desactivado</p>
  <div class="tbox">
    <div class="row">
      <span class="lbl">Máx. encendido</span>
      <span><input type="number" id="ton" min="0" max="1440"> min
            <button onclick="setV('ton')">✓</button></span>
    </div>
    <div class="row">
      <span class="lbl">Cooldown (espera)</span>
      <span><input type="number" id="tof" min="0" max="1440"> min
            <button onclick="setV('tof')">✓</button></span>
    </div>
  </div>
</div>

<script>
var AQI_LBL=['','Buena','Moderada','Sensible','Mala','Pésima'];
function refresh(){
  fetch('/api').then(r=>r.json()).then(d=>{
    var aq=document.getElementById('vaqi');
    aq.textContent=d.aqi+(d.aqi>=1&&d.aqi<=5?' – '+AQI_LBL[d.aqi]:'');
    aq.className='val aqi'+d.aqi;
    document.getElementById('vtvoc').textContent=d.tvoc+' ppb';
    document.getElementById('veco2').textContent=d.eco2+' ppm';
    document.getElementById('vhum').textContent=d.humidity.toFixed(1)+'%';
    document.getElementById('vtemp').textContent=d.temp.toFixed(1)+'°C';
    var ext=document.getElementById('ext-s');
    ext.textContent=d.relay?'ON':'OFF'; ext.className=d.relay?'on':'off';
    var mb=document.getElementById('mode-b');
    if(d.cooldown){mb.textContent='COOLDOWN';mb.className='badge bc';}
    else if(d.auto){mb.textContent='AUTO';mb.className='badge ba';}
    else{mb.textContent='MANUAL';mb.className='badge bm';}
    var ti=document.getElementById('timer-info');
    if(d.cooldown&&d.tof>0) ti.textContent='Cooldown: '+d.tof+' min';
    else if(d.relay&&d.ton>0) ti.textContent='Timer ON: max '+d.ton+' min';
    else ti.textContent='';
    document.getElementById('t1').value=d.t1;
    document.getElementById('t2').value=d.t2;
    document.getElementById('ton').value=d.ton;
    document.getElementById('tof').value=d.tof;
  }).catch(()=>{});
}
function cmd(u){fetch(u).then(refresh);}
function setV(id){cmd('/set?'+id+'='+document.getElementById(id).value);}
refresh(); setInterval(refresh,8000);
</script></body></html>
)html";

void handleRoot() { webServer.send_P(200, "text/html", HTML); }

void handleApi() {
    JsonDocument doc;
    doc["aqi"]        = gAQI;
    doc["tvoc"]       = gTVOC;
    doc["eco2"]       = gECO2;
    doc["humidity"]   = (float)round(gHumidity * 10) / 10.0f;
    doc["temp"]       = (float)round(gTemp     * 10) / 10.0f;
    doc["relay"]      = gExtractor;
    doc["auto"]       = gAutoMode;
    doc["cooldown"]   = gInCooldown;
    doc["t1"]         = gThresh1;
    doc["t2"]         = gThresh2;
    doc["ton"]        = gTimerOnMin;
    doc["tof"]        = gTimerOffMin;
    String out; serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void handleSet() {
    bool changed = false;
    if (webServer.hasArg("relay")) {
        gAutoMode = false; gInCooldown = false;
        setExtractor(webServer.arg("relay") == "ON"); changed = true;
    }
    if (webServer.hasArg("mode")) {
        gAutoMode = (webServer.arg("mode") == "AUTO");
        if (gAutoMode) evaluateAuto(); changed = true;
    }
    if (webServer.hasArg("t1")) {
        gThresh1 = constrain(webServer.arg("t1").toFloat(), 1.0f, 5.0f);
        evaluateAuto(); changed = true;
    }
    if (webServer.hasArg("t2")) {
        gThresh2 = constrain(webServer.arg("t2").toFloat(), 0.0f, 100.0f);
        evaluateAuto(); changed = true;
    }
    if (webServer.hasArg("ton")) {
        gTimerOnMin  = (uint16_t)constrain(webServer.arg("ton").toInt(), 0, 1440);
        changed = true;
    }
    if (webServer.hasArg("tof")) {
        gTimerOffMin = (uint16_t)constrain(webServer.arg("tof").toInt(), 0, 1440);
        changed = true;
    }
    if (changed) { saveConfig(); publishAll(); }
    webServer.send(200, "text/plain", "OK");
}

/* ══════════════════════════════════════════════════════════
   OTA
   ══════════════════════════════════════════════════════════ */
void setupOTA() {
    ArduinoOTA.setHostname("extractor-esp");
    ArduinoOTA.onStart([]()  { Serial.println("OTA: inicio"); });
    ArduinoOTA.onEnd([]()    { Serial.println("\nOTA: fin");  });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("OTA error [%u]\n", e);
    });
    ArduinoOTA.begin();
    Serial.println("OTA listo → 'extractor-esp'.");
}

/* ══════════════════════════════════════════════════════════
   Setup
   ══════════════════════════════════════════════════════════ */
void saveConfigCallback() { needSave = true; }

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Extractor ESP8266 ===");

    /* ── Relay: forzar OFF físicamente antes de cualquier otra init.
     * Se escribe directamente (sin pasar por setExtractor) para
     * garantizar el estado del pin independientemente del valor
     * inicial de gExtractor.                                        */
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, RELAY_OFF);
    gExtractor = false;

    /* ── I2C ──────────────────────────────────────────────────────
     * Wire.begin() sin argumentos: SDA=D2(GPIO4), SCL=D1(GPIO5)
     * Estos son los pines estándar I2C del NodeMCU v2.             */
    Wire.begin();

    /* ── ENS160 ───────────────────────────────────────────────── */
    ens160.begin(&Wire, ENS160_I2C_ADDRESS);

    Serial.println("begin ens160..");
    int retries = 0;
    while (ens160.init() != true && retries < 500)
    {
        Serial.print(".");
        delay(1000);
        retries++;
    }
    if (retries >= 500)
    {
        Serial.println("Error: ENS160 no responde. Iniciando sin sensor.");
     
    }
    else
    {
        Serial.println("success");
    }

    /* ── AHT21 ────────────────────────────────────────────────── */
    if (!aht.begin()) {
        Serial.println("[AHT21] no encontrado – verifica cableado I2C.");
    } else {
        Serial.println("[AHT21] OK.");
    }

    loadConfig();

    /* ── WiFiManager ──────────────────────────────────────────────
     * WiFi.persistent(false): impide que el SDK de ESP8266 escriba
     * credenciales en su propio sector flash en cada reconexión.
     * WiFiManager gestiona sus credenciales en LittleFS de forma
     * independiente y no se ve afectado por esta opción.
     * Debe llamarse ANTES de autoConnect().                        */
    WiFi.persistent(false);

    WiFiManagerParameter pHost("h", "MQTT Host", cfgHost, 63);
    WiFiManagerParameter pPort("p", "MQTT Port", cfgPort, 5);
    WiFiManagerParameter pUser("u", "MQTT User", cfgUser, 31);
    WiFiManagerParameter pPass("w", "MQTT Pass", cfgPass, 31);

    WiFiManager wm;
    wm.addParameter(&pHost);
    wm.addParameter(&pPort);
    wm.addParameter(&pUser);
    wm.addParameter(&pPass);
    wm.setSaveConfigCallback(saveConfigCallback);
    wm.setConfigPortalTimeout(180);

    if (!wm.autoConnect("ExtractorAP")) {
        Serial.println("Sin WiFi – reiniciando en 5s…");
        delay(5000);
        ESP.restart();
    }
    Serial.println("WiFi OK: " + WiFi.localIP().toString());

    /* ── WIFI_MODEM_SLEEP ─────────────────────────────────────────
     * El radio se apaga entre intervalos de beacon del AP (~100 ms),
     * reduciendo el consumo mientras mantiene la asociación intacta.
     * El ESP se despierta automáticamente en TX o al recibir datos.
     *
     * IMPORTANTE: se fija DESPUÉS de autoConnect(). Configurarlo
     * antes puede interferir con el portal captivo de WiFiManager.
     *
     * setAutoReconnect(true): el SDK reconecta automáticamente si
     * el AP desaparece y vuelve, sin intervención del firmware.    */
    WiFi.setSleepMode(WIFI_MODEM_SLEEP);
    WiFi.setAutoReconnect(true);

    if (needSave) {
        strlcpy(cfgHost, pHost.getValue(), sizeof(cfgHost));
        strlcpy(cfgPort, pPort.getValue(), sizeof(cfgPort));
        strlcpy(cfgUser, pUser.getValue(), sizeof(cfgUser));
        strlcpy(cfgPass, pPass.getValue(), sizeof(cfgPass));
        saveConfig();
    }

    Serial.printf("Heap libre tras WiFi: %u bytes\n", ESP.getFreeHeap());

    mqtt.setServer(cfgHost, atoi(cfgPort));
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);
    mqtt.setBufferSize(512);

    webServer.on("/",    handleRoot);
    webServer.on("/api", handleApi);
    webServer.on("/set", handleSet);
    webServer.begin();

    setupOTA();

    readSensors();
    evaluateAuto();
}

/* ══════════════════════════════════════════════════════════
   Loop
   ══════════════════════════════════════════════════════════ */
void loop() {
    /* ── WiFi watchdog ────────────────────────────────────────────
     * setAutoReconnect gestiona la reconexión habitual, pero en
     * algunos casos el SDK de ESP8266 puede quedarse en un estado
     * corrupto indefinidamente. Un reinicio limpio garantiza la
     * recuperación. Umbral: 2 minutos sin WiFi → ESP.restart().   */
    if (WiFi.status() != WL_CONNECTED) {
        if (!wifiWasLost) {
            wifiWasLost = true;
            tWifiLost   = millis();
            Serial.println("WiFi desconectado.");
        } else if (millis() - tWifiLost > INTERVAL_WIFI_WDT) {
            Serial.println("WiFi perdido > 2 min → reiniciando.");
            ESP.restart();
        }
    } else {
        wifiWasLost = false;
    }

    ArduinoOTA.handle();
    webServer.handleClient();
    mqttReconnect();
    mqtt.loop();

    unsigned long now = millis();

    if (now - tSensor >= INTERVAL_SENSOR) {
        tSensor = now;
        readSensors();
        evaluateAuto();
    }
    if (now - tMqtt >= INTERVAL_MQTT) {
        tMqtt = now;
        Serial.printf("[heap] libre: %u B  frag: %u%%\n",
                      ESP.getFreeHeap(), ESP.getHeapFragmentation());
        publishAll();
    }
}