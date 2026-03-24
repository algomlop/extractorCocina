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
 * D5 → Relay IN (GPIO14) – NO active-low  ← usa contacto Normalmente Abierto
 *   PIN LOW  → bobina activada  → NO cerrado → extractor ON
 *   PIN HIGH → bobina en reposo → NO abierto → extractor OFF  (fail-safe)
 *
 * A0 → Sensor humedad suelo (capacitivo, 0–3,3 V):
 *   Raw alto (~820) = seco  →  0 % humedad suelo
 *   Raw bajo (~380) = húmedo → 100 % humedad suelo
 *   Calibrar SOIL_DRY / SOIL_WET según tu sensor específico.
 *
 * Lógica de disparo (modo AUTO):
 *   ENCENDER: AQI >= thresh1  O  Humedad_aire >= thresh2  O  Suelo >= thresh3
 *   APAGAR:   AQI <  thresh1  Y  Humedad_aire < (thresh2 - HYST_HUM)
 *                              Y  Suelo < (thresh3 - HYST_SOIL)
 *   Si ENS160 aún está calentando, el AQI se ignora.
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ScioSense_ENS16x.h>
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include "include/secrets.h"
#include <TelnetStream.h>


void logToFile(const char* msg);

#define LOG_PRINT(x)                                                           \
    {                                                                          \
        Serial.print(x);                                                       \
        if (telnetReady)                                                       \
            TelnetStream.print(x);                                             \
        /* LOG_PRINT no escribe a fichero — sin newline, se acumularía roto */ \
    }

#define LOG_PRINTLN(x)                    \
    {                                     \
        Serial.println(x);                \
        if (telnetReady)                  \
            TelnetStream.println(x);      \
        {                                 \
            String _s = String(x) + "\n"; \
            logToFile(_s.c_str());        \
        }                                 \
    }

#define LOG_PRINTLN0()              \
    {                               \
        Serial.println();           \
        if (telnetReady)            \
            TelnetStream.println(); \
        logToFile("\n");            \
    }

#define LOG_PRINTF(fmt, ...)                                  \
    {                                                         \
        Serial.printf(fmt, ##__VA_ARGS__);                    \
        if (telnetReady)                                      \
        {                                                     \
            char _buf[256];                                   \
            snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
            TelnetStream.print(_buf);                         \
            logToFile(_buf);                                  \
        }                                                     \
        else                                                  \
        {                                                     \
            char _buf[256];                                   \
            snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
            logToFile(_buf);                                  \
        }                                                     \
    }

#define LOG_PRINTF_P(fmt, ...)                                  \
    {                                                           \
        Serial.printf_P(fmt, ##__VA_ARGS__);                    \
        {                                                       \
            char _buf[256];                                     \
            snprintf_P(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
            if (telnetReady)                                    \
                TelnetStream.print(_buf);                       \
            logToFile(_buf);                                    \
        }                                                       \
    }
#define MAX_LOG_BYTES 16384UL // 16 KB — ajusta según tu LittleFS


/* ─── Pins ────────────────────────────────────────────────────
 * I2C usa D1(SCL/GPIO5) y D2(SDA/GPIO4) — pines Wire por defecto.
 * El relay está en D5 para liberar el bus I2C.
 * A0 es el único pin ADC del NodeMCU v2.                        */
#define PIN_RELAY D5     // GPIO14
#define PIN_SOIL A0      // ADC – sensor humedad suelo (capacitivo)
#define PIN_CFG_FORCE D3 // GPIO0 – mantener pulsado en boot para forzar portal WiFiManager

/* ─── Relay NO  ─────────────────────────*/
#define RELAY_ON HIGH // bobina activada  → NO cerrado → extractor ON
#define RELAY_OFF LOW // bobina en reposo → NO abierto → extractor OFF

/* ─── Calibración sensor suelo ────────────────────────────────
 *  Ajusta estos valores midiendo con tu sensor:
 *    analogRead() con el sensor en aire seco  → SOIL_DRY
 *    analogRead() con el sensor sumergido     → SOIL_WET */
#define SOIL_DRY 964 // ADC raw ~ suelo completamente seco
#define SOIL_WET 339 // ADC raw ~ suelo completamente húmedo

/* ─── Histéresis ──────────────────────────────────────────────
 *  Evita chatter en los disparadores de humedad.               */
#define HYST_HUM 5.0f  // ±5 % humedad aire
#define HYST_SOIL 5.0f // ±5 % humedad suelo

/* ─── Intervalos ──────────────────────────────────────────────*/
#define INTERVAL_SENSOR 10000UL    // leer sensores cada 10 s
#define INTERVAL_MQTT 60000UL      // publicar MQTT cada 60 s
#define INTERVAL_RECONNECT 5000UL  // reintentar conexión MQTT cada 5 s
#define INTERVAL_WIFI_WDT 120000UL // reiniciar si WiFi caído > 2 min

/* ─── MQTT topics ─────────────────────────────────────────────*/
#define BASE "extractor"
// ← Publicar
#define T_AQI BASE "/s1/aqi"                // 1–5
#define T_TVOC BASE "/s1/tvoc"              // ppb
#define T_ECO2 BASE "/s1/eco2"              // ppm
#define T_HUMIDITY BASE "/s1/humidity"      // % aire
#define T_TEMP BASE "/s1/temperature"       // °C
#define T_SOIL BASE "/s2/soil"              // % humedad suelo
#define T_STATE BASE "/relay/state"         // ON|OFF
#define T_MODE BASE "/mode/state"           // AUTO|MANUAL
#define T_THRESH1 BASE "/cfg/thresh1/state" // AQI umbral (1–5)
#define T_THRESH2 BASE "/cfg/thresh2/state" // Humedad aire %
#define T_THRESH3 BASE "/cfg/thresh3/state" // Humedad suelo %
#define T_TIMER_ON BASE "/cfg/timer_on/state"
#define T_TIMER_OFF BASE "/cfg/timer_off/state"
#define T_TIMER_STATUS BASE "/timer/status" // IDLE|ON_TIMER|COOLDOWN
// → Comandos (suscribir)
#define T_SET_RELAY BASE "/relay/set"
#define T_SET_MODE BASE "/mode/set"
#define T_SET_THRESH1 BASE "/cfg/thresh1/set"
#define T_SET_THRESH2 BASE "/cfg/thresh2/set"
#define T_SET_THRESH3 BASE "/cfg/thresh3/set"
#define T_SET_TIMER_ON BASE "/cfg/timer_on/set"
#define T_SET_TIMER_OFF BASE "/cfg/timer_off/set"

#define T_STATUS BASE "/status"

#define ENS160_I2C_ADDRESS 0x52

#define PIN_SOIL_VCC D6 // GPIO12 — alimentación del sensor

/* ─── Objetos ─────────────────────────────────────────────────*/
ENS160 ens160;
Adafruit_AHTX0 aht;
BearSSL::WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);
ESP8266WebServer webServer(80);

bool telnetReady = false;
bool littleFSok=false;

/* ─── Estado sensores ─────────────────────────────────────────*/
uint8_t gAQI = 1;
uint16_t gTVOC = 0;
uint16_t gECO2 = 400;
bool gENSValid = false; // true solo cuando ENS160 reporta NORMAL
float gHumidity = 0.0f; // % humedad relativa aire (AHT21)
float gTemp = 0.0f;     // °C (AHT21)
float gSoilHum = 0.0f;  // % humedad suelo (A0)

bool gAHTok = false;
bool gENSok = false;

/* ─── Control extractor ───────────────────────────────────────*/
bool gExtractor = false;
bool gAutoMode = true;
float gThresh1 = 3.0f;  // AQI umbral: encender si AQI >= este valor
float gThresh2 = 80.0f; // Humedad aire %: encender si >= este valor
float gThresh3 = 60.0f; // Humedad suelo %: encender si >= este valor

/* ─── Temporizador ────────────────────────────────────────────*/
uint16_t gTimerOnMin = 0;
uint16_t gTimerOffMin = 0;
unsigned long tExtractorOn = 0;
unsigned long tCooldownStart = 0;
bool gInCooldown = false;

/* ─── Config MQTT ─────────────────────────────────────────────*/
char cfgHost[64] = SECRET_MQTT_HOST;
char cfgPort[6] = SECRET_MQTT_PORT;
char cfgUser[32] = SECRET_MQTT_USER;
char cfgPass[32] = SECRET_MQTT_PASS;

/* ─── Timers globales ─────────────────────────────────────────*/
unsigned long tSensor = 0;
unsigned long tMqtt = 0;
unsigned long tRecon = 0;
unsigned long tWifiLost = 0;
bool wifiWasLost = false;

/* ══════════════════════════════════════════════════════════
   LittleFS – carga y guardado de configuración
   ══════════════════════════════════════════════════════════ */
void loadConfig()
{
    if (!littleFSok)
    {
        LOG_PRINTLN(F("El begin del LittleFS dio error"));
        return;
    }
    // borrado /cfg.json
    // LittleFS.remove("/cfg.json");

    File f = LittleFS.open("/cfg.json", "r");
    if (!f)
        return;
    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok)
    {

        gThresh1 = doc["t1"] | 3.0f;
        gThresh2 = doc["t2"] | 80.0f;
        gThresh3 = doc["t3"] | 60.0f;
        gTimerOnMin = doc["ton"] | 0;
        gTimerOffMin = doc["tof"] | 0;
    }
    f.close();
    LOG_PRINTF_P(PSTR("Config OK. MQTT: %s:%s\n"), cfgHost, cfgPort);
}

void saveConfig()
{
    if (!littleFSok)
    {
        LOG_PRINTF_P(PSTR("El begin del LittleFS no funcionó")); 
        return;
    }
    File f = LittleFS.open("/cfg.json", "w");
    if (!f)
        return;
    JsonDocument doc;
    doc["t1"] = gThresh1;
    doc["t2"] = gThresh2;
    doc["t3"] = gThresh3;
    doc["ton"] = gTimerOnMin;
    doc["tof"] = gTimerOffMin;
    serializeJson(doc, f);
    f.close();
    LOG_PRINTLN(F("Config guardado."));
}


void logToFile(const char *msg)
{
    File f = LittleFS.open("/log.txt", "a");
    if (!f)
        return;

    // Rotación si supera el límite
    if (f.size() >= MAX_LOG_BYTES)
    {
        f.close();

        // Copiar la 2ª mitad a /log.tmp en chunks (RAM mínima)
        File src = LittleFS.open("/log.txt", "r");
        File dst = LittleFS.open("/log.tmp", "w");
        if (src && dst)
        {
            src.seek(MAX_LOG_BYTES / 2);
            uint8_t chunk[256];
            while (src.available())
            {
                size_t n = src.read(chunk, sizeof(chunk));
                dst.write(chunk, n);
            }
        }
        src.close();
        dst.close();

        LittleFS.remove("/log.txt");
        LittleFS.rename("/log.tmp", "/log.txt");

        f = LittleFS.open("/log.txt", "a");
        if (!f)
            return;
    }

    f.print(msg);
    f.close();
}

/* ══════════════════════════════════════════════════════════
   Relay
   ══════════════════════════════════════════════════════════ */
void setExtractor(bool on)
{
    if (on == gExtractor)
        return;
    gExtractor = on;
    digitalWrite(PIN_RELAY, on ? RELAY_ON : RELAY_OFF);
    if (on)
    {
        tExtractorOn = millis();
        gInCooldown = false;
    }
    LOG_PRINTF_P(PSTR("Extractor: %s\n"), on ? "ON" : "OFF");
}

/* ══════════════════════════════════════════════════════════
   Lógica automática: histéresis + temporizador
   ══════════════════════════════════════════════════════════ */
void evaluateAuto()
{
    if (!gAutoMode)
        return;
    unsigned long now = millis();

    /* ── Timer: apagado por tiempo máximo encendido ── */
    if (gExtractor && gTimerOnMin > 0)
    {
        if (now - tExtractorOn >= (unsigned long)gTimerOnMin * 60000UL)
        {
            LOG_PRINTLN(F("Timer ON expirado → apagando."));
            setExtractor(false);
            if (gTimerOffMin > 0)
            {
                gInCooldown = true;
                tCooldownStart = now;
                LOG_PRINTF_P(PSTR("Cooldown: %u min.\n"), gTimerOffMin);
            }
            return;
        }
    }

    /* ── Cooldown: espera antes de volver a encender ── */
    if (gInCooldown)
    {
        if (now - tCooldownStart < (unsigned long)gTimerOffMin * 60000UL)
            return;
        gInCooldown = false;
        LOG_PRINTLN(F("Cooldown terminado."));
    }

    /* ── Disparadores ──────────────────────────────────────────
     *  ENCENDER: AQI >= thresh1  O  Humedad_aire >= thresh2  O  Suelo >= thresh3
     *  APAGAR:   AQI <  thresh1  Y  Humedad_aire < (thresh2 - HYST_HUM)
     *                             Y  Suelo < (thresh3 - HYST_SOIL)
     *
     *  gENSValid = false durante WARMUP/INITIAL (~3 min, primer arranque).
     *  En ese período el AQI se ignora para evitar falsos positivos.
     * ─────────────────────────────────────────────────────────── */
    bool trigAQI = gENSValid && ((float)gAQI >= gThresh1);
    bool trigHum = (gHumidity >= gThresh2);
    bool trigSoil = (gSoilHum >= gThresh3);

    if (!gExtractor)
    {
        if (trigAQI || trigHum || trigSoil)
            setExtractor(true);
    }
    else
    {
        bool okAQI = !gENSValid || ((float)gAQI < gThresh1);
        bool okHum = (gHumidity < gThresh2 - HYST_HUM);
        bool okSoil = (gSoilHum < gThresh3 - HYST_SOIL);
        if (okAQI && okHum && okSoil)
            setExtractor(false);
    }
}

/* ══════════════════════════════════════════════════════════
   Sensores
   ══════════════════════════════════════════════════════════ */
void readSensors()
{
    /* 1. AHT21 primero – sus datos se usan para compensar el ENS160 */
    if (gAHTok)
    {

        sensors_event_t evtHum, evtTemp;
        if (aht.getEvent(&evtHum, &evtTemp))
        {
            if (!isnan(evtTemp.temperature))
                gTemp = evtTemp.temperature;
            if (!isnan(evtHum.relative_humidity))
                gHumidity = evtHum.relative_humidity;
            ens160.writeCompensation(gTemp, gHumidity);
        }
        else
        {
            LOG_PRINTLN(F("[AHT21] error de lectura."));
        }
    }

    /* 2. ENS160 */
    if (gENSok)
    {
        // ens160.wait();
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
            LOG_PRINTLN(F("[ENS160] error de lectura."));
            gENSValid = false;
        }
    }

    /* 3. Sensor humedad suelo (A0, capacitivo)
     *  El ADC del NodeMCU v2 mide 0–3,3 V → valores 0–1023.
     *  El sensor capacitivo da voltaje ALTO en seco y BAJO en húmedo.
     *  Mapeamos inversamente: 100 % = muy húmedo, 0 % = seco.         */

    analogWrite(PIN_SOIL_VCC, 128); // PWM 50% duty cycle
    delay(200);                     // dejar estabilizar el filtro RC
    int raw = analogRead(PIN_SOIL);
    digitalWrite(PIN_SOIL_VCC, LOW); // apagar inmediatamente

    int clampedRaw = constrain(raw, SOIL_WET, SOIL_DRY);
    gSoilHum = 100.0f * (SOIL_DRY - clampedRaw) / (float)(SOIL_DRY - SOIL_WET);

    LOG_PRINTF_P(
        PSTR("ENS160 AQI:%u  TVOC:%u ppb  eCO2:%u ppm | "
             "AHT21  HR:%.1f%%  T:%.1f°C | "
             "Suelo  raw:%d  %.1f%%\n"),
        gAQI, gTVOC, gECO2, gHumidity, gTemp, raw, gSoilHum);
}

/* ══════════════════════════════════════════════════════════
   MQTT – publicar todo el estado
   ══════════════════════════════════════════════════════════ */
void publishAll()
{
    if (!mqtt.connected())
        return;

    char buf[12];

    auto pubF = [&](const char *t, float v, uint8_t dec = 1)
    {
        dtostrf(v, 1, dec, buf);
        mqtt.publish(t, buf, true);
    };
    auto pubU = [&](const char *t, uint32_t v)
    {
        snprintf(buf, sizeof(buf), "%u", v);
        mqtt.publish(t, buf, true);
    };

    pubU(T_AQI, gAQI);
    pubU(T_TVOC, gTVOC);
    pubU(T_ECO2, gECO2);

    pubF(T_HUMIDITY, gHumidity);
    pubF(T_TEMP, gTemp);
    pubF(T_SOIL, gSoilHum);

    pubF(T_THRESH1, gThresh1, 0); // AQI sin decimales
    pubF(T_THRESH2, gThresh2);
    pubF(T_THRESH3, gThresh3);

    pubU(T_TIMER_ON, gTimerOnMin);
    pubU(T_TIMER_OFF, gTimerOffMin);

    mqtt.publish(T_STATE, gExtractor ? "ON" : "OFF", true);
    mqtt.publish(T_MODE, gAutoMode ? "AUTO" : "MANUAL", true);

    const char *ts = gInCooldown                       ? "COOLDOWN"
                     : (gExtractor && gTimerOnMin > 0) ? "ON_TIMER"
                                                       : "IDLE";
    mqtt.publish(T_TIMER_STATUS, ts, true);
}

/* ══════════════════════════════════════════════════════════
   MQTT – callback de comandos entrantes
   ══════════════════════════════════════════════════════════ */
void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    char msg[64] = {0};
    memcpy(msg, payload, min(len, (unsigned int)63));
    LOG_PRINTF_P(PSTR("MQTT ← [%s]: %s\n"), topic, msg);

    if (strcmp(topic, T_SET_RELAY) == 0)
    {
        gAutoMode = false;
        gInCooldown = false;
        setExtractor(strcmp(msg, "ON") == 0);
    }
    else if (strcmp(topic, T_SET_MODE) == 0)
    {
        gAutoMode = (strcmp(msg, "AUTO") == 0);
        if (gAutoMode)
            evaluateAuto();
    }
    else if (strcmp(topic, T_SET_THRESH1) == 0)
    {
        gThresh1 = constrain(atof(msg), 1.0f, 5.0f);
        saveConfig();
        evaluateAuto();
    }
    else if (strcmp(topic, T_SET_THRESH2) == 0)
    {
        gThresh2 = constrain(atof(msg), 0.0f, 100.0f);
        saveConfig();
        evaluateAuto();
    }
    else if (strcmp(topic, T_SET_THRESH3) == 0)
    {
        gThresh3 = constrain(atof(msg), 0.0f, 100.0f);
        saveConfig();
        evaluateAuto();
    }
    else if (strcmp(topic, T_SET_TIMER_ON) == 0)
    {
        gTimerOnMin = (uint16_t)constrain(atoi(msg), 0, 1440);
        saveConfig();
    }
    else if (strcmp(topic, T_SET_TIMER_OFF) == 0)
    {
        gTimerOffMin = (uint16_t)constrain(atoi(msg), 0, 1440);
        saveConfig();
    }
    publishAll();
}

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

    LOG_PRINTF_P(PSTR("MQTT → %s:%s  (heap: %u B)…\n"), cfgHost, cfgPort, ESP.getFreeHeap());
    String cid = "esp-ext-" + String(ESP.getChipId(), HEX);

    bool ok = mqtt.connect(
        cid.c_str(),
        cfgUser[0] ? cfgUser : nullptr,
        cfgPass[0] ? cfgPass : nullptr,
        T_STATUS, 0, true, "offline",
        true);

    if (ok)
    {
        LOG_PRINTLN(F("MQTT OK."));
        mqtt.publish(T_STATUS, "online", true);

        mqtt.subscribe(T_SET_RELAY);
        mqtt.subscribe(T_SET_MODE);
        mqtt.subscribe(T_SET_THRESH1);
        mqtt.subscribe(T_SET_THRESH2);
        mqtt.subscribe(T_SET_THRESH3);
        mqtt.subscribe(T_SET_TIMER_ON);
        mqtt.subscribe(T_SET_TIMER_OFF);
        publishAll();
    }
    else
    {
        LOG_PRINTF_P(PSTR("MQTT error %d\n"), mqtt.state());
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
  /* Barra de humedad suelo */
  .soil-bar-wrap{flex:1;margin:0 10px;height:10px;background:#e0e0e0;border-radius:5px;overflow:hidden}
  .soil-bar{height:100%;border-radius:5px;transition:width .6s,background .6s}
</style></head><body>
<h2>Extractor</h2>

<div class="card">
  <h3>Calidad del aire</h3>
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
  <h3>🪴 Humedad de suelo</h3>
  <div class="row">
    <span class="lbl">Suelo</span>
    <div class="soil-bar-wrap"><div class="soil-bar" id="soil-bar" style="width:0%"></div></div>
    <span class="val" id="vsoil">–</span>
  </div>
  <p class="note" style="margin-top:6px">Calibrar SOIL_DRY / SOIL_WET en el firmware según tu sensor.</p>
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
  <h3>Umbrales <span style="font-weight:400;color:#aaa;font-size:.85em">(±5 % histéresis en humedad)</span></h3>
  <div class="row">
    <span class="lbl">AQI mín. (1–5)</span>
    <span><input type="number" id="t1" min="1" max="5" step="1">
          <button onclick="setV('t1')">✓</button></span>
  </div>
  <div class="row">
    <span class="lbl">Humedad aire mín.</span>
    <span><input type="number" id="t2" min="0" max="100">%
          <button onclick="setV('t2')">✓</button></span>
  </div>
  <div class="row">
    <span class="lbl">🪴 Humedad suelo mín.</span>
    <span><input type="number" id="t3" min="0" max="100">%
          <button onclick="setV('t3')">✓</button></span>
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
<div class="card">
  <h3>Logs</h3>
  <p class="note"><a href="/log" target="_blank" style="font-size:.85em;color:#1565c0">📋 Ver log</a></p>
  
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
    // Suelo
    var sp=Math.min(100,Math.max(0,d.soil));
    document.getElementById('vsoil').textContent=sp.toFixed(0)+'%';
    var bar=document.getElementById('soil-bar');
    bar.style.width=sp+'%';
    bar.style.background=sp>70?'#1565c0':sp>40?'#43a047':'#ef6c00';
    // Extractor
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
    document.getElementById('t3').value=d.t3;
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

void handleApi()
{
    JsonDocument doc;
    doc["aqi"] = gAQI;
    doc["tvoc"] = gTVOC;
    doc["eco2"] = gECO2;
    doc["humidity"] = (float)round(gHumidity * 10) / 10.0f;
    doc["temp"] = (float)round(gTemp * 10) / 10.0f;
    doc["soil"] = (float)round(gSoilHum * 10) / 10.0f;
    doc["relay"] = gExtractor;
    doc["auto"] = gAutoMode;
    doc["cooldown"] = gInCooldown;
    doc["t1"] = gThresh1;
    doc["t2"] = gThresh2;
    doc["t3"] = gThresh3;
    doc["ton"] = gTimerOnMin;
    doc["tof"] = gTimerOffMin;
    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void handleSet()
{
    bool changed = false;
    if (webServer.hasArg("relay"))
    {
        gAutoMode = false;
        gInCooldown = false;
        setExtractor(webServer.arg("relay") == "ON");
        changed = true;
    }
    if (webServer.hasArg("mode"))
    {
        gAutoMode = (webServer.arg("mode") == "AUTO");
        if (gAutoMode)
            evaluateAuto();
        changed = true;
    }
    if (webServer.hasArg("t1"))
    {
        gThresh1 = constrain(webServer.arg("t1").toFloat(), 1.0f, 5.0f);
        evaluateAuto();
        changed = true;
    }
    if (webServer.hasArg("t2"))
    {
        gThresh2 = constrain(webServer.arg("t2").toFloat(), 0.0f, 100.0f);
        evaluateAuto();
        changed = true;
    }
    if (webServer.hasArg("t3"))
    {
        gThresh3 = constrain(webServer.arg("t3").toFloat(), 0.0f, 100.0f);
        evaluateAuto();
        changed = true;
    }
    if (webServer.hasArg("ton"))
    {
        gTimerOnMin = (uint16_t)constrain(webServer.arg("ton").toInt(), 0, 1440);
        changed = true;
    }
    if (webServer.hasArg("tof"))
    {
        gTimerOffMin = (uint16_t)constrain(webServer.arg("tof").toInt(), 0, 1440);
        changed = true;
    }
    if (changed)
    {
        saveConfig();
        publishAll();
    }
    webServer.send(200, "text/plain", "OK");
}

/* ══════════════════════════════════════════════════════════
   OTA
   ══════════════════════════════════════════════════════════ */
void setupOTA()
{
    if (!MDNS.begin("extractor-esp"))
    {
        LOG_PRINTLN("Error configurando mDNS");
    }
    else
    {
        LOG_PRINTLN("mDNS configurado: extractor-esp");
    }

    ArduinoOTA.setHostname("extractor-esp");
    ArduinoOTA.onStart([]()
                       {
    WiFi.setSleepMode(WIFI_NONE_SLEEP); // radio siempre activa durante OTA
    LOG_PRINTLN(F("OTA: inicio")); });
    ArduinoOTA.onEnd([]()
                     {
    WiFi.setSleepMode(WIFI_MODEM_SLEEP); // vuelve a dormir al terminar
    LOG_PRINTLN(F("\nOTA: fin")); });
    ArduinoOTA.onError([](ota_error_t e)
                       { LOG_PRINTF_P(PSTR("OTA error [%u]\n"), e); });
    ArduinoOTA.begin();
    LOG_PRINTLN(F("OTA listo → 'extractor-esp'."));
}


void handleLog() {
    File f = LittleFS.open("/log.txt", "r");
    if (!f) {
        webServer.send(200, "text/html",
            "<pre>Log vacío.</pre>");
        return;
    }
    // Cabecera HTML con auto-scroll y refresco
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html", "");
    webServer.sendContent(
        "<!DOCTYPE html><html><head>"
        "<meta charset=UTF-8>"
        "<meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Log – Extractor</title>"
        "<style>body{background:#111;color:#cfc;font-family:monospace;font-size:.82em;"
        "padding:10px}pre{white-space:pre-wrap;word-break:break-all}"
        "a{color:#7bf;text-decoration:none;margin-right:12px}</style></head><body>"
        "<p><a href='/'>← Inicio</a>"
        "<a href='/log'>↺ Refrescar</a>"
        "<a href='/log/clear'>🗑 Borrar</a></p><pre>"
    );
    // Stream del fichero en chunks — 0 heap extra
    uint8_t chunk[256];
    while (f.available()) {
        size_t n = f.read(chunk, sizeof(chunk));
        webServer.sendContent(reinterpret_cast<char*>(chunk), n);
    }
    f.close();
    webServer.sendContent("</pre><script>window.scrollTo(0,document.body.scrollHeight)</script>"
                          "</body></html>");
    webServer.sendContent("");
}

void handleLogClear() {
    LittleFS.remove("/log.txt");
    webServer.sendHeader("Location", "/log");
    webServer.send(303);
}


/* ══════════════════════════════════════════════════════════
   Setup
   ══════════════════════════════════════════════════════════ */

void setup()
{
    Serial.begin(115200);
    LOG_PRINTLN(F("\n=== Extractor ESP8266 ==="));

    if (!LittleFS.begin())
    {
        LOG_PRINTLN(F("ERROR: LittleFS no montado"));
        littleFSok = false;
    }
    else
    {
        LOG_PRINTLN(F("LittleFS OK."));
        littleFSok = true;
    }

    pinMode(PIN_SOIL_VCC, OUTPUT);
    digitalWrite(PIN_SOIL_VCC, LOW);
    analogWriteFreq(500);

    /* ── Relay: forzar OFF físicamente antes de cualquier otra init.
     * Con RELAY_OFF = HIGH (bobina en reposo) → contacto NO abierto
     * → extractor desconectado. Fail-safe real.                      */
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, RELAY_OFF);

    gExtractor = false;

    /* ── A0 no necesita pinMode() en ESP8266 (solo entrada ADC)      */

    /* ── I2C ────────────────────────────────────────────────────── */
    Wire.begin(4, 5); // SDA=GPIO4, SCL=GPIO5

    for (byte a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0)
        {
            LOG_PRINTF_P(PSTR("Dispositivo en 0x%02X\n"), a);
        }
        else
        {
            LOG_PRINTF_P(PSTR("Nada encontrado en 0x%02X\n"), a);
        }
    }

    loadConfig();

    /* ── WiFiManager ─────────────────────────────────────────────*/
    WiFi.persistent(false);
    WiFi.hostname("extractor-esp");
    /* D3 pulsado en boot → forzar portal de configuración        */
    pinMode(PIN_CFG_FORCE, INPUT_PULLUP);
    bool forcePortal = (digitalRead(PIN_CFG_FORCE) == LOW);
    if (forcePortal)
        LOG_PRINTLN(F("PIN_CFG_FORCE activo → forzando portal WiFi."));

    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    wm.setHostname("extractor-esp");

    bool connected;
    if (forcePortal)
    {
        connected = wm.startConfigPortal("ExtractorAP");
    }
    else
    {
        connected = wm.autoConnect("ExtractorAP");
    }

    if (!connected)
    {
        LOG_PRINTLN(F("Sin WiFi – reiniciando en 5s…"));
        delay(5000);
        ESP.restart();
    }

    LOG_PRINT(F("WiFi OK: "));
    LOG_PRINTLN(WiFi.localIP().toString().c_str());

    WiFi.setSleepMode(WIFI_MODEM_SLEEP);
    WiFi.setAutoReconnect(true);

    TelnetStream.begin();
    telnetReady = true;

    LOG_PRINTF_P(PSTR("Heap libre tras WiFi: %u bytes\n"), ESP.getFreeHeap());

    /* ── MQTT sobre TLS (puerto 8883) – sin verificar certificado */
    wifiClient.setInsecure();

    mqtt.setServer(cfgHost, atoi(cfgPort));
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);
    mqtt.setBufferSize(256);

    webServer.on("/", handleRoot);
    webServer.on("/api", handleApi);
    webServer.on("/set", handleSet);
    webServer.on("/log", handleLog);
    webServer.on("/log/clear", handleLogClear);
    webServer.begin();

    setupOTA();

    /* ── ENS160 ───────────────────────────────────────────────── */
    ens160.begin(&Wire, ENS160_I2C_ADDRESS);

    LOG_PRINTLN(F("begin ens160.."));
    int retries = 0;
    bool initOk = ens160.init();
    LOG_PRINTF_P(PSTR("ENS160 init() = %d\n"), initOk);

    while (!initOk && retries < 20)
    {
        LOG_PRINTLN(F("."));
        delay(1000);
        retries++;
    }
    gENSok = (retries < 20);
    if (!gENSok)
    {
        LOG_PRINTLN(F("Error: ENS160 no responde. Iniciando sin sensor."));
    }
    else
    {
        LOG_PRINTLN(F("success"));
    }

    /* ── AHT21 ────────────────────────────────────────────────── */
    gAHTok = aht.begin();
    if (!gAHTok)
    {
        LOG_PRINTLN(F("[AHT21] no encontrado – verifica cableado I2C."));
    }
    else
    {
        LOG_PRINTLN(F("[AHT21] OK."));
    }

    tSensor = millis() - INTERVAL_SENSOR;
}

/* ══════════════════════════════════════════════════════════
   Loop
   ══════════════════════════════════════════════════════════ */
void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (!wifiWasLost)
        {
            wifiWasLost = true;
            tWifiLost = millis();
            LOG_PRINTLN(F("WiFi desconectado."));
        }
        else if (millis() - tWifiLost > INTERVAL_WIFI_WDT)
        {
            LOG_PRINTLN(F("WiFi perdido > 2 min → reiniciando."));
            ESP.restart();
        }
    }
    else
    {
        wifiWasLost = false;
    }

    ArduinoOTA.handle();
    MDNS.update();
    webServer.handleClient();
    mqttReconnect();
    mqtt.loop();

    unsigned long now = millis();

    if (now - tSensor >= INTERVAL_SENSOR)
    {
        tSensor = now;
        readSensors();
        evaluateAuto();
    }
    if (now - tMqtt >= INTERVAL_MQTT)
    {
        tMqtt = now;
        publishAll();
    }
}