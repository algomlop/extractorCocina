/**
 * Control de Extractor – ESP8266 (NodeMCU v2)
 * ============================================
 * I2C (Wire por defecto en ESP8266):
 *   D1 → SCL (GPIO5)
 *   D2 → SDA (GPIO4)
 *
 * Módulo ENS160+AHT21 (mismo bus I2C):
 *   ENS160 @ 0x53 → AQI (1–5) / TVOC (ppb) / eCO₂ (ppm)
 *   AHT21  @ 0x38 → Temperatura (°C) / Humedad relativa (%)
 *
 * D5 → Relay IN (GPIO14)  ← usa contacto Normalmente Abierto
 *
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
#include <DNSServer.h>
#include <PubSubClient.h>
#include "ScioSense_ENS160.h"
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include "include/secrets.h"
#include <TelnetStream.h>
#include <time.h>

// comentado a proposito. ESCRITURA EN FLASH DESACTIVADA PARA EVITAR DESGASTE
// void logToFile(const char *msg);

bool timeOK = false;
struct tm timeinfo;
bool telnetReady = false;

#define RTC_MAGIC 0xCAFE1234UL

struct RTCCrashInfo
{
    uint32_t magic;      // RTC_MAGIC si los datos son válidos
    uint32_t resetCount; // reinicios consecutivos anómalos
    uint32_t reason;     // rst_reason del SDK (0=power-on, 6=exception, etc.)
    uint32_t exccause;   // causa de excepción (solo válida si reason==6)
    char lastMsg[76];    // último mensaje LOG antes del crash
    uint32_t crc;        // suma de comprobación sencilla
    // Total: 4+4+4+4+76+4 = 96 bytes = 24 words (múltiplo de 4 ✓)
};

static bool gCrashInfoValid = false;

static RTCCrashInfo gCrashInfo;

static uint32_t rtcCrc(const RTCCrashInfo &d)
{
    uint32_t c = 0;
    const uint8_t *p = (const uint8_t *)&d;
    for (size_t i = 0; i < offsetof(RTCCrashInfo, crc); i++)
        c += p[i];
    return c;
}

// Llama a esto desde LOG_PRINTLN antes de un punto crítico,
// o usa updateRTCLastMsg() periódicamente para dejar rastro.
void updateRTCLastMsg(const char *msg)
{
    if (!msg)
        return;
    struct rst_info *ri = ESP.getResetInfoPtr();
    gCrashInfo.magic = RTC_MAGIC;
    gCrashInfo.reason = ri ? ri->reason : 0;
    gCrashInfo.exccause = ri ? ri->exccause : 0;
    // No incrementamos resetCount aquí; lo hace el arranque si detecta crash.
    strncpy(gCrashInfo.lastMsg, msg, sizeof(gCrashInfo.lastMsg) - 1);
    gCrashInfo.lastMsg[sizeof(gCrashInfo.lastMsg) - 1] = '\0';
    gCrashInfo.crc = rtcCrc(gCrashInfo);
    ESP.rtcUserMemoryWrite(0, (uint32_t *)&gCrashInfo, sizeof(gCrashInfo));
}

//  Helper para obtener el timestamp según disponibilidad de hora
String getLogHeader()
{
    char header[32];
    if (timeOK && getLocalTime(&timeinfo))
    {
        strftime(header, sizeof(header), "%Y-%m-%d %H:%M:%S", &timeinfo);
    }
    else
    {
        snprintf(header, sizeof(header), "%8lums", millis());
    }
    return "[" + String(header) + "] ";
}

// Función centralizada de logging
void logLine(const char *msg)
{
    if (!msg || msg[0] == '\0')
        return;

    // 1. "Caja Negra" (RTC): Guardamos el mensaje actual
    // por si el sistema crashea en el siguiente paso.
    updateRTCLastMsg(msg);

    // 2. Formatear para salida visual
    String header = getLogHeader();
    String fullMsg = header + msg;

    // 3. Salidas volátiles (No escriben en disco)
    Serial.print(fullMsg);
    if (telnetReady)
        TelnetStream.print(fullMsg);

    // 4. Flash (COMENTADO para no degradar memoria)
    // logToFile(fullMsg.c_str());
}

#define LOG_PRINT(x) logLine(String(x).c_str())

#define LOG_PRINTLN(x)                \
    {                                 \
        String _s = String(x) + "\n"; \
        logLine(_s.c_str());          \
    }

#define LOG_PRINTLN0() logLine("\n")

#define LOG_PRINTF(fmt, ...)                              \
    {                                                     \
        char _buf[256];                                   \
        snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
        logLine(_buf);                                    \
    }

#define LOG_PRINTF_P(fmt, ...)                              \
    {                                                       \
        char _buf[256];                                     \
        snprintf_P(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
        logLine(_buf);                                      \
    }

/* --- FUNCIÓN LOGTOFILE MODIFICADA (COMENTADA) --- ESCRITURA EN FLASH DESACTIVADA PARA EVITAR DESGASTE --- */

/*void logToFile(const char *msg)
{

    static uint16_t writesThisBoot = 0;
    if (writesThisBoot > 500) return;

    File f = LittleFS.open("/log.txt", "a");
    if (!f) return;

    if (f.size() >= MAX_LOG_BYTES) {
        f.close();
        File src = LittleFS.open("/log.txt", "r");
        File dst = LittleFS.open("/log.tmp", "w");
        if (src && dst) {
            src.seek(MAX_LOG_BYTES / 2);
            uint8_t chunk[256];
            while (src.available()) {
                size_t n = src.read(chunk, sizeof(chunk));
                dst.write(chunk, n);
            }
        }
        src.close();
        dst.close();
        LittleFS.remove("/log.txt");
        LittleFS.rename("/log.tmp", "/log.txt");
        f = LittleFS.open("/log.txt", "a");
        if (!f) return;
    }

    f.print(msg);
    f.close();
    writesThisBoot++;

}*/
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
#define INTERVAL_SENSOR 10000UL        // leer sensores cada 10 s
#define INTERVAL_MQTT 60000UL          // publicar MQTT cada 60 s
#define INTERVAL_RECONNECT 5000UL      // reintentar conexión MQTT cada 5 s
#define INTERVAL_WIFI_WDT 180000UL     // entrar en modo AP si WiFi caído > 3 min
#define INTERVAL_AP_RECONNECT 180000UL // en modo AP, reintentar WiFi cada 3 min

/* ─── AP WiFi ────────────────────────────────────────────────*/
#define AP_SSID "ExtractorAP"

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

#define ENS160_I2C_ADDRESS 0x53

#define PIN_SOIL_VCC D6 // GPIO12 — alimentación del sensor

#define MY_NTP_SERVER "pool.ntp.org"
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03" // for Central Europe

/* ─── Objetos ─────────────────────────────────────────────────*/
ScioSense_ENS160 ens160(ENS160_I2C_ADDRESS); // 0x53..ENS160+AHT21
Adafruit_AHTX0 aht;
BearSSL::WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);
ESP8266WebServer webServer(80);
DNSServer dnsServer; // captive portal en modo AP

bool littleFSok = false;

/* ─── Estado WiFi/AP ──────────────────────────────────────────*/
bool gApMode = false;           // true = funcionando como AP (sin WiFi)
bool gPortalRequested = false;  // true = usuario pidió portal WiFiManager
unsigned long tApReconnect = 0; // temporizador de reintento en modo AP

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
   WiFi Test (/testwifi)
   Prueba 3 modos PHY × niveles de potencia
   State machine no bloqueante: avanza en loop().
   Los resultados se guardan en /wifitest.json (LittleFS).
   ══════════════════════════════════════════════════════════ */

static const WiFiPhyMode_t kWtModes[] = {WIFI_PHY_MODE_11B, WIFI_PHY_MODE_11G, WIFI_PHY_MODE_11N};
static const float kWtPowers[] = {5.0f, 10.0f, 15.0f, 17.5f, 18.5f, 19.5f, 20.5f};
#define WTEST_N_MODES 3
#define WTEST_N_POWERS 7
#define WTEST_TOTAL (WTEST_N_MODES * WTEST_N_POWERS) // 12

// Códigos de error por paso
#define WTEST_ERR_OK 0         // sin error
#define WTEST_ERR_TIMEOUT 1    // timeout de conexión WiFi
#define WTEST_ERR_SPEED 2      // test de velocidad fallido
#define WTEST_ERR_GLOBAL_TMO 3 // test abortado por timeout global
#define WTEST_ERR_ABORTED 4    // abortado (p.ej. reinicio inesperado)

// Timeout global: si el test no termina en este tiempo, se aborta y se guarda el estado
#define WTEST_GLOBAL_TIMEOUT_MS (12UL * 60UL * 1000UL) // 12 minutos

struct WifiTestEntry
{
    uint8_t phyMode;     // 1=11b 2=11g 3=11n
    float power;         // dBm
    bool connected;      // ¿logró conectar?
    int8_t rssi;         // dBm señal (0 si no conectó)
    uint16_t connectMs;  // ms hasta WL_CONNECTED
    int32_t downloadBps; // bytes/s promedio (-1 = sin dato)
    uint8_t errCode;     // WTEST_ERR_* — motivo de fallo (0=OK)
};

enum WtPhase
{
    WTP_IDLE,
    WTP_START_DELAY,
    WTP_CONNECT,
    WTP_WAITING,
    WTP_SPEED,
    WTP_DONE_STEP,
    WTP_RESTORE_WAIT
};

WifiTestEntry gWtResults[WTEST_TOTAL];
int gWtStep = 0;
bool gWtRunning = false;
bool gWtDone = false; // ¿hay resultados cargados/terminados?
String gWtTimestamp = "";
WtPhase gWtPhase = WTP_IDLE;
unsigned long gWtPhaseTimer = 0;

// Configuración original del WiFi para restaurar al terminar
uint8_t gWtOrigPhyMode = WIFI_PHY_MODE_11B;
float gWtOrigPower = 18.5f;

unsigned long gWtStartTime = 0; // para vigilar el timeout global del test

/* ══════════════════════════════════════════════════════════
   RTC crash tracking – sobrevive reinicios (no apagados)
   Usa los 256 bytes de user RTC memory del ESP8266.
   ══════════════════════════════════════════════════════════ */

void loadRTCCrash()
{
    RTCCrashInfo tmp;
    ESP.rtcUserMemoryRead(0, (uint32_t *)&tmp, sizeof(tmp));
    if (tmp.magic == RTC_MAGIC && tmp.crc == rtcCrc(tmp))
    {
        gCrashInfo = tmp;
        gCrashInfoValid = true;
    }
}

// Nombres legibles del rst_reason del SDK ESP8266
static const char *rstReasonStr(uint32_t r)
{
    switch (r)
    {
    case 0:
        return "power-on";
    case 1:
        return "hw-watchdog";
    case 2:
        return "exception";
    case 3:
        return "sw-watchdog";
    case 4:
        return "soft-restart";
    case 5:
        return "deep-sleep-wake";
    case 6:
        return "ext-reset";
    default:
        return "unknown";
    }
}

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

/* ══════════════════════════════════════════════════════════
   WiFi – helpers
   ══════════════════════════════════════════════════════════ */

/**
 * Activa el modo AP con SSID+contraseña y arranca el servidor DNS
 * para redirigir cualquier dominio al portal cautivo (192.168.4.1).
 */
void startApMode()
{
    LOG_PRINTLN(F("Iniciando modo AP..."));
    gApMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, SECRET_AP_PASS);
    delay(100); // esperar a que el AP esté listo
    dnsServer.start(53, "*", WiFi.softAPIP());
    tApReconnect = millis(); // primer intento de reconexión tras 60 s
    LOG_PRINTF_P(PSTR("AP activo: %s  IP: %s\n"),
                 AP_SSID, WiFi.softAPIP().toString().c_str());
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
        }
        else
        {
            LOG_PRINTLN(F("[AHT21] error de lectura."));
        }
    }

    /* 2. ENS160 */
    if (gENSok && ens160.available())
    {

        gENSValid = true;
        // Give values to Air Quality Sensor.

        ens160.set_envdata(gTemp, gHumidity);

        ens160.measure(true);
        ens160.measureRaw(true);

        gAQI = ens160.getAQI();
        gTVOC = ens160.getTVOC();
        gECO2 = ens160.geteCO2();
    }
    else
    {
        LOG_PRINTLN(F("[ENS160] error de lectura."));
        gENSValid = false;
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
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Extractor</title>
</head><body>
<h2>Extractor</h2>

<div id="ap-banner" style="display:none">
  ⚠️ Modo AP activo — sin WiFi. IP: <span id="vip">192.168.4.1</span> | Reintentar cada 3 min.
</div>

<h3>Calidad del aire</h3>
AQI: <span id="vaqi">–</span><br>
TVOC: <span id="vtvoc">–</span><br>
eCO2: <span id="veco2">–</span>

<h3>Ambiente</h3>
Humedad: <span id="vhum">–</span><br>
Temperatura: <span id="vtemp">–</span>

<h3>Humedad suelo</h3>
<span id="vsoil">–</span>
<div id="soil-bar" style="display:inline-block;height:8px;vertical-align:middle"></div>

<h3>Extractor — <span id="mode-b">–</span></h3>
Estado: <span id="ext-s">–</span> <span id="timer-info"></span><br>
<button onclick="cmd('/set?relay=ON')">Encender</button>
<button onclick="cmd('/set?relay=OFF')">Apagar</button>
<button onclick="cmd('/set?mode=AUTO')">Auto</button>

<h3>Umbrales (±5% histéresis)</h3>
AQI (1–5): <input type="number" id="t1" min="1" max="5" step="1"> <button onclick="setV('t1')">✓</button><br>
Humedad aire: <input type="number" id="t2" min="0" max="100">% <button onclick="setV('t2')">✓</button><br>
Humedad suelo: <input type="number" id="t3" min="0" max="100">% <button onclick="setV('t3')">✓</button>

<h3>Temporizador (0 = desactivado)</h3>
Máx. encendido: <input type="number" id="ton" min="0" max="1440"> min <button onclick="setV('ton')">✓</button><br>
Cooldown: <input type="number" id="tof" min="0" max="1440"> min <button onclick="setV('tof')">✓</button>

<h3>Logs &amp; Sistema</h3>
<div id="crash-info" style="display:none"></div>
Heap: <span id="vheap">–</span><br>
<a href="/log" target="_blank">Ver log</a> |
<button onclick="clearLog()">Borrar log</button>

<h3>Red WiFi</h3>
IP: <span id="vip-sta">–</span><br>
<button onclick="toggleWifiForm()">Cambiar WiFi</button>
<button onclick="wifiPortal()">Desconectar + AP</button>
<div id="wifi-form" style="display:none">
  SSID: <input type="text" id="wssid" placeholder="Nombre red"><br>
  Pass: <input type="password" id="wpass" placeholder="Contraseña"><br>
  <button onclick="submitWifiSta()">Aplicar y reiniciar</button>
  <button onclick="toggleWifiForm()">Cancelar</button>
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
    // Red
    if(d.ip) document.getElementById('vip-sta').textContent=d.ip;
    var banner=document.getElementById('ap-banner');
    var vip=document.getElementById('vip');
    if(d.ap){
      banner.style.display='block';
      if(d.ip) vip.textContent=d.ip;
    } else {
      banner.style.display='none';
    }
  }).catch(()=>{});
}
// Cargar info de crash y heap al inicio (solo una vez)
function loadCrashInfo(){
  fetch('/api/crash').then(r=>r.json()).then(d=>{
    if(d.free_heap) document.getElementById('vheap').textContent=Math.round(d.free_heap/1024)+'KB';
    var box=document.getElementById('crash-info');
    if(d.prev_valid && d.prev_reason_n!==0 && d.prev_reason_n!==4 && d.prev_reason_n!==5){
      box.style.display='block';
      var isCrash=(d.prev_reason_n===1||d.prev_reason_n===2||d.prev_reason_n===3);
      box.innerHTML=(isCrash?'⚠️ <strong>Crash detectado</strong>':'ℹ️ Reset previo')+
        ' — motivo: <strong>'+d.prev_reason+'</strong>'+
        (d.reset_count>0?' (×'+d.reset_count+')'  :'')+
        (d.prev_last_msg?'<br>Último log: <em>'+d.prev_last_msg+'</em>':'');
    }
  }).catch(()=>{});
}
function cmd(u){fetch(u).then(refresh);}
function setV(id){cmd('/set?'+id+'='+document.getElementById(id).value);}
function clearLog(){if(confirm('¿Borrar el log de sistema?'))fetch('/log/clear');}
function toggleWifiForm(){
  var f=document.getElementById('wifi-form');
  f.style.display=f.style.display==='none'?'block':'none';
}
function submitWifiSta(){
  var s=document.getElementById('wssid').value.trim();
  var p=document.getElementById('wpass').value;
  if(!s){alert('Introduce el SSID.');return;}
  if(confirm('¿Conectar a "'+s+'" y reiniciar el dispositivo?')){
    fetch('/wifi/sta?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p))
      .then(()=>alert('Reiniciando… Reconéctate a la misma red en unos segundos.'))
      .catch(()=>alert('Reiniciando…'));
  }
}
function wifiPortal(){
  if(confirm('¿Desconectar del WiFi y abrir portal de configuración?\n\nSe creará el AP "ExtractorAP". El dispositivo se reiniciará al terminar (o en 3 min).')){
    fetch('/wifi').then(()=>{
      alert('Portal iniciado.\nConéctate al AP "ExtractorAP" y abre 192.168.4.1');
    });
  }
}
refresh(); setInterval(refresh,8000);
loadCrashInfo();
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
    // Info de red
    doc["ap"] = gApMode;
    doc["ip"] = gApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
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
   Web – configurar WiFi STA desde la conexión actual (/wifi/sta)
   No lanza AP: guarda las credenciales y reinicia.
   ══════════════════════════════════════════════════════════ */
void handleWifiSta()
{
    if (!webServer.hasArg("ssid") || webServer.arg("ssid").length() == 0)
    {
        webServer.send(400, "text/plain", "ssid requerido");
        return;
    }
    String ssid = webServer.arg("ssid");
    String pass = webServer.arg("pass");

    webServer.send(200, "text/html",
                   F("<!DOCTYPE html><html><head>"
                     "<meta charset=UTF-8>"
                     "<meta name=viewport content='width=device-width,initial-scale=1'>"
                     "<title>WiFi – Extractor</title>"
                     "<style>body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:16px}"
                     "h2{color:#1565c0}</style></head><body>"
                     "<h2>🔄 Aplicando nueva red WiFi...</h2>"
                     "<p>El dispositivo se reiniciará en 2 segundos e intentará conectarse a la red indicada.</p>"
                     "<p>Si la conexión falla, volverá a modo AP automáticamente.</p>"
                     "</body></html>"));

    delay(500);
    // Guardar credenciales en flash de forma persistente (una sola vez)
    WiFi.persistent(true);
    WiFi.begin(ssid.c_str(), pass.c_str());
    WiFi.persistent(false);
    delay(300);
    ESP.restart();
}

/* ══════════════════════════════════════════════════════════
   Web – info de crash (/api/crash)
   ══════════════════════════════════════════════════════════ */
void handleCrashApi()
{
    JsonDocument doc;
    struct rst_info *ri = ESP.getResetInfoPtr();
    doc["current_reason"] = ri ? rstReasonStr(ri->reason) : "?";
    doc["current_reason_n"] = ri ? (int)ri->reason : -1;
    doc["free_heap"] = (int)ESP.getFreeHeap();
    doc["chip_id"] = String(ESP.getChipId(), HEX);
    if (gCrashInfoValid)
    {
        doc["prev_valid"] = true;
        doc["prev_reason"] = rstReasonStr(gCrashInfo.reason);
        doc["prev_reason_n"] = (int)gCrashInfo.reason;
        doc["prev_exccause"] = (int)gCrashInfo.exccause;
        doc["prev_last_msg"] = gCrashInfo.lastMsg;
        doc["reset_count"] = (int)gCrashInfo.resetCount;
    }
    else
    {
        doc["prev_valid"] = false;
    }
    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

/* ══════════════════════════════════════════════════════════
   Web – portal WiFiManager (endpoint /wifi)
   ══════════════════════════════════════════════════════════ */
void handleWifiPortal()
{
    // Enviar respuesta antes de activar el flag
    webServer.send(200, "text/html",
                   F("<!DOCTYPE html><html><head>"
                     "<meta charset=UTF-8>"
                     "<meta name=viewport content='width=device-width,initial-scale=1'>"
                     "<title>WiFi – Extractor</title>"
                     "<style>body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:16px}"
                     "h2{color:#6a1b9a}p{line-height:1.6}strong{color:#333}</style>"
                     "</head><body>"
                     "<h2>⚙️ Portal WiFi iniciando...</h2>"
                     "<p>En unos segundos se activará el AP <strong>ExtractorAP</strong>.</p>"
                     "<p>Conéctate a ese AP (contraseña: la configurada) y navega a "
                     "<strong>192.168.4.1</strong> para configurar el WiFi.</p>"
                     "<p>El portal expira en <strong>3 minutos</strong>; "
                     "el dispositivo se reiniciará automáticamente.</p>"
                     "<p><small>Si no aparece el portal, navega manualmente a 192.168.4.1</small></p>"
                     "</body></html>"));
    gPortalRequested = true;
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
                       { LOG_PRINTLN(F("OTA: inicio")); });
    ArduinoOTA.onEnd([]()
                     { LOG_PRINTLN(F("\nOTA: fin")); });
    ArduinoOTA.onError([](ota_error_t e)
                       { LOG_PRINTF_P(PSTR("OTA error [%u]\n"), e); });
    ArduinoOTA.begin();
    LOG_PRINTLN(F("OTA listo → 'extractor-esp'."));
}

void handleLog()
{
    // Si quieres ver algo en /log aunque la flash esté apagada:
    String s = "<!DOCTYPE html><html><head><meta charset=UTF-8><title>Log</title>";
    s += "<style>body{background:#111;color:#cfc;font-family:monospace;padding:20px}</style></head><body>";
    s += "<h3>Estado de Memoria Flash: <span style='color:orange'>DESACTIVADA</span></h3>";
    s += "<p>Los logs en tiempo real se ven por Serial o Telnet.</p>";

    if (gCrashInfoValid)
    {
        s += "<hr><h4>ÚLTIMO RASTRO EN RTC (Caja Negra):</h4>";
        s += "<b>Motivo:</b> " + String(rstReasonStr(gCrashInfo.reason)) + "<br>";
        s += "<b>Contador de Resets:</b> " + String(gCrashInfo.resetCount) + "<br>";
        s += "<b>Último mensaje antes de reset:</b> <i style='color:#7bf'>" + String(gCrashInfo.lastMsg) + "</i>";
    }

    s += "<br><br><a href='/' style='color:#fff'>[ Volver al Inicio ]</a></body></html>";
    webServer.send(200, "text/html", s);
}

void handleLogClear()
{
    LittleFS.remove("/log.txt");
    webServer.sendHeader("Location", "/log");
    webServer.send(303);
}

/* ══════════════════════════════════════════════════════════
   WiFi Test – persistencia en flash
   ══════════════════════════════════════════════════════════ */
/* ══════════════════════════════════════════════════════════
   WiFi Test – persistencia en flash
   saveWifiTestCheckpoint() → guarda progreso parcial tras cada paso.
   saveWifiTestResults()    → alias final (marca done=true).
   ══════════════════════════════════════════════════════════ */

/**
 * Guarda en /wifitest.json el estado actual del test:
 * - Si el test está en curso  → running=true, los pasos ya completos y errores.
 * - Si el test ha terminado   → running=false/done=true, todos los resultados.
 * Se llama después de cada paso y al finalizar, para no perder nada ante un reinicio.
 */
void saveWifiTestCheckpoint()
{
    if (!littleFSok)
        return;
    File f = LittleFS.open("/wifitest.json", "w");
    if (!f)
    {
        LOG_PRINTLN(F("[WTest] ERROR: no se pudo abrir /wifitest.json para escritura."));
        return;
    }
    JsonDocument doc;
    doc["ts"] = gWtTimestamp;
    doc["running"] = gWtRunning; // true si el test aún no ha terminado
    doc["done"] = gWtDone;
    doc["step"] = gWtStep; // último paso completado

    JsonArray arr = doc["results"].to<JsonArray>();
    // Guardar solo los pasos ya terminados (índices 0..gWtStep-1 si en curso, todos si done)
    int limit = gWtDone ? WTEST_TOTAL : gWtStep;
    for (int i = 0; i < limit; i++)
    {
        JsonObject o = arr.add<JsonObject>();
        o["mode"] = gWtResults[i].phyMode;
        o["power"] = gWtResults[i].power;
        o["ok"] = gWtResults[i].connected;
        o["rssi"] = gWtResults[i].rssi;
        o["cMs"] = gWtResults[i].connectMs;
        o["bps"] = gWtResults[i].downloadBps;
        o["err"] = gWtResults[i].errCode;
    }
    serializeJson(doc, f);
    f.close();
    LOG_PRINTF_P(PSTR("[WTest] Checkpoint guardado: paso %d/%d, done=%d\n"),
                 gWtStep, WTEST_TOTAL, (int)gWtDone);
}

void saveWifiTestResults()
{
    // Alias semántico para el guardado final: asegura que done=true está reflejado.
    saveWifiTestCheckpoint();
}

bool loadWifiTestResults()
{
    if (!littleFSok)
        return false;
    File f = LittleFS.open("/wifitest.json", "r");
    if (!f)
        return false;
    JsonDocument doc;
    if (deserializeJson(doc, f) != DeserializationError::Ok)
    {
        f.close();
        return false;
    }
    f.close();
    gWtTimestamp = doc["ts"] | "";

    // Si el JSON dice que el test estaba en curso → fue abortado por reinicio/crash.
    // Marcamos los pasos no completados con WTEST_ERR_ABORTED.
    bool wasRunning = doc["running"] | false;
    int savedStep = doc["step"] | 0;

    JsonArray arr = doc["results"].as<JsonArray>();
    int i = 0;
    for (JsonObject o : arr)
    {
        if (i >= WTEST_TOTAL)
            break;
        gWtResults[i].phyMode = o["mode"] | 0;
        gWtResults[i].power = o["power"] | 0.0f;
        gWtResults[i].connected = o["ok"] | false;
        gWtResults[i].rssi = o["rssi"] | 0;
        gWtResults[i].connectMs = o["cMs"] | 0;
        gWtResults[i].downloadBps = o["bps"] | (int)-1;
        gWtResults[i].errCode = o["err"] | (int)WTEST_ERR_OK;
        i++;
    }

    if (wasRunning)
    {
        // El test se interrumpió: rellenar los pasos pendientes con ABORTED
        for (int j = i; j < WTEST_TOTAL; j++)
        {
            gWtResults[j].phyMode = kWtModes[j / WTEST_N_POWERS];
            gWtResults[j].power = kWtPowers[j % WTEST_N_POWERS];
            gWtResults[j].connected = false;
            gWtResults[j].rssi = 0;
            gWtResults[j].connectMs = 0;
            gWtResults[j].downloadBps = -1;
            gWtResults[j].errCode = WTEST_ERR_ABORTED;
        }
        gWtDone = true; // mostrar como finalizado (con errores de aborto)
        LOG_PRINTF_P(PSTR("[WTest] Test anterior abortado en paso %d — pasos pendientes marcados.\n"),
                     savedStep);
        // Persistir el estado corregido para que la web lo muestre correctamente
        gWtTimestamp += " [abortado]";
        saveWifiTestCheckpoint();
        return true;
    }

    gWtDone = (i == WTEST_TOTAL) && (doc["done"] | false);
    return gWtDone;
}

/* ══════════════════════════════════════════════════════════
   WiFi Test – test de velocidad (bloqueante ~5 s, usa yield)
   Descarga de un servidor HTTP conocido y mide throughput.
   ══════════════════════════════════════════════════════════ */
int32_t doWifiSpeedTest()
{
    // Servidor HTTP plano (sin TLS) con ficheros de tamaño fijo para tests
    const char *host = "speedtest.tele2.net";
    const uint16_t port = 80;
    const char *path = "/100KB.zip"; // 102400 bytes controlados
    // const char* path = "sha1sum.txt";
    WiFiClient client;
    client.setTimeout(6000);

    unsigned long tConn = millis();
    if (!client.connect(host, port))
    {
        LOG_PRINTLN(F("[WTest] Speed: no pudo conectar al servidor."));
        return -1;
    }
    LOG_PRINTF_P(PSTR("[WTest] Speed: conectado en %lu ms\n"), millis() - tConn);

    // Petición HTTP/1.0 (sin chunked transfer)
    client.print(F("GET "));
    client.print(path);
    client.print(F(" HTTP/1.0\r\nHost: "));
    client.print(host);
    client.print(F("\r\nConnection: close\r\n\r\n"));

    // Saltar cabeceras HTTP
    unsigned long tHdr = millis();
    bool headersDone = false;
    while (client.connected() && millis() - tHdr < 4000UL)
    {
        String line = client.readStringUntil('\n');
        yield();
        if (line == "\r" || line == "")
        {
            headersDone = true;
            break;
        }
    }
    if (!headersDone)
    {
        client.stop();
        return -1;
    }

    // Contar bytes descargados durante hasta 5 segundos
    uint32_t totalBytes = 0;
    uint8_t buf[256];
    unsigned long tStart = millis();
    while (client.connected() && millis() - tStart < 5000UL)
    {
        int avail = client.available();
        if (avail > 0)
        {
            int n = client.read(buf, min((int)sizeof(buf), avail));
            if (n > 0)
                totalBytes += (uint32_t)n;
        }
        else
        {
            delay(2);
        }
        yield();
    }
    client.stop();

    uint32_t elapsed = millis() - tStart;
    if (elapsed == 0 || totalBytes == 0)
        return -1;

    // bytes/s
    int32_t bps = (int32_t)((uint64_t)totalBytes * 1000UL / elapsed);
    LOG_PRINTF_P(PSTR("[WTest] Speed: %lu bytes en %lu ms → %ld B/s\n"),
                 totalBytes, elapsed, bps);
    return bps;
}

/* ══════════════════════════════════════════════════════════
   WiFi Test – state machine, llamar desde loop()
   ══════════════════════════════════════════════════════════ */

// Helper interno: finalizar el test de forma ordenada (éxito o forzado).
// Restaura WiFi, guarda resultados y limpia el estado.
static void wtFinalize(bool abortedByTimeout)
{
    LOG_PRINTF_P(PSTR("[WTest] Finalizando test (abortado=%d). Restaurando WiFi...\n"),
                 (int)abortedByTimeout);

    // Si se abortó por timeout global, marcar los pasos no completados
    if (abortedByTimeout)
    {
        for (int j = gWtStep; j < WTEST_TOTAL; j++)
        {
            gWtResults[j].phyMode = kWtModes[j / WTEST_N_POWERS];
            gWtResults[j].power = kWtPowers[j % WTEST_N_POWERS];
            gWtResults[j].connected = false;
            gWtResults[j].rssi = 0;
            gWtResults[j].connectMs = 0;
            gWtResults[j].downloadBps = -1;
            gWtResults[j].errCode = WTEST_ERR_GLOBAL_TMO;
        }
    }

    // Timestamp del fin
    if (timeOK && getLocalTime(&timeinfo))
    {
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        gWtTimestamp = String(buf);
        if (abortedByTimeout)
            gWtTimestamp += " [timeout]";
    }
    else
    {
        gWtTimestamp = "t+" + String(millis() / 1000) + "s";
        if (abortedByTimeout)
            gWtTimestamp += " [timeout]";
    }

    gWtRunning = false;
    gWtDone = true;
    gWtPhase = WTP_IDLE;

    saveWifiTestResults(); // guardado final
    LOG_PRINTLN(F("[WTest] Test WiFi finalizado y guardado."));
}

void runWifiTestStep()
{
    if (!gWtRunning)
        return;

    // ── Timeout global: si el test lleva demasiado tiempo, abortarlo ──
    if (millis() - gWtStartTime > WTEST_GLOBAL_TIMEOUT_MS)
    {
        LOG_PRINTF_P(PSTR("[WTest] TIMEOUT GLOBAL (%lu ms) — abortando test.\n"),
                     millis() - gWtStartTime);
        // Restaurar WiFi antes de finalizar
        WiFi.disconnect(false);
        delay(200);
        yield();
        WiFi.setOutputPower(gWtOrigPower);
        WiFi.setPhyMode((WiFiPhyMode_t)gWtOrigPhyMode);
        WiFi.begin();
        wtFinalize(true);
        return;
    }

    int modeIdx = gWtStep / WTEST_N_POWERS;
    int powerIdx = gWtStep % WTEST_N_POWERS;

    switch (gWtPhase)
    {
    case WTP_START_DELAY:
        // Dar ~2 s para que la respuesta HTTP de /testwifi/run llegue al navegador
        // antes de cortar el WiFi con disconnect(). Sin esto el fetch() falla.
        if (millis() - gWtPhaseTimer >= 2000UL)
        {
            LOG_PRINTLN(F("[WTest] Iniciando primera combinación..."));
            gWtPhase = WTP_CONNECT;
        }
        break;

    case WTP_CONNECT:
    {
        WiFiPhyMode_t mode = kWtModes[modeIdx];
        float pwr = kWtPowers[powerIdx];

        static const char *modeNames[] = {"11b", "11g", "11n"};
        LOG_PRINTF_P(PSTR("[WTest] Paso %d/%d: modo=%s pow=%.1f dBm\n"),
                     gWtStep + 1, WTEST_TOTAL, modeNames[modeIdx], pwr);

        gWtResults[gWtStep].phyMode = (uint8_t)mode;
        gWtResults[gWtStep].power = pwr;
        gWtResults[gWtStep].errCode = WTEST_ERR_OK; // se sobreescribirá si hay error

        WiFi.setOutputPower(pwr);
        WiFi.setPhyMode(mode);
        WiFi.disconnect(false);
        delay(300);
        yield();
        WiFi.begin(); // credenciales guardadas por WiFiManager

        gWtPhaseTimer = millis();
        gWtPhase = WTP_WAITING;
        break;
    }

    case WTP_WAITING:
        if (WiFi.status() == WL_CONNECTED)
        {
            uint16_t elapsed = (uint16_t)min((unsigned long)65535UL, millis() - gWtPhaseTimer);
            gWtResults[gWtStep].connected = true;
            gWtResults[gWtStep].connectMs = elapsed;
            gWtResults[gWtStep].rssi = (int8_t)WiFi.RSSI();
            gWtResults[gWtStep].errCode = WTEST_ERR_OK;
            LOG_PRINTF_P(PSTR("[WTest] Conectado en %u ms, RSSI=%d dBm\n"),
                         elapsed, WiFi.RSSI());
            gWtPhase = WTP_SPEED;
        }
        else if (millis() - gWtPhaseTimer > 15000UL)
        {
            LOG_PRINTF_P(PSTR("[WTest] Timeout en paso %d → sin conexión.\n"), gWtStep + 1);
            gWtResults[gWtStep].connected = false;
            gWtResults[gWtStep].rssi = 0;
            gWtResults[gWtStep].connectMs = 0;
            gWtResults[gWtStep].downloadBps = -1;
            gWtResults[gWtStep].errCode = WTEST_ERR_TIMEOUT;
            gWtPhase = WTP_DONE_STEP;
        }
        break;

    case WTP_SPEED:
    {
        int32_t bps = doWifiSpeedTest();
        gWtResults[gWtStep].downloadBps = bps;
        if (bps < 0)
        {
            LOG_PRINTF_P(PSTR("[WTest] Speed test fallido en paso %d.\n"), gWtStep + 1);
            gWtResults[gWtStep].errCode = WTEST_ERR_SPEED;
        }
        gWtPhase = WTP_DONE_STEP;
        break;
    }

    case WTP_DONE_STEP:
        // ── Guardar en flash el resultado de este paso ─────────────
        gWtStep++;
        saveWifiTestCheckpoint(); // persiste progreso ANTES de pasar al siguiente

        if (gWtStep >= WTEST_TOTAL)
        {
            /* ── Test completado: iniciar restauración no-bloqueante ── */
            LOG_PRINTLN(F("[WTest] Todas las combinaciones probadas. Restaurando WiFi..."));
            WiFi.disconnect(false);
            delay(300);
            yield();
            WiFi.setOutputPower(gWtOrigPower);
            WiFi.setPhyMode((WiFiPhyMode_t)gWtOrigPhyMode);
            WiFi.begin();

            gWtPhaseTimer = millis();
            gWtPhase = WTP_RESTORE_WAIT;
        }
        else
        {
            gWtPhase = WTP_CONNECT;
            delay(800); // pausa breve entre combinaciones
            yield();
        }
        break;

    case WTP_RESTORE_WAIT:
    {
        // Esperar reconexión final de forma NO bloqueante (max 20 s)
        bool connected = (WiFi.status() == WL_CONNECTED);
        bool timedOut = (millis() - gWtPhaseTimer > 20000UL);

        if (connected)
        {
            LOG_PRINTF_P(PSTR("[WTest] WiFi restaurado: %s\n"),
                         WiFi.localIP().toString().c_str());
            wtFinalize(false);
        }
        else if (timedOut)
        {
            LOG_PRINTLN(F("[WTest] Aviso: WiFi no reconectó tras restaurar (timeout 20 s)."));
            wtFinalize(false); // el test en sí terminó OK; solo la reconexión tardó
        }
        // Si ninguno de los dos: esperar al siguiente tick del loop
        break;
    }

    default:
        break;
    }
}

/* ══════════════════════════════════════════════════════════
   Web – handlers /testwifi
   ══════════════════════════════════════════════════════════ */

// Página HTML principal del test WiFi
const char HTML_TESTWIFI[] PROGMEM = R"html(
<!DOCTYPE html><html lang="es"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Test WiFi – Extractor</title>
<style>
body{font-family:sans-serif;max-width:720px;margin:24px auto;padding:16px;background:#fafafa;color:#222}
h2{color:#1565c0;margin-bottom:4px}
.sub{color:#555;font-size:13px;margin-bottom:16px}
.btn{display:inline-block;padding:10px 22px;margin:6px 4px;border:none;border-radius:6px;
     cursor:pointer;font-size:14px;text-decoration:none}
.bp{background:#1565c0;color:#fff}.bs{background:#546e7a;color:#fff}
.btn:disabled,.btn[disabled]{opacity:.45;cursor:not-allowed}
#progress{display:none;margin:16px 0;padding:14px 16px;background:#e3f2fd;
          border-radius:8px;border-left:4px solid #1565c0}
.bar-bg{background:#bbdefb;border-radius:4px;height:10px;margin:8px 0}
.bar-fg{background:#1565c0;height:10px;border-radius:4px;transition:width .6s}
#prog-label{font-weight:bold;font-size:14px}
#status-msg{color:#555;font-size:12px;margin-top:4px}
#results{margin-top:20px}
table{border-collapse:collapse;width:100%;font-size:13px}
th,td{border:1px solid #ccc;padding:7px 10px;text-align:center}
th{background:#1565c0;color:#fff;font-size:12px}
tr:nth-child(even){background:#f5f5f5}
.ok{color:#2e7d32;font-weight:bold}.fail{color:#c62828;font-weight:bold}
.best{background:#e8f5e9 !important}
.warn{color:#e65100}
</style></head><body>
<h2>🔬 Test WiFi por Modo &amp; Potencia</h2>
<p class="sub">Prueba la conexión con los datos WiFi almacenados por WifiManager<br>
en <b>3 modos</b> (802.11b / g / n) × <b>diferentes potencias</b>.<br>
⚠️ El test interrumpe WiFi/MQTT durante ~3 minutos. No tocar el dispositivo mientras corre.</p>

<button class="btn bp" id="btn-run" onclick="startTest()">▶ Realizar Test</button>
<button class="btn bs" id="btn-view" onclick="loadResults()">📊 Ver Últimos Resultados</button>
<a href="/" class="btn bs">← Inicio</a>

<div id="progress">
  <div id="prog-label">Preparando...</div>
  <div class="bar-bg"><div class="bar-fg" id="prog-bar" style="width:0%"></div></div>
  <div id="status-msg"></div>
</div>
<div id="results"></div>

<script>
var polling=null;
var modeNames={1:'802.11b',2:'802.11g',3:'802.11n'};
var powers=[5.0, 10.0, 15.0, 17.5, 18.5, 19.5, 20.5];
var errLabels={0:'',1:'timeout',2:'speed fail',3:'global timeout',4:'abortado'};

function startTest(){
  document.getElementById('btn-run').disabled=true;
  document.getElementById('btn-view').disabled=true;
  document.getElementById('progress').style.display='block';
  document.getElementById('prog-label').textContent='Iniciando...';
  document.getElementById('results').innerHTML='';
  fetch('/testwifi/run').then(function(){
    polling=setInterval(pollStatus,2500);
    pollStatus();
  }).catch(function(e){
    document.getElementById('prog-label').textContent='Error al iniciar: '+e;
    document.getElementById('btn-run').disabled=false;
    document.getElementById('btn-view').disabled=false;
  });
}

function loadResults(){
  fetch('/testwifi/status').then(function(r){return r.json();}).then(function(d){
    if(!d.results||d.results.length===0){
      document.getElementById('results').innerHTML='<p>Sin resultados guardados aún.</p>';
    }else{
      showResults(d);
    }
  }).catch(function(){
    document.getElementById('results').innerHTML='<p>Error al obtener resultados.</p>';
  });
}

function pollStatus(){
  fetch('/testwifi/status').then(function(r){return r.json();}).then(function(d){
    var pct=d.total>0?Math.round(100*d.step/d.total):0;
    document.getElementById('prog-bar').style.width=pct+'%';
    if(d.running){
      var mi=Math.floor(d.step/7); var pi=d.step%7;
      document.getElementById('prog-label').textContent=
        'Probando '+( d.step+1)+'/'+d.total+': '+
        (mi<3?modeNames[mi+1]:'?')+' @ '+(pi<7?powers[pi]:'?')+' dBm…';
      document.getElementById('status-msg').textContent='Puede tardar ~6 min en total. No recargar.';
    }else{
      clearInterval(polling); polling=null;
      document.getElementById('prog-bar').style.width='100%';
      document.getElementById('prog-label').textContent='✅ Test completado';
      document.getElementById('status-msg').textContent='';
      document.getElementById('btn-run').disabled=false;
      document.getElementById('btn-view').disabled=false;
      showResults(d);
    }
  }).catch(function(){});
}

function fmtSpeed(bps){
  if(bps<0) return '—';
  if(bps>=1048576) return (bps/1048576).toFixed(2)+' MB/s';
  if(bps>=1024)    return (bps/1024).toFixed(1)+' KB/s';
  return bps+' B/s';
}

function showResults(d){
  if(!d.results||d.results.length===0){
    document.getElementById('results').innerHTML='<p>Sin resultados.</p>';
    return;
  }
  // Encontrar la mejor combinación (mayor bps entre las conectadas)
  var bestBps=-1,bestIdx=-1;
  for(var i=0;i<d.results.length;i++){
    var r=d.results[i];
    if(r.ok&&r.bps>bestBps){bestBps=r.bps;bestIdx=i;}
  }
  var html='<h3>Resultados'+(d.ts?' &mdash; <small style="font-weight:normal">'+d.ts+'</small>':'')+' </h3>';
  html+='<table><tr><th>Modo</th><th>Potencia</th><th>Estado</th>';
  html+='<th>T. conexión</th><th>RSSI</th><th>Velocidad</th><th>Error</th></tr>';
  for(var i=0;i<d.results.length;i++){
    var r=d.results[i];
    var cls=i===bestIdx?' class="best"':'';
    var okS=r.ok?'<span class="ok">✓ OK</span>':'<span class="fail">✗ Fallo</span>';
    var ct=r.cMs?r.cMs+' ms':'—';
    var rssi=r.rssi?r.rssi+' dBm':'—';
    var spd=fmtSpeed(r.bps)+(i===bestIdx?' 🏆':'');
    var errS=(r.err&&r.err>0)?'<span class="warn">'+( errLabels[r.err]||'err'+r.err)+'</span>':'';
    html+='<tr'+cls+'><td>'+(modeNames[r.mode]||r.mode)+'</td><td>'+r.power.toFixed(1)+' dBm</td>';
    html+='<td>'+okS+'</td><td>'+ct+'</td><td>'+rssi+'</td><td>'+spd+'</td><td>'+errS+'</td></tr>';
  }
  html+='</table>';
  if(bestIdx>=0){
    var b=d.results[bestIdx];
    html+='<p>🏆 <b>Mejor combinación:</b> '+(modeNames[b.mode]||b.mode)+' @ '+
          b.power.toFixed(1)+' dBm → '+fmtSpeed(b.bps)+'</p>';
  }
  document.getElementById('results').innerHTML=html;
}
</script></body></html>
)html";

void handleTestWifi()
{
    webServer.send_P(200, "text/html", HTML_TESTWIFI);
}

void handleTestWifiRun()
{
    if (gApMode)
    {
        webServer.send(400, "text/plain", "Test no disponible en modo AP (sin WiFi).");
        return;
    }
    if (gWtRunning)
    {
        webServer.send(200, "application/json", "{\"started\":false,\"reason\":\"ya en curso\"}");
        return;
    }

    // Reiniciar estado
    gWtStep = 0;
    gWtRunning = true;
    gWtDone = false;
    gWtPhase = WTP_START_DELAY; // esperar a que la respuesta HTTP llegue al navegador
    gWtPhaseTimer = millis();
    gWtStartTime = millis(); // arrancar el cronómetro del timeout global
    gWtTimestamp = "";
    memset(gWtResults, 0, sizeof(gWtResults));
    for (int i = 0; i < WTEST_TOTAL; i++)
        gWtResults[i].downloadBps = -1;

    LOG_PRINTLN(F("[WTest] Test WiFi iniciado desde web."));
    webServer.send(200, "application/json", "{\"started\":true}");
}

void handleTestWifiStatus()
{
    JsonDocument doc;
    doc["running"] = gWtRunning;
    doc["done"] = gWtDone;
    doc["step"] = gWtStep;
    doc["total"] = WTEST_TOTAL;
    doc["ts"] = gWtTimestamp;

    JsonArray arr = doc["results"].to<JsonArray>();
    // Devolver solo los pasos ya completados (o todos si done)
    int limit = gWtDone ? WTEST_TOTAL : gWtStep;
    for (int i = 0; i < limit; i++)
    {
        JsonObject o = arr.add<JsonObject>();
        o["mode"] = gWtResults[i].phyMode;
        o["power"] = gWtResults[i].power;
        o["ok"] = gWtResults[i].connected;
        o["rssi"] = gWtResults[i].rssi;
        o["cMs"] = gWtResults[i].connectMs;
        o["bps"] = gWtResults[i].downloadBps;
        o["err"] = gWtResults[i].errCode;
    }

    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

/* ══════════════════════════════════════════════════════════
   Setup
   ══════════════════════════════════════════════════════════ */

void setup()
{
    Serial.begin(115200);

    /* ── Leer RTC crash info PRIMERO: antes de cualquier logLine()     ──
     * Si lo hacemos después, updateRTCLastMsg() ya habrá sobreescrito
     * el último mensaje del crash anterior y lo perderíamos para siempre. */
    loadRTCCrash();

    // Nota: LOG_PRINTLN ya incluirá el timestamp automáticamente
    LOG_PRINTLN(F("\n=== Extractor ESP8266 ==="));

    if (!LittleFS.begin())
    {
        Serial.println(F("ERROR: LittleFS no montado"));
        littleFSok = false;
    }
    else
    {
        Serial.println(F("LittleFS OK."));
        littleFSok = true;
    }

    /* ── RTC crash info ya cargado; procesar estado del reset actual: ── */
    {
        struct rst_info *ri = ESP.getResetInfoPtr();
        uint32_t reason = ri ? ri->reason : 99;

        // Formatear info de arranque para RTC
        char bootHdr[100];
        snprintf(bootHdr, sizeof(bootHdr), "BOOT: %s(%u)", rstReasonStr(reason), reason);

        // ¿Fue un crash inesperado?
        bool isCrash = (reason == 1 || reason == 2 || reason == 3);
        if (isCrash)
        {
            gCrashInfo.magic = RTC_MAGIC;
            gCrashInfo.reason = reason;
            gCrashInfo.exccause = ri ? ri->exccause : 0;
            gCrashInfo.resetCount = gCrashInfoValid ? gCrashInfo.resetCount + 1 : 1;
            gCrashInfo.crc = rtcCrc(gCrashInfo);
            ESP.rtcUserMemoryWrite(0, (uint32_t *)&gCrashInfo, sizeof(gCrashInfo));
            gCrashInfoValid = true;

            LOG_PRINTF_P(PSTR("*** CRASH #%u detectado! Motivo: %s\n"),
                         gCrashInfo.resetCount, rstReasonStr(gCrashInfo.reason));
        }
        else
        {
            gCrashInfo.magic = RTC_MAGIC;
            gCrashInfo.reason = reason;
            gCrashInfo.exccause = 0;
            gCrashInfo.resetCount = 0;
            // No borramos lastMsg aquí para poder verlo en la web tras un reinicio limpio
            gCrashInfo.crc = rtcCrc(gCrashInfo);
            ESP.rtcUserMemoryWrite(0, (uint32_t *)&gCrashInfo, sizeof(gCrashInfo));
        }
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
    Wire.begin(D2, D1);    // SDA=D2(GPIO4), SCL=D1(GPIO5)
    Wire.setClock(100000); // bajar a 100 kHz — el ENS160 es sensible a velocidades altas

    loadConfig();
    loadWifiTestResults(); // cargar resultados previos del test WiFi (si existen)

    /* ── WiFi ──────────────────────── */
    WiFi.persistent(false); // no desgastar flash en cada reconexión
    WiFi.hostname("extractor-esp");
    WiFi.setSleepMode(WIFI_NONE_SLEEP); // radio siempre activa — sin latencia
    WiFi.setOutputPower(gWtOrigPower);
    WiFi.setPhyMode(WIFI_PHY_MODE_11B); // modo 802.11b para mejor cobertura
    WiFi.setAutoReconnect(true);

    /* ── Comprobar pin de forzado de portal ─────────────────────── */
    pinMode(PIN_CFG_FORCE, INPUT_PULLUP);
    bool forcePortal = (digitalRead(PIN_CFG_FORCE) == LOW);
    if (forcePortal)
        LOG_PRINTLN(F("PIN_CFG_FORCE activo → forzando portal WiFi."));

    /* ── Portal forzado por pin físico ─────────────────────────── */
    if (forcePortal)
    {
        LOG_PRINTLN(F("Iniciando portal WiFiManager (forzado por pin)..."));
        WiFiManager wm;
        wm.setConfigPortalTimeout(180);
        wm.setHostname("extractor-esp");
        // startConfigPortal es bloqueante hasta que se configure o expire
        if (wm.startConfigPortal(AP_SSID, SECRET_AP_PASS))
        {
            LOG_PRINTLN(F("Portal: WiFi configurado correctamente."));
        }
        else
        {
            LOG_PRINTLN(F("Portal: expirado sin configurar."));
        }
        // Continuar igualmente — intentar conectar con lo que haya
    }

    /* ── Intentar conectar con credenciales guardadas (máx. 20 s) ─ */
    LOG_PRINTLN(F("Conectando a WiFi guardado..."));
    WiFi.mode(WIFI_STA);
    WiFi.begin(); // usa las credenciales almacenadas por WiFiManager
    {
        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL)
        {
            delay(200);
            yield();
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        gApMode = false;
        LOG_PRINT(F("WiFi OK: "));
        LOG_PRINTLN(WiFi.localIP().toString().c_str());
        LOG_PRINTF_P(PSTR("Heap libre tras WiFi: %u bytes\n"), ESP.getFreeHeap());
    }
    else
    {
        LOG_PRINTLN(F("Sin WiFi → arrancando en modo AP."));
        startApMode();
    }

    /* --- Sincronización de hora --- */
    configTime(0, 0, MY_NTP_SERVER);
    configTzTime(MY_TZ, MY_NTP_SERVER);

    LOG_PRINT(F("Sincronizando hora NTP..."));
    int horaRetries = 0;
    while (!getLocalTime(&timeinfo) && horaRetries < 10)
    {
        Serial.print(".");
        delay(500);
        horaRetries++;
    }

    if (horaRetries >= 10)
    {
        LOG_PRINTLN(F("\n[!] No se obtuvo hora. Usando timestamps relativos."));
        timeOK = false;
    }
    else
    {
        timeOK = true;
        LOG_PRINTLN(F("\n[OK] Hora sincronizada. Logs con fecha activa."));
    }

    // Telnet y WebServer
    TelnetStream.begin();
    telnetReady = true;

    /* ── MQTT sobre TLS (puerto 8883) – sin verificar certificado  */
    wifiClient.setInsecure();
    mqtt.setServer(cfgHost, atoi(cfgPort));
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);
    mqtt.setBufferSize(256);

    /* ── Rutas del servidor web ─────────────────────────────────── */
    webServer.on("/", handleRoot);
    webServer.on("/api", handleApi);
    webServer.on("/set", handleSet);
    webServer.on("/log", handleLog);
    webServer.on("/log/clear", handleLogClear);
    webServer.on("/wifi", handleWifiPortal);                // desconectar + AP + WiFiManager
    webServer.on("/wifi/sta", handleWifiSta);               // cambiar WiFi desde conexión actual
    webServer.on("/api/crash", handleCrashApi);             // info de crash/reset
    webServer.on("/testwifi", handleTestWifi);              // página del test WiFi
    webServer.on("/testwifi/run", handleTestWifiRun);       // disparar test
    webServer.on("/testwifi/status", handleTestWifiStatus); // estado/resultados JSON
    webServer.begin();
    LOG_PRINTLN(F("WebServer iniciado en puerto 80."));

    /* ── OTA ─────────────────────────────────────────────────────*/
    setupOTA();

    /* ── ENS160 ───────────────────────────────────────────────── */
    ens160.begin();

    LOG_PRINTF_P(PSTR("begin ens160.."));
    LOG_PRINTF_P(ens160.available() ? PSTR("done.") : PSTR("failed!"));
    if (ens160.available())
    {
        gENSok = true;
        // Print ENS160 versions
        LOG_PRINTF_P(PSTR("\tens160 RevMajor: %u."), ens160.getMajorRev());
        LOG_PRINTF_P(PSTR("\tens160 RevMinor: %u."), ens160.getMinorRev());
        LOG_PRINTF_P(PSTR("\tens160 RevBuild: %u."), ens160.getBuild());

        LOG_PRINTF_P(PSTR("\tStandard mode "));
        LOG_PRINTF_P(ens160.setMode(ENS160_OPMODE_STD) ? PSTR("done.") : PSTR("failed!"));
    }
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
    /* ── Portal WiFiManager solicitado desde la web ─────────────
     * Se procesa al inicio del loop para que la respuesta HTTP
     * haya sido enviada en la iteración anterior.               */
    if (gPortalRequested)
    {
        gPortalRequested = false;
        LOG_PRINTLN(F("Iniciando portal WiFiManager solicitado por web..."));
        webServer.stop();
        dnsServer.stop(); // detener DNS propio (WiFiManager lanzará el suyo)
        WiFiManager wm;
        wm.setConfigPortalTimeout(180);
        wm.setHostname("extractor-esp");
        if (wm.startConfigPortal(AP_SSID, SECRET_AP_PASS))
        {
            LOG_PRINTLN(F("Portal: WiFi configurado → reiniciando."));
        }
        else
        {
            LOG_PRINTLN(F("Portal: expirado → reiniciando."));
        }
        delay(500);
        ESP.restart();
    }

    /* ── Gestión de conectividad WiFi ─────────────────────────── */
    if (gApMode)
    {
        // Procesar peticiones DNS del captive portal
        dnsServer.processNextRequest();

        // Cada 3 min intentar reconectar al WiFi (manteniendo AP activo)
        if (millis() - tApReconnect >= INTERVAL_AP_RECONNECT)
        {
            tApReconnect = millis();
            LOG_PRINTLN(F("AP: probando reconexión WiFi..."));

            // WIFI_AP_STA: mantenemos el AP activo mientras intentamos STA
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAP(AP_SSID, SECRET_AP_PASS); // re-asegurar AP
            WiFi.begin();                         // intenta con credenciales guardadas

            unsigned long t = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t < 10000UL)
            {
                dnsServer.processNextRequest(); // seguir sirviendo DNS
                delay(200);
                yield();
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                LOG_PRINT(F("WiFi recuperado: "));
                LOG_PRINTLN(WiFi.localIP().toString().c_str());
                LOG_PRINTLN(F("Reiniciando en modo normal..."));
                delay(500);
                ESP.restart(); // reinicio limpio en modo normal
            }
            else
            {
                // No conectó: volver al modo AP puro
                LOG_PRINTLN(F("Sin WiFi — continuando en modo AP."));
                WiFi.mode(WIFI_AP);
                // El softAP ya estaba activo en WIFI_AP_STA — reconfirmar
                WiFi.softAP(AP_SSID, SECRET_AP_PASS);
                delay(100);
                dnsServer.start(53, "*", WiFi.softAPIP());
            }
        }
    }
    else
    {
        /* ── Modo normal (STA): vigilar pérdida de WiFi ───────── */
        // Durante el test WiFi el watchdog se suspende: las desconexiones son intencionadas.
        if (!gWtRunning && WiFi.status() != WL_CONNECTED)
        {
            if (!wifiWasLost)
            {
                wifiWasLost = true;
                tWifiLost = millis();
                LOG_PRINTLN(F("WiFi desconectado."));
            }
            else if (millis() - tWifiLost > INTERVAL_WIFI_WDT)
            {
                LOG_PRINTLN(F("WiFi perdido > 2 min → modo AP."));
                wifiWasLost = false;
                // Detener lo que dependa del WiFi
                mqtt.disconnect();
                // Iniciar modo AP y reiniciar webServer en el nuevo entorno
                webServer.stop();
                startApMode();
                webServer.begin();
            }
        }
        else if (!gWtRunning)
        {
            wifiWasLost = false;
        }
        else
        {
            // Test en curso: resetear el temporizador continuamente para que
            // al terminar el test no haya un disparo inmediato del watchdog.
            tWifiLost = millis();
            wifiWasLost = false;
        }
    }

    /* ── Servicios comunes ──────────────────────────────────────*/
    ArduinoOTA.handle();
    MDNS.update();
    webServer.handleClient();

    /* ── WiFi Test: avanzar un paso de la state machine ─────────*/
    runWifiTestStep();

    /* ── MQTT solo en modo normal (necesita acceso a internet/LAN)*/
    if (!gApMode)
    {
        mqttReconnect();
        mqtt.loop();
    }

    /* ── Sensores y publicación periódica ───────────────────────*/
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