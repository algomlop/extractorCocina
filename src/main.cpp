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
 * D5 → Relay IN (GPIO14)  ← contacto Normalmente Abierto
 * D6 → Alimentación (PWM) del sensor de suelo
 * A0 → Sensor humedad suelo (capacitivo). Calibración desde la web.
 *
 * Lógica AUTO (prioridad de mayor a menor):
 *   1. Programación horaria (cron): ENCENDIDO mientras coincida. Ignora timer/cooldown.
 *   2. Timer máx. encendido + cooldown (mín. 1 min).
 *   3. Sensores:
 *      ENCENDER: AQI >= thresh1  O  HumAire >= thresh2  O  Suelo >= thresh3
 *      APAGAR:   AQI <= thresh1-1 Y HumAire < thresh2-5 Y Suelo < thresh3-5
 *      thresh1 = 6 → el AQI nunca dispara (el AQI máximo es 5).
 *      Protección del relé: tiempo mínimo ON/OFF entre conmutaciones por sensores.
 *   Si el ENS160 no reporta validez NORMAL (calentamiento), el AQI se ignora.
 *
 * Programación (cron): "min hora díaMes mes díaSem", 4 huecos.
 *   Soporta *, n, a-b, a,b,c, *​/n, a-b/n. díaSem: 0 y 7 = domingo.
 *   díaMes y díaSem se combinan con AND. Sin hora válida (NTP) no se dispara.
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
#include <coredecls.h> // settimeofday_cb()

// true  = modo AP permanente (sin conectar al router); solo se conecta cada hora
//         para sincronizar hora (NTP) y enviar/recibir MQTT.
// false = modo normal: conectado al router y MQTT continuo.
// Se puede cambiar desde la web o MQTT (se guarda en cfg.json).
bool modeAP = true;

/* ─── Hora (NTP) ──────────────────────────────────────────────*/
bool timeOK = false;
volatile uint32_t gSyncCount = 0; // nº de sincronizaciones SNTP recibidas
time_t gLastSync = 0;
bool telnetReady = false;

// La llama SNTP al fijar la hora: solo banderas, sin logs ni delay.
void onTimeSet()
{
    timeOK = true;
    gLastSync = time(nullptr);
    gSyncCount++;
}

// Hora local sin bloquear. false si aún no hay hora válida.
static bool nowLocal(struct tm &t)
{
    if (!timeOK)
        return false;
    time_t n = time(nullptr);
    localtime_r(&n, &t);
    return true;
}

/* ══════════════════════════════════════════════════════════
   RTC "caja negra" – sobrevive reinicios (no apagados)
   ══════════════════════════════════════════════════════════ */
#define RTC_MAGIC 0xCAFE1234UL

struct RTCCrashInfo
{
    uint32_t magic;      // RTC_MAGIC si los datos son válidos
    uint32_t resetCount; // reinicios anómalos consecutivos
    uint32_t reason;     // rst_reason del SDK (0=power-on, 2=excepción, ...)
    uint32_t exccause;   // causa de excepción (solo válida si reason==2)
    char lastMsg[76];    // último mensaje de log antes del crash
    uint32_t crc;        // suma de comprobación sencilla
    // Total: 4+4+4+4+76+4 = 96 bytes
};

static RTCCrashInfo gCrashInfo; // buffer vivo: se reescribe en cada log
static RTCCrashInfo gPrevCrash; // copia del arranque anterior (solo lectura)
static bool gPrevValid = false;

static uint32_t rtcCrc(const RTCCrashInfo &d)
{
    uint32_t c = 0;
    const uint8_t *p = (const uint8_t *)&d;
    for (size_t i = 0; i < offsetof(RTCCrashInfo, crc); i++)
        c += p[i];
    return c;
}

void updateRTCLastMsg(const char *msg)
{
    if (!msg)
        return;
    struct rst_info *ri = ESP.getResetInfoPtr();
    gCrashInfo.magic = RTC_MAGIC;
    gCrashInfo.reason = ri ? ri->reason : 0;
    gCrashInfo.exccause = ri ? ri->exccause : 0;
    strncpy(gCrashInfo.lastMsg, msg, sizeof(gCrashInfo.lastMsg) - 1);
    gCrashInfo.lastMsg[sizeof(gCrashInfo.lastMsg) - 1] = '\0';
    gCrashInfo.crc = rtcCrc(gCrashInfo);
    ESP.rtcUserMemoryWrite(0, (uint32_t *)&gCrashInfo, sizeof(gCrashInfo));
}

/* ══════════════════════════════════════════════════════════
   Logging: Serial + Telnet + buffer circular en RAM (/log)
   ══════════════════════════════════════════════════════════ */
#define LOG_RING_SIZE 2048
static char gLogRing[LOG_RING_SIZE];
static uint16_t gLogHead = 0;
static bool gLogWrapped = false;

static void ringAppend(const char *s)
{
    while (*s)
    {
        gLogRing[gLogHead++] = *s++;
        if (gLogHead >= LOG_RING_SIZE)
        {
            gLogHead = 0;
            gLogWrapped = true;
        }
    }
}

void logLine(const char *msg)
{
    if (!msg || msg[0] == '\0')
        return;

    // "Caja negra": guardar el mensaje por si el sistema crashea después
    updateRTCLastMsg(msg);

    char header[32];
    struct tm t;
    if (nowLocal(t))
    {
        char ts[24];
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &t);
        snprintf(header, sizeof(header), "[%s] ", ts);
    }
    else
    {
        snprintf(header, sizeof(header), "[%8lums] ", millis());
    }

    Serial.print(header);
    Serial.print(msg);
    if (telnetReady)
    {
        TelnetStream.print(header);
        TelnetStream.print(msg);
    }
    ringAppend(header);
    ringAppend(msg);
}

#define LOG_PRINT(x) logLine(String(x).c_str())

#define LOG_PRINTLN(x)                    \
    do                                    \
    {                                     \
        String _s = String(x) + "\n";     \
        logLine(_s.c_str());              \
    } while (0)

#define LOG_PRINTF_P(fmt, ...)                              \
    do                                                      \
    {                                                       \
        char _buf[256];                                     \
        snprintf_P(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
        logLine(_buf);                                      \
    } while (0)

/* ─── Pins ────────────────────────────────────────────────────
 * I2C usa D1(SCL/GPIO5) y D2(SDA/GPIO4) — pines Wire por defecto.
 * A0 es el único pin ADC del NodeMCU v2.                        */
#define PIN_RELAY D5     // GPIO14
#define PIN_SOIL A0      // ADC – sensor humedad suelo (capacitivo)
#define PIN_SOIL_VCC D6  // GPIO12 — alimentación (PWM) del sensor de suelo

/* ─── Relay NO ────────────────────────────────────────────────*/
#define RELAY_ON HIGH  // bobina activada  → NO cerrado → extractor ON
#define RELAY_OFF LOW  // bobina en reposo → NO abierto → extractor OFF

/* ─── Calibración sensor suelo (valores por defecto) ──────────
 *  Se pueden recalibrar desde la web (botones "Calibrar seco/mojado"). */
#define SOIL_DRY_DEFAULT 964 // ADC raw con el sensor en aire seco
#define SOIL_WET_DEFAULT 339 // ADC raw con el sensor sumergido
#define SOIL_SAMPLES 8       // lecturas promediadas
#define SOIL_SETTLE_MS 200UL // estabilización tras alimentar el sensor

/* ─── Histéresis ──────────────────────────────────────────────*/
#define HYST_HUM 5.0f  // ±5 % humedad aire
#define HYST_SOIL 5.0f // ±5 % humedad suelo
#define HYST_AQI 1.0f  // unidades AQI

/* ─── Protección del relé (solo conmutaciones por sensores) ───*/
#define RELAY_MIN_ON_MS 60000UL
#define RELAY_MIN_OFF_MS 60000UL

/* ─── Intervalos ──────────────────────────────────────────────*/
#define INTERVAL_SENSOR 10000UL              // leer ENS160/AHT21 cada 10 s
#define INTERVAL_SOIL 120000UL               // leer suelo cada 2 min
#define INTERVAL_MQTT 60000UL                // publicar MQTT cada 60 s
#define INTERVAL_RECONNECT 5000UL            // reintentar MQTT cada 5 s
#define INTERVAL_WIFI_WDT 180000UL           // modo AP si WiFi caído > 3 min
#define INTERVAL_AP_RECONNECT 180000UL       // en AP por fallo de WiFi, reintentar cada 3 min
#define INTERVAL_AP_RECONNECT_MQTT 3600000UL // en AP permanente, conectar cada 60 min (NTP+MQTT)
#define CFG_SAVE_DELAY 5000UL                // agrupar escrituras de cfg.json

/* ─── AP WiFi ────────────────────────────────────────────────*/
#define AP_SSID "ExtractorAP"

/* ─── MQTT topics ─────────────────────────────────────────────*/
#define BASE "extractor"
// ← Publicar
#define T_AQI BASE "/s1/aqi"
#define T_TVOC BASE "/s1/tvoc"
#define T_ECO2 BASE "/s1/eco2"
#define T_HUMIDITY BASE "/s1/humidity"
#define T_TEMP BASE "/s1/temperature"
#define T_SOIL BASE "/s2/soil"
#define T_STATE BASE "/relay/state"         // ON|OFF
#define T_MODE BASE "/mode/state"           // AUTO|MANUAL
#define T_THRESH1 BASE "/cfg/thresh1/state" // AQI umbral (1–6, 6 = desactivado)
#define T_THRESH2 BASE "/cfg/thresh2/state"
#define T_THRESH3 BASE "/cfg/thresh3/state"
#define T_TIMER_ON BASE "/cfg/timer_on/state"
#define T_TIMER_OFF BASE "/cfg/timer_off/state"
#define T_TIMER_STATUS BASE "/timer/status" // IDLE|ON_TIMER|COOLDOWN|SCHEDULE
#define T_MODEAP BASE "/cfg/modeap/state"   // ON|OFF
#define T_SCHED_PREFIX BASE "/cfg/sched/"   // + N + "/state" | "/enabled" | "/set"
// → Comandos (suscribir)
#define T_SET_RELAY BASE "/relay/set"
#define T_SET_MODE BASE "/mode/set"
#define T_SET_THRESH1 BASE "/cfg/thresh1/set"
#define T_SET_THRESH2 BASE "/cfg/thresh2/set"
#define T_SET_THRESH3 BASE "/cfg/thresh3/set"
#define T_SET_TIMER_ON BASE "/cfg/timer_on/set"
#define T_SET_TIMER_OFF BASE "/cfg/timer_off/set"
#define T_SET_MODEAP BASE "/cfg/modeap/set"
#define T_SET_SCHED_ALL BASE "/cfg/sched/+/set" // payload: cron | ON | OFF | vacío (borrar)

#define T_STATUS BASE "/status"

#define ENS160_I2C_ADDRESS 0x53

#define MY_NTP_SERVER "pool.ntp.org"
#define MY_TZ "CET-1CEST,M3.5.0/02,M10.5.0/03" // Europa Central

/* ─── Objetos ─────────────────────────────────────────────────*/
ScioSense_ENS160 ens160(ENS160_I2C_ADDRESS);
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
bool ignoregENSValid = true; // lo pongo a true para ignorar el gENSValid que viene del sensor y que parece que no funciona
bool gENSValid = ignoregENSValid; // true solo cuando ENS160 reporta validez NORMAL
float gHumidity = 0.0f; // % humedad relativa aire (AHT21)
float gTemp = 0.0f;     // °C (AHT21)
float gSoilHum = 0.0f;  // % humedad suelo
int gSoilRaw = 0;       // última lectura ADC promediada
bool gSoilHasData = false;
int gSoilDry = SOIL_DRY_DEFAULT;
int gSoilWet = SOIL_WET_DEFAULT;
bool gSoilPowered = false;
unsigned long tSoil = 0;
unsigned long tSoilPower = 0;

bool gAHTok = false;
bool gENSok = false;

/* ─── Control extractor ───────────────────────────────────────*/
bool gExtractor = false;
bool gAutoMode = true;
float gThresh1 = 3.0f;  // AQI: encender si AQI >= valor (6 = desactivado)
float gThresh2 = 80.0f; // Humedad aire %
float gThresh3 = 60.0f; // Humedad suelo %
unsigned long tLastSwitch = 0;

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

/* ─── Guardado diferido de config (menos escrituras en flash) ─*/
bool gCfgDirty = false;
unsigned long tCfgDirty = 0;

void saveConfig(); // definida más abajo
void saveConfigLater()
{
    gCfgDirty = true;
    tCfgDirty = millis();
}

/* ══════════════════════════════════════════════════════════
   Programación horaria (cron simplificado)
   ══════════════════════════════════════════════════════════ */
#define MAX_SCHEDULES 4

struct Schedule
{
    char expr[40]; // "min hora dia mes dsem"
    bool enabled;
    uint64_t minutes; // bits 0-59
    uint32_t hours;   // bits 0-23
    uint32_t dom;     // bits 1-31
    uint16_t months;  // bits 1-12
    uint8_t dow;      // bits 0-6 (0 = domingo)
};

Schedule gSched[MAX_SCHEDULES]; // global => inicializado a 0
bool gSchedActive = false;

// Campo cron ("*", "5", "1-4", "*/15", "1,3,10-12", "0-30/5") -> máscara de bits
static bool parseCronField(const char *src, int lo, int hi, uint64_t &mask)
{
    char buf[40];
    strncpy(buf, src, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    mask = 0;
    char *save = nullptr;
    for (char *tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(nullptr, ",", &save))
    {
        int step = 1;
        char *slash = strchr(tok, '/');
        if (slash)
        {
            *slash = '\0';
            if (!isdigit((unsigned char)slash[1]))
                return false;
            step = atoi(slash + 1);
            if (step < 1)
                return false;
        }
        int a, b;
        if (strcmp(tok, "*") == 0)
        {
            a = lo;
            b = hi;
        }
        else
        {
            if (!isdigit((unsigned char)tok[0]))
                return false;
            char *dash = strchr(tok, '-');
            if (dash)
            {
                *dash = '\0';
                if (!isdigit((unsigned char)dash[1]))
                    return false;
                a = atoi(tok);
                b = atoi(dash + 1);
            }
            else
            {
                a = atoi(tok);
                b = slash ? hi : a; // "5/10" = desde 5 hasta el máximo, paso 10
            }
        }
        if (a < lo || b > hi || a > b)
            return false;
        for (int v = a; v <= b; v += step)
            mask |= (1ULL << v);
    }
    return mask != 0;
}

// Parsea "m h dom mon dow". No toca s.enabled.
bool parseSchedule(const char *expr, Schedule &s)
{
    if (!expr || strlen(expr) >= sizeof(s.expr))
        return false;
    char buf[sizeof(s.expr)];
    strcpy(buf, expr);
    char *f[5];
    int n = 0;
    char *save = nullptr;
    for (char *t = strtok_r(buf, " \t", &save); t; t = strtok_r(nullptr, " \t", &save))
    {
        if (n == 5)
            return false;
        f[n++] = t;
    }
    if (n != 5)
        return false;

    uint64_t m, h, d, mo, w;
    if (!parseCronField(f[0], 0, 59, m) || !parseCronField(f[1], 0, 23, h) ||
        !parseCronField(f[2], 1, 31, d) || !parseCronField(f[3], 1, 12, mo) ||
        !parseCronField(f[4], 0, 7, w))
        return false;
    if (w & (1ULL << 7))
        w |= 1; // 7 = domingo

    s.minutes = m;
    s.hours = (uint32_t)h;
    s.dom = (uint32_t)d;
    s.months = (uint16_t)mo;
    s.dow = (uint8_t)(w & 0x7F);
    snprintf(s.expr, sizeof(s.expr), "%s %s %s %s %s", f[0], f[1], f[2], f[3], f[4]);
    return true;
}

static bool scheduleMatches(const Schedule &s, const struct tm &t)
{
    return s.enabled && s.expr[0] &&
           ((s.minutes >> t.tm_min) & 1ULL) &&
           ((s.hours >> t.tm_hour) & 1UL) &&
           ((s.dom >> t.tm_mday) & 1UL) &&
           ((s.months >> (t.tm_mon + 1)) & 1U) &&
           ((s.dow >> t.tm_wday) & 1U);
}

bool scheduleActive()
{
    struct tm t;
    if (!nowLocal(t))
        return false; // sin hora válida, nunca se dispara
    for (int i = 0; i < MAX_SCHEDULES; i++)
        if (scheduleMatches(gSched[i], t))
            return true;
    return false;
}

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
#define WTEST_TOTAL (WTEST_N_MODES * WTEST_N_POWERS) // 21

#define WTEST_ERR_OK 0         // sin error
#define WTEST_ERR_TIMEOUT 1    // timeout de conexión WiFi
#define WTEST_ERR_SPEED 2      // test de velocidad fallido
#define WTEST_ERR_GLOBAL_TMO 3 // test abortado por timeout global
#define WTEST_ERR_ABORTED 4    // abortado (p.ej. reinicio inesperado)

#define WTEST_GLOBAL_TIMEOUT_MS (12UL * 60UL * 1000UL) // 12 minutos

struct WifiTestEntry
{
    uint8_t phyMode;     // 1=11b 2=11g 3=11n
    float power;         // dBm
    bool connected;      // ¿logró conectar?
    int8_t rssi;         // dBm señal (0 si no conectó)
    uint16_t connectMs;  // ms hasta WL_CONNECTED
    int32_t downloadBps; // bytes/s promedio (-1 = sin dato)
    uint8_t errCode;     // WTEST_ERR_* (0=OK)
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
bool gWtDone = false;
String gWtTimestamp = "";
WtPhase gWtPhase = WTP_IDLE;
unsigned long gWtPhaseTimer = 0;

uint8_t gWtOrigPhyMode = WIFI_PHY_MODE_11B;
float gWtOrigPower = 19.5f;

unsigned long gWtStartTime = 0;

/* ══════════════════════════════════════════════════════════
   RTC crash tracking: carga del arranque anterior
   ══════════════════════════════════════════════════════════ */
void loadRTCCrash()
{
    RTCCrashInfo tmp;
    ESP.rtcUserMemoryRead(0, (uint32_t *)&tmp, sizeof(tmp));
    if (tmp.magic == RTC_MAGIC && tmp.crc == rtcCrc(tmp))
    {
        gPrevCrash = tmp;
        gCrashInfo = tmp;
        gPrevValid = true;
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
        modeAP = doc["ap"] | true;
        gSoilDry = doc["sd"] | SOIL_DRY_DEFAULT;
        gSoilWet = doc["sw"] | SOIL_WET_DEFAULT;

        int si = 0;
        for (JsonObject o : doc["sch"].as<JsonArray>())
        {
            if (si >= MAX_SCHEDULES)
                break;
            Schedule tmp = {};
            if (parseSchedule(o["e"] | "", tmp))
            {
                tmp.enabled = o["on"] | false;
                gSched[si] = tmp;
            }
            si++;
        }
    }
    f.close();
    LOG_PRINTF_P(PSTR("Config OK. MQTT: %s:%s\n"), cfgHost, cfgPort);
}

void saveConfig()
{
    if (!littleFSok)
    {
        LOG_PRINTLN(F("LittleFS no montado: config no guardada."));
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
    doc["ap"] = modeAP;
    doc["sd"] = gSoilDry;
    doc["sw"] = gSoilWet;
    JsonArray sch = doc["sch"].to<JsonArray>();
    for (int i = 0; i < MAX_SCHEDULES; i++)
    {
        JsonObject o = sch.add<JsonObject>();
        o["e"] = gSched[i].expr;
        o["on"] = gSched[i].enabled;
    }
    serializeJson(doc, f);
    f.close();
    LOG_PRINTLN(F("Config guardada."));
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
    delay(100);
    dnsServer.start(53, "*", WiFi.softAPIP());
    tApReconnect = millis();
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
    tLastSwitch = millis();
    if (on)
    {
        tExtractorOn = tLastSwitch;
        gInCooldown = false;
    }
    LOG_PRINTF_P(PSTR("Extractor: %s\n"), on ? "ON" : "OFF");
    if (mqtt.connected())
        mqtt.publish(T_STATE, on ? "ON" : "OFF", true);
}

/* ══════════════════════════════════════════════════════════
   Lógica automática: programación + timer + sensores
   ══════════════════════════════════════════════════════════ */
static uint16_t cooldownMin() { return gTimerOffMin > 0 ? gTimerOffMin : 1; }

void evaluateAuto()
{
    if (!gAutoMode)
        return;
    unsigned long now = millis();

    /* ── Programación horaria: prioridad máxima ── */
    static bool prevSched = false;
    gSchedActive = scheduleActive();
    if (gSchedActive != prevSched)
    {
        LOG_PRINTLN(gSchedActive ? F("Programación: ventana ACTIVA") : F("Programación: ventana terminada"));
        if (!gSchedActive)
        {
            tExtractorOn = now; // reiniciar contador de "máx. encendido"
            bool trig = (gENSValid && (float)gAQI >= gThresh1) ||
                        gHumidity >= gThresh2 || gSoilHum >= gThresh3;
            if (!trig)
                setExtractor(false); // ningún sensor lo pide: apagar al acabar la ventana
        }
        prevSched = gSchedActive;
    }
    if (gSchedActive)
    {
        gInCooldown = false;
        setExtractor(true);
        return;
    }

    /* ── Timer: apagado por tiempo máximo encendido ── */
    if (gExtractor && gTimerOnMin > 0 &&
        now - tExtractorOn >= (unsigned long)gTimerOnMin * 60000UL)
    {
        LOG_PRINTLN(F("Timer ON expirado → apagando."));
        setExtractor(false);
        gInCooldown = true; // cooldown mínimo 1 min para evitar reencendido inmediato
        tCooldownStart = now;
        LOG_PRINTF_P(PSTR("Cooldown: %u min.\n"), (unsigned)cooldownMin());
        return;
    }

    /* ── Cooldown: espera antes de volver a encender ── */
    if (gInCooldown)
    {
        if (now - tCooldownStart < (unsigned long)cooldownMin() * 60000UL)
            return;
        gInCooldown = false;
        LOG_PRINTLN(F("Cooldown terminado."));
    }

    /* ── Disparadores por sensores ── */
    bool trigAQI = gENSValid && ((float)gAQI >= gThresh1); // thresh1=6 => nunca
    bool trigHum = (gHumidity >= gThresh2);
    bool trigSoil = (gSoilHum >= gThresh3);

    if (!gExtractor)
    {
        if ((trigAQI || trigHum || trigSoil) && now - tLastSwitch >= RELAY_MIN_OFF_MS)
            setExtractor(true);
    }
    else
    {
        bool okAQI = !gENSValid || ((float)gAQI <= gThresh1 - HYST_AQI);
        bool okHum = (gHumidity < gThresh2 - HYST_HUM);
        bool okSoil = (gSoilHum < gThresh3 - HYST_SOIL);
        if (okAQI && okHum && okSoil && now - tLastSwitch >= RELAY_MIN_ON_MS)
            setExtractor(false);
    }
}

/* ══════════════════════════════════════════════════════════
   Sensores
   ══════════════════════════════════════════════════════════ */

// Validez del ENS160 (registro DEVICE_STATUS 0x20, bits 3:2):
// 0 = normal, 1 = calentando, 2 = arranque inicial, 3 = inválido. -1 = error I2C.
static int8_t ens160Validity()
{
    Wire.beginTransmission(ENS160_I2C_ADDRESS);
    Wire.write(0x20);
    if (Wire.endTransmission() != 0)
        return -1;
    if (Wire.requestFrom((uint8_t)ENS160_I2C_ADDRESS, (uint8_t)1) != 1)
        return -1;
    return (Wire.read() >> 2) & 0x03;
}

static void soilCompute()
{
    int lo = gSoilWet, hi = gSoilDry;
    if (hi - lo < 10)
    {
        gSoilHum = 0.0f;
        return;
    }
    int c = constrain(gSoilRaw, lo, hi);
    gSoilHum = 100.0f * (hi - c) / (float)(hi - lo);
}

// Lectura de suelo NO bloqueante: alimenta el sensor, espera, promedia y apaga.
void soilTick()
{
    unsigned long now = millis();
    if (!gSoilPowered)
    {
        if (now - tSoil < INTERVAL_SOIL)
            return;
        analogWrite(PIN_SOIL_VCC, 128); // PWM 50 %
        tSoilPower = now;
        gSoilPowered = true;
        return;
    }
    if (now - tSoilPower < SOIL_SETTLE_MS)
        return;

    long sum = 0;
    for (int i = 0; i < SOIL_SAMPLES; i++)
    {
        sum += analogRead(PIN_SOIL);
        delay(2);
    }
    digitalWrite(PIN_SOIL_VCC, LOW);
    gSoilPowered = false;
    tSoil = now;
    gSoilRaw = (int)(sum / SOIL_SAMPLES);
    gSoilHasData = true;
    soilCompute();
}

void readSensors()
{
    /* 1. AHT21 primero – sus datos compensan el ENS160 */
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
    if (gENSok)
    {
        if (gAHTok)
            ens160.set_envdata(gTemp, gHumidity);
        if (!ignoregENSValid)
            gENSValid = (ens160Validity() == 0);
        ens160.measure(false); // sin esperar: si no hay dato nuevo, se mantiene el anterior
        gAQI = ens160.getAQI();
        gTVOC = ens160.getTVOC();
        gECO2 = ens160.geteCO2();
    }

    LOG_PRINTF_P(
        PSTR("ENS160 AQI:%u TVOC:%u ppb eCO2:%u ppm%s | "
             "AHT21 HR:%.1f%% T:%.1f°C | "
             "Suelo raw:%d %.1f%%\n"),
        gAQI, gTVOC, gECO2, gENSValid ? "" : " (no válido)",
        gHumidity, gTemp, gSoilRaw, gSoilHum);
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
        snprintf(buf, sizeof(buf), "%u", (unsigned)v);
        mqtt.publish(t, buf, true);
    };

    pubU(T_AQI, gAQI);
    pubU(T_TVOC, gTVOC);
    pubU(T_ECO2, gECO2);

    pubF(T_HUMIDITY, gHumidity);
    pubF(T_TEMP, gTemp);
    pubF(T_SOIL, gSoilHum);

    pubF(T_THRESH1, gThresh1, 0);
    pubF(T_THRESH2, gThresh2);
    pubF(T_THRESH3, gThresh3);

    pubU(T_TIMER_ON, gTimerOnMin);
    pubU(T_TIMER_OFF, gTimerOffMin);

    mqtt.publish(T_STATE, gExtractor ? "ON" : "OFF", true);
    mqtt.publish(T_MODE, gAutoMode ? "AUTO" : "MANUAL", true);
    mqtt.publish(T_MODEAP, modeAP ? "ON" : "OFF", true);

    const char *ts = gSchedActive                      ? "SCHEDULE"
                     : gInCooldown                     ? "COOLDOWN"
                     : (gExtractor && gTimerOnMin > 0) ? "ON_TIMER"
                                                       : "IDLE";
    mqtt.publish(T_TIMER_STATUS, ts, true);

    char tb[40];
    for (int i = 0; i < MAX_SCHEDULES; i++)
    {
        snprintf(tb, sizeof(tb), T_SCHED_PREFIX "%d/state", i);
        mqtt.publish(tb, gSched[i].expr, true);
        snprintf(tb, sizeof(tb), T_SCHED_PREFIX "%d/enabled", i);
        mqtt.publish(tb, gSched[i].enabled ? "ON" : "OFF", true);
    }
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
        gThresh1 = constrain(atof(msg), 1.0f, 6.0f);
        saveConfigLater();
        evaluateAuto();
    }
    else if (strcmp(topic, T_SET_THRESH2) == 0)
    {
        gThresh2 = constrain(atof(msg), 0.0f, 100.0f);
        saveConfigLater();
        evaluateAuto();
    }
    else if (strcmp(topic, T_SET_THRESH3) == 0)
    {
        gThresh3 = constrain(atof(msg), 0.0f, 100.0f);
        saveConfigLater();
        evaluateAuto();
    }
    else if (strcmp(topic, T_SET_TIMER_ON) == 0)
    {
        gTimerOnMin = (uint16_t)constrain(atoi(msg), 0, 1440);
        saveConfigLater();
    }
    else if (strcmp(topic, T_SET_TIMER_OFF) == 0)
    {
        gTimerOffMin = (uint16_t)constrain(atoi(msg), 0, 1440);
        saveConfigLater();
    }
    else if (strcmp(topic, T_SET_MODEAP) == 0)
    {
        bool v = (strcmp(msg, "ON") == 0);
        if (v != modeAP) // solo actúa si cambia (evita bucles con mensajes retenidos)
        {
            modeAP = v;
            saveConfig();
            gCfgDirty = false;
            delay(200);
            ESP.restart();
        }
    }
    else if (strncmp(topic, T_SCHED_PREFIX, strlen(T_SCHED_PREFIX)) == 0)
    {
        int i = topic[strlen(T_SCHED_PREFIX)] - '0'; // calcular antes de cualquier publish
        if (i >= 0 && i < MAX_SCHEDULES)
        {
            if (strcmp(msg, "ON") == 0 || strcmp(msg, "OFF") == 0)
            {
                gSched[i].enabled = (msg[1] == 'N');
            }
            else if (msg[0] == '\0')
            {
                memset(&gSched[i], 0, sizeof(Schedule));
            }
            else
            {
                Schedule tmp = {};
                if (parseSchedule(msg, tmp))
                {
                    tmp.enabled = true;
                    gSched[i] = tmp;
                }
                else
                {
                    LOG_PRINTLN(F("MQTT: cron inválido"));
                }
            }
            saveConfigLater();
            evaluateAuto();
        }
    }
    publishAll();
}

/* ══════════════════════════════════════════════════════════
   MQTT – reconexión (no bloqueante entre intentos)
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
        mqtt.subscribe(T_SET_MODEAP);
        mqtt.subscribe(T_SET_SCHED_ALL);
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
<style>
body{font-family:sans-serif;max-width:520px;margin:16px auto;padding:0 12px;line-height:1.7}
h2{color:#1565c0;margin-bottom:4px}
h3{margin:16px 0 4px;border-bottom:1px solid #ccc;padding-bottom:2px}
.val{font-weight:bold}
.aqi1{color:#2e7d32}.aqi2{color:#7cb342}.aqi3{color:#f9a825}.aqi4{color:#ef6c00}.aqi5{color:#c62828}
.on{color:#2e7d32;font-weight:bold}.off{color:#757575;font-weight:bold}
.badge{padding:2px 8px;border-radius:10px;color:#fff;font-size:12px;vertical-align:middle}
.ba{background:#1565c0}.bm{background:#6a1b9a}.bc{background:#ef6c00}
#ap-banner{display:none;background:#fff3e0;border-left:4px solid #ef6c00;padding:8px;margin:8px 0}
#crash-info{display:none;background:#ffebee;border-left:4px solid #c62828;padding:8px;margin:8px 0}
#soil-track{width:200px;height:8px;background:#e0e0e0;border-radius:4px;overflow:hidden}
#soil-bar{height:100%;width:0}
#wifi-form{display:none}
input[type=number]{width:70px}
button{margin:2px}
</style>
</head><body>
<h2>Extractor</h2>

<div id="ap-banner">
  ⚠️ Modo AP activo — sin WiFi. IP: <span id="vip">192.168.4.1</span>
</div>
<div>Hora: <span id="vtime">–</span></div>

<h3>Calidad del aire</h3>
AQI: <span id="vaqi">–</span><br>
TVOC: <span id="vtvoc">–</span><br>
eCO2: <span id="veco2">–</span>

<h3>Ambiente</h3>
Humedad: <span id="vhum">–</span><br>
Temperatura: <span id="vtemp">–</span>

<h3>Humedad suelo</h3>
<span id="vsoil">–</span> <small>(raw <span id="vraw">–</span>)</small>
<div id="soil-track"><div id="soil-bar"></div></div>
<button onclick="calSoil('dry')">Calibrar seco</button>
<button onclick="calSoil('wet')">Calibrar mojado</button>

<h3>Extractor — <span id="mode-b" class="badge">–</span></h3>
Estado: <span id="ext-s">–</span> <span id="timer-info"></span><br>
<button onclick="cmd('/set?relay=ON')">Encender</button>
<button onclick="cmd('/set?relay=OFF')">Apagar</button>
<button onclick="cmd('/set?mode=AUTO')">Auto</button>

<h3>Umbrales (±5% histéresis)</h3>
AQI (1–5, 6 = desactivado): <input type="number" id="t1" min="1" max="6" step="1"> <button onclick="setV('t1')">✓</button><br>
Humedad aire: <input type="number" id="t2" min="0" max="100">% <button onclick="setV('t2')">✓</button><br>
Humedad suelo: <input type="number" id="t3" min="0" max="100">% <button onclick="setV('t3')">✓</button>

<h3>Temporizador (0 = desactivado)</h3>
Máx. encendido: <input type="number" id="ton" min="0" max="1440"> min <button onclick="setV('ton')">✓</button><br>
Cooldown: <input type="number" id="tof" min="0" max="1440"> min <button onclick="setV('tof')">✓</button><br>
<small>Si hay máx. encendido, el cooldown mínimo es 1 min.</small>

<h3>Programación horaria</h3>
<small>Cron: <code>min hora díaMes mes díaSem</code>. Ej.: <code>0-4 13-14 * * *</code> = minutos 0–4 de las 13h y 14h.
Campo vacío + ✓ = borrar. Solo en modo AUTO.</small>
<div id="sched"></div>

<h3>Logs &amp; Sistema</h3>
<div id="crash-info"></div>
Heap: <span id="vheap">–</span><br>
<a href="/log" target="_blank">Ver log</a>

<h3>Red WiFi</h3>
IP: <span id="vip-sta">–</span><br>
<label><input type="checkbox" id="apmode" onchange="setApMode()"> Modo AP permanente (conecta al router solo cada hora para NTP y MQTT)</label><br>
<button onclick="toggleWifiForm()">Cambiar WiFi</button>
<button onclick="wifiPortal()">Desconectar + AP</button>
<div id="wifi-form">
  SSID: <input type="text" id="wssid" placeholder="Nombre red"><br>
  Pass: <input type="password" id="wpass" placeholder="Contraseña"><br>
  <button onclick="submitWifiSta()">Aplicar y reiniciar</button>
  <button onclick="toggleWifiForm()">Cancelar</button>
</div>

<script>
var AQI_LBL=['','Buena','Moderada','Sensible','Mala','Pésima'];
function $(id){return document.getElementById(id);}
function setIn(id,v){var e=$(id);if(document.activeElement!==e)e.value=v;}
function refresh(){
  fetch('/api').then(function(r){return r.json();}).then(function(d){
    var aq=$('vaqi');
    aq.textContent=d.aqi+(d.aqi>=1&&d.aqi<=5?' – '+AQI_LBL[d.aqi]:'')+(d.ensv?'':' (calentando / no válido)');
    aq.className='val aqi'+d.aqi;
    $('vtvoc').textContent=d.tvoc+' ppb';
    $('veco2').textContent=d.eco2+' ppm';
    $('vhum').textContent=d.humidity.toFixed(1)+'%';
    $('vtemp').textContent=d.temp.toFixed(1)+'°C';
    $('vtime').textContent=d.sync?d.time:'sin sincronizar (sin NTP)';
    // Suelo
    var sp=Math.min(100,Math.max(0,d.soil));
    $('vsoil').textContent=sp.toFixed(0)+'%';
    $('vraw').textContent=d.soilraw;
    var bar=$('soil-bar');
    bar.style.width=sp+'%';
    bar.style.background=sp>70?'#1565c0':sp>40?'#43a047':'#ef6c00';
    // Extractor
    var ext=$('ext-s');
    ext.textContent=d.relay?'ON':'OFF'; ext.className=d.relay?'on':'off';
    var mb=$('mode-b');
    if(d.cooldown){mb.textContent='COOLDOWN';mb.className='badge bc';}
    else if(d.auto){mb.textContent='AUTO';mb.className='badge ba';}
    else{mb.textContent='MANUAL';mb.className='badge bm';}
    var ti=$('timer-info');
    if(d.sched) ti.textContent='Programación activa';
    else if(d.cooldown) ti.textContent='Cooldown: '+Math.max(1,d.tof)+' min';
    else if(d.relay&&d.ton>0) ti.textContent='Timer ON: max '+d.ton+' min';
    else ti.textContent='';
    setIn('t1',d.t1); setIn('t2',d.t2); setIn('t3',d.t3);
    setIn('ton',d.ton); setIn('tof',d.tof);
    // Programación
    for(var i=0;i<d.sch.length;i++){
      setIn('s'+i,d.sch[i].e);
      $('s'+i+'on').checked=d.sch[i].on;
    }
    // Red
    if(d.ip) $('vip-sta').textContent=d.ip;
    var apm=$('apmode');
    if(document.activeElement!==apm) apm.checked=d.apmode;
    var banner=$('ap-banner');
    if(d.ap){
      banner.style.display='block';
      if(d.ip) $('vip').textContent=d.ip;
    } else {
      banner.style.display='none';
    }
  }).catch(function(){});
}
// Info de crash y heap (solo una vez)
function loadCrashInfo(){
  fetch('/api/crash').then(function(r){return r.json();}).then(function(d){
    if(d.free_heap) $('vheap').textContent=Math.round(d.free_heap/1024)+'KB';
    var box=$('crash-info');
    if(d.prev_valid && d.prev_reason_n!==0 && d.prev_reason_n!==4 && d.prev_reason_n!==5){
      box.style.display='block';
      var isCrash=(d.prev_reason_n===1||d.prev_reason_n===2||d.prev_reason_n===3);
      box.innerHTML=(isCrash?'⚠️ <strong>Crash detectado</strong>':'ℹ️ Reset previo')+
        ' — motivo: <strong>'+d.prev_reason+'</strong>'+
        (d.reset_count>0?' (×'+d.reset_count+')':'')+
        (d.prev_last_msg?'<br>Último log: <em>'+d.prev_last_msg+'</em>':'');
    }
  }).catch(function(){});
}
function cmd(u){fetch(u).then(refresh);}
function setV(id){cmd('/set?'+id+'='+$(id).value);}
function calSoil(t){
  var m=t==='dry'?'Pon el sensor al aire (seco) y confirma.':'Sumerge el sensor en agua y confirma.';
  if(confirm(m)) fetch('/set?cal='+t).then(function(r){
    if(!r.ok) alert('Lectura no válida o sin datos aún.');
    refresh();
  });
}
// Programación
function buildSched(){
  var c=$('sched');
  for(var i=0;i<4;i++){
    var d=document.createElement('div');
    d.innerHTML='<input type="checkbox" id="s'+i+'on" onchange="setSchOn('+i+')"> '+
      '<input type="text" id="s'+i+'" size="20" placeholder="vacío = sin programa"> '+
      '<button onclick="setSched('+i+')">✓</button>';
    c.appendChild(d);
  }
}
function setSched(i){
  var v=$('s'+i).value.trim();
  fetch('/set?s'+i+'='+encodeURIComponent(v)).then(function(r){
    if(!r.ok) alert('Expresión cron no válida');
    refresh();
  });
}
function setSchOn(i){
  cmd('/set?s'+i+'on='+($('s'+i+'on').checked?1:0));
}
function setApMode(){
  var c=$('apmode').checked;
  if(confirm('Se reiniciará el dispositivo. ¿Continuar?')){
    fetch('/set?ap='+(c?1:0))
      .then(function(){alert('Reiniciando… vuelve a conectarte en unos segundos.');})
      .catch(function(){alert('Reiniciando…');});
  } else { $('apmode').checked=!c; }
}
function toggleWifiForm(){
  var f=$('wifi-form');
  f.style.display=f.style.display==='block'?'none':'block';
}
function submitWifiSta(){
  var s=$('wssid').value.trim();
  var p=$('wpass').value;
  if(!s){alert('Introduce el SSID.');return;}
  if(confirm('¿Conectar a "'+s+'" y reiniciar el dispositivo?')){
    fetch('/wifi/sta?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p))
      .then(function(){alert('Reiniciando… Reconéctate a la misma red en unos segundos.');})
      .catch(function(){alert('Reiniciando…');});
  }
}
function wifiPortal(){
  if(confirm('¿Desconectar del WiFi y abrir portal de configuración?\n\nSe creará el AP "ExtractorAP". El dispositivo se reiniciará al terminar (o en 3 min).')){
    fetch('/wifi').then(function(){
      alert('Portal iniciado.\nConéctate al AP "ExtractorAP" y abre 192.168.4.1');
    });
  }
}
buildSched(); refresh(); setInterval(refresh,8000);
loadCrashInfo();
</script><br>v2.1</body></html>
)html";

void handleRoot()
{
    webServer.sendHeader("Cache-Control", "no-cache");
    webServer.send_P(200, "text/html", HTML);
}

void handleApi()
{
    JsonDocument doc;
    doc["aqi"] = gAQI;
    doc["tvoc"] = gTVOC;
    doc["eco2"] = gECO2;
    doc["ensv"] = gENSValid;
    doc["humidity"] = (float)round(gHumidity * 10) / 10.0f;
    doc["temp"] = (float)round(gTemp * 10) / 10.0f;
    doc["soil"] = (float)round(gSoilHum * 10) / 10.0f;
    doc["soilraw"] = gSoilRaw;
    doc["relay"] = gExtractor;
    doc["auto"] = gAutoMode;
    doc["cooldown"] = gInCooldown;
    doc["sched"] = gSchedActive;
    doc["t1"] = gThresh1;
    doc["t2"] = gThresh2;
    doc["t3"] = gThresh3;
    doc["ton"] = gTimerOnMin;
    doc["tof"] = gTimerOffMin;
    // Red
    doc["ap"] = gApMode;
    doc["apmode"] = modeAP;
    doc["ip"] = gApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    // Hora
    struct tm t;
    bool haveTime = nowLocal(t);
    doc["sync"] = haveTime;
    if (haveTime)
    {
        char b[24];
        strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &t);
        doc["time"] = b;
    }
    // Programaciones
    JsonArray sch = doc["sch"].to<JsonArray>();
    for (int i = 0; i < MAX_SCHEDULES; i++)
    {
        JsonObject o = sch.add<JsonObject>();
        o["e"] = gSched[i].expr;
        o["on"] = gSched[i].enabled;
    }
    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void handleSet()
{
    bool changed = false;
    bool bad = false;

    /* Modo AP permanente: se guarda y se reinicia */
    if (webServer.hasArg("ap"))
    {
        bool v = (webServer.arg("ap") == "1");
        if (v != modeAP)
        {
            modeAP = v;
            saveConfig();
            gCfgDirty = false;
            webServer.send(200, "text/plain", "OK");
            delay(300);
            ESP.restart();
            return;
        }
    }
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
        changed = true;
    }
    if (webServer.hasArg("t1"))
    {
        gThresh1 = constrain(webServer.arg("t1").toFloat(), 1.0f, 6.0f);
        changed = true;
    }
    if (webServer.hasArg("t2"))
    {
        gThresh2 = constrain(webServer.arg("t2").toFloat(), 0.0f, 100.0f);
        changed = true;
    }
    if (webServer.hasArg("t3"))
    {
        gThresh3 = constrain(webServer.arg("t3").toFloat(), 0.0f, 100.0f);
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

    /* Calibración del sensor de suelo con la última lectura */
    if (webServer.hasArg("cal"))
    {
        String c = webServer.arg("cal");
        if (gSoilHasData && c == "dry" && gSoilRaw > gSoilWet + 20)
        {
            gSoilDry = gSoilRaw;
            soilCompute();
            changed = true;
        }
        else if (gSoilHasData && c == "wet" && gSoilRaw < gSoilDry - 20)
        {
            gSoilWet = gSoilRaw;
            soilCompute();
            changed = true;
        }
        else
        {
            bad = true;
        }
    }

    /* Programaciones: s0..s3 = expresión (vacío = borrar), s0on..s3on = 1/0 */
    for (int i = 0; i < MAX_SCHEDULES; i++)
    {
        String k = "s" + String(i);
        if (webServer.hasArg(k))
        {
            String v = webServer.arg(k);
            v.trim();
            if (v.length() == 0)
            {
                memset(&gSched[i], 0, sizeof(Schedule));
                changed = true;
            }
            else
            {
                Schedule tmp = {};
                if (parseSchedule(v.c_str(), tmp))
                {
                    tmp.enabled = true;
                    gSched[i] = tmp;
                    changed = true;
                }
                else
                {
                    bad = true;
                }
            }
        }
        if (webServer.hasArg(k + "on"))
        {
            gSched[i].enabled = (webServer.arg(k + "on") == "1");
            changed = true;
        }
    }

    if (changed)
    {
        evaluateAuto();
        saveConfigLater();
        publishAll();
    }
    webServer.send(bad ? 400 : 200, "text/plain", bad ? "invalido" : "OK");
}

/* ══════════════════════════════════════════════════════════
   Web – configurar WiFi STA desde la conexión actual (/wifi/sta)
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
    if (gCfgDirty)
        saveConfig();
    WiFi.persistent(true);
    WiFi.begin(ssid.c_str(), pass.c_str());
    WiFi.persistent(false);
    delay(300);
    ESP.restart();
}

/* ══════════════════════════════════════════════════════════
   Web – info de crash (/api/crash)
   prev_* = por qué terminó la ejecución anterior (motivo del
   reinicio actual) y su último mensaje de log.
   ══════════════════════════════════════════════════════════ */
void handleCrashApi()
{
    JsonDocument doc;
    struct rst_info *ri = ESP.getResetInfoPtr();
    int r = ri ? (int)ri->reason : -1;
    doc["current_reason"] = ri ? rstReasonStr(ri->reason) : "?";
    doc["current_reason_n"] = r;
    doc["free_heap"] = (int)ESP.getFreeHeap();
    doc["chip_id"] = String(ESP.getChipId(), HEX);
    doc["prev_valid"] = gPrevValid;
    if (gPrevValid)
    {
        doc["prev_reason"] = ri ? rstReasonStr(ri->reason) : "?";
        doc["prev_reason_n"] = r;
        doc["prev_exccause"] = ri ? (int)ri->exccause : 0;
        doc["prev_last_msg"] = gPrevCrash.lastMsg;
        doc["reset_count"] = (int)gCrashInfo.resetCount;
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
                     "<strong>192.168.4.1</strong> para configurar el WiFi. "
                     "Al terminar, el dispositivo se reiniciará automáticamente.</p>"
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

/* ══════════════════════════════════════════════════════════
   Web – /log: motivo del último reinicio + log reciente en RAM
   ══════════════════════════════════════════════════════════ */
static void appendEsc(String &o, const char *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
    {
        char c = p[i];
        if (c == '<')
            o += F("&lt;");
        else if (c == '>')
            o += F("&gt;");
        else if (c == '&')
            o += F("&amp;");
        else
            o += c;
    }
}

static void sendRingPart(const char *p, size_t n)
{
    String o;
    while (n)
    {
        size_t k = n > 64 ? 64 : n;
        o = "";
        appendEsc(o, p, k);
        webServer.sendContent(o);
        p += k;
        n -= k;
        yield();
    }
}

void handleLog()
{
    webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
    webServer.send(200, "text/html", "");
    webServer.sendContent(F("<!DOCTYPE html><html><head><meta charset=UTF-8><title>Log</title>"
                            "<style>body{background:#111;color:#cfc;font-family:monospace;padding:20px}"
                            "pre{white-space:pre-wrap;word-break:break-all}a{color:#fff}</style></head><body>"
                            "<p><a href='/'>[ Inicio ]</a> <a href='/log'>[ Refrescar ]</a></p>"));

    struct rst_info *ri = ESP.getResetInfoPtr();
    String s = F("<h4>Arranque anterior</h4><p><b>Motivo del reinicio:</b> ");
    s += ri ? rstReasonStr(ri->reason) : "?";
    s += F("<br><b>Reinicios an&oacute;malos consecutivos:</b> ");
    s += String(gCrashInfo.resetCount);
    if (gPrevValid)
    {
        s += F("<br><b>&Uacute;ltimo mensaje:</b> <i>");
        appendEsc(s, gPrevCrash.lastMsg, strlen(gPrevCrash.lastMsg));
        s += F("</i>");
    }
    s += F("</p><h4>Log reciente (RAM)</h4><pre>");
    webServer.sendContent(s);

    if (gLogWrapped)
        sendRingPart(gLogRing + gLogHead, LOG_RING_SIZE - gLogHead);
    sendRingPart(gLogRing, gLogHead);

    webServer.sendContent(F("</pre></body></html>"));
    webServer.sendContent("");
}

/* ══════════════════════════════════════════════════════════
   WiFi Test – persistencia en flash
   saveWifiTestCheckpoint() → guarda progreso parcial tras cada paso.
   saveWifiTestResults()    → alias final (done=true).
   ══════════════════════════════════════════════════════════ */
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
    doc["running"] = gWtRunning;
    doc["done"] = gWtDone;
    doc["step"] = gWtStep;

    JsonArray arr = doc["results"].to<JsonArray>();
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
        gWtDone = true;
        LOG_PRINTF_P(PSTR("[WTest] Test anterior abortado en paso %d — pasos pendientes marcados.\n"),
                     savedStep);
        gWtTimestamp += " [abortado]";
        saveWifiTestCheckpoint();
        return true;
    }

    gWtDone = (i == WTEST_TOTAL) && (doc["done"] | false);
    return gWtDone;
}

/* ══════════════════════════════════════════════════════════
   WiFi Test – test de velocidad (bloqueante ~5 s, usa yield)
   ══════════════════════════════════════════════════════════ */
int32_t doWifiSpeedTest()
{
    const char *host = "speedtest.tele2.net";
    const uint16_t port = 80;
    const char *path = "/100KB.zip"; // 102400 bytes
    WiFiClient client;
    client.setTimeout(6000);

    unsigned long tConn = millis();
    if (!client.connect(host, port))
    {
        LOG_PRINTLN(F("[WTest] Speed: no pudo conectar al servidor."));
        return -1;
    }
    LOG_PRINTF_P(PSTR("[WTest] Speed: conectado en %lu ms\n"), millis() - tConn);

    client.print(F("GET "));
    client.print(path);
    client.print(F(" HTTP/1.0\r\nHost: "));
    client.print(host);
    client.print(F("\r\nConnection: close\r\n\r\n"));

    // Línea de estado: debe ser 200
    String status = client.readStringUntil('\n');
    if (status.indexOf(" 200") < 0)
    {
        LOG_PRINTLN(F("[WTest] Speed: respuesta HTTP no válida."));
        client.stop();
        return -1;
    }

    // Saltar cabeceras (una línea vacía = timeout/cierre → fallo)
    unsigned long tHdr = millis();
    bool headersDone = false;
    while (client.connected() && millis() - tHdr < 4000UL)
    {
        String line = client.readStringUntil('\n');
        yield();
        if (line.length() == 0)
            break;
        if (line == "\r")
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

    int32_t bps = (int32_t)((uint64_t)totalBytes * 1000UL / elapsed);
    LOG_PRINTF_P(PSTR("[WTest] Speed: %lu bytes en %lu ms → %ld B/s\n"),
                 (unsigned long)totalBytes, (unsigned long)elapsed, (long)bps);
    return bps;
}

/* ══════════════════════════════════════════════════════════
   WiFi Test – state machine, llamar desde loop()
   ══════════════════════════════════════════════════════════ */

static void wtFinalize(bool abortedByTimeout)
{
    LOG_PRINTF_P(PSTR("[WTest] Finalizando test (abortado=%d). Restaurando WiFi...\n"),
                 (int)abortedByTimeout);

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

    struct tm t;
    if (nowLocal(t))
    {
        char buf[32];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
        gWtTimestamp = String(buf);
    }
    else
    {
        gWtTimestamp = "t+" + String(millis() / 1000) + "s";
    }
    if (abortedByTimeout)
        gWtTimestamp += " [timeout]";

    gWtRunning = false;
    gWtDone = true;
    gWtPhase = WTP_IDLE;

    saveWifiTestResults();
    LOG_PRINTLN(F("[WTest] Test WiFi finalizado y guardado."));
}

void runWifiTestStep()
{
    if (!gWtRunning)
        return;

    if (millis() - gWtStartTime > WTEST_GLOBAL_TIMEOUT_MS)
    {
        LOG_PRINTF_P(PSTR("[WTest] TIMEOUT GLOBAL (%lu ms) — abortando test.\n"),
                     millis() - gWtStartTime);
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
        // ~2 s para que la respuesta HTTP de /testwifi/run llegue al navegador
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
        gWtResults[gWtStep].errCode = WTEST_ERR_OK;

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
        gWtStep++;
        saveWifiTestCheckpoint(); // persiste progreso ANTES de pasar al siguiente

        if (gWtStep >= WTEST_TOTAL)
        {
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
            delay(800);
            yield();
        }
        break;

    case WTP_RESTORE_WAIT:
    {
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
            wtFinalize(false);
        }
        break;
    }

    default:
        break;
    }
}

/* ══════════════════════════════════════════════════════════
   Web – handlers /testwifi
   ══════════════════════════════════════════════════════════ */

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
⚠️ El test interrumpe WiFi/MQTT durante ~6 minutos. No tocar el dispositivo mientras corre.</p>

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
      var np=d.npow||7;
      var mi=Math.floor(d.step/np); var pi=d.step%np;
      document.getElementById('prog-label').textContent=
        'Probando '+(d.step+1)+'/'+d.total+': '+
        (mi<3?modeNames[mi+1]:'?')+' @ '+(pi<powers.length?powers[pi]:'?')+' dBm…';
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
  var bestBps=-1,bestIdx=-1;
  for(var i=0;i<d.results.length;i++){
    var r=d.results[i];
    if(r.ok&&r.bps>bestBps){bestBps=r.bps;bestIdx=i;}
  }
  var html='<h3>Resultados'+(d.ts?' &mdash; <small style="font-weight:normal">'+d.ts+'</small>':'')+'</h3>';
  html+='<table><tr><th>Modo</th><th>Potencia</th><th>Estado</th>';
  html+='<th>T. conexión</th><th>RSSI</th><th>Velocidad</th><th>Error</th></tr>';
  for(var i=0;i<d.results.length;i++){
    var r=d.results[i];
    var cls=i===bestIdx?' class="best"':'';
    var okS=r.ok?'<span class="ok">✓ OK</span>':'<span class="fail">✗ Fallo</span>';
    var ct=r.cMs?r.cMs+' ms':'—';
    var rssi=r.rssi?r.rssi+' dBm':'—';
    var spd=fmtSpeed(r.bps)+(i===bestIdx?' 🏆':'');
    var errS=(r.err&&r.err>0)?'<span class="warn">'+(errLabels[r.err]||'err'+r.err)+'</span>':'';
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

    gWtStep = 0;
    gWtRunning = true;
    gWtDone = false;
    gWtPhase = WTP_START_DELAY;
    gWtPhaseTimer = millis();
    gWtStartTime = millis();
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
    doc["npow"] = WTEST_N_POWERS;
    doc["ts"] = gWtTimestamp;

    JsonArray arr = doc["results"].to<JsonArray>();
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

    /* Cargar la copia RTC del arranque anterior ANTES de cualquier logLine().
     * gPrevCrash es una copia aparte: los logs nuevos no la sobrescriben. */
    loadRTCCrash();

    LOG_PRINTLN(F("\n=== Extractor ESP8266 ==="));
    LOG_PRINTLN(F("Build: " __DATE__ " " __TIME__));

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

    /* ── Estado del reset actual ── */
    {
        struct rst_info *ri = ESP.getResetInfoPtr();
        uint32_t reason = ri ? ri->reason : 99;
        bool isCrash = (reason == 1 || reason == 2 || reason == 3);

        gCrashInfo.magic = RTC_MAGIC;
        gCrashInfo.reason = reason;
        gCrashInfo.exccause = (isCrash && ri) ? ri->exccause : 0;
        gCrashInfo.resetCount = isCrash ? (gPrevValid ? gPrevCrash.resetCount + 1 : 1) : 0;
        gCrashInfo.crc = rtcCrc(gCrashInfo);
        ESP.rtcUserMemoryWrite(0, (uint32_t *)&gCrashInfo, sizeof(gCrashInfo));

        if (isCrash)
            LOG_PRINTF_P(PSTR("*** CRASH #%u detectado! Motivo: %s (exccause=%u epc1=0x%08x)\n"),
                (unsigned)gCrashInfo.resetCount, rstReasonStr(reason),
                (unsigned)gCrashInfo.exccause, ri ? (unsigned)ri->epc1 : 0u);
    }

    pinMode(PIN_SOIL_VCC, OUTPUT);
    digitalWrite(PIN_SOIL_VCC, LOW);
    analogWriteFreq(500);

    /* ── Relay: forzar OFF antes de cualquier otra init (fail-safe) ── */
    pinMode(PIN_RELAY, OUTPUT);
    digitalWrite(PIN_RELAY, RELAY_OFF);
    gExtractor = false;
    tLastSwitch = millis() - RELAY_MIN_OFF_MS; // permitir encender desde el arranque

    /* ── I2C ── */
    Wire.begin(D2, D1);    // SDA=D2(GPIO4), SCL=D1(GPIO5)
    Wire.setClock(100000); // 100 kHz — el ENS160 es sensible a velocidades altas

    loadConfig(); // define también modeAP
    loadWifiTestResults();

    /* ── WiFi ── */
    WiFi.persistent(false); // no desgastar flash en cada reconexión
    WiFi.hostname("extractor-esp");
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.setOutputPower(gWtOrigPower);
    WiFi.setPhyMode(WIFI_PHY_MODE_11B); // 802.11b para mejor cobertura
    WiFi.setAutoReconnect(true);

    if (!modeAP)
    {
        LOG_PRINTLN(F("Conectando a WiFi guardado..."));
        WiFi.mode(WIFI_STA);
        WiFi.begin();
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
    }
    if (modeAP || WiFi.status() != WL_CONNECTED)
    {
        if (modeAP)
            WiFi.setAutoReconnect(false);

        LOG_PRINTLN(F("Sin WiFi → arrancando en modo AP."));
        startApMode();
        // En AP permanente: primer intento de NTP+MQTT a los ~20 s del arranque
        if (modeAP)
            tApReconnect = millis() - INTERVAL_AP_RECONNECT_MQTT + 20000UL;
    }

    /* ── Hora: SNTP en segundo plano; timeOK lo fija onTimeSet() ── */
    settimeofday_cb(onTimeSet);
    configTzTime(MY_TZ, MY_NTP_SERVER);
    LOG_PRINTLN(F("NTP configurado: la hora se fijará cuando haya WiFi."));

    // Telnet y WebServer
    TelnetStream.begin();
    telnetReady = true;

    /* ── MQTT sobre TLS (puerto 8883) – sin verificar certificado ── */
    wifiClient.setInsecure();
    mqtt.setServer(cfgHost, atoi(cfgPort));
    mqtt.setCallback(mqttCallback);
    mqtt.setKeepAlive(60);
    mqtt.setBufferSize(256);

    /* ── Rutas del servidor web ── */
    webServer.on("/", handleRoot);
    webServer.on("/api", handleApi);
    webServer.on("/set", handleSet);
    webServer.on("/log", handleLog);
    webServer.on("/wifi", handleWifiPortal);
    webServer.on("/wifi/sta", handleWifiSta);
    webServer.on("/api/crash", handleCrashApi);
    webServer.on("/testwifi", handleTestWifi);
    webServer.on("/testwifi/run", handleTestWifiRun);
    webServer.on("/testwifi/status", handleTestWifiStatus);
    webServer.begin();
    LOG_PRINTLN(F("WebServer iniciado en puerto 80."));

    /* ── OTA ── */
    setupOTA();

    /* ── ENS160 ── */
    ens160.begin();
    if (ens160.available())
    {
        gENSok = true;
        bool modeOk = ens160.setMode(ENS160_OPMODE_STD);
        LOG_PRINTF_P(PSTR("ENS160 OK rev %u.%u.%u, modo estándar: %s\n"),
                     (unsigned)ens160.getMajorRev(), (unsigned)ens160.getMinorRev(),
                     (unsigned)ens160.getBuild(), modeOk ? "OK" : "FALLO");
    }
    else
    {
        LOG_PRINTLN(F("Error: ENS160 no responde. Iniciando sin sensor."));
    }

    /* ── AHT21 ── */
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
    tSoil = millis() - INTERVAL_SOIL; // primera lectura de suelo al arrancar
}

/* ══════════════════════════════════════════════════════════
   Loop
   ══════════════════════════════════════════════════════════ */
void loop()
{
    /* ── Portal WiFiManager solicitado desde la web ── */
    if (gPortalRequested)
    {
        gPortalRequested = false;
        LOG_PRINTLN(F("Iniciando portal WiFiManager solicitado por web..."));
        if (gCfgDirty)
        {
            saveConfig();
            gCfgDirty = false;
        }
        webServer.stop();
        dnsServer.stop(); // WiFiManager lanzará el suyo
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

    /* ── Gestión de conectividad WiFi ── */
    if (gApMode)
    {
        dnsServer.processNextRequest();

        // AP permanente: conectar cada hora (NTP+MQTT). AP por fallo de WiFi: cada 3 min.
        unsigned long tApInterval = modeAP ? INTERVAL_AP_RECONNECT_MQTT : INTERVAL_AP_RECONNECT;
        if (millis() - tApReconnect >= tApInterval)
        {
            tApReconnect = millis();
            LOG_PRINTLN(F("AP: probando reconexión WiFi..."));

            // WIFI_AP_STA: mantenemos el AP activo mientras intentamos STA
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAP(AP_SSID, SECRET_AP_PASS);
            WiFi.begin();

            unsigned long t = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t < 10000UL)
            {
                dnsServer.processNextRequest();
                delay(200);
                yield();
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                LOG_PRINT(F("WiFi recuperado: "));
                LOG_PRINTLN(WiFi.localIP().toString().c_str());

                if (!modeAP)
                {
                    LOG_PRINTLN(F("Reiniciando en modo normal..."));
                    delay(500);
                    ESP.restart();
                }
                else
                {
                    // 1) Hora: forzar petición SNTP inmediata y esperar hasta 10 s
                    uint32_t before = gSyncCount;
                    configTzTime(MY_TZ, MY_NTP_SERVER);
                    unsigned long tn = millis();
                    while (gSyncCount == before && millis() - tn < 10000UL)
                    {
                        delay(100);
                        yield();
                    }
                    LOG_PRINTLN(gSyncCount != before ? F("NTP: hora sincronizada.") : F("NTP: sin respuesta."));

                    // 2) MQTT: conectar (con reintentos), publicar y atender comandos ~5 s
                    unsigned long tm = millis(), tOk = 0;
                    while (millis() - tm < 15000UL)
                    {
                        if (!mqtt.connected())
                        {
                            if (mqttReconnect())
                                tOk = millis();
                        }
                        else
                        {
                            mqtt.loop();
                        }
                        if (tOk && millis() - tOk > 5000UL)
                            break;
                        delay(20);
                    }
                    mqtt.disconnect();
                    delay(500);

                    // 3) Volver a AP puro
                    webServer.stop();
                    startApMode();
                    webServer.begin();
                }
            }
            else
            {
                LOG_PRINTLN(F("Sin WiFi — continuando en modo AP."));
                WiFi.mode(WIFI_AP);
                WiFi.softAP(AP_SSID, SECRET_AP_PASS);
                delay(100);
                dnsServer.start(53, "*", WiFi.softAPIP());
            }
        }
    }
    else
    {
        /* ── Modo normal (STA): vigilar pérdida de WiFi ── */
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
                LOG_PRINTLN(F("WiFi perdido > 3 min → modo AP."));
                wifiWasLost = false;
                mqtt.disconnect();
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
            tWifiLost = millis();
            wifiWasLost = false;
        }
    }

    /* ── Servicios comunes ── */
    ArduinoOTA.handle();
    MDNS.update();
    webServer.handleClient();

    /* ── WiFi Test: avanzar un paso de la state machine ── */
    runWifiTestStep();

    /* ── MQTT solo en modo normal ── */
    if (!gApMode)
    {
        mqttReconnect();
        mqtt.loop();
    }

    /* ── Sensores, lógica y publicación periódica ── */
    soilTick();
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

    /* ── Guardado diferido de configuración ── */
    if (gCfgDirty && millis() - tCfgDirty >= CFG_SAVE_DELAY)
    {
        gCfgDirty = false;
        saveConfig();
    }
}