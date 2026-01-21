#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
// ---------- SERIAL COMMAND CONSOLE ----------
String serialLine = "";
static const uint16_t SERIAL_LINE_MAX = 96;

struct PiezoStats;  // forward declaration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- WIFI ----------------
const char* ssid = "G3 QuickShifter Basic";
String wifiPassword = "12345678";  // caricata da EEPROM se presente

DNSServer dnsServer;
WebServer server(80);

// ---------------- VERSION ----------------
static const char* FW_VERSION = "v1.8.1";

// ---------------- PIN ----------------
const int btnActivate = 3;  // ON quando premuto, OFF quando rilasciato
const int btnDec = 4;
const int btnInc = 5;
const int btnShift = 7;  // opzionale debug, non usato come trigger principale
const int ledPin = 6;
const int mosfetPin = 8;
// Puntalino / bottone cambio (DIGITALE)
const int btnPuntalinoPin = 2;  


// Piezo / sensore cambiata
const int piezoPin = A2;  // sensore su A2

// Battery pin
const int battPin = A3;
static float envSlope = 0.0f;
// ---------------- LIMITI CUT ----------------
const int CUT_MIN = 5;
const int CUT_MAX = 150;

// ---------------- SHIFT COOLDOWN LIMITS ----------------
const uint16_t COOLDOWN_MIN_MS = 0;
const uint16_t COOLDOWN_MAX_MS = 3000;

// ---------------- STATE ----------------
int cutTime = 30;
bool isActivated = false;

// MOSFET shift cut
bool shiftActive = false;
unsigned long shiftStartTime = 0;

// anti double-cut cooldown
uint16_t shiftCooldownMs = 500;
unsigned long lastShiftTriggerMs = 0;

// blocked shifts
uint32_t blockedShifts = 0;
uint32_t blockedAntiTheftShifts = 0;


enum ShiftSensorType : uint8_t { SENSOR_PIEZO = 0, SENSOR_PUNTALINO = 1 };
ShiftSensorType shiftSensorType = SENSOR_PIEZO;
bool sensorTypeUnset = false; // per mostrare overlay prima volta

// LCD management
enum LCDDisplay { LCD_STATUS,
                  CUT,
                  MODE,
                  BATTERY };
LCDDisplay lcdMode = BATTERY;
unsigned long lcdTimer = 0;

// SHIFT message LCD
bool showShiftMessage = false;
unsigned long shiftMessageStart = 0;
const unsigned long shiftMessageDuration = 300;

// ---------------- MODES ----------------
enum Mode : uint8_t { MODE_RACE = 0,
                      MODE_CUSTOM = 1,
                      MODE_STREET = 2 };
Mode currentMode = MODE_RACE;

String modeToStr(Mode m) {
  switch (m) {
    case MODE_RACE: return "RACE";
    case MODE_CUSTOM: return "CUSTOM";
    case MODE_STREET: return "STREET";
    default: return "RACE";
  }
}
Mode strToMode(const String& s) {
  if (s == "RACE") return MODE_RACE;
  if (s == "CUSTOM") return MODE_CUSTOM;
  if (s == "STREET") return MODE_STREET;
  return MODE_RACE;
}

// ---------------- SCOPPI / BANG ----------------
bool scoppiEnabled = false;

// Bang parameters
uint8_t bangPulses = 3;         // 1..8
uint16_t bangCutMs = 18;        // duration CUT segment
uint16_t bangPassMs = 10;       // duration PASS segment
uint16_t bangMaxTotalMs = 120;  // safety cap: total window = min(cutTime, bangMaxTotalMs)

// runtime state
bool bangActive = false;
bool bangIsCut = true;
unsigned long bangStageStartMs = 0;
uint32_t bangPulseCount = 0;

// --- LIVE AI CONTROL warning + LED blink globals ---
uint16_t liveAiSuggestedTh = 0;

bool aiBlinkActive = false;
bool aiBlinkState = false;

uint32_t aiPiezoSamples = 0;
float aiPiezoEnvSum = 0.0f;

// --- AUTO MODE gating ---
uint32_t liveAiLastNoisyEventMs = 0;
static const uint16_t LIVEAI_EVENT_HOLDOFF_MS = 60;   // conta max ~16 eventi/s
static const float    LIVEAI_NOISE_FRAC = 0.85f;      // più severo di 0.75
static const uint16_t AUTOMODE_MIN_SHIFTS = 12;             // prima di decidere: almeno 12 cambiate accettate
static const uint32_t AUTOMODE_MIN_TIME_MS = 30000;         // e almeno 30s da quando abiliti
static const uint32_t AUTOMODE_DECISION_PERIOD_MS = 20000;  // non più spesso di 20s
static const uint32_t AUTOMODE_IDLE_FREEZE_MS = 15000;      // se non cambi da 15s => NON cambiare nulla

// tracking
uint32_t autoModeEnabledSinceMs = 0;
uint32_t autoModeShiftSamples = 0;
uint32_t autoModeLastDecisionMs = 0;

// evita spam/applicazioni ripetute
Mode autoModeLastAppliedMode = MODE_RACE;
int autoModeLastAppliedCut = -1; 
// ================= ADVANCED CALIBRATING (runtime) =================
uint16_t advMinAboveMs = 10;           // MIN_ABOVE_MS
float    advMinSlope   = 4.0f;         // MIN_SLOPE
float    advEnvAlpha   = 0.12f;        // PIEZO_ENV_ALPHA
float    advBaseAlpha  = 0.0025f;      // PIEZO_BASE_ALPHA

uint16_t advHoldoffMs      = 140;      // PIEZO_HOLDOFF_MS
float    advNoisyLevel     = 1.25f;    // PIEZO_NOISY_LEVEL
uint16_t advNoisyHoldoffMs = 260;      // PIEZO_HOLDOFF_NOISY_MS
// ================================================================


// --- AUTO MODE (AI) globals ---
bool autoModeEnabled = false;
String autoModeLastDecision = "";

// ---------------- ANTI THEFT ----------------
bool antiTheftEnabled = false;

// ---------------- LCD ENABLE ----------------
bool lcdEnabled = true;

// ---------------- BATTERY ----------------
float batteryScale = 4.85f;
float batteryFiltered = 0.0f;
const float UNDER_VOLTAGE_TH = 11.0f;
bool underVoltageLatched = false;

// Undervoltage debounce: evita spam errori
unsigned long uvLastChangeMs = 0;
const unsigned long UV_DEBOUNCE_MS = 1500;

// ---------------- TELEMETRY ----------------
uint32_t shiftCount = 0;
uint32_t errorCount = 0;
// ================= AI LIVE CONTROL (CORE) =================

// shift count window
unsigned long aiWindowStartMs = 0;
uint8_t aiShiftCounter = 0;

// evaluation thresholds
static const unsigned long AI_WINDOW_MS = 6000;
static const uint8_t AI_MAX_SHIFTS = 7;

// state
bool aiWarningActive = false;
String aiWarningReason = "";
uint16_t aiSuggestedPiezo = 0;

// ================= END AI LIVE CONTROL =================


// ---------------- ACTIVE TIME ----------------
uint32_t activeSeconds = 0;
unsigned long lastActiveTickMs = 0;
unsigned long lastEepromCommitMs = 0;

// ---------------- PIEZO SENSITIVITY ----------------
// più basso = più sensibile (trigger più facile) — soglia sul valore "ENV" filtrato
uint16_t piezoTh = 300;  // default. Cambialo da web

// --- PIEZO robust processing ---
static const uint16_t PIEZO_ADC_MAX = 4095;
uint16_t piezoRaw = 0;   // ultima lettura grezza
float piezoBase = 0.0f;  // baseline (drift lento)
float piezoEnv = 0.0f;   // envelope (energia vibrazione)
unsigned long lastPiezoProcMs = 0;

// Anti-rimbalzo trigger
unsigned long lastPiezoTrigMs = 0;

// HOLD per evitare “raffiche”
const unsigned long PIEZO_HOLDOFF_MS = 140;  // tempo minimo tra trigger
// Limite baseline: non inseguire durante un colpo
const float PIEZO_BASE_ALPHA = 0.0025f;  // lento
// Envelope smoothing (più alto = più reattivo)
const float PIEZO_ENV_ALPHA = 0.12f;  // medio

// ---------------- PIEZO HISTORY (for graph) ----------------
static const int PH_CAP = 300;  // 300 punti
float ph[PH_CAP];
uint16_t phHead = 0;
uint16_t phCount = 0;
unsigned long lastPhSampleMs = 0;
const unsigned long PH_PERIOD_MS = 200;  // 5 Hz grafico
const unsigned long PIEZO_HOLDOFF_NOISY_MS = 260; // 220..320
const float PIEZO_NOISY_LEVEL = 1.25f;            // 1.15..1.35



// ---------------- LED LOGIC ----------------
// ---------------- LIVE AI WARNING LED (5 blink ogni 2s) ----------------
static const uint8_t  LIVEAI_LED_BLINKS     = 5;     // 5 lampeggi
static const uint16_t LIVEAI_LED_TOGGLE_MS  = 140;   // velocità ON/OFF del blink
static const uint16_t LIVEAI_LED_PERIOD_MS  = 2000;  // ogni 2 secondi riparte il ciclo

uint32_t aiLedCycleStartMs  = 0;
uint8_t  aiLedBlinkCount    = 0;   // conteggio blink completati (OFF raggiunto)
uint32_t aiLedLastToggleMs  = 0;
bool     aiLedState         = false; // stato attuale LED durante warning

unsigned long ledOnTransitionUntilMs = 0;  // 2s blink when QS becomes ON
unsigned long ledBlinkLastToggleMs = 0;
bool ledBlinkState = false;
bool rebootPending = false;
uint32_t rebootAtMs = 0;


// MODIFICA: ora il “lampeggio evento” inverte lo stato base (visibile anche se LED già ON)
unsigned long ledInvertUntilMs = 0;
const unsigned long LED_PULSE_MS = 90;

unsigned long antiTheftCycleStartMs = 0;  // 10s cycle
const unsigned long ANTITHEFT_PERIOD_MS = 10000;
const unsigned long ANTITHEFT_BLINK_WINDOW_MS = 3000;
const unsigned long LED_BLINK_PERIOD_MS = 180;

// ===== AUTO MODE LED BLINK (non-blocking) =====
bool autoModeBlinkActive = false;
bool autoModeBlinkState = false;
uint8_t autoModeBlinkCount = 0;
uint32_t autoModeBlinkLastMs = 0;

static const uint8_t AUTOMODE_LED_BLINKS = 3;
static const uint16_t AUTOMODE_LED_TOGGLE_MS = 90;

// ---------------- EEPROM MAP ----------------
const int EEPROM_SIZE = 512;

// addresses
const int EEPROM_ADDR_SENSOR_TYPE_U8 = 128;
const int EEPROM_ADDR_CUT_U16 = 0;    // uint16
const int EEPROM_ADDR_MODE_U8 = 2;    // uint8
const int EEPROM_ADDR_ANTIFT_U8 = 3;  // uint8
const int EEPROM_ADDR_LCD_U8 = 4;     // uint8

const int EEPROM_ADDR_ERR_U32 = 8;        // uint32
const int EEPROM_ADDR_ACTS_U32 = 12;      // uint32
const int EEPROM_ADDR_SCALE_F32 = 16;     // float
const int EEPROM_ADDR_COOLDOWN_U16 = 20;  // uint16
const int EEPROM_ADDR_BLOCKED_U32 = 24;   // uint32

// SCOPPI / BANG saved
const int EEPROM_ADDR_SCOPPI_U8 = 28;      // uint8
const int EEPROM_ADDR_BANG_PULSES = 29;    // uint8
const int EEPROM_ADDR_BANG_CUT_U16 = 30;   // uint16
const int EEPROM_ADDR_BANG_PASS_U16 = 32;  // uint16
const int EEPROM_ADDR_BANG_MAX_U16 = 34;   // uint16
const int EEPROM_ADDR_LIVEAI_U8 = 39;

// ---- Advanced calibrating EEPROM ----
const int EEPROM_ADDR_ADV_MAGIC_U32 = 124;  // 4 bytes (usa vicino al tuo magic)
const uint32_t ADV_MAGIC = 0x41445631UL;    // "ADV1"

// Metti il blocco dopo, es. da 140 in poi (allineato)
const int EEPROM_ADDR_ADV_BLOCK = 140;      // inizio struct

// --- LIVE AI CONTROL (global) ---
bool liveAiEnabled = true;       // default ON
bool liveAiWarning = false;



// PIEZO threshold
const int EEPROM_ADDR_PIEZO_TH_U16 = 36;  // uint16
// --- AUTO MODE AI (EEPROM) ---
const int EEPROM_ADDR_AUTOMODE_U8 = 38;  // 0/1 (assicurati non confligga con altri)


// WiFi password storage
const int EEPROM_ADDR_WIFI_PW_LEN = 40;  // uint8
const int EEPROM_ADDR_WIFI_PW = 41;      // 31 bytes
const int WIFI_PW_MAX_LEN = 31;

const int EEPROM_ADDR_MAGIC_U32 = 120;       // uint32
const uint32_t EEPROM_MAGIC = 0x47535153UL;  // "GSQS"

// ---------------- EEPROM helpers ----------------
static void eepromWriteU32(int addr, uint32_t v) {
  EEPROM.write(addr + 0, (uint8_t)(v & 0xFF));
  EEPROM.write(addr + 1, (uint8_t)((v >> 8) & 0xFF));
  EEPROM.write(addr + 2, (uint8_t)((v >> 16) & 0xFF));
  EEPROM.write(addr + 3, (uint8_t)((v >> 24) & 0xFF));
}
static uint32_t eepromReadU32(int addr) {
  uint32_t v = 0;
  v |= (uint32_t)EEPROM.read(addr + 0);
  v |= ((uint32_t)EEPROM.read(addr + 1)) << 8;
  v |= ((uint32_t)EEPROM.read(addr + 2)) << 16;
  v |= ((uint32_t)EEPROM.read(addr + 3)) << 24;
  return v;
}
static void eepromWriteU16(int addr, uint16_t v) {
  EEPROM.write(addr + 0, (uint8_t)(v & 0xFF));
  EEPROM.write(addr + 1, (uint8_t)((v >> 8) & 0xFF));
}
static uint16_t eepromReadU16(int addr) {
  uint16_t v = 0;
  v |= (uint16_t)EEPROM.read(addr + 0);
  v |= ((uint16_t)EEPROM.read(addr + 1)) << 8;
  return v;
}
static void eepromWriteFloat(int addr, float val) {
  byte* p = (byte*)(void*)&val;
  for (int i = 0; i < 4; i++) EEPROM.write(addr + i, p[i]);
}
static float eepromReadFloat(int addr) {
  float val;
  byte* p = (byte*)(void*)&val;
  for (int i = 0; i < 4; i++) p[i] = EEPROM.read(addr + i);
  return val;
}

bool isValidWifiPassword(const String& p) {
  if (p.length() < 8 || p.length() > WIFI_PW_MAX_LEN) return false;
  for (size_t i = 0; i < p.length(); i++) {
    char c = p[i];
    if ((uint8_t)c < 32 || (uint8_t)c > 126) return false;
  }
  return true;
}
void saveWifiPasswordToEeprom(const String& p) {
  uint8_t len = (uint8_t)min((int)p.length(), WIFI_PW_MAX_LEN);
  EEPROM.write(EEPROM_ADDR_WIFI_PW_LEN, len);
  for (int i = 0; i < WIFI_PW_MAX_LEN; i++) {
    EEPROM.write(EEPROM_ADDR_WIFI_PW + i, (i < len) ? (uint8_t)p[i] : 0);
  }
}
String loadWifiPasswordFromEeprom() {
  int len = EEPROM.read(EEPROM_ADDR_WIFI_PW_LEN);
  if (len < 8 || len > WIFI_PW_MAX_LEN) return "";
  char buf[WIFI_PW_MAX_LEN + 1];
  for (int i = 0; i < len; i++) buf[i] = (char)EEPROM.read(EEPROM_ADDR_WIFI_PW + i);
  buf[len] = '\0';
  String p(buf);
  if (!isValidWifiPassword(p)) return "";
  return p;
}

// ---------------- LOG BUFFER ----------------
static const int LOG_CAP = 160;
String logs[LOG_CAP];
int logHead = 0;
int logCount = 0;

String ts() {
  unsigned long s = millis() / 1000UL;
  unsigned long m = s / 60UL;
  s %= 60UL;
  unsigned long h = m / 60UL;
  m %= 60UL;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  return String(buf);
}
void addLog(const String& level, const String& msg) {
  logs[logHead] = "[" + ts() + "] [" + level + "] " + msg;
  logHead = (logHead + 1) % LOG_CAP;
  if (logCount < LOG_CAP) logCount++;
}

// ---------------- VOLTAGE HISTORY ----------------
static const int VH_CAP = 240;  // ~8 min @ 2s
float vh[VH_CAP];
uint16_t vhHead = 0;
uint16_t vhCount = 0;
unsigned long lastVhSampleMs = 0;
const unsigned long VH_PERIOD_MS = 2000;

// ---------------- LCD helpers ----------------
void lcdPrintClean(int col, int row, String text) {
  if (!lcdEnabled) return;
  lcd.setCursor(col, row);
  lcd.print("                ");
  lcd.setCursor(col, row);
  lcd.print(text);
}
void lcdApplyEnabled() {
  if (lcdEnabled) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
    lcd.clear();
  }
}

static void serialPrintHelp() {
  Serial.println();
  Serial.println("=== G3D QuickShifter Serial Commands ===");
  Serial.println("HELP                          -> show this help");
  Serial.println("SHOW                          -> show current settings summary");
  Serial.println("FACTORYRESET                  -> wipe EEPROM (0xFF) + reboot");
  Serial.println("SENSORRESET                   -> unset sensor type (0xFF) + reboot");
  Serial.println("WIFIPW <8..31 ASCII chars>     -> set WiFi AP password + reboot");
  Serial.println("========================================");
  Serial.println();
}

static void serialShowStatus() {
  Serial.println();
  Serial.println("=== STATUS ===");
  Serial.print("FW: "); Serial.println(FW_VERSION);

  Serial.print("WiFi SSID: "); Serial.println(ssid);
  Serial.print("WiFi PW len: "); Serial.println(wifiPassword.length());

  Serial.print("SensorTypeUnset: "); Serial.println(sensorTypeUnset ? "true" : "false");
  Serial.print("ShiftSensorType: ");
  Serial.println((shiftSensorType == SENSOR_PUNTALINO) ? "PUNTALINO" : "PIEZO");

  Serial.print("cutTime: "); Serial.println(cutTime);
  Serial.print("mode: "); Serial.println(modeToStr(currentMode));
  Serial.print("antiTheft: "); Serial.println(antiTheftEnabled ? "ON" : "OFF");
  Serial.println("=============");
  Serial.println();
}

static bool serialIsAsciiPrintable(const String& s) {
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if ((uint8_t)c < 32 || (uint8_t)c > 126) return false;
  }
  return true;
}

static void serialDoFactoryReset() {
  Serial.println("[SERIAL] FACTORYRESET: wiping EEPROM and rebooting...");
  addLog("WARN", "SERIAL FACTORYRESET");

  for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0xFF);
  EEPROM.commit();

  rebootPending = true;
  rebootAtMs = millis() + 900;
}

static void serialDoSensorReset() {
  Serial.println("[SERIAL] SENSORRESET: unsetting sensor type and rebooting...");
  addLog("WARN", "SERIAL SENSORRESET");

  sensorTypeUnset = true;
  EEPROM.write(EEPROM_ADDR_SENSOR_TYPE_U8, 0xFF);
  EEPROM.commit();

  rebootPending = true;
  rebootAtMs = millis() + 900;
}

static void serialDoWifiPw(const String& pw) {
  if (pw.length() < 8 || pw.length() > WIFI_PW_MAX_LEN || !serialIsAsciiPrintable(pw)) {
    Serial.println("[SERIAL] WIFIPW: invalid password. Must be 8..31 printable ASCII chars.");
    return;
  }

  Serial.print("[SERIAL] WIFIPW: saving new password (len=");
  Serial.print(pw.length());
  Serial.println(") and rebooting...");
  addLog("WARN", "SERIAL WIFIPW changed");

  wifiPassword = pw;
  saveWifiPasswordToEeprom(pw);
  EEPROM.commit();

  rebootPending = true;
  rebootAtMs = millis() + 900;
}

static void serialHandleCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // uppercase command token, keep original for args
  String cmd = line;
  int sp = cmd.indexOf(' ');
  if (sp >= 0) cmd = cmd.substring(0, sp);
  cmd.toUpperCase();

  if (cmd == "HELP" || cmd == "?") {
    serialPrintHelp();
    return;
  }

  if (cmd == "SHOW") {
    serialShowStatus();
    return;
  }

  if (cmd == "FACTORYRESET") {
    serialDoFactoryReset();
    return;
  }

  if (cmd == "SENSORRESET") {
    serialDoSensorReset();
    return;
  }

  if (cmd == "WIFIPW") {
    String pw = "";
    if (sp >= 0) pw = line.substring(sp + 1);
    pw.trim();
    serialDoWifiPw(pw);
    return;
  }

  Serial.print("[SERIAL] Unknown command: ");
  Serial.println(line);
  Serial.println("Type HELP");
}


// ---------------- Battery ----------------
float readBatteryRaw() {
  int raw = analogRead(battPin);
  float voltageADC = (raw / 4095.0f) * 3.3f;
  return voltageADC * batteryScale;
}
void updateBattery() {
  float v = readBatteryRaw();
  if (batteryFiltered < 0.1f) batteryFiltered = v;
  batteryFiltered = batteryFiltered * 0.90f + v * 0.10f;

  bool uvNow = (batteryFiltered > 0.1f && batteryFiltered < UNDER_VOLTAGE_TH);
  unsigned long now = millis();

  if (uvNow != underVoltageLatched) {
    if (uvLastChangeMs == 0) uvLastChangeMs = now;
    if (now - uvLastChangeMs >= UV_DEBOUNCE_MS) {
      underVoltageLatched = uvNow;
      uvLastChangeMs = 0;

      if (underVoltageLatched) {
        errorCount++;
        addLog("ERROR", "UNDER_VOLTAGE " + String(batteryFiltered, 2) + "V");
      } else {
        addLog("INFO", "UNDER_VOLTAGE cleared");
      }
    }
  } else {
    uvLastChangeMs = 0;
  }

  // history
  if (now - lastVhSampleMs >= VH_PERIOD_MS) {
    lastVhSampleMs = now;
    vh[vhHead] = batteryFiltered;
    vhHead = (vhHead + 1) % VH_CAP;
    if (vhCount < VH_CAP) vhCount++;
  }
}
void startAutoModeBlink() {
  autoModeBlinkActive = true;
  autoModeBlinkState = false;
  autoModeBlinkCount = 0;
  autoModeBlinkLastMs = millis();
}
void stopAutoModeBlink() {
  autoModeBlinkActive = false;
  autoModeBlinkState  = false;
  autoModeBlinkCount  = 0;
  autoModeBlinkLastMs = 0;
}

struct AdvSettings {
  uint16_t minAboveMs;
  float    minSlope;
  float    envAlpha;
  float    baseAlpha;
  uint16_t holdoffMs;
  float    noisyLevel;
  uint16_t noisyHoldoffMs;
};

static void advLoadFromEeprom() {
  uint32_t m = eepromReadU32(EEPROM_ADDR_ADV_MAGIC_U32);
  if (m != ADV_MAGIC) return; // non presente, tieni i default

  AdvSettings s;
  EEPROM.get(EEPROM_ADDR_ADV_BLOCK, s);

  // validazione minima + clamp
  if (s.minAboveMs < 2 || s.minAboveMs > 50) return;
  if (s.minSlope < 0.0f || s.minSlope > 50.0f) return;
  if (s.envAlpha < 0.01f || s.envAlpha > 0.50f) return;
  if (s.baseAlpha < 0.0001f || s.baseAlpha > 0.02f) return;
  if (s.holdoffMs < 20 || s.holdoffMs > 800) return;
  if (s.noisyLevel < 1.00f || s.noisyLevel > 2.00f) return;
  if (s.noisyHoldoffMs < 60 || s.noisyHoldoffMs > 1200) return;

  // apply
  advMinAboveMs      = s.minAboveMs;
  advMinSlope        = s.minSlope;
  advEnvAlpha        = s.envAlpha;
  advBaseAlpha       = s.baseAlpha;
  advHoldoffMs       = s.holdoffMs;
  advNoisyLevel      = s.noisyLevel;
  advNoisyHoldoffMs  = s.noisyHoldoffMs;
}

static void advSaveToEeprom() {
  AdvSettings s;
  s.minAboveMs      = advMinAboveMs;
  s.minSlope        = advMinSlope;
  s.envAlpha        = advEnvAlpha;
  s.baseAlpha       = advBaseAlpha;
  s.holdoffMs       = advHoldoffMs;
  s.noisyLevel      = advNoisyLevel;
  s.noisyHoldoffMs  = advNoisyHoldoffMs;

  EEPROM.put(EEPROM_ADDR_ADV_BLOCK, s);
  eepromWriteU32(EEPROM_ADDR_ADV_MAGIC_U32, ADV_MAGIC);
  EEPROM.commit();
}

// ---------------- Piezo processing ----------------
void updatePiezoProcessing(unsigned long now) {
  // process a ~1kHz/500Hz-ish loop without blocking: every 2ms..5ms
  if (now - lastPiezoProcMs < 3) return;
  lastPiezoProcMs = now;

  uint16_t r = (uint16_t)analogRead(piezoPin);
  piezoRaw = r;

  // baseline (drift lento): solo se envelope non è in "colpo"
  float centered = (float)r - piezoBase;

  // envelope: |centered| filtrato
  float absx = fabs(centered);
  piezoEnv = piezoEnv + advEnvAlpha * (absx - piezoEnv);
  // ---------- SLOPE DETECTION (anti vibrazione) ----------
  static float lastEnv = 0.0f;

  float d = piezoEnv - lastEnv;
  lastEnv = piezoEnv;

  // slope filtrata (più stabile)
  envSlope = envSlope * 0.75f + d * 0.25f;


  // aggiorna baseline SOLO se non stai “vibrando forte”
  if (piezoEnv < (float)piezoTh * 0.35f) {
    piezoBase = piezoBase + advBaseAlpha * ((float)r - piezoBase);

  }

  // history per grafico
  if (now - lastPhSampleMs >= PH_PERIOD_MS) {
    lastPhSampleMs = now;
    ph[phHead] = piezoEnv;  // grafichiamo envelope stabile
    phHead = (phHead + 1) % PH_CAP;
    if (phCount < PH_CAP) phCount++;
  }
  // ================= AI LIVE CONTROL (PIEZO HOOK) =================


  // ================= END AI LIVE CONTROL =================
  aiPiezoEnvSum += piezoEnv;
  aiPiezoSamples++;
}


bool piezoDetect(unsigned long now) {
  static bool wasAbove = false;
  static unsigned long aboveSince = 0;

  // holdoff base tra trigger
  if (now - lastPiezoTrigMs < advHoldoffMs) return false;

  const bool above = (piezoEnv >= (float)piezoTh);

  if (above) {
    if (!wasAbove) {
      // fronte di salita: parte evento
      aboveSince = now;
    }

    // deve restare sopra soglia per un minimo (ma piccolo => reattivo)
    if ((now - aboveSince) < advMinAboveMs) {
      wasAbove = above;
      return false;
    }

    // slope solo come “guard rail” (non aggressivo)
    if (envSlope < advMinSlope) {
      wasAbove = above;
      return false;
    }

    lastPiezoTrigMs = now;
    wasAbove = above;
    // se sei in rumore forte (tipico alti giri), allunga il refractory
    if (piezoEnv > (float)piezoTh * advNoisyLevel) {
      lastPiezoTrigMs = now - advHoldoffMs + advNoisyHoldoffMs;
    }

    return true;
  } else {
    aboveSince = 0;
  }

  wasAbove = above;
  return false;
}

bool buttonDetect(unsigned long now) {
  static bool lastRead = false;
  static bool stableState = false;
  static unsigned long lastChangeMs = 0;
  static unsigned long lastTrigMs = 0;

  const unsigned long DEBOUNCE_MS = 10;
  const unsigned long BUTTON_HOLDOFF_MS = 140; // simile a piezo holdoff

  bool readNow = (digitalRead(btnPuntalinoPin) == LOW); // premuto = LOW (pullup)

  if (readNow != lastRead) {
    lastRead = readNow;
    lastChangeMs = now;
  }

  if ((now - lastChangeMs) >= DEBOUNCE_MS) {
    if (stableState != readNow) {
      stableState = readNow;

      // trigger sul fronte di pressione
      if (stableState) {
        if (now - lastTrigMs >= BUTTON_HOLDOFF_MS) {
          lastTrigMs = now;
          return true;
        }
      }
    }
  }
  return false;
}



// ---------------- Active time ----------------
void updateActiveTime() {
  unsigned long now = millis();

  if (isActivated) {
    if (lastActiveTickMs == 0) lastActiveTickMs = now;
    unsigned long dt = now - lastActiveTickMs;

    if (dt >= 1000) {
      uint32_t addS = dt / 1000;
      activeSeconds += addS;
      lastActiveTickMs += addS * 1000UL;
    }

    if (now - lastEepromCommitMs >= 60000UL) {
      lastEepromCommitMs = now;
      eepromWriteU32(EEPROM_ADDR_ACTS_U32, activeSeconds);
      EEPROM.commit();
    }
  } else {
    lastActiveTickMs = 0;
  }
}

// ---------------- Save/Load ----------------
void saveAllSettings() {
  EEPROM.write(EEPROM_ADDR_LIVEAI_U8, (uint8_t)(liveAiEnabled ? 1 : 0));
  EEPROM.write(EEPROM_ADDR_AUTOMODE_U8, (uint8_t)(autoModeEnabled ? 1 : 0));
  eepromWriteU16(EEPROM_ADDR_CUT_U16, (uint16_t)cutTime);
  EEPROM.write(EEPROM_ADDR_MODE_U8, (uint8_t)currentMode);
  EEPROM.write(EEPROM_ADDR_ANTIFT_U8, (uint8_t)(antiTheftEnabled ? 1 : 0));
  EEPROM.write(EEPROM_ADDR_LCD_U8, (uint8_t)(lcdEnabled ? 1 : 0));
  EEPROM.write(EEPROM_ADDR_SENSOR_TYPE_U8, sensorTypeUnset ? 0xFF : (uint8_t)shiftSensorType);

  eepromWriteU32(EEPROM_ADDR_ERR_U32, errorCount);
  eepromWriteU32(EEPROM_ADDR_ACTS_U32, activeSeconds);
  eepromWriteFloat(EEPROM_ADDR_SCALE_F32, batteryScale);
  eepromWriteU16(EEPROM_ADDR_COOLDOWN_U16, shiftCooldownMs);
  eepromWriteU32(EEPROM_ADDR_BLOCKED_U32, blockedShifts);

  EEPROM.write(EEPROM_ADDR_SCOPPI_U8, (uint8_t)(scoppiEnabled ? 1 : 0));
  EEPROM.write(EEPROM_ADDR_BANG_PULSES, (uint8_t)bangPulses);
  eepromWriteU16(EEPROM_ADDR_BANG_CUT_U16, bangCutMs);
  eepromWriteU16(EEPROM_ADDR_BANG_PASS_U16, bangPassMs);
  eepromWriteU16(EEPROM_ADDR_BANG_MAX_U16, bangMaxTotalMs);

  eepromWriteU16(EEPROM_ADDR_PIEZO_TH_U16, piezoTh);

  eepromWriteU32(EEPROM_ADDR_MAGIC_U32, EEPROM_MAGIC);

  EEPROM.commit();
}

// ---------------- Apply settings ----------------
void applyActivation(bool on) {
  if (antiTheftEnabled) {
    if (on) addLog("WARN", "Activation blocked (AntiTheft)");
    isActivated = false;
    return;
  }

  bool prev = isActivated;
  isActivated = on;

  if (!prev && isActivated) {
    ledOnTransitionUntilMs = millis() + 2000UL;
    ledBlinkLastToggleMs = millis();
    ledBlinkState = false;
    addLog("INFO", "Activation=ON (momentary)");
  }
  if (prev && !isActivated) {
    addLog("INFO", "Activation=OFF (released)");
  }

  if (lcdEnabled) {
    lcdPrintClean(0, 1, "Status: " + String(isActivated ? "ON" : "OFF"));
    lcdMode = LCD_STATUS;
    lcdTimer = millis();
  }

  saveAllSettings();
}

void applyCut(int ms) {
  cutTime = constrain(ms, CUT_MIN, CUT_MAX);
  if (lcdEnabled) {
    lcdPrintClean(0, 0, "Cut: " + String(cutTime) + "ms");
    lcdMode = CUT;
    lcdTimer = millis();
  }
  addLog("INFO", "CutTime=" + String(cutTime) + "ms");
  saveAllSettings();
}

void applyMode(Mode m) {
  currentMode = m;
  if (lcdEnabled) {
    lcdPrintClean(0, 1, "Mode: " + modeToStr(currentMode));
    lcdMode = MODE;
    lcdTimer = millis();
  }
  addLog("INFO", "Mode=" + modeToStr(currentMode));

  if (currentMode == MODE_STREET) {
    cutTime = 75;
    addLog("INFO", "Street fixed cut=75ms");
  }
  saveAllSettings();
}

void applyAntiTheft(bool en) {
  antiTheftEnabled = en;
  addLog("WARN", String("AntiTheft=") + (antiTheftEnabled ? "ON" : "OFF"));

  if (antiTheftEnabled) {
    isActivated = false;
    shiftActive = false;
    bangActive = false;
    digitalWrite(mosfetPin, LOW);  // CUT held
    antiTheftCycleStartMs = millis();
  } else {
    digitalWrite(mosfetPin, HIGH);
    antiTheftCycleStartMs = millis();
  }
  saveAllSettings();
}

void applyLcdEnabled(bool en) {
  lcdEnabled = en;
  lcdApplyEnabled();
  addLog("INFO", String("LCD=") + (lcdEnabled ? "ON" : "OFF"));
  saveAllSettings();
}

void applyCooldown(uint16_t ms) {
  shiftCooldownMs = (uint16_t)constrain((int)ms, (int)COOLDOWN_MIN_MS, (int)COOLDOWN_MAX_MS);
  addLog("INFO", "Cooldown=" + String(shiftCooldownMs) + "ms");
  saveAllSettings();
}

void applyScoppi(bool en) {
  scoppiEnabled = en;
  addLog("WARN", String("SCOPPI=") + (scoppiEnabled ? "ON" : "OFF"));
  saveAllSettings();
}

void applyBangParams(uint8_t pulses, uint16_t cutms, uint16_t passms, uint16_t maxTotal) {
  bangPulses = (uint8_t)constrain((int)pulses, 1, 8);
  bangCutMs = (uint16_t)constrain((int)cutms, 3, 60);
  bangPassMs = (uint16_t)constrain((int)passms, 2, 60);
  bangMaxTotalMs = (uint16_t)constrain((int)maxTotal, 10, 250);

  addLog("INFO", "Bang params pulses=" + String(bangPulses) + " cut=" + String(bangCutMs) + " pass=" + String(bangPassMs) + " max=" + String(bangMaxTotalMs));
  saveAllSettings();
}

void applyPiezoThreshold(uint16_t th) {
  piezoTh = (uint16_t)constrain((int)th, 20, 2500);
  addLog("INFO", "PiezoTh=" + String(piezoTh));
  saveAllSettings();
}

// ---------------- Auto-calibrate piezo (single pass) ----------------
uint16_t autoCalibratePiezo() {
  const unsigned long T_MS = 2500;
  unsigned long t0 = millis();

  float tmpBase = (float)analogRead(piezoPin);
  float tmpEnv = 0.0f;

  double mean = 0.0;
  double m2 = 0.0;
  uint32_t n = 0;

  while (millis() - t0 < T_MS) {
    uint16_t r = (uint16_t)analogRead(piezoPin);
    float centered = (float)r - tmpBase;
    float absx = fabs(centered);
    tmpEnv = tmpEnv + PIEZO_ENV_ALPHA * (absx - tmpEnv);

    if (tmpEnv < 80.0f) tmpBase = tmpBase + 0.003f * ((float)r - tmpBase);

    n++;
    double x = (double)tmpEnv;
    double delta = x - mean;
    mean += delta / (double)n;
    double delta2 = x - mean;
    m2 += delta * delta2;

    // 👇 IMPORTANTISSIMO: lascia respirare WiFi/RTOS
    if ((n & 0x3F) == 0) yield();  // ogni 64 campioni

    delay(3);
  }


  double var = (n > 10) ? (m2 / (double)(n - 1)) : 0.0;
  double sd = sqrt(var);

  double thr = mean + 6.0 * sd;
  if (thr < 120.0) thr = 120.0;
  if (thr > 2500.0) thr = 2500.0;

  return (uint16_t)thr;
}

// ---------------- Guided "AI-like" piezo calibration ----------------
struct PiezoStats {
  double mean = 0.0;
  double sd = 0.0;
  double mx = 0.0;
  uint32_t n = 0;
};

PiezoStats samplePiezoEnvStats(uint16_t durationMs) {
  PiezoStats st;
  unsigned long t0 = millis();

  float tmpBase = (float)analogRead(piezoPin);
  float tmpEnv = 0.0f;

  double mean = 0.0;
  double m2 = 0.0;
  double mx = 0.0;
  uint32_t n = 0;

  while (millis() - t0 < durationMs) {
    uint16_t r = (uint16_t)analogRead(piezoPin);
    float centered = (float)r - tmpBase;
    float absx = fabs(centered);
    tmpEnv = tmpEnv + PIEZO_ENV_ALPHA * (absx - tmpEnv);

    if (tmpEnv < 80.0f) tmpBase = tmpBase + 0.003f * ((float)r - tmpBase);

    n++;
    double x = (double)tmpEnv;
    if (x > mx) mx = x;

    double delta = x - mean;
    mean += delta / (double)n;
    double delta2 = x - mean;
    m2 += delta * delta2;

    if ((n & 0x3F) == 0) yield();  // 👈 uguale

    delay(3);
  }

  double var = (n > 10) ? (m2 / (double)(n - 1)) : 0.0;
  double sd = sqrt(var);

  st.mean = mean;
  st.sd = sd;
  st.mx = mx;
  st.n = n;
  return st;
}

bool aiCalHasIdle = false;
PiezoStats aiIdle;

uint16_t computeGuidedThreshold(const PiezoStats& idle, const PiezoStats& rev) {
  // Obiettivo: stare sopra il rumore al minimo, ma non troppo alto da perdere i colpi.
  // Formula robusta: base = idle.mean + 4*idle.sd
  // + offset proporzionale alla differenza rev-idle (25%)
  double base = idle.mean + 4.0 * idle.sd;
  double delta = (rev.mean - idle.mean);
  if (delta < 0) delta = 0;
  double thr = base + 0.25 * delta;

  // min/margini
  if (thr < 120.0) thr = 120.0;
  if (thr > 2500.0) thr = 2500.0;
  return (uint16_t)thr;
}

// ---------------- Test mode ----------------
bool testArmed = false;
unsigned long testArmUntil = 0;

void armTest() {
  testArmed = true;
  testArmUntil = millis() + 10000UL;
  addLog("WARN", "TEST ARMED (10s)");
  server.send(200, "application/json", "{\"armed\":true,\"ttlMs\":10000}");
}

void testPulse() {
  if (!testArmed || millis() > testArmUntil) {
    testArmed = false;
    errorCount++;
    addLog("ERROR", "TEST_NOT_ARMED");
    saveAllSettings();
    server.send(403, "application/json", "{\"error\":\"TEST_NOT_ARMED\"}");
    return;
  }
  if (antiTheftEnabled) {
    server.send(423, "application/json", "{\"error\":\"ANTITHEFT_ON\"}");
    return;
  }
  int ms = 50;
  if (server.hasArg("ms")) ms = server.arg("ms").toInt();
  ms = constrain(ms, 10, 500);

  addLog("WARN", "TEST PULSE " + String(ms) + "ms");
  digitalWrite(mosfetPin, LOW);
  delay(ms);
  digitalWrite(mosfetPin, HIGH);

  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------- API: logs/history ----------------
void handleLogs() {
  String j = "{\"count\":" + String(logCount) + ",\"items\":[";
  int start = (logHead - logCount + LOG_CAP) % LOG_CAP;
  for (int i = 0; i < logCount; i++) {
    int idx = (start + i) % LOG_CAP;
    String line = logs[idx];
    line.replace("\\", "\\\\");
    line.replace("\"", "\\\"");
    j += "\"" + line + "\"";
    if (i < logCount - 1) j += ",";
  }
  j += "]}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", j);
}

void clearLogs() {
  logHead = 0;
  logCount = 0;
  addLog("INFO", "Logs cleared");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleHistory() {
  String j = "{\"count\":" + String(vhCount) + ",\"values\":[";
  int start = (vhHead - vhCount + VH_CAP) % VH_CAP;
  for (int i = 0; i < vhCount; i++) {
    int idx = (start + i) % VH_CAP;
    j += String(vh[idx], 2);
    if (i < vhCount - 1) j += ",";
  }
  j += "]}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", j);
}

void handlePiezoHistory() {
  String j = "{\"count\":" + String(phCount) + ",\"values\":[";
  int start = (phHead - phCount + PH_CAP) % PH_CAP;
  for (int i = 0; i < phCount; i++) {
    int idx = (start + i) % PH_CAP;
    j += String(ph[idx], 1);
    if (i < phCount - 1) j += ",";
  }
  j += "]}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", j);
}

// ---------------- Settings API ----------------
void resetCounters() {
  errorCount = 0;
  activeSeconds = 0;
  shiftCount = 0;
  blockedShifts = 0;
  blockedAntiTheftShifts = 0;
  addLog("WARN", "Counters reset");
  saveAllSettings();
  server.send(200, "application/json", "{\"ok\":true}");
}

void setScale() {
  if (!server.hasArg("v")) {
    server.send(400, "application/json", "{\"error\":\"missing v\"}");
    return;
  }
  float v = server.arg("v").toFloat();
  if (v < 0.1f || v > 50.0f) {
    server.send(400, "application/json", "{\"error\":\"out_of_range\"}");
    return;
  }
  batteryScale = v;
  addLog("INFO", "Scale set=" + String(batteryScale, 4));
  saveAllSettings();
  server.send(200, "application/json", "{\"ok\":true}");
}

void setCooldownApi() {
  if (!server.hasArg("ms")) {
    server.send(400, "application/json", "{\"error\":\"missing ms\"}");
    return;
  }
  int ms = server.arg("ms").toInt();
  ms = constrain(ms, (int)COOLDOWN_MIN_MS, (int)COOLDOWN_MAX_MS);
  applyCooldown((uint16_t)ms);
  server.send(200, "application/json", "{\"ok\":true}");
}

void setPiezoApi() {
  if (!server.hasArg("th")) {
    server.send(400, "application/json", "{\"error\":\"missing th\"}");
    return;
  }
  int th = server.arg("th").toInt();
  applyPiezoThreshold((uint16_t)th);
  server.send(200, "application/json", "{\"ok\":true}");
}

void factoryReset() {
  addLog("WARN", "FACTORY RESET requested");

  for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0xFF);
  EEPROM.commit();

  // rispondi subito
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true,\"reboot\":true}");

  // programma reboot (NON qui subito)
  rebootPending = true;
  rebootAtMs = millis() + 800;  // 0.8s per far finire la risposta
}


// ---------------- JSON state ----------------
String stateJson() {
  int sta = WiFi.softAPgetStationNum();
  float activeHours = activeSeconds / 3600.0f;
  bool uvNow = (batteryFiltered > 0.1f && batteryFiltered < UNDER_VOLTAGE_TH);

  String j = "{";
  j += "\"advMinAboveMs\":" + String(advMinAboveMs) + ",";
  j += "\"advMinSlope\":" + String(advMinSlope,2) + ",";
  j += "\"advEnvAlpha\":" + String(advEnvAlpha,3) + ",";
  j += "\"advBaseAlpha\":" + String(advBaseAlpha,4) + ",";
  j += "\"advHoldoffMs\":" + String(advHoldoffMs) + ",";
  j += "\"advNoisyLevel\":" + String(advNoisyLevel,2) + ",";
  j += "\"advNoisyHoldoffMs\":" + String(advNoisyHoldoffMs) + ",";
  j += "\"blockedAntiTheft\":" + String(blockedAntiTheftShifts) + ",";
  j += "\"autoMode\":" + String(autoModeEnabled ? "true" : "false") + ",";
  j += "\"autoModeLast\":\"" + autoModeLastDecision + "\",";
  j += "\"on\":" + String(isActivated ? "true" : "false") + ",";
  j += "\"mode\":\"" + modeToStr(currentMode) + "\",";
  j += "\"cutMs\":" + String(cutTime) + ",";
  j += "\"batteryV\":" + String(batteryFiltered, 2) + ",";
  j += "\"uv\":" + String(uvNow ? "true" : "false") + ",";
  j += "\"shiftActive\":" + String(shiftActive ? "true" : "false") + ",";
  j += "\"shiftCount\":" + String(shiftCount) + ",";
  j += "\"blockedShifts\":" + String(blockedShifts) + ",";
  j += "\"stations\":" + String(sta) + ",";
  j += "\"sensorType\":" + String((uint8_t)shiftSensorType) + ",";
  j += "\"sensorTypeUnset\":" + String(sensorTypeUnset ? "true" : "false") + ",";
  j += "\"errors\":" + String(errorCount) + ",";
  j += "\"activeHours\":" + String(activeHours, 2) + ",";
  j += "\"antiTheft\":" + String(antiTheftEnabled ? "true" : "false") + ",";
  j += "\"lcdEnabled\":" + String(lcdEnabled ? "true" : "false") + ",";
  j += "\"scale\":" + String(batteryScale, 4) + ",";
  j += "\"cooldownMs\":" + String(shiftCooldownMs) + ",";
  j += "\"scoppi\":" + String(scoppiEnabled ? "true" : "false") + ",";
  j += "\"bangPulses\":" + String(bangPulses) + ",";
  j += "\"bangCutMs\":" + String(bangCutMs) + ",";
  j += "\"bangPassMs\":" + String(bangPassMs) + ",";
  j += "\"bangMaxTotalMs\":" + String(bangMaxTotalMs) + ",";
  j += "\"piezoTh\":" + String(piezoTh) + ",";
  j += "\"piezoRaw\":" + String(piezoRaw) + ",";
  j += "\"piezoEnv\":" + String(piezoEnv, 1) + ",";
  j += "\"liveAiEnabled\":" + String(liveAiEnabled ? "true" : "false") + ",";
  j += "\"liveAiWarning\":" + String(liveAiWarning ? "true" : "false") + ",";
  j += "\"liveAiSuggestedTh\":" + String(liveAiSuggestedTh) + ",";
  j += "\"fw\":\"" + String(FW_VERSION) + "\"";
  j += "}";
  return j;
}

// ---------------- WEB UI ----------------
String minimalPage() {
  return String(
    "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width'>"
    "<title>QuickShifter</title></head><body style='font-family:system-ui'>"
    "<h3>QuickShifter</h3><p>WebApp non inclusa in questo file. Aggiungila dopo in webPage().</p>"
    "</body></html>");
}

String webPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>G3D QuickShifter</title>
<style>
:root{
  --bg:#070a12; --bg2:#0a0e1b; --stroke: rgba(255,255,255,.10);
  --text:#e8ecf7; --muted:#9aa2c4;
  --racing:#7c3aed; --custom:#3b82f6; --street:#22c55e; --danger:#ef4444;
  --radius:22px; --shadow:0 22px 60px rgba(0,0,0,.55);
  --t:220ms; --spring:cubic-bezier(.2,.9,.2,1);
}
*{box-sizing:border-box}
html,body{height:100%}
body{
  margin:0;color:var(--text);
  font-family: ui-sans-serif, system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial;
  background:
    radial-gradient(900px 520px at 18% -10%, rgba(124,58,237,.20), transparent 62%),
    radial-gradient(900px 520px at 115% 6%, rgba(59,130,246,.16), transparent 62%),
    linear-gradient(180deg, var(--bg), var(--bg2));
  background-attachment: fixed;
  overflow-x:hidden;position:relative;
}
body::before{
  content:"";position:fixed; inset:0;pointer-events:none;
  background-image:
    radial-gradient(circle at 20% 30%, rgba(255,255,255,.02), transparent 55%),
    radial-gradient(circle at 80% 10%, rgba(255,255,255,.015), transparent 60%),
    linear-gradient(0deg, rgba(255,255,255,.015), rgba(255,255,255,.00));
  opacity:.9;mix-blend-mode: overlay;
}
.app{min-height:100%;display:flex;flex-direction:column}
.container{max-width:560px;margin:0 auto;padding:14px 14px 28px;width:100%;flex:1;display:flex;flex-direction:column}

/* Intro (controlled by JS) */
#intro{
  position:fixed; inset:0; display:flex; align-items:center; justify-content:center;
  background:
    radial-gradient(1200px 600px at 50% 20%, rgba(255,255,255,.10), transparent 58%),
    linear-gradient(180deg, rgba(0,0,0,.92), rgba(0,0,0,.78));
  backdrop-filter: blur(10px);
  z-index:9999;
}
#intro.out{ animation: introOut 1.0s var(--spring) forwards; }

#intro .wrap{
  text-align:center;
  transform: translateY(10px) scale(.98);
  animation: introIn .9s var(--spring) forwards;
}
#intro .logo{
  font-weight:1000;letter-spacing:.18em;text-transform:uppercase;
  font-size:44px;color:rgba(255,255,255,.96);
  text-shadow:0 0 14px rgba(255,255,255,.35),
              0 0 36px rgba(255,255,255,.20),
              0 0 70px rgba(255,255,255,.14);
}
#intro .sub{
  margin-top:14px;color:rgba(255,255,255,.55);
  font-size:12px;letter-spacing:.22em;text-transform:uppercase;
}
#intro .bar{
  width:220px;height:8px;border-radius:999px;margin:22px auto 0;
  background: rgba(255,255,255,.10);overflow:hidden;
  border:1px solid rgba(255,255,255,.10);
}
#intro .bar>span{
  display:block;height:100%;width:0%;
  background: linear-gradient(90deg, rgba(255,255,255,.95), rgba(255,255,255,.35));
  box-shadow: 0 0 18px rgba(255,255,255,.25);
  animation: introLoad 2.6s var(--spring) forwards;
}
/* Sensor overlay fade out */
#sensorOverlay{
  transition: opacity 260ms var(--spring), transform 260ms var(--spring), filter 260ms var(--spring);
}
#sensorOverlay.out{
  opacity:0;
  transform: scale(.985);
  filter: blur(10px);
  pointer-events:none;
}

@keyframes introIn{
  from{opacity:0;transform:translateY(14px) scale(.96);filter:blur(6px)}
  to{opacity:1;transform:translateY(0) scale(1);filter:blur(0)}
}
@keyframes introLoad{from{width:0%}to{width:100%}}
@keyframes introOut{from{opacity:1}to{opacity:0;visibility:hidden;pointer-events:none}}

.introBottom{
  position:absolute; left:14px; bottom:14px;
  display:flex; align-items:center; gap:10px;
  color:rgba(255,255,255,.65);
  font-size:11px; letter-spacing:.08em; text-transform:uppercase;
}
.spin{
  width:14px;height:14px;border-radius:50%;
  border:2px solid rgba(255,255,255,.25);
  border-top-color: rgba(255,255,255,.85);
  animation: spin 800ms linear infinite;
}
@keyframes spin{to{transform:rotate(360deg)}}

.introDone{
  position:absolute; right:14px; bottom:14px;
  color:rgba(34,197,94,.92);
  font-weight:1000;
  letter-spacing:.10em;
  text-transform:uppercase;
  font-size:12px;
  opacity:0;
  transform: translateY(6px);
  transition: opacity 220ms var(--spring), transform 220ms var(--spring);
}
.introDone.on{opacity:1; transform: translateY(0);}

.top{display:flex;align-items:center;justify-content:space-between;padding:10px 6px 10px}

/* TITLE + BASIC */
.titleWrap{display:inline-flex;flex-direction:column;align-items:flex-start}
.title{font-size:20px;font-weight:1000;letter-spacing:.2px}
.badgeBasic{
  align-self:flex-end;margin-top:2px;
  font-size:12px;font-weight:1000;letter-spacing:.12em;text-transform:uppercase;
  color: rgba(124,58,237,.95);
  text-shadow: 0 0 18px rgba(124,58,237,.25);
}

.status{display:inline-flex;align-items:center;gap:10px;padding:8px 12px;border-radius:999px;background:rgba(255,255,255,.05);
  border:1px solid var(--stroke);color:var(--muted);font-size:12px;font-weight:950;letter-spacing:.10em;text-transform:uppercase}
.dot{width:10px;height:10px;border-radius:50%;background:#444}
.dot.on{background:var(--street);box-shadow:0 0 20px rgba(34,197,94,.55)}

.menu{display:flex;gap:10px;padding:0 6px 12px}
.tab{
  flex:1;border-radius:16px;border:1px solid rgba(255,255,255,.10);background: rgba(255,255,255,.05);
  padding:12px 10px;cursor:pointer;text-align:center;
  font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px;
  transition: transform var(--t) var(--spring), box-shadow var(--t) var(--spring), border-color var(--t) var(--spring), background var(--t) var(--spring);
  position:relative;overflow:hidden;
}
.tab:active{transform:translateY(1px) scale(.99)}
.tab.active{border-color: rgba(255,255,255,.18);background: rgba(255,255,255,.10);
  box-shadow: 0 0 26px rgba(255,255,255,.10), 0 0 60px rgba(255,255,255,.04) inset;}

@keyframes warnBlink{0%,49%{opacity:1}50%,100%{opacity:.12}}
.tabWarn{display:none;margin-left:8px;vertical-align:middle;filter: drop-shadow(0 0 8px rgba(239,68,68,.9));}
.tabWarn.on{display:inline-flex;animation: warnBlink .85s linear infinite}

.page{display:none}
.page.active{display:block;animation: pageIn 280ms var(--spring) forwards}
@keyframes pageIn{from{opacity:0;transform:translateY(10px) scale(.995)}to{opacity:1;transform:translateY(0) scale(1)}}

.card{border-radius:var(--radius);border:1px solid rgba(255,255,255,.10);
  background: linear-gradient(180deg, rgba(255,255,255,.07), rgba(255,255,255,.03));
  box-shadow:var(--shadow);padding:16px;margin:12px 0}
.kicker{font-size:12px;font-weight:1000;letter-spacing:.12em;text-transform:uppercase;color:rgba(255,255,255,.78)}
.sub{color:var(--muted);font-size:13px;line-height:1.25}
.bigV{font-size:38px;font-weight:1000}
.row{display:flex;align-items:center;justify-content:space-between;gap:12px}
.hr{height:1px;background:rgba(255,255,255,.08);margin:12px 0}

@keyframes glowPulse{0%{box-shadow:0 0 0 rgba(255,255,255,0)}50%{box-shadow:0 0 34px rgba(255,255,255,.16)}100%{box-shadow:0 0 0 rgba(255,255,255,0)}}
.btn{
  -webkit-tap-highlight-color:transparent;
  border:1px solid rgba(255,255,255,.14);background: rgba(255,255,255,.06);color: var(--text);
  padding:12px 14px;border-radius:16px;font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px;
  cursor:pointer;position:relative;overflow:hidden;
  transition: transform var(--t) var(--spring), border-color var(--t) var(--spring), background var(--t) var(--spring), box-shadow var(--t) var(--spring);
  animation: glowPulse 2.0s ease-in-out infinite;
}
.btn:active{transform:translateY(1px) scale(.99)}
.btn.on{
  border-color: rgba(34,197,94,.75);background: rgba(34,197,94,.18);
  animation:none;box-shadow: 0 0 28px rgba(34,197,94,.38), 0 0 74px rgba(34,197,94,.14);
}
.btn.danger{border-color: rgba(239,68,68,.60);background: rgba(239,68,68,.12);animation: glowPulse 1.6s ease-in-out infinite;box-shadow: 0 0 18px rgba(239,68,68,.14);}
@keyframes redBlink{0%,45%{box-shadow:0 0 0 rgba(239,68,68,0);filter:saturate(1)}50%{box-shadow:0 0 36px rgba(239,68,68,.55);filter:saturate(1.25)}100%{box-shadow:0 0 0 rgba(239,68,68,0);filter:saturate(1)}}
.btn.alert{border-color: rgba(239,68,68,.78) !important;background: rgba(239,68,68,.20) !important;animation: redBlink 1.0s ease-in-out infinite !important;}
.btn.presetSel{
  border-color: rgba(124,58,237,.75) !important;
  background: rgba(124,58,237,.18) !important;
  box-shadow: 0 0 34px rgba(124,58,237,.22) !important;
  animation: none !important;
}

.modeRail{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-top:12px}
.modeChip{border-radius:18px;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.18);padding:12px;cursor:pointer}
.modeChip .t{font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px}
.modeChip .d{font-size:12px;color:var(--muted);margin-top:6px;line-height:1.2}
.modeChip.active.race{border-color:rgba(124,58,237,.75);background:rgba(124,58,237,.18);box-shadow:0 0 34px rgba(124,58,237,.26)}
.modeChip.active.custom{border-color:rgba(59,130,246,.75);background:rgba(59,130,246,.18);box-shadow:0 0 34px rgba(59,130,246,.22)}
.modeChip.active.street{border-color:rgba(34,197,94,.75);background:rgba(34,197,94,.18);box-shadow:0 0 34px rgba(34,197,94,.22)}

.cutTop{display:flex;justify-content:space-between;align-items:baseline;gap:10px;flex-wrap:wrap}
.cutTop .stat{color:rgba(34,197,94,.92);font-weight:1000;font-size:12px;letter-spacing:.10em;text-transform:uppercase}

input[type=range]{-webkit-appearance:none;width:100%;height:16px;border-radius:999px;outline:none;background: rgba(255,255,255,.12);margin:16px 0 12px;box-shadow: 0 0 22px rgba(255,255,255,.08);}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:30px;height:30px;border-radius:50%;background:#fff;border:4px solid var(--racing);box-shadow:0 0 26px rgba(124,58,237,.34);}

.presetRow{display:grid;grid-template-columns:repeat(3, 1fr);gap:10px;margin-top:12px}
.preset{border-radius:18px;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.16);padding:12px;cursor:pointer;text-align:center}
.preset .ms{font-weight:1000;font-size:22px}
.preset .lbl{font-weight:1000;font-size:12px;letter-spacing:.10em;text-transform:uppercase;color:rgba(255,255,255,.78);margin-top:6px}
.preset.active{border-color:rgba(255,255,255,.18);background:rgba(255,255,255,.08);box-shadow:0 0 34px rgba(255,255,255,.10)}

.grid2{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
.kpi{border-radius:18px;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.16);padding:12px;}
.kpi .v{font-size:22px;font-weight:1000}
.kpi .k{font-size:12px;letter-spacing:.10em;text-transform:uppercase;color:var(--muted);font-weight:1000;margin-top:6px}

.list{margin-top:12px;border-radius:18px;overflow:hidden;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.14);}
.li{display:flex;justify-content:space-between;gap:10px;padding:12px 14px;border-top:1px solid rgba(255,255,255,.08)}
.li:first-child{border-top:none}
.li .k{font-size:12px;letter-spacing:.10em;text-transform:uppercase;color:var(--muted);font-weight:1000}
.li .v{font-weight:1000}

.console{
  font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono","Courier New", monospace;
  white-space:pre-wrap;border-radius:18px;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.22);
  padding:12px;min-height:120px;max-height:240px;overflow:auto;font-size:12px;color:rgba(255,255,255,.88);
}
.chartWrap{margin-top:12px;border-radius:18px;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.18);padding:10px;}
canvas{width:100%;height:170px;display:block}

.warnIcon{display:none;width:16px;height:16px;margin-left:8px;vertical-align:middle;}
.warnIcon.on{display:inline-block;animation: warnBlink 0.8s linear infinite;filter: drop-shadow(0 0 8px rgba(239,68,68,.9));}

.theftWarn{
  margin-top:12px;border-radius:18px;border:1px solid rgba(239,68,68,.35);
  background: rgba(239,68,68,.10);padding:12px 14px;display:none;align-items:center;gap:10px;
}
.theftWarn.on{display:flex;animation: warnBlink .85s linear infinite}
.theftWarn .t{font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px;color: rgba(255,255,255,.88)}
.theftWarn .d{font-size:12px;color: rgba(255,255,255,.65);line-height:1.2}

/* SCOPPI selector */
.scoppiRow{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-top:12px}
.scoppiToggle{
  display:inline-flex;align-items:center;gap:10px;
  padding:10px 12px;border-radius:16px;cursor:pointer;
  border:1px solid rgba(255,255,255,.14);background: rgba(255,255,255,.06);
  font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px;
}
.box{width:18px;height:18px;border-radius:6px;border:2px solid rgba(255,255,255,.35);background:transparent;position:relative}
.box.on{border-color: rgba(239,68,68,.85); box-shadow:0 0 18px rgba(239,68,68,.28)}
.box.on::after{
  content:""; position:absolute; left:4px; top:1px; width:6px; height:10px;
  border-right:3px solid rgba(239,68,68,.95);
  border-bottom:3px solid rgba(239,68,68,.95);
  transform: rotate(40deg);
}
.scoppiWarn{
  margin-top:12px;border-radius:18px;border:1px solid rgba(239,68,68,.35);
  background: rgba(239,68,68,.10);padding:12px 14px;display:none;
}
.scoppiWarn.on{display:block}
.scoppiWarn .t{font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px;color: rgba(255,255,255,.88)}
.scoppiWarn .d{margin-top:6px;font-size:12px;color: rgba(255,255,255,.65);line-height:1.25}

.devBox{margin-top:12px;border-radius:18px;border:1px solid rgba(255,255,255,.10);background: rgba(0,0,0,.14);overflow:hidden;}
.devHead{padding:12px 14px;display:flex;justify-content:space-between;align-items:center;cursor:pointer;user-select:none;}
.devHead .k{font-size:12px;letter-spacing:.12em;text-transform:uppercase;color:rgba(255,255,255,.75);font-weight:1000}
.devBody{display:none;padding:12px 14px;border-top:1px solid rgba(255,255,255,.08)}
.devBox.open .devBody{display:block}

.footer{margin-top:auto;padding:18px 0 10px;text-align:center;color:rgba(255,255,255,.30);
  font-size:11px;letter-spacing:.12em;text-transform:uppercase;}

/* Live AI warning panel */
.aiWarn{
  margin-top:12px;border-radius:18px;border:1px solid rgba(239,68,68,.35);
  background: rgba(239,68,68,.10);padding:12px 14px;display:none;
}
.aiWarn.on{display:block;animation: warnBlink .85s linear infinite}
.aiWarn .t{font-weight:1000;letter-spacing:.10em;text-transform:uppercase;font-size:12px;color: rgba(255,255,255,.88)}
.aiWarn .d{margin-top:6px;font-size:12px;color: rgba(255,255,255,.65);line-height:1.25}
.aiWarn .a{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}

/* Small info chips */
.chipRow{display:flex;gap:10px;flex-wrap:wrap;margin-top:10px}
.chip{
  border-radius:999px;border:1px solid rgba(255,255,255,.12);
  background: rgba(0,0,0,.18);
  padding:8px 10px;font-size:11px;font-weight:1000;letter-spacing:.10em;text-transform:uppercase;
  color:rgba(255,255,255,.70);
}
.chip.ok{border-color: rgba(34,197,94,.28); background: rgba(34,197,94,.10); color: rgba(34,197,94,.90)}
.chip.bad{border-color: rgba(239,68,68,.28); background: rgba(239,68,68,.10); color: rgba(239,68,68,.92)}
.chip.ai{border-color: rgba(124,58,237,.28); background: rgba(124,58,237,.10); color: rgba(124,58,237,.92)}

.hidden{display:none!important}
</style>
</head>
<body>

<div id="intro">
  <div class="wrap">
    <div class="logo">G3Design</div>
    <div class="sub">Performance electronics</div>
    <div class="bar"><span></span></div>
  </div>

  <div class="introBottom">
    <div class="spin"></div>
    <div id="introMsg">Initializing system…</div>
  </div>

  <div id="introDone" class="introDone">Done!</div>
</div>

<div class="app"><div class="container">
  <div class="top">
    <div>
      <div class="titleWrap">
        <div class="title">G3D QuickShifter</div>
        <div class="badgeBasic">Basic</div>
      </div>
      <div class="chipRow">
        <div class="chip" id="chipFw">FW: --</div>
        <div class="chip" id="chipStations">Clients: --</div>
        <div class="chip ai" id="chipAutoMode" style="display:none">AutoMode: ON</div>
        <div class="chip" id="chipLiveAi" style="display:none">LiveAI: ON</div>
      </div>
    </div>
    <div class="status"><span id="dot" class="dot"></span><span id="conn">offline</span></div>
  </div>

  <div class="menu">
    <div class="tab active" id="tabControl" onclick="showPage('control')">Control Panel</div>
    <div class="tab" id="tabTheft" onclick="showPage('theft')">
      Anti Theft <span id="tabTheftWarn" class="tabWarn" aria-hidden="true">
        <svg viewBox="0 0 24 24" width="14" height="14"><path fill="rgba(239,68,68,.95)" d="M12 2 1 21h22L12 2zm0 6c.6 0 1 .4 1 1v6c0 .6-.4 1-1 1s-1-.4-1-1V9c0-.6.4-1 1-1zm0 12a1.3 1.3 0 1 0 0-2.6A1.3 1.3 0 0 0 12 20z"/></svg>
      </span>
    </div>
    <div class="tab" id="tabSettings" onclick="showPage('settings')">Settings</div>
  </div>

  <!-- CONTROL -->
  <div id="pageControl" class="page active">

    <!-- LIVE AI WARNING -->
    <div id="aiWarn" class="aiWarn">
      <div class="t">ai live warning</div>
      <div class="d" id="aiWarnText">L’AI ha rilevato rumore sul sensore. Consigliato aumentare la soglia.</div>
      <div class="a">
        <button class="btn" id="aiApplyBtn" onclick="applyAiSuggested()">IMPOSTA SUGGERITA</button>
        <button class="btn danger" onclick="dismissAiWarning()">CHIUDI</button>
      </div>
    </div>

    <section class="card">
      <div class="kicker">battery voltage</div>
      <div class="row" style="margin-top:8px">
        <div class="bigV">
          <span id="batteryV">--.-</span> V
          <svg id="uvIcon" class="warnIcon" viewBox="0 0 24 24" aria-hidden="true">
            <path fill="rgba(239,68,68,.95)" d="M12 2 1 21h22L12 2zm0 6c.6 0 1 .4 1 1v6c0 .6-.4 1-1 1s-1-.4-1-1V9c0-.6.4-1 1-1zm0 12a1.3 1.3 0 1 0 0-2.6A1.3 1.3 0 0 0 12 20z"/>
          </svg>
        </div>
        <button class="btn" id="toggleBtn" onclick="toggle()">OFF</button>
      </div>
      <div class="sub" style="margin-top:6px">Undervoltage warning &lt; 11.0V</div>
    </section>

    <section class="card">
      <div class="kicker">operating modes</div>

      <div class="modeRail">
        <div class="modeChip" id="cRace" onclick="setMode('RACE')"><div class="t">Racing</div><div class="d">Massima reattività</div></div>
        <div class="modeChip" id="cCustom" onclick="setMode('CUSTOM')"><div class="t">Custom</div><div class="d">Range esteso</div></div>
        <div class="modeChip" id="cStreet" onclick="setMode('STREET')"><div class="t">Street</div><div class="d">Setup morbido</div></div>
      </div>

      <div class="scoppiRow">
        <div>
          <div class="kicker" style="margin-top:6px">bang</div>
          <div class="sub">Attiva sequenza CUT/PASS durante la cambiata.</div>
        </div>
        <div class="scoppiToggle" onclick="toggleScoppi()">
          <span id="scoppiBox" class="box"></span>
          SCOPPI
        </div>
      </div>

      <div id="scoppiWarn" class="scoppiWarn">
        <div class="t">avviso</div>
        <div class="d">
          la modalità scoppi può danneggiare il motore non ci assumiamo nessuna responsabilità è fatta solo per divertimento non per uso continuo
        </div>
      </div>
    </section>

    <section class="card">
      <div class="cutTop">
        <div><div class="kicker">cut time</div><div class="sub">TIP: soglia consigliata sopra i 20ms</div></div>
        <div class="stat" id="cutStat">CURRENT: --ms</div>
      </div>

      <input id="range" type="range" min="20" max="40" value="30"
        oninput="previewCut(this.value)" onchange="applyCut(this.value)">

      <div class="sub" style="text-align:center;font-weight:1000;letter-spacing:.10em;text-transform:uppercase;">
        <span id="modeLabel">Racing</span> • <span id="cutMid">--</span>ms <span id="setLabel" style="display:none; color:var(--muted)">SET</span>
      </div>

      <div class="presetRow">
        <div class="preset" id="p1" onclick="setPreset(30,'RACE')"><div class="ms">30</div><div class="lbl">Racing</div></div>
        <div class="preset" id="p2" onclick="setPreset(50,'CUSTOM')"><div class="ms">50</div><div class="lbl">Custom</div></div>
        <div class="preset" id="p3" onclick="setPreset(75,'STREET')"><div class="ms">75</div><div class="lbl">Street</div></div>
      </div>
    </section>

    <!-- AI FEATURES (separata, sotto Modes + Cut Time) -->
    <section class="card">
      <div class="kicker">ai features</div>
      <div class="hr"></div>

      <div class="row">
        <div>
          <div class="kicker">auto mode ai</div>
          <div class="sub" style="margin-top:6px">L'auto mode permette all'AI di esaminare lo stile di guida e cambiare il cut time per avere una guida più adattiva</div>
        </div>
        <button class="btn" id="autoModeBtn" onclick="toggleAutoMode()">OFF</button>
      </div>
      <div class="sub" id="autoModeInfo" style="margin-top:10px;opacity:.7"></div>

      <div class="hr"></div>

      <div class="row piezoOnly" id="liveAiRow">
        <div>
          <div class="kicker">live ai control</div>
          <div class="sub" style="margin-top:6px">Live AI Control verifica se ci sono problemi di cablaggio o sensibilità/calibrazione del sensore piezo e avvisa tramite il led nel bottone a manubrio. (SOLO PER SENSORE PIEZO NO PUNTALINO)</div>
        </div>
        <button class="btn" id="liveAiBtn" onclick="toggleLiveAi()">ON</button>
      </div>
      <div class="sub piezoOnly" id="liveAiInfo" style="margin-top:10px;opacity:.7"></div>
    </section>

    <section class="card">
      <div class="kicker">telemetry</div>
      <div class="grid2">
        <div class="kpi"><div class="v" id="activeHours">--</div><div class="k">active hours</div></div>
        <div class="kpi"><div class="v" id="errors">--</div><div class="k">errors</div></div>
        <div class="kpi"><div class="v" id="modeKpi">--</div><div class="k">mode</div></div>
        <div class="kpi"><div class="v" id="shifts">--</div><div class="k">shifts</div></div>
      </div>

      <div class="list">
        <div class="li"><div class="k">cut time</div><div class="v" id="cutTable">--ms</div></div>
        <div class="li"><div class="k">cooldown</div><div class="v" id="cooldownShow">--ms</div></div>
        <div class="li"><div class="k">scoppi</div><div class="v" id="scoppiShow">--</div></div>
        <div class="li piezoOnly" id="piezoShowLi"><div class="k">sensor sensitivity</div><div class="v" id="piezoShow">--</div></div>
        <div class="li piezoOnly" id="liveAiShowLi"><div class="k">live ai</div><div class="v" id="liveAiShow">--</div></div>
        <div class="li"><div class="k">firmware</div><div class="v" id="fwShow">--</div></div>
        <div class="li"><div class="k">wifi clients</div><div class="v" id="stations">--</div></div>
      </div>
    </section>

    <section class="card">
      <div class="row">
        <div><div class="kicker">diagnostics</div><div class="sub" style="margin-top:6px">Grafici di diagnostica batteria/voltaggio sensore piezo e console attività.</div></div>
        <button class="btn" onclick="clearLogs()">CLEAR</button>
      </div>

      <div class="chartWrap">
        <div class="sub" style="margin-bottom:8px">Battery Voltage</div>
        <canvas id="chart" width="600" height="220"></canvas>
      </div>

      <div class="chartWrap piezoOnly" id="piezoChartWrap" style="margin-top:12px">
        <div class="sub" style="margin-bottom:8px">Piezo Voltage</div>
        <canvas id="chartPiezo" width="600" height="220"></canvas>
      </div>

      <div class="hr"></div>
      <div id="console" class="console">[--:--:--] [INIT] Console ready...</div>
    </section>

    <section class="card">
      <div class="kicker">test mode</div>
      <div class="hr"></div>
      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <button class="btn danger" onclick="armTest()">ARM 10s</button>
        <button class="btn" onclick="pulse(50)">PULSE 50ms</button>
        <button class="btn" onclick="pulse(100)">PULSE 100ms</button>
        <button class="btn" onclick="pulse(200)">PULSE 200ms</button>
      </div>
    </section>
  </div>

  <!-- ANTI THEFT -->
  <div id="pageTheft" class="page">
    <section class="card">
      <div class="kicker">anti theft</div>
      <div class="sub" style="margin-top:6px">
        Protezione antifurto per impedire l’uso non autorizzato del veicolo.
      </div>
      <div class="hr"></div>
      <div class="row">
        <div class="sub">Status</div>
        <button class="btn danger" id="theftBtn" onclick="toggleTheft()">OFF</button>
      </div>

      <div id="theftWarn" class="theftWarn">
        <svg viewBox="0 0 24 24" width="18" height="18"><path fill="rgba(239,68,68,.95)" d="M12 2 1 21h22L12 2zm0 6c.6 0 1 .4 1 1v6c0 .6-.4 1-1 1s-1-.4-1-1V9c0-.6.4-1 1-1zm0 12a1.3 1.3 0 1 0 0-2.6A1.3 1.3 0 0 0 12 20z"/></svg>
        <div><div class="t">Anti Theft Active</div><div class="d">Sistema antifurto attivo.</div></div>
      </div>

      <div class="hr"></div>

      <!-- SOLO contatore shift bloccati PER anti-theft -->
      <div class="grid2" style="grid-template-columns:1fr;">
        <div class="kpi">
          <div class="v" id="blockedShifts">--</div>
          <div class="k">anti theft blocked shifts</div>
        </div>
      </div>

      <!-- Disclaimer rosso: niente dettagli di funzionamento -->
      <div class="theftWarn on" style="display:flex;animation:none;margin-top:12px;border-color:rgba(239,68,68,.35);">
        <svg viewBox="0 0 24 24" width="18" height="18"><path fill="rgba(239,68,68,.95)" d="M12 2 1 21h22L12 2zm0 6c.6 0 1 .4 1 1v6c0 .6-.4 1-1 1s-1-.4-1-1V9c0-.6.4-1 1-1zm0 12a1.3 1.3 0 1 0 0-2.6A1.3 1.3 0 0 0 12 20z"/></svg>
        <div>
          <div class="t">warning</div>
          <div class="d">
            Anti Theft può essere usato come deterrente antifurto. Non ci assumiamo alcuna responsabilità per manomissioni, usi impropri o danni.
            Si consiglia di cambiare la password del Wi-Fi per evitare accessi non autorizzati.
          </div>
        </div>
      </div>
    </section>
  </div>

  <!-- SETTINGS -->
  <div id="pageSettings" class="page">
   <section class="card">
    <div class="kicker">shift sensor input</div>
    <div class="sub" style="margin-top:6px">
      Seleziona quale sensore usare per il trigger della cambiata.
    </div>
    <div class="hr"></div>

    <div style="display:flex; gap:10px; flex-wrap:wrap">
      <button class="btn" id="sensorPiezoBtn" onclick="setSensorType('piezo')">PIEZO</button>
      <button class="btn" id="sensorPuntBtn" onclick="setSensorType('puntalino')">PUNTALINO</button>
    </div>
    <div style="display:flex; gap:10px; flex-wrap:wrap; margin-top:10px">
    </div>


    <div class="sub" id="sensorTypeInfo" style="margin-top:10px;opacity:.75"></div>
   </section>


    <!-- SETTINGS (base) -->
    <section class="card">
      <div class="kicker">settings</div>
      <div class="hr"></div>

      <div class="row" style="align-items:flex-end">
        <div style="flex:1">
          <div class="sub" style="font-weight:1000; letter-spacing:.10em; text-transform:uppercase">Battery scale</div>
          <div class="sub">Valore attuale: <span id="scaleNow">--</span></div>
          <input id="scaleInput" type="number" step="0.0001" min="0.1" max="50"
            style="width:100%;margin-top:10px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>
        <button class="btn" style="margin-left:10px" onclick="saveScale()">SAVE</button>
      </div>

      <div class="hr"></div>

      <div class="row" style="align-items:flex-end">
        <div style="flex:1">
          <div class="sub" style="font-weight:1000; letter-spacing:.10em; text-transform:uppercase">Shift cooldown (ms)</div>
          <div class="sub">Valore attuale: <span id="cooldownNow">--</span> ms</div>
          <input id="cooldownInput" type="number" step="10" min="0" max="3000"
            style="width:100%;margin-top:10px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>
        <button class="btn" style="margin-left:10px" onclick="saveCooldown()">SAVE</button>
      </div>

      <div class="hr"></div>

      <!-- PIEZO SENSITIVITY -->
      <div class="row piezoOnly" id="piezoSensRow" style="align-items:flex-end">
        <div style="flex:1">
          <div class="sub" style="font-weight:1000; letter-spacing:.10em; text-transform:uppercase">Shift sensor sensitivity</div>
          <div class="sub">Valore attuale: <span id="sensNow">--</span> (più basso = più sensibile)</div>
          <input id="sensInput" type="number" step="5" min="20" max="2500"
            style="width:100%;margin-top:10px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>

        <div style="display:flex; gap:10px; margin-left:10px; flex-wrap:wrap">
          <button class="btn" onclick="saveSens()">SAVE</button>
          <button class="btn" onclick="autoSens()">AUTO (AI)</button>
        </div>
      </div>

      <div class="hr"></div>

      <section class="card" style="margin-top:12px">
        <div class="kicker">security &amp; privacy</div>
        <div class="sub" style="margin-top:6px">Cambia la password del Wi-Fi AP. Dopo il salvataggio il dispositivo si riavvia.</div>
        <div class="hr"></div>

        <div class="sub" style="font-weight:1000; letter-spacing:.10em; text-transform:uppercase">Wi-Fi password (8–31 chars)</div>
        <div style="display:flex; gap:10px; align-items:center; margin-top:10px">
          <input id="wifiPw" type="password" placeholder="New password"
            style="flex:1;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
          <button class="btn" onclick="togglePw()">SHOW</button>
        </div>

        <div style="display:flex; gap:10px; flex-wrap:wrap; margin-top:12px">
          <button class="btn danger" onclick="saveWifiPw()">SAVE &amp; REBOOT</button>
          <div class="sub" id="wifiInfo" style="align-self:center; opacity:.7"></div>
        </div>
      </section>

      <div class="hr"></div>
      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <button class="btn danger" onclick="resetCounters()">RESET COUNTERS</button>
        <button class="btn danger" onclick="factoryReset()">FACTORY RESET</button>
      </div>

      <div class="devBox" id="devBox">
        <div class="devHead" onclick="toggleDev()">
          <div class="k">Developers & testers only</div>
          <div style="color:rgba(255,255,255,.55);font-weight:1000">▼</div>
        </div>
        <div class="devBody">
          <div class="sub">Abilita/disabilita LCD fisico.</div>
          <div class="hr"></div>
          <div class="row">
            <div class="sub">LCD</div>
            <button class="btn" id="lcdBtn" onclick="toggleLcd()">ON</button>
          </div>
        </div>
      </div>
    </section>

    <!-- ADVANCED CALIBRATING (card separata) -->
    <section class="card">
      <div class="kicker">advanced calibrating</div>
      <div class="sub" style="margin-top:6px">Parametri avanzati per ridurre falsi trigger. Se non sai cosa indicano, lascia default.</div>

      <div class="devBox" id="advBox">
        <div class="devHead" onclick="toggleAdv()">
          <div>
            <div class="k">Advanced Calibrating</div>
            <div class="sub" style="margin-top:6px;font-size:11px;opacity:.75">
              se non sai cosa indicano questi valori non toccare lascia default
            </div>
          </div>
          <div style="color:rgba(255,255,255,.55);font-weight:1000">▼</div>
        </div>

        <div class="devBody piezoOnly" id="advPiezoBox">
          <div class="sub" style="opacity:.8">
            Questi parametri servono a ridurre falsi trigger ad alti giri senza rallentare lo shift.
          </div>
          <div class="hr"></div>

          <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px">
            <div>
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">MIN_ABOVE_MS</div>
              <input id="advMinAbove" type="number" min="2" max="50" step="1"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>

            <div>
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">MIN_SLOPE</div>
              <input id="advMinSlope" type="number" min="0" max="50" step="0.1"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>

            <div>
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">PIEZO_ENV_ALPHA</div>
              <input id="advEnvAlpha" type="number" min="0.01" max="0.50" step="0.01"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>

            <div>
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">PIEZO_BASE_ALPHA</div>
              <input id="advBaseAlpha" type="number" min="0.0001" max="0.0200" step="0.0001"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>

            <div>
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">PIEZO_HOLDOFF_MS</div>
              <input id="advHoldoff" type="number" min="20" max="800" step="5"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>

            <div>
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">PIEZO_NOISY_LEVEL</div>
              <input id="advNoisyLevel" type="number" min="1.00" max="2.00" step="0.05"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>

            <div style="grid-column:1 / -1;">
              <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">PIEZO_HOLDOFF_NOISY_MS</div>
              <input id="advNoisyHoldoff" type="number" min="60" max="1200" step="10"
                style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
            </div>
          </div>

          <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:12px">
            <button class="btn" onclick="saveAdvanced()">SAVE ADVANCED</button>
            <button class="btn danger" onclick="resetAdvancedDefaults()">RESET DEFAULT</button>
            <div class="sub" id="advInfo" style="align-self:center;opacity:.7"></div>
          </div>
        </div>
      </div>
    </section>

    <!-- BANG SETTINGS (card separata) -->
    <section class="card">
      <div class="kicker">bang settings</div>
      <div class="hr"></div>

      <div style="display:flex;gap:10px;flex-wrap:wrap">
        <button class="btn" id="bangMild" onclick="applyBangPreset('mild')">MILD</button>
        <button class="btn" id="bangBalanced" onclick="applyBangPreset('balanced')">BALANCED</button>
        <button class="btn" id="bangCrackle" onclick="applyBangPreset('crackle')">MORE CRACKLE</button>
      </div>

      <div class="hr"></div>

      <div class="sub" style="font-weight:1000; letter-spacing:.10em; text-transform:uppercase">Custom</div>
      <div class="sub" style="margin-top:6px">Questi parametri vengono usati quando SCOPPI è attivo.</div>

      <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px">
        <div>
          <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">Pulses cap</div>
          <input id="bangPulses" type="number" min="1" max="8"
            style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>
        <div>
          <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">Max total (ms)</div>
          <input id="bangMax" type="number" min="10" max="250"
            style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>
        <div>
          <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">Cut (ms)</div>
          <input id="bangCut" type="number" min="3" max="60"
            style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>
        <div>
          <div class="sub" style="font-weight:1000;letter-spacing:.10em;text-transform:uppercase">Pass (ms)</div>
          <input id="bangPass" type="number" min="2" max="60"
            style="width:100%;margin-top:8px;padding:12px;border-radius:14px;border:1px solid rgba(255,255,255,.12);background:rgba(0,0,0,.20);color:var(--text)">
        </div>
      </div>

      <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:12px">
        <button class="btn" onclick="saveBangCustom()">SAVE CUSTOM</button>
        <div class="sub" id="bangNow" style="align-self:center;opacity:.7"></div>
      </div>
    </section>

  </div>

  <div class="footer">developed and produced by g3design</div>
</div></div>

<script>
const $ = (id)=>document.getElementById(id);

function applyPiezoOnlyVisibility(piezoOn){
  // elementi marcati come piezoOnly (settings + control + dev)
  document.querySelectorAll('.piezoOnly').forEach(el=>{
    el.classList.toggle('hidden', !piezoOn);
  });
}
function uiLog(tag, msg){
  const c = $('console');
  const now = new Date();
  const hh = String(now.getHours()).padStart(2,'0');
  const mm = String(now.getMinutes()).padStart(2,'0');
  const ss = String(now.getSeconds()).padStart(2,'0');
  const line = `[${hh}:${mm}:${ss}] [${tag}] ${msg}`;
  if(c){
    c.textContent += "\n" + line;
    if(consoleAutoScroll) c.scrollTop = c.scrollHeight;
  }
}

function resetSensorAndReboot(){
  const ok = confirm('Reset scelta sensore e riavvio ESP. La webapp verrà disconnessa. Continuare?');
  if(!ok) return;

  // effetto “chiusura” visiva
  showExitOverlay('Reset sensore… reboot in corso');

  fetch('/api/settings/sensortype/reset_reboot', {
    method:'POST',
    cache:'no-store',
    keepalive:true
  }).catch(()=>{});

  setTimeout(()=>{ try{ window.close(); }catch(e){} }, 900);
}

let adjusting=false;
let currentMode='RACE';
let lastOk=0;
let advLoadedOnce = false;


/* Bang preset UI state */
let bangPresetSel = 'mild';
function setBangPresetUI(name){
  bangPresetSel = name;
  const ids = ['bangMild','bangBalanced','bangCrackle'];
  ids.forEach(id=>{ const el = $(id); if(el) el.classList.remove('presetSel'); });
  if(name==='mild' && $('bangMild')) $('bangMild').classList.add('presetSel');
  if(name==='balanced' && $('bangBalanced')) $('bangBalanced').classList.add('presetSel');
  if(name==='crackle' && $('bangCrackle')) $('bangCrackle').classList.add('presetSel');
}

// intro remove
window.addEventListener('load', ()=>{
  const intro = document.getElementById('intro');
  const msgEl = document.getElementById('introMsg');
  const doneEl = document.getElementById('introDone');
  if(!intro) return;

  const msgs = [
    'Initializing system…',
    'Loading calibration profiles…',
    'Syncing sensor filters…',
    'Starting Wi-Fi services…',
    'Checking EEPROM map…',
    'Booting UI modules…',
    'Applying safety limits…',
    'Linking telemetry…',
    'Finalizing…'
  ];

  const totalMs = 3000;
  const tickMs  = 300;
  let i = 0;
  const t0 = Date.now();

  const timer = setInterval(()=>{
    if(msgEl) msgEl.textContent = msgs[i++ % msgs.length];

    if(Date.now() - t0 >= totalMs){
      clearInterval(timer);
      if(doneEl) doneEl.classList.add('on');
      setTimeout(()=>{
        intro.classList.add('out');
        setTimeout(()=>intro.remove(), 1100);
      }, 300);
    }
  }, tickMs);
});

// console scroll lock
let consoleAutoScroll=true;
document.addEventListener('DOMContentLoaded', ()=>{
  const c=$('console');
  c.addEventListener('scroll', ()=>{
    const nearBottom = (c.scrollTop + c.clientHeight) >= (c.scrollHeight - 24);
    consoleAutoScroll = nearBottom;
  }, {passive:true});
});
function showExitOverlay(msg){
  let ov = document.getElementById('exitOverlay');
  if(!ov){
    ov = document.createElement('div');
    ov.id = 'exitOverlay';
    ov.style.cssText =
      'position:fixed;inset:0;display:flex;align-items:center;justify-content:center;z-index:99999;' +
      'background:linear-gradient(180deg, rgba(0,0,0,.92), rgba(0,0,0,.78));backdrop-filter:blur(10px);';
    ov.innerHTML =
      '<div style="width:min(520px,92vw);border-radius:22px;border:1px solid rgba(255,255,255,.12);' +
      'background:linear-gradient(180deg, rgba(255,255,255,.08), rgba(255,255,255,.03));' +
      'box-shadow:0 22px 60px rgba(0,0,0,.55);padding:18px;text-align:center;">' +
      '<div style="font-weight:1000;letter-spacing:.12em;text-transform:uppercase;font-size:12px;color:rgba(255,255,255,.78)">system</div>' +
      '<div id="exitMsg" style="margin-top:10px;font-size:18px;font-weight:1000"></div>' +
      '<div style="margin-top:10px;color:rgba(255,255,255,.55);font-size:12px">Connessione in chiusura. Riconnettiti dopo il reboot.</div>' +
      '</div>';
    document.body.appendChild(ov);
  }
  const m = document.getElementById('exitMsg');
  if(m) m.textContent = msg || '...';
  ov.style.display = 'flex';
}
function setConn(ok){
  $('conn').textContent = ok ? 'online' : 'offline';
  $('dot').classList.toggle('on', ok);
}
function resetSensorAndReboot(){
  const ok = confirm('Reset scelta sensore e riavvio ESP. La webapp verrà disconnessa. Continuare?');
  if(!ok) return;

  // Schermo “chiusura” (non possiamo chiudere davvero la tab, ma sembra chiusa)
  showExitOverlay('Reset sensore… reboot in corso');

  fetch('/api/settings/sensortype/reset_reboot', {
    method:'POST',
    cache:'no-store',
    keepalive:true
  }).catch(()=>{});

  // Prova a chiudere (spesso non permesso dal browser, ma non importa)
  setTimeout(()=>{ try{ window.close(); }catch(e){} }, 900);
}


function showPage(p){
  $('tabControl').classList.toggle('active', p==='control');
  $('tabTheft').classList.toggle('active', p==='theft');
  $('tabSettings').classList.toggle('active', p==='settings');
  $('pageControl').classList.toggle('active', p==='control');
  $('pageTheft').classList.toggle('active', p==='theft');
  $('pageSettings').classList.toggle('active', p==='settings');
  window.scrollTo(0,0);
}
function toggleDev(){ $('devBox').classList.toggle('open'); }
function toggleAdv(){ $('advBox').classList.toggle('open'); }

// Defaults “consigliati”
const ADV_DEFAULTS = {
  minAboveMs: 10,
  minSlope: 4.0,
  envAlpha: 0.12,
  baseAlpha: 0.0025,
  holdoffMs: 140,
  noisyLevel: 1.25,
  noisyHoldoffMs: 260
};

function fillAdvanced(d){
  $('advMinAbove').value = d.minAboveMs;
  $('advMinSlope').value = d.minSlope;
  $('advEnvAlpha').value = d.envAlpha;
  $('advBaseAlpha').value = d.baseAlpha;
  $('advHoldoff').value = d.holdoffMs;
  $('advNoisyLevel').value = d.noisyLevel;
  $('advNoisyHoldoff').value = d.noisyHoldoffMs;
  $('advInfo').textContent = 'defaults loaded';
}

function resetAdvancedDefaults(){
  fillAdvanced(ADV_DEFAULTS);
  $('advInfo').textContent = 'reset to default';
}


function saveAdvanced(){
  const payload = {
    minAboveMs: Number($('advMinAbove').value),
    minSlope: Number($('advMinSlope').value),
    envAlpha: Number($('advEnvAlpha').value),
    baseAlpha: Number($('advBaseAlpha').value),
    holdoffMs: Number($('advHoldoff').value),
    noisyLevel: Number($('advNoisyLevel').value),
    noisyHoldoffMs: Number($('advNoisyHoldoff').value)
  };
  if(!isFinite(payload.minAboveMs) || !isFinite(payload.minSlope) || !isFinite(payload.envAlpha)) return;

  const body =
    `minAboveMs=${encodeURIComponent(Math.round(payload.minAboveMs))}` +
    `&minSlope=${encodeURIComponent(payload.minSlope.toFixed(2))}` +
    `&envAlpha=${encodeURIComponent(payload.envAlpha.toFixed(3))}` +
    `&baseAlpha=${encodeURIComponent(payload.baseAlpha.toFixed(4))}` +
    `&holdoffMs=${encodeURIComponent(Math.round(payload.holdoffMs))}` +
    `&noisyLevel=${encodeURIComponent(payload.noisyLevel.toFixed(2))}` +
    `&noisyHoldoffMs=${encodeURIComponent(Math.round(payload.noisyHoldoffMs))}`;

  fetch('/api/settings/advancedcal', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body
  }).then(r=>r.json()).then(d=>{
    $('advInfo').textContent = d && d.ok ? 'saved' : ('error: ' + (d.error||'unknown'));
  }).catch(()=>{ $('advInfo').textContent = 'network error'; });
}

function setActiveMode(m){
  currentMode=m;
  const cRace=$('cRace'), cCustom=$('cCustom'), cStreet=$('cStreet');
  [cRace,cCustom,cStreet].forEach(x=>x.classList.remove('active','race','custom','street'));
  if(m==='RACE'){ cRace.classList.add('active','race'); $('modeLabel').textContent='Racing'; }
  if(m==='CUSTOM'){ cCustom.classList.add('active','custom'); $('modeLabel').textContent='Custom'; }
  if(m==='STREET'){ cStreet.classList.add('active','street'); $('modeLabel').textContent='Street'; }
  applyModeRange(); updateRangeStyle(); updatePresetUI();
}

function applyModeRange(){
  const r = $('range');
  if(currentMode==='RACE'){ r.min=20; r.max=40; r.value = Math.min(40, Math.max(20, Number(r.value))); }
  if(currentMode==='CUSTOM'){ r.min=5; r.max=150; r.value = Math.min(150, Math.max(5, Number(r.value))); }
  if(currentMode==='STREET'){ r.min=75; r.max=75; r.value=75; previewCut(75); applyCut(75); }
}

function modeColor(){
  if(currentMode==='CUSTOM') return 'var(--custom)';
  if(currentMode==='STREET') return 'var(--street)';
  return 'var(--racing)';
}

function updateRangeStyle(){
  const slider = $('range');
  const min = Number(slider.min);
  const max = Number(slider.max);
  const val = Number(slider.value);
  let pct = 100;
  if (max > min) pct = ((val - min) / (max - min)) * 100;
  const color = modeColor();
  const rest = 'rgba(255,255,255,.12)';
  slider.style.background = `linear-gradient(90deg, ${color} 0%, ${color} ${pct}%, ${rest} ${pct}%, ${rest} 100%)`;
}

function previewCut(v){
  $('cutMid').textContent = v;
  $('setLabel').style.display='inline';
  $('cutStat').textContent = `CURRENT: ${v}ms • RANGE: ${$('range').min}-${$('range').max}ms`;
  adjusting=true; updateRangeStyle();
}
function applyCut(v){
  $('setLabel').style.display='none';
  adjusting=false;
  fetch('/set?cut='+v).catch(()=>{});
}
function setPreset(ms, m){
  if(m==='STREET'){ setMode('STREET'); return; }
  if(m==='RACE') setMode('RACE');
  if(m==='CUSTOM') setMode('CUSTOM');
  $('range').value = ms; previewCut(ms); applyCut(ms); updatePresetUI();
}
function updatePresetUI(){
  const v = Number($('range').value);
  const p1=$('p1'), p2=$('p2'), p3=$('p3');
  [p1,p2,p3].forEach(p=>p.classList.remove('active'));
  if(v===30) p1.classList.add('active');
  if(v===50) p2.classList.add('active');
  if(v===75) p3.classList.add('active');
}

function toggle(){ fetch('/toggle').catch(()=>{}); }
function setMode(m){ fetch('/setmode?mode='+m).then(()=>{ setActiveMode(m); }).catch(()=>{}); }
function toggleTheft(){ fetch('/api/antitheft/toggle', {method:'POST'}).catch(()=>{}); }
function setSensorType(t){
  // UI: fade-out overlay subito (se presente)
  const ov = $('sensorOverlay');
  if(ov && ov.style.display !== 'none'){
    ov.classList.add('out');
    setTimeout(()=>{
      ov.style.display = 'none';
      ov.classList.remove('out');
    }, 300);
  }

  // invio al firmware
  fetch('/api/settings/sensortype', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'type=' + encodeURIComponent(t),
    cache:'no-store',
    keepalive:true
  })
  .then(()=>{ setTimeout(()=>updateState(), 250); })
  .catch(()=>{});
}


function resetSensorChoice(){
  const ok = confirm('Reset scelta sensore: alla prossima apertura ti richiede PIEZO/PUNTALINO. Continuare?');
  if(!ok) return;

  const tries = [
    {url:'/api/settings/sensortype/reset', method:'POST'},
    {url:'/api/settings/sensortype/reset', method:'GET'},
    {url:'/api/settings/sensortype?reset=1', method:'POST'},
    {url:'/api/settings/sensortype?reset=1', method:'GET'},
    {url:'/api/settings/sensortype?unset=1', method:'POST'},
    {url:'/api/settings/sensortype?unset=1', method:'GET'},
  ];

  (async ()=>{
    uiLog('SENSOR', 'Reset choice requested…');

    for(const t of tries){
      try{
        const r = await fetch(t.url, {
          method: t.method,
          cache: 'no-store',
          headers: (t.method==='POST') ? {'Content-Type':'application/x-www-form-urlencoded'} : undefined,
          body: (t.method==='POST') ? '' : undefined
        });

        const text = await r.text().catch(()=> '');
        uiLog('SENSOR', `${t.method} ${t.url} -> ${r.status} ${text ? ('| '+text.slice(0,80)) : ''}`);

        if(r.ok){
          // forza refresh state “subito”, così vedi overlay se il firmware lo imposta
          setTimeout(()=>updateState(), 250);
          return;
        }
      }catch(e){
        uiLog('SENSOR', `${t.method} ${t.url} -> NETWORK ERROR`);
      }
    }

    alert('Reset scelta sensore: endpoint non trovato nel firmware (controlla handler).');
  })();
}



function toggleLcd(){
  const wantOn = $('lcdBtn').textContent.trim() !== 'ON';
  fetch('/api/settings/lcd?en='+(wantOn?1:0), {method:'POST'}).catch(()=>{});
}

function toggleAutoMode(){
  const wantOn = $('autoModeBtn').textContent.trim() !== 'ON';
  fetch('/api/settings/automode?en='+(wantOn?1:0), {method:'POST'}).catch(()=>{});
}

function toggleLiveAi(){
  const wantOn = $('liveAiBtn').textContent.trim() !== 'ON';
  fetch('/api/settings/liveai?en='+(wantOn?1:0), {method:'POST'}).catch(()=>{});
}

function armTest(){ fetch('/api/test/arm', {method:'POST'}).catch(()=>{}); }
function pulse(ms){ fetch('/api/test/pulse?ms='+ms, {method:'POST'}).catch(()=>{}); }

function clearLogs(){ fetch('/api/logs/clear',{method:'POST'}).then(()=>updateLogs()).catch(()=>{}); }
function updateLogs(){
  fetch('/api/logs').then(r=>r.json()).then(d=>{
    const c=$('console');
    const prev = c.scrollTop;
    c.textContent = (d.items || []).join('\n');
    if(consoleAutoScroll) c.scrollTop = c.scrollHeight;
    else c.scrollTop = prev;
  }).catch(()=>{});
}

function drawChart(values, canvasId, lineColor){
  const canvas = document.getElementById(canvasId);
  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle = 'rgba(0,0,0,.15)';
  ctx.fillRect(0,0,W,H);

  if(!values || values.length < 2){
    ctx.fillStyle = 'rgba(255,255,255,.55)';
    ctx.font = '14px system-ui';
    ctx.fillText('No data yet...', 16, 30);
    return;
  }

  let mn=1e9, mx=-1e9;
  for(const v of values){ if(v<mn) mn=v; if(v>mx) mx=v; }
  if(mx-mn < 1){ mx+=1; mn-=1; }

  const left=44, right=16, top=16, bottom=26;
  const gw = W-left-right, gh = H-top-bottom;

  ctx.strokeStyle = 'rgba(255,255,255,.08)';
  ctx.lineWidth = 1;
  for(let i=0;i<=4;i++){
    const y = top + (gh*i/4);
    ctx.beginPath(); ctx.moveTo(left,y); ctx.lineTo(left+gw,y); ctx.stroke();
  }

  ctx.fillStyle = 'rgba(255,255,255,.55)';
  ctx.font = '12px ui-monospace, Menlo, Consolas, monospace';
  ctx.fillText(mx.toFixed(0), 8, top+10);
  ctx.fillText(mn.toFixed(0), 8, top+gh);

  ctx.strokeStyle = lineColor;
  ctx.lineWidth = 2;
  ctx.beginPath();
  for(let i=0;i<values.length;i++){
    const x = left + gw*(i/(values.length-1));
    const y = top + gh*(1 - (values[i]-mn)/(mx-mn));
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

function updateBatteryHistory(){
  fetch('/api/history')
    .then(r=>r.json())
    .then(d=>{ drawChart(d.values || [], 'chart', 'rgba(34,197,94,.78)'); })
    .catch(()=>{});
}

function updatePiezoHistory(){
  fetch('/api/piezo/history')
    .then(r=>r.json())
    .then(d=>{ drawChart(d.values || [], 'chartPiezo', 'rgba(124,58,237,.78)'); })
    .catch(()=>{});
}

function resetCounters(){ fetch('/api/settings/reset', {method:'POST'}).catch(()=>{}); }
function saveScale(){
  const v = Number($('scaleInput').value);
  if(!isFinite(v) || v<=0) return;
  fetch('/api/settings/scale?v='+encodeURIComponent(v.toFixed(4)), {method:'POST'}).catch(()=>{});
}
function saveCooldown(){
  const v = Number($('cooldownInput').value);
  if(!isFinite(v) || v<0) return;
  fetch('/api/settings/cooldown?ms='+encodeURIComponent(Math.round(v)), {method:'POST'}).catch(()=>{});
}
function saveSens(){
  const v = Number($('sensInput').value);
  if(!isFinite(v)) return;
  fetch('/api/settings/piezo?th='+encodeURIComponent(Math.round(v)), {method:'POST'}).catch(()=>{});
}
function autoSens(){
  if(!window.__piezoOn){
    alert("AUTO (AI) disponibile solo con sensore PIEZO.");
    return;
  }
  openAutoSens();
}

let autoSensStep = 0; // 0=idle,1=rev
let autoSensBusy = false;

function openAutoSens(){
  autoSensStep = 0;
  autoSensBusy = false;
  renderAutoSensStep();
  const ov = $('autoSensOverlay');
  if(ov){
    ov.style.display = 'flex';
    ov.classList.remove('out');
  }
}

function cancelAutoSens(){
  const ov = $('autoSensOverlay');
  if(ov) ov.style.display = 'none';
  autoSensBusy = false;
  autoSensStep = 0;
  // best-effort: cancella stato lato ESP
  fetch('/api/settings/piezo/autocal_guided?step=cancel', {method:'POST'}).catch(()=>{});
}

function renderAutoSensStep(){
  const title = $('autoSensTitle');
  const desc = $('autoSensDesc');
  const state = $('autoSensState');
  const btn = $('autoSensProceedBtn');
  if(!title || !desc || !state || !btn) return;

  btn.disabled = autoSensBusy;

  if(autoSensStep === 0){
    title.textContent = "Step 1/2 — Minimo (idle)";
    desc.innerHTML = "Tieni la moto al <b>minimo</b>. <b>NON</b> toccare la leva cambio.<br>Premi <b>PROCEDI</b> per campionare ~2.5s di vibrazioni/rumore dal sensore piezo.";
    state.textContent = autoSensBusy ? "Campionamento..." : "Pronto";
  }else{
    title.textContent = "Step 2/2 — Un po’ più alta di giri";
    desc.innerHTML = "Tieni la moto <b>leggermente più alta di giri</b> (senza toccare la leva cambio).<br>Premi <b>PROCEDI</b> per campionare ~2.5s e calcolare la sensibilità consigliata.";
    state.textContent = autoSensBusy ? "Campionamento..." : "Pronto";
  }
}

async function proceedAutoSens(){
  if(autoSensBusy) return;
  autoSensBusy = true;
  renderAutoSensStep();

  const stepArg = (autoSensStep === 0) ? 'idle' : 'rev';

  try{
    const r = await fetch('/api/settings/piezo/autocal_guided?step='+encodeURIComponent(stepArg), {
      method:'POST',
      cache:'no-store'
    });
    const d = await r.json().catch(()=>null);

    if(!d || !d.ok){
      autoSensBusy = false;
      renderAutoSensStep();
      alert("Errore auto calibrazione.");
      return;
    }

    if(autoSensStep === 0){
      // passo successivo
      autoSensStep = 1;
      autoSensBusy = false;
      renderAutoSensStep();
      return;
    }

    // step 2: finito
    autoSensBusy = false;
    renderAutoSensStep();

    const ov = $('autoSensOverlay');
    if(ov) ov.style.display = 'none';

    const th = (d.th !== undefined) ? d.th : '--';
    alert("Auto calibrazione completata.\nSoglia consigliata: " + th);

    // refresh status UI
    updateStatus();
  }catch(e){
    autoSensBusy = false;
    renderAutoSensStep();
    alert("Errore di rete");
  }
}


function factoryReset(){
  const ok = confirm('FACTORY RESET: cancella EEPROM e riavvia. Continuare?');
  if(!ok) return;

  const tries = [
    {url:'/api/settings/factoryreset', method:'POST'},
    {url:'/api/settings/factoryreset', method:'GET'},
    {url:'/api/settings/factoryreset?go=1', method:'POST'},
    {url:'/api/settings/factoryreset?go=1', method:'GET'},
    {url:'/factoryreset', method:'POST'},
    {url:'/factoryreset', method:'GET'},
  ];

  (async ()=>{
    uiLog('RESET', 'Factory reset requested…');

    for(const t of tries){
      try{
        const r = await fetch(t.url, {
          method: t.method,
          cache: 'no-store',
          keepalive: true, // IMPORTANTISSIMO: aiuta quando l’ESP rebootta subito
          headers: (t.method==='POST') ? {'Content-Type':'application/x-www-form-urlencoded'} : undefined,
          body: (t.method==='POST') ? '' : undefined
        });

        const text = await r.text().catch(()=> '');
        uiLog('RESET', `${t.method} ${t.url} -> ${r.status} ${text ? ('| '+text.slice(0,80)) : ''}`);

        if(r.ok){
          alert('Comando inviato. Se il dispositivo riavvia, il Wi-Fi cadrà per qualche secondo.');
          return;
        }
      }catch(e){
        uiLog('RESET', `${t.method} ${t.url} -> NETWORK ERROR`);
      }
    }

    alert('Factory reset: endpoint non trovato nel firmware (controlla handler).');
  })();
}


/* WiFi password UI */
function togglePw(){ const i=$('wifiPw'); i.type = (i.type === 'password') ? 'text' : 'password'; }
function loadWifiInfo(){
  fetch('/api/settings/wifi').then(r=>r.json()).then(d=>{
    $('wifiInfo').textContent = `SSID: ${d.ssid} • Password length: ${d.pwLen}`;
  }).catch(()=>{ $('wifiInfo').textContent=''; });
}
function saveWifiPw(){
  const pw = $('wifiPw').value.trim();
  if(pw.length < 8 || pw.length > 31){ alert('Password non valida. Usa 8–31 caratteri ASCII.'); return; }
  fetch('/api/settings/wifi', { method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body:'pw='+encodeURIComponent(pw) })
    .then(r=>r.json()).then(d=>{
      if(d.ok) alert('Salvato. Il dispositivo si riavvia: riconnettiti al Wi-Fi con la nuova password.');
      else alert('Errore: '+(d.error||'unknown'));
    }).catch(()=>alert('Errore di rete'));
}

/* SCOPPI toggle + Bang settings */
function toggleScoppi(){ fetch('/api/scoppi/toggle', {method:'POST'}).catch(()=>{}); }

function applyBangPreset(name){
  setBangPresetUI(name);

  let payload;
  if(name==='mild'){
    payload = {pulses:2, cut:12, pass:12, maxTotal:90};
  } else if(name==='balanced'){
    payload = {pulses:3, cut:16, pass:10, maxTotal:110};
  } else {
    payload = {pulses:5, cut:18, pass:8, maxTotal:130};
  }
  fetch('/api/settings/bang', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:`pulses=${payload.pulses}&cut=${payload.cut}&pass=${payload.pass}&max=${payload.maxTotal}`
  }).catch(()=>{});
}

function saveBangCustom(){
  const pulses = Number($('bangPulses').value);
  const cut = Number($('bangCut').value);
  const pass = Number($('bangPass').value);
  const maxT = Number($('bangMax').value);
  if(!isFinite(pulses)||!isFinite(cut)||!isFinite(pass)||!isFinite(maxT)) return;
  fetch('/api/settings/bang', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:`pulses=${Math.round(pulses)}&cut=${Math.round(cut)}&pass=${Math.round(pass)}&max=${Math.round(maxT)}`
  }).catch(()=>{});
}

function updateBangInputs(s){
  $('bangPulses').value = s.bangPulses;
  $('bangCut').value = s.bangCutMs;
  $('bangPass').value = s.bangPassMs;
  $('bangMax').value = s.bangMaxTotalMs;
  $('bangNow').textContent = `pulses=${s.bangPulses} • cut=${s.bangCutMs}ms • pass=${s.bangPassMs}ms • max=${s.bangMaxTotalMs}ms`;
}

/* Live AI warning actions */
let aiSuggestedTh = 0;
function applyAiSuggested(){
  if(!aiSuggestedTh || aiSuggestedTh <= 0) return;
  fetch('/api/settings/piezo?th='+encodeURIComponent(Math.round(aiSuggestedTh)), {method:'POST'}).catch(()=>{});
  dismissAiWarning();
}
function dismissAiWarning(){
  $('aiWarn').classList.remove('on');
  fetch('/api/ai/warning/clear', {method:'POST'}).catch(()=>{});
}

/* --- Voice control (single implementation) --- */
let SR = null;
let srActive = false;

function voiceSetDiag(msg){
  const d = $('voiceDiag');
  d.style.display = msg ? 'block' : 'none';
  d.textContent = msg || '';
}

function openVoiceOverlay(){
  $('voiceOverlay').style.display='flex';
  $('voiceHeard').textContent='…';
  $('voiceStatus').textContent='Sto ascoltando…';
  voiceSetDiag('');
}
function closeVoiceOverlay(){
  $('voiceOverlay').style.display='none';
  voiceSetDiag('');
}

function startVoice(){
  const isSecure = window.isSecureContext;
  if(!isSecure){
    openVoiceOverlay();
    $('voiceStatus').textContent = 'Microfono non disponibile in HTTP.';
    voiceSetDiag('SpeechRecognition richiede HTTPS/localhost. Su AP ESP32 in http spesso è bloccato.');
    return;
  }

  const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
  if(!SpeechRecognition){
    openVoiceOverlay();
    $('voiceStatus').textContent = 'Non supportato.';
    voiceSetDiag('Questo browser non supporta SpeechRecognition. Prova Chrome su Android/desktop.');
    return;
  }

  if(SR && srActive){
    stopVoice();
    return;
  }

  SR = new SpeechRecognition();
  SR.lang = 'it-IT';
  SR.interimResults = true;
  SR.continuous = false;

  SR.onstart = ()=>{
    srActive = true;
    openVoiceOverlay();
    $('voiceStatus').textContent = 'Sto ascoltando…';
    voiceSetDiag('');
  };

  SR.onerror = (e)=>{
    $('voiceStatus').textContent = 'Errore microfono.';
    voiceSetDiag('SpeechRecognition error: ' + (e.error || 'unknown'));
    srActive = false;
  };

  SR.onend = ()=>{
    srActive = false;
  };

  SR.onresult = (evt)=>{
    let finalText = '';
    let interim = '';
    for(let i=evt.resultIndex; i<evt.results.length; i++){
      const txt = evt.results[i][0].transcript;
      if(evt.results[i].isFinal) finalText += txt;
      else interim += txt;
    }
    const heard = (finalText || interim || '').trim();
    $('voiceHeard').textContent = heard || '…';

    if(finalText){
      $('voiceStatus').textContent = 'Comando ricevuto.';
      const ok = applyVoiceCommand(finalText);
      if(!ok) voiceSetDiag('Comando non riconosciuto dalle regole locali.');
      else {
        voiceSetDiag('');
        setTimeout(()=>closeVoiceOverlay(), 450);
      }
    }
  };

  try{
    SR.start();
  }catch(err){
    openVoiceOverlay();
    $('voiceStatus').textContent = 'Impossibile avviare.';
    voiceSetDiag(String(err));
    srActive = false;
  }
}

function stopVoice(){
  try{ if(SR) SR.stop(); }catch(e){}
  srActive = false;
  closeVoiceOverlay();
}

function applyVoiceCommand(text){
  const t = (text||'').toLowerCase().trim();

  if(t.includes('modalità race') || t.includes('mode race') || t === 'race'){
    setMode('RACE'); return true;
  }
  if(t.includes('modalità custom') || t.includes('mode custom') || t === 'custom'){
    setMode('CUSTOM'); return true;
  }
  if(t.includes('modalità street') || t.includes('mode street') || t === 'street'){
    setMode('STREET'); return true;
  }

  if(t.includes('attiva scoppi') || t.includes('scoppi on') || t.includes('bang on')){
    toggleScoppi(); return true;
  }
  if(t.includes('disattiva scoppi') || t.includes('scoppi off') || t.includes('bang off')){
    toggleScoppi(); return true;
  }

  if(t.includes('anti furto on') || t.includes('antifurto on') || t.includes('anti theft on')){
    toggleTheft(); return true;
  }
  if(t.includes('anti furto off') || t.includes('antifurto off') || t.includes('anti theft off')){
    toggleTheft(); return true;
  }

  {
    const m = t.match(/(?:cut|cut time|imposta cut)\s*([0-9]{1,3})/);
    if(m){
      const v = Math.round(Number(m[1]));
      if(isFinite(v)){ $('range').value = v; previewCut(v); applyCut(v); return true; }
    }
  }

  {
    const m = t.match(/(?:sensibilit[aà]|sensitivity)\s*([0-9]{2,4})/);
    if(m){
      const v = Math.round(Number(m[1]));
      if(isFinite(v)){ $('sensInput').value = v; saveSens(); return true; }
    }
  }
  return false;
}

/* --- State update --- */
function updateState(){
  fetch('/api/state').then(r=>r.json()).then(s=>{
    lastOk = Date.now(); setConn(true);

    $('chipFw').textContent = 'FW: ' + (s.fw || '--');
    $('chipStations').textContent = 'Clients: ' + (s.stations ?? '--');

    $('fwShow').textContent = s.fw || '--';
    $('stations').textContent = s.stations;

    // AutoMode chip
    const am = !!s.autoMode;
    $('chipAutoMode').style.display = am ? 'inline-flex' : 'none';
    if(am) $('chipAutoMode').textContent = 'AutoMode: ON';

    // LiveAI chip + settings button
    const laEn = (s.liveAiEnabled === undefined) ? true : !!s.liveAiEnabled;
    $('chipLiveAi').style.display = 'inline-flex';
    $('chipLiveAi').textContent = laEn ? 'LiveAI: ON' : 'LiveAI: OFF';
    $('liveAiShow').textContent = laEn ? 'ON' : 'OFF';

    $('liveAiBtn').textContent = laEn ? 'ON' : 'OFF';
    $('liveAiBtn').classList.toggle('on', laEn);
    $('liveAiInfo').textContent = laEn ? 'Live AI attivo.' : 'Live AI disattivato.';

    // Live AI warning: mostra SOLO se liveAiEnabled=true
    const warn = laEn && !!s.liveAiWarning;
    aiSuggestedTh = Number(s.liveAiSuggestedTh || 0);
    if(warn){
      const txt = (aiSuggestedTh>0)
        ? (`L’AI ha rilevato troppo rumore/trigger ravvicinati. Suggerito piezoTh ≈ ${aiSuggestedTh}.`)
        : (`L’AI ha rilevato troppo rumore/trigger ravvicinati. Suggerito aumentare la soglia piezo.`);
      $('aiWarnText').textContent = txt;
      $('aiApplyBtn').style.display = (aiSuggestedTh>0) ? 'inline-flex' : 'none';
      $('aiWarn').classList.add('on');
    } else {
      $('aiWarn').classList.remove('on');
    }

    $('batteryV').textContent = Number(s.batteryV).toFixed(2);
    $('uvIcon').classList.toggle('on', !!s.uv);

    $('toggleBtn').textContent = s.on ? 'ON' : 'OFF';
    $('toggleBtn').classList.toggle('on', !!s.on);

    $('theftBtn').textContent = s.antiTheft ? 'ON' : 'OFF';
    $('theftBtn').classList.toggle('on', !!s.antiTheft);
    $('theftBtn').classList.toggle('alert', !!s.antiTheft);
    $('theftWarn').classList.toggle('on', !!s.antiTheft);
    $('tabTheftWarn').classList.toggle('on', !!s.antiTheft);

    $('lcdBtn').textContent = s.lcdEnabled ? 'ON' : 'OFF';
    $('lcdBtn').classList.toggle('on', !!s.lcdEnabled);

    $('scaleNow').textContent = Number(s.scale).toFixed(4);
    if(document.activeElement !== $('scaleInput')) $('scaleInput').value = Number(s.scale).toFixed(4);

    $('cooldownNow').textContent = Number(s.cooldownMs);
    $('cooldownShow').textContent = Number(s.cooldownMs) + 'ms';
    if(document.activeElement !== $('cooldownInput')) $('cooldownInput').value = Number(s.cooldownMs);

    $('sensNow').textContent = Number(s.piezoTh);
    $('piezoShow').textContent = Number(s.piezoTh);
    if(document.activeElement !== $('sensInput')) $('sensInput').value = Number(s.piezoTh);

    // Anti Theft blocked shifts (solo antifurto)
    $('blockedShifts').textContent = Number(s.blockedAntiTheft ?? 0);

    // AutoMode UI
    const autoMode = !!s.autoMode;
    $('autoModeBtn').textContent = autoMode ? 'ON' : 'OFF';
    $('autoModeBtn').classList.toggle('on', autoMode);
    $('autoModeInfo').textContent = autoMode ? ('Decision: ' + (s.autoModeLast || '')) : '';

    // Mode
    if(s.mode && s.mode !== currentMode){ setActiveMode(s.mode); }
    else { applyModeRange(); updateRangeStyle(); }

    if(!adjusting){
      $('range').value = s.cutMs;
      $('cutMid').textContent = s.cutMs;
      $('cutStat').textContent = `CURRENT: ${s.cutMs}ms • RANGE: ${$('range').min}-${$('range').max}ms`;
      updateRangeStyle(); updatePresetUI();
    }

    $('cutTable').textContent = s.cutMs + 'ms';
    $('shifts').textContent = s.shiftCount;
    $('modeKpi').textContent = s.mode;
    $('errors').textContent = s.errors;
    $('activeHours').textContent = Number(s.activeHours).toFixed(2);

    // SCOPPI UI + warning
    $('scoppiBox').classList.toggle('on', !!s.scoppi);
    $('scoppiWarn').classList.toggle('on', !!s.scoppi);
    $('scoppiShow').textContent = s.scoppi ? 'ON' : 'OFF';

    // ===== Advanced calibrating: carica UNA SOLA VOLTA =====
   if(!advLoadedOnce){
     $('advMinAbove').value    = Number(s.advMinAboveMs);
     $('advMinSlope').value    = Number(s.advMinSlope);
     $('advEnvAlpha').value    = Number(s.advEnvAlpha);
     $('advBaseAlpha').value   = Number(s.advBaseAlpha);
     $('advHoldoff').value     = Number(s.advHoldoffMs);
     $('advNoisyLevel').value  = Number(s.advNoisyLevel);
     $('advNoisyHoldoff').value= Number(s.advNoisyHoldoffMs);
     advLoadedOnce = true;
    }
    // ===== Shift sensor type UI =====
   const st = Number(s.sensorType ?? 0);
   const piezoOn = (st === 0);
   window.__piezoOn = piezoOn;
   applyPiezoOnlyVisibility(piezoOn);
   const unset = !!s.sensorTypeUnset;

   if($('sensorPiezoBtn') && $('sensorPuntBtn')){
    $('sensorPiezoBtn').classList.toggle('on', piezoOn);
    $('sensorPuntBtn').classList.toggle('on', !piezoOn);
    $('sensorTypeInfo').textContent = unset
      ? 'Scelta non impostata: seleziona un sensore.'
      : ('Attivo: ' + (piezoOn ? 'PIEZO' : 'PUNTALINO'));
    }

   // overlay first setup
   const ov = $('sensorOverlay');
   if(ov){
   if(unset){
     ov.style.display = 'flex';
     ov.classList.remove('out'); // assicurati che sia visibile
   }else{
     ov.style.display = 'none';
    }
   }



    updateBangInputs(s);
  }).catch(()=>{
    if(Date.now()-lastOk > 2000) setConn(false);
  });
}

function updateLogsAndHistory(){
  updateLogs();
  updateBatteryHistory();
  if(window.__piezoOn) updatePiezoHistory();
}

showPage('control');
setActiveMode('RACE');
setBangPresetUI('mild');

updateState(); updateLogsAndHistory(); loadWifiInfo();
setInterval(()=>{ updateState(); updateLogs(); }, 900);
setInterval(()=>{ updateBatteryHistory(); updatePiezoHistory(); }, 2000);
</script>

<!-- VOICE OVERLAY -->
<div id="voiceOverlay" style="position:fixed;inset:0;display:none;align-items:center;justify-content:center;z-index:9998;
  background: radial-gradient(900px 520px at 50% 20%, rgba(255,255,255,.10), transparent 58%),
              linear-gradient(180deg, rgba(0,0,0,.92), rgba(0,0,0,.78));
  backdrop-filter: blur(10px);">
  <div style="width:min(520px,92vw);border-radius:22px;border:1px solid rgba(255,255,255,.12);
    background: linear-gradient(180deg, rgba(255,255,255,.08), rgba(255,255,255,.03));
    box-shadow:0 22px 60px rgba(0,0,0,.55); padding:18px;">
    <div style="display:flex;align-items:center;justify-content:space-between;gap:12px;">
      <div>
        <div style="font-weight:1000;letter-spacing:.12em;text-transform:uppercase;font-size:12px;color:rgba(255,255,255,.78)">Voice control</div>
        <div id="voiceStatus" style="margin-top:6px;font-size:14px;color:rgba(255,255,255,.85)">Sto ascoltando…</div>
      </div>
      <button class="btn danger" onclick="stopVoice()" style="animation:none;">CLOSE</button>
    </div>

    <div style="margin-top:14px;display:flex;align-items:center;gap:14px;">
      <div style="width:58px;height:58px;border-radius:18px;border:1px solid rgba(255,255,255,.14);
        background: rgba(124,58,237,.18); box-shadow:0 0 34px rgba(124,58,237,.22);
        position:relative; overflow:hidden;">
        <div style="position:absolute;inset:-40px;background: radial-gradient(circle at 50% 50%, rgba(255,255,255,.30), transparent 55%);
          animation: vp 1.1s ease-in-out infinite;"></div>
      </div>
      <div style="flex:1;">
        <div id="voiceHeard" style="font-weight:1000;font-size:16px;min-height:22px">…</div>
        <div style="margin-top:6px;color:rgba(255,255,255,.55);font-size:12px;line-height:1.25">
          Esempi: “imposta modalità race”, “cut 20”, “attiva scoppi”, “anti furto on”, “sensibilità 320”.
        </div>
        <div id="voiceDiag" style="margin-top:8px;color:rgba(239,68,68,.85);font-size:12px;display:none"></div>
      </div>
    </div>
  </div>
</div>

<style>
@keyframes vp{0%{transform:translateY(0);opacity:.7}50%{transform:translateY(22px);opacity:.25}100%{transform:translateY(0);opacity:.7}}
</style>

<!-- MIC BUTTON (single) -->
<button class="btn"
  style="position:fixed;right:16px;bottom:16px;width:56px;height:56px;border-radius:18px;padding:0;
  display:flex;align-items:center;justify-content:center;z-index:9997;"
  onclick="startVoice()"
  aria-label="Voice">
  <svg viewBox="0 0 24 24" width="22" height="22">
    <path fill="rgba(255,255,255,.92)" d="M12 14a3 3 0 0 0 3-3V6a3 3 0 0 0-6 0v5a3 3 0 0 0 3 3zm5-3a5 5 0 0 1-10 0H5a7 7 0 0 0 6 6.92V21h2v-3.08A7 7 0 0 0 19 11h-2z"/>
  </svg>
</button>

<div id="autoSensOverlay" style="position:fixed; inset:0; display:none; align-items:center; justify-content:center; z-index:9995;
  background: radial-gradient(900px 520px at 50% 20%, rgba(255,255,255,.10), transparent 58%),
              linear-gradient(180deg, rgba(0,0,0,.92), rgba(0,0,0,.78));
  backdrop-filter: blur(10px);">
  <div style="width:min(540px,92vw);border-radius:22px;border:1px solid rgba(255,255,255,.12);
    background: linear-gradient(180deg, rgba(255,255,255,.08), rgba(255,255,255,.03));
    box-shadow:0 22px 60px rgba(0,0,0,.55); padding:18px; text-align:left;">
    <div style="display:flex;align-items:flex-start;justify-content:space-between;gap:12px">
      <div>
        <div style="font-weight:1000;letter-spacing:.12em;text-transform:uppercase;font-size:12px;color:rgba(255,255,255,.78)">
          auto calibrazione piezo
        </div>
        <div id="autoSensTitle" style="margin-top:8px;font-size:18px;font-weight:1000">Step 1/2 — Minimo (idle)</div>
        <div id="autoSensDesc" style="margin-top:8px;color:rgba(255,255,255,.78);line-height:1.35">
          Tieni la moto al <b>minimo</b>. <b>NON</b> toccare la leva cambio.
          Premi <b>PROCEDI</b> per campionare ~2.5s di vibrazioni/rumore dal sensore piezo.
        </div>
      </div>
      <div style="min-width:96px;text-align:right">
        <div id="autoSensState" style="font-size:12px;color:rgba(255,255,255,.65)">Pronto</div>
      </div>
    </div>

    <div class="hr" style="margin-top:14px"></div>

    <div style="display:flex;gap:10px;justify-content:flex-end;flex-wrap:wrap">
      <button class="btn" onclick="cancelAutoSens()">ANNULLA</button>
      <button class="btn" id="autoSensProceedBtn" onclick="proceedAutoSens()">PROCEDI</button>
    </div>
  </div>
</div>
<div id="sensorOverlay" style="position:fixed; inset:0; display:none; align-items:center; justify-content:center; z-index:9996;
  background: radial-gradient(900px 520px at 50% 20%, rgba(255,255,255,.10), transparent 58%),
              linear-gradient(180deg, rgba(0,0,0,.92), rgba(0,0,0,.78));
  backdrop-filter: blur(10px);">
  <div style="width:min(520px,92vw);border-radius:22px;border:1px solid rgba(255,255,255,.12);
    background: linear-gradient(180deg, rgba(255,255,255,.08), rgba(255,255,255,.03));
    box-shadow:0 22px 60px rgba(0,0,0,.55); padding:18px; text-align:center;">
    <div style="font-weight:1000;letter-spacing:.12em;text-transform:uppercase;font-size:12px;color:rgba(255,255,255,.78)">
      first setup
    </div>
    <div style="margin-top:10px;font-size:18px;font-weight:1000">
      Seleziona il sensore cambio
    </div>
    <div style="margin-top:8px;color:rgba(255,255,255,.60);font-size:12px;line-height:1.3">
      Questa scelta viene salvata e usata ad ogni avvio.
    </div>

    <div style="display:flex; gap:10px; justify-content:center; flex-wrap:wrap; margin-top:14px">
      <button class="btn" onclick="setSensorType('piezo')">PIEZO</button>
      <button class="btn" onclick="setSensorType('puntalino')">PUNTALINO</button>
    </div>
  </div>
</div>

</body>
</html>
)rawliteral";
}




// ---------------- BANG runtime helpers ----------------
void bangStart(unsigned long now) {
  bangActive = true;
  bangIsCut = true;
  bangStageStartMs = now;
  bangPulseCount = 0;
  digitalWrite(mosfetPin, LOW);  // start CUT
}

void bangStop() {
  bangActive = false;
  digitalWrite(mosfetPin, HIGH);
}

void bangTick(unsigned long now) {
  unsigned long totalWindow = (unsigned long)min((int)cutTime, (int)bangMaxTotalMs);
  if (now - shiftStartTime >= totalWindow) {
    bangStop();
    shiftActive = false;
    addLog("SHIFT", "SCOPPI done");
    return;
  }

  unsigned long elapsed = now - bangStageStartMs;
  uint16_t dur = bangIsCut ? bangCutMs : bangPassMs;

  if (elapsed >= dur) {
    bangStageStartMs = now;
    bangIsCut = !bangIsCut;

    if (bangIsCut) {
      bangPulseCount++;
      if (bangPulseCount >= bangPulses) {
        bangIsCut = false;
        digitalWrite(mosfetPin, HIGH);
        return;
      }
      digitalWrite(mosfetPin, LOW);
    } else {
      digitalWrite(mosfetPin, HIGH);
    }
  }
}

// ---------------- LED update ----------------
void pulseLedAcceptedShift(unsigned long now) {
  // Inverte lo stato base (blink visibile anche se LED già ON)
  ledInvertUntilMs = now + LED_PULSE_MS;
}

// Ritorna true se sta gestendo lei il LED (quindi updateStatusLed deve fare "return")
bool aiHandleWarningLed(unsigned long now) {
  if (!liveAiEnabled || !liveAiWarning) {
    aiLedCycleStartMs = 0;
    aiLedBlinkCount = 0;
    aiLedLastToggleMs = 0;
    aiLedState = false;
    return false;
  }

  if (aiLedCycleStartMs == 0) aiLedCycleStartMs = now;

  if (now - aiLedCycleStartMs >= LIVEAI_LED_PERIOD_MS) {
    aiLedCycleStartMs = now;
    aiLedBlinkCount = 0;
    aiLedState = false;
    aiLedLastToggleMs = 0;
  }

  if (aiLedBlinkCount < LIVEAI_LED_BLINKS) {
    if (aiLedLastToggleMs == 0 || (now - aiLedLastToggleMs) >= LIVEAI_LED_TOGGLE_MS) {
      aiLedLastToggleMs = now;
      aiLedState = !aiLedState;
      digitalWrite(ledPin, aiLedState ? HIGH : LOW);
      if (!aiLedState) aiLedBlinkCount++;
    }
    return true;
  }

  digitalWrite(ledPin, HIGH); // oppure LOW fisso, come preferisci
  return true;
}


void updateStatusLed() {
  unsigned long now = millis();
    // PRIORITÀ ASSOLUTA: warning live AI
  if (aiHandleWarningLed(millis())) return;



  // =======================
  // 0) LIVE AI WARNING LED (massima priorità)
  // Pattern: LIVEAI_LED_BLINKS lampeggi ogni LIVEAI_LED_PERIOD_MS
  // =======================


  // =======================
  // 2) Stato base LED (antiTheft / system ON)
  // =======================
  bool base = false;

  if (antiTheftEnabled) {
    if (antiTheftCycleStartMs == 0) antiTheftCycleStartMs = now;
    unsigned long t = (now - antiTheftCycleStartMs) % ANTITHEFT_PERIOD_MS;

    if (t < ANTITHEFT_BLINK_WINDOW_MS) {
      if (now - ledBlinkLastToggleMs >= LED_BLINK_PERIOD_MS) {
        ledBlinkLastToggleMs = now;
        ledBlinkState = !ledBlinkState;
      }
      base = ledBlinkState;
    } else {
      base = false;
    }
  } else {
    if (isActivated) {
      if (now < ledOnTransitionUntilMs) {
        if (now - ledBlinkLastToggleMs >= LED_BLINK_PERIOD_MS) {
          ledBlinkLastToggleMs = now;
          ledBlinkState = !ledBlinkState;
        }
        base = ledBlinkState;
      } else {
        base = true;
      }
    } else {
      base = false;
    }
  }

  // =======================
  // 3) AI blink extra (se lo usi) - solo se non c'è warning (già gestito sopra)
  // =======================
  if (aiBlinkActive) {
    digitalWrite(ledPin, aiBlinkState ? HIGH : LOW);
    return;
  }

  // =======================
  // 4) Overlay pulse (inverti per LED_PULSE_MS)
  // =======================
  if (now < ledInvertUntilMs) {
    digitalWrite(ledPin, base ? LOW : HIGH);
  } else {
    digitalWrite(ledPin, base ? HIGH : LOW);
  }
}

// ================= AI LIVE CONTROL EVALUATION =================
void aiEvaluateLiveControl(unsigned long now) {
  if (!autoModeEnabled) return;

  // Se sei fermo / non ci sono cambiate recenti: NON cambiare nulla
  if (lastShiftTriggerMs != 0 && (now - lastShiftTriggerMs) > AUTOMODE_IDLE_FREEZE_MS) {
    autoModeLastDecision = "IDLE_FREEZE";
    return;
  }

  // Warm-up: tempo minimo + cambiate minime
  if (autoModeEnabledSinceMs == 0) autoModeEnabledSinceMs = now;

  if ((now - autoModeEnabledSinceMs) < AUTOMODE_MIN_TIME_MS) {
    autoModeLastDecision = "WARMUP_TIME";
    return;
  }
  if (autoModeShiftSamples < AUTOMODE_MIN_SHIFTS) {
    autoModeLastDecision = "WARMUP_SHIFTS " + String(autoModeShiftSamples) + "/" + String(AUTOMODE_MIN_SHIFTS);
    return;
  }

  // Rate limit decisioni
  if (autoModeLastDecisionMs != 0 && (now - autoModeLastDecisionMs) < AUTOMODE_DECISION_PERIOD_MS) {
    return;
  }
  autoModeLastDecisionMs = now;

  // --- QUI sotto ci metti la tua logica di scelta (race/custom/street) ---
  // MA: non usare "no activity => street". Se non hai dati => return.

  // Esempio minimo (placeholder): non cambiare mai a STREET automaticamente
  // finché non hai una metrica "aggressiva" reale.
  // Scegli targetMode e targetCutMs basandoti su rumore/burst ecc.
  Mode targetMode = currentMode;
  int targetCut = cutTime;

  // ... tua logica ...

  // Ultima riga: applica solo se cambia davvero (anti-spam)



  // reset window
  aiWindowStartMs = 0;
  aiShiftCounter = 0;
  aiPiezoEnvSum = 0;
  aiPiezoSamples = 0;
}

// ---------------- Shift trigger core ----------------
void handleShiftTrigger(unsigned long now, const String& src) {
  addLog("INPUT", "SHIFT trigger (" + src + ") env=" + String(piezoEnv, 1));

  if (shiftActive) {
    blockedShifts++;
    addLog("WARN", "SHIFT ignored (already active)");
    saveAllSettings();
    return;
  }

  if (antiTheftEnabled) {
    blockedAntiTheftShifts++;
    addLog("WARN", "SHIFT blocked (AntiTheft)");
    saveAllSettings();
    return;
  }

  if (!isActivated) {
    blockedShifts++;
    addLog("WARN", "SHIFT blocked (System OFF)");
    saveAllSettings();
    return;
  }

  if (now - lastShiftTriggerMs < (unsigned long)shiftCooldownMs) {
    blockedShifts++;
    addLog("WARN", "SHIFT blocked (cooldown)");
    saveAllSettings();
    return;
  }

  // OK: execute shift (QUI lampeggio LED)
  pulseLedAcceptedShift(now);

  lastShiftTriggerMs = now;
  shiftActive = true;
  shiftStartTime = now;
  showShiftMessage = true;
  shiftMessageStart = now;

  shiftCount++;
  liveAiOnAcceptedShift(now);

  if (autoModeEnabled) {
    autoModeShiftSamples++;
  }

  // AI Live Control: count shifts
  if (aiWindowStartMs == 0) aiWindowStartMs = now;
  aiShiftCounter++;

  bool uvNow = (batteryFiltered > 0.1f && batteryFiltered < UNDER_VOLTAGE_TH);

  if (scoppiEnabled && !uvNow) {
    addLog("SHIFT", "SCOPPI start");
    bangStart(now);
  } else {
    bangActive = false;
    addLog("SHIFT", "NORMAL cut mode=" + modeToStr(currentMode) + " cut=" + String(cutTime) + "ms");
    digitalWrite(mosfetPin, LOW);
  }
}

void autoModeApplyIfChanged(Mode targetMode, int targetCutMs, const String& reason) {
  bool changed = false;

  if (currentMode != targetMode) {
    applyMode(targetMode);
    changed = true;
  }

  if (targetMode != MODE_STREET) {
    targetCutMs = constrain(targetCutMs, CUT_MIN, CUT_MAX);
    if (cutTime != targetCutMs) {
      applyCut(targetCutMs);
      changed = true;
    }
  }

  if (!changed) return;

  addLog("AI", "AUTO apply -> " + modeToStr(currentMode) + " cut=" + String(cutTime) + " (" + reason + ")");

  startAutoModeBlink();  // <<< QUI
}

// ================= END AI LIVE CONTROL =================

// ---------------- LIVE AI CONTROL (offline heuristic) ----------------
// NOTE: "AI" qui = euristica offline, zero internet. Obiettivo:
// - se piezoTh troppo basso => troppi trigger ravvicinati (rumore)
// - accende warning + LED pattern 5 lampeggi ogni 2 secondi
// - l'utente può "dismissare" dalla webapp e il LED torna normale subito
// ---------------- AUTO MODE AI (offline heuristic) ----------------

uint32_t autoModeLastEvalMs = 0;
uint32_t autoModeLastApplyMs = 0;

// ring buffer shifts accettati (per stimare stile guida)
static const uint8_t AI_SHIFT_BUF = 14;
uint32_t aiShiftTs[AI_SHIFT_BUF];
uint8_t aiShiftHead = 0;
uint8_t aiShiftCount = 0;

// LED 3-blink quando AI cambia

uint8_t aiBlinkTogglesLeft = 0;  // 3 blink = 6 toggles
uint32_t aiBlinkLastToggleMs = 0;
static const uint16_t AI_BLINK_PERIOD_MS = 120;

// debug/telemetry

     // default ON (come richiesto)   // soglia consigliata (se warning)
uint32_t liveAiNoisyEvents = 0;  // contatore rumore
uint32_t liveAiLastEvalMs = 0;

// Finestra di valutazione rumore: conta quante volte il sensore supera 0.75*th
// e quante SHIFT avvengono in poco tempo (qui usiamo gli eventi reali + envelope)
static const uint32_t LIVEAI_WINDOW_MS = 6000;     // 6s
static const uint16_t LIVEAI_MIN_EVENTS = 22;      // soglia eventi per warning (tarabile)
static const uint16_t LIVEAI_MIN_SHIFT_BURST = 4;  // 4 cambiate in 6s = sospetto

// LED warning pattern: 5 blink ogni 2 secondi


// tracking shift burst
uint8_t liveAiShiftBurst = 0;
uint32_t liveAiShiftBurstStartMs = 0;

void aiClearWarning() {
  autoModeBlinkActive = false;
  digitalWrite(ledPin, LOW);
  stopAutoModeBlink();
  ledInvertUntilMs = 0;  // se stavi anche invertendo per pulse evento (opzionale ma utile)


  liveAiWarning = false;
  liveAiSuggestedTh = 0;
  liveAiNoisyEvents = 0;

  // IMPORTANT: stop blink immediately
  aiLedBlinkCount = 0;
  aiLedState = false;
  aiLedLastToggleMs = 0;
  aiLedCycleStartMs = 0;
}

void aiRaiseWarning(uint16_t suggestedTh) {
  liveAiWarning = true;
  liveAiSuggestedTh = suggestedTh;

  // start blink cycle fresh
  aiLedBlinkCount = 0;
  aiLedState = false;
  aiLedLastToggleMs = millis();
  aiLedCycleStartMs = millis();
}




// Chiamala spesso nel loop: valuta se "rumore eccessivo"
void liveAiTick(uint32_t nowMs, float env, uint16_t th) {
  if (!liveAiEnabled) return;

  // evento reale con holdoff
  if (env > (float)th * LIVEAI_NOISE_FRAC) {
    if (nowMs - liveAiLastNoisyEventMs >= LIVEAI_EVENT_HOLDOFF_MS) {
      liveAiLastNoisyEventMs = nowMs;
      liveAiNoisyEvents++;
    }
  }

  if (liveAiLastEvalMs == 0) liveAiLastEvalMs = nowMs;

  if (nowMs - liveAiLastEvalMs >= LIVEAI_WINDOW_MS) {
    const bool noisy = (liveAiNoisyEvents >= LIVEAI_MIN_EVENTS);
    const bool burst = (liveAiShiftBurst >= LIVEAI_MIN_SHIFT_BURST);

    if ((noisy || burst) && !liveAiWarning) {
      uint16_t bump = 0;
      if (liveAiNoisyEvents > LIVEAI_MIN_EVENTS) bump = (uint16_t)((liveAiNoisyEvents - LIVEAI_MIN_EVENTS) * 10);
      if (bump < 40) bump = 40;

      uint16_t suggested = (uint16_t)constrain((int)th + (int)bump, 20, 2500);
      aiRaiseWarning(suggested);

      addLog("AI", "LiveAI warning: events=" + String(liveAiNoisyEvents) +
                   " burst=" + String(liveAiShiftBurst) +
                   " suggestTh=" + String(suggested));
    }

    // reset finestra
    liveAiNoisyEvents = 0;
    liveAiShiftBurst = 0;
    liveAiShiftBurstStartMs = nowMs;
    liveAiLastEvalMs = nowMs;
  }
}


// chiamala quando una SHIFT è stata ACCETTATA (non bloccata)
void liveAiOnAcceptedShift(uint32_t nowMs) {
  if (!liveAiEnabled) return;

  if (liveAiShiftBurstStartMs == 0) liveAiShiftBurstStartMs = nowMs;
  if (nowMs - liveAiShiftBurstStartMs > LIVEAI_WINDOW_MS) {
    liveAiShiftBurstStartMs = nowMs;
    liveAiShiftBurst = 0;
  }
  liveAiShiftBurst++;
}

void aiRequestBlink3() {
  // Se c'è warning liveAI attivo, non sovrascrivere il pattern warning
  if (liveAiWarning) return;

  aiBlinkActive = true;
  aiBlinkTogglesLeft = 6;  // 3 blink = ON/OFF x3
  aiBlinkLastToggleMs = millis();
  aiBlinkState = false;
}

void aiBlinkTick() {
  if (!aiBlinkActive) return;
  uint32_t now = millis();
  if (now - aiBlinkLastToggleMs >= AI_BLINK_PERIOD_MS) {
    aiBlinkLastToggleMs = now;
    aiBlinkState = !aiBlinkState;

    if (aiBlinkTogglesLeft > 0) aiBlinkTogglesLeft--;
    if (aiBlinkTogglesLeft == 0) {
      aiBlinkActive = false;
    }
  }
}

void aiTrackAcceptedShift(uint32_t now) {
  aiShiftTs[aiShiftHead] = now;
  aiShiftHead = (aiShiftHead + 1) % AI_SHIFT_BUF;
  if (aiShiftCount < AI_SHIFT_BUF) aiShiftCount++;
}

uint8_t aiCountShiftsInWindow(uint32_t now, uint32_t windowMs) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < aiShiftCount; i++) {
    int idx = (int)aiShiftHead - 1 - i;
    while (idx < 0) idx += AI_SHIFT_BUF;
    uint32_t t = aiShiftTs[idx];
    if (now - t <= windowMs) c++;
    else break;  // timestamps in ordine decrescente
  }
  return c;
}
void autoModeEvaluateAndApply(uint32_t now) {
    if (!autoModeEnabled) {
    autoModeLastDecision = "OFF";
    return;
  }
  if (autoModeEnabledSinceMs == 0) autoModeEnabledSinceMs = now;

  // warm-up: non decidere finché non fai un po' di guida reale
  if ((now - autoModeEnabledSinceMs) < AUTOMODE_MIN_TIME_MS) {
    autoModeLastDecision = "WARMUP_TIME";
    return;
  }
  if (autoModeShiftSamples < AUTOMODE_MIN_SHIFTS) {
    autoModeLastDecision = "WARMUP_SHIFTS " + String(autoModeShiftSamples) + "/" + String(AUTOMODE_MIN_SHIFTS);
    return;
  }
  if (antiTheftEnabled) return;

  // throttle: valuta ogni 4s
  if (now - autoModeLastEvalMs < 4000) return;
  autoModeLastEvalMs = now;

  // Non fare modifiche troppo frequenti (evita EEPROM spam)
  if (now - autoModeLastApplyMs < 8000) {
    // puoi comunque aggiornare la decision string
  }

  // Indicatori:
  // - shift burst ultimi 10s (stile guida)
  // - warning liveAi (se troppo rumore => preferisci STREET o CUSTOM)
  const uint32_t WIN_MS = 10000;
  uint8_t shifts10s = aiCountShiftsInWindow(now, WIN_MS);

  bool uvNow = (batteryFiltered > 0.1f && batteryFiltered < UNDER_VOLTAGE_TH);

  Mode wantMode = currentMode;
  int wantCut = cutTime;

  if (uvNow) {
    // sicurezza: street stabile
    wantMode = MODE_STREET;
    wantCut = 75;
    autoModeLastDecision = "UV->STREET";
  } else if (liveAiWarning) {
    // se rumore/sens troppo bassa: evita settaggi aggressivi
    wantMode = MODE_STREET;
    wantCut = 75;
    autoModeLastDecision = "NOISE->STREET";
  } else {
    // stile guida basato su frequenza cambi
    if (shifts10s >= 5) {
      // guida aggressiva
      wantMode = MODE_RACE;
      wantCut = 25;  // rapido ma non estremo
      autoModeLastDecision = "AGGR->RACE(25)";
    } else if (shifts10s >= 2) {
      // guida dinamica/media
      wantMode = MODE_CUSTOM;
      wantCut = 50;
      autoModeLastDecision = "MID->CUSTOM(50)";
    } else {
      // guida tranquilla / città
      wantMode = MODE_STREET;
      wantCut = 75;
      autoModeLastDecision = "CALM->STREET(75)";
    }
  }

  // Applica SOLO se cambia davvero e se non stiamo spammando
  bool changed = false;

  if (wantMode != currentMode) {
    applyMode(wantMode);
    changed = true;
  }

  // Street forza 75 già dentro applyMode, ma lo teniamo coerente
  if (wantMode == MODE_STREET) wantCut = 75;

  if (wantCut != cutTime) {
    // se STREET, cut fisso
    if (wantMode != MODE_STREET) applyCut(wantCut);
    changed = true;
  }

  if (changed) {
    autoModeLastApplyMs = now;
    aiRequestBlink3();
    addLog("AI", "AutoMode -> " + autoModeLastDecision);
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("G3D QuickShifter Serial Console ready. Type HELP");

  pinMode(btnActivate, INPUT_PULLUP);
  pinMode(btnDec, INPUT_PULLUP);
  pinMode(btnInc, INPUT_PULLUP);
  pinMode(btnShift, INPUT_PULLUP);

  pinMode(ledPin, OUTPUT);
  pinMode(mosfetPin, OUTPUT);
  pinMode(btnPuntalinoPin, INPUT_PULLUP);

  digitalWrite(ledPin, LOW);
  digitalWrite(mosfetPin, HIGH);

  lcd.init();
  EEPROM.begin(EEPROM_SIZE);
  advLoadFromEeprom();

// --- LOAD sensor type (Piezo / Puntalino) ---
uint8_t st = EEPROM.read(EEPROM_ADDR_SENSOR_TYPE_U8);
if (st == 0xFF) {
  sensorTypeUnset = true;
  shiftSensorType = SENSOR_PIEZO; // default interno, ma overlay mostrerà scelta
} else {
  sensorTypeUnset = false;
  shiftSensorType = (st == 1) ? SENSOR_PUNTALINO : SENSOR_PIEZO;
}



  // 1) LOAD LiveAI da EEPROM (DOPO begin)
  uint8_t la = EEPROM.read(EEPROM_ADDR_LIVEAI_U8);
  if (la == 0xFF) {
    liveAiEnabled = true;      // default
  } else {
    liveAiEnabled = (la == 1);
  }

  // 2) se LiveAI è OFF, spegni subito warning/LED
  if (!liveAiEnabled) {
    aiClearWarning();
  }

  // 3) registra endpoint LiveAI (NON usare server.send fuori handler)
  server.on("/api/settings/liveai", HTTP_POST, []() {
    bool en = true;
    if (server.hasArg("en")) en = (server.arg("en").toInt() == 1);

    liveAiEnabled = en;

    if (!liveAiEnabled) {
      aiClearWarning();
    }

    saveAllSettings();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
  });

  
  server.on("/api/settings/advancedcal", HTTP_POST, []() {
  // leggi e valida
  if (!server.hasArg("minAboveMs") || !server.hasArg("minSlope") ||
      !server.hasArg("envAlpha")   || !server.hasArg("baseAlpha") ||
      !server.hasArg("holdoffMs")  || !server.hasArg("noisyLevel") ||
      !server.hasArg("noisyHoldoffMs")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing_args\"}");
    return;
  }

  int   minAbove = server.arg("minAboveMs").toInt();
  float minSlope = server.arg("minSlope").toFloat();
  float envA     = server.arg("envAlpha").toFloat();
  float baseA    = server.arg("baseAlpha").toFloat();
  int   holdoff  = server.arg("holdoffMs").toInt();
  float nLevel   = server.arg("noisyLevel").toFloat();
  int   nHoldoff = server.arg("noisyHoldoffMs").toInt();

  // clamp
  minAbove = constrain(minAbove, 2, 50);
  if (minSlope < 0.0f) minSlope = 0.0f;
  if (minSlope > 50.0f) minSlope = 50.0f;

  if (envA < 0.01f) envA = 0.01f;
  if (envA > 0.50f) envA = 0.50f;

  if (baseA < 0.0001f) baseA = 0.0001f;
  if (baseA > 0.0200f) baseA = 0.0200f;

  holdoff = constrain(holdoff, 20, 800);

  if (nLevel < 1.00f) nLevel = 1.00f;
  if (nLevel > 2.00f) nLevel = 2.00f;

  nHoldoff = constrain(nHoldoff, 60, 1200);

  // apply runtime
  advMinAboveMs      = (uint16_t)minAbove;
  advMinSlope        = minSlope;
  advEnvAlpha        = envA;
  advBaseAlpha       = baseA;
  advHoldoffMs       = (uint16_t)holdoff;
  advNoisyLevel      = nLevel;
  advNoisyHoldoffMs  = (uint16_t)nHoldoff;

  // save
  advSaveToEeprom();

  addLog("INFO", "ADV saved: above=" + String(advMinAboveMs) +
                 " slope=" + String(advMinSlope,2) +
                 " envA=" + String(advEnvAlpha,3) +
                 " baseA=" + String(advBaseAlpha,4) +
                 " hold=" + String(advHoldoffMs) +
                 " noisyL=" + String(advNoisyLevel,2) +
                 " noisyHold=" + String(advNoisyHoldoffMs));

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/liveai/state", []() {
    String j = "{";
    j += "\"enabled\":" + String(liveAiEnabled ? "true" : "false") + ",";
    j += "\"warning\":" + String(liveAiWarning ? "true" : "false") + ",";
    j += "\"suggestTh\":" + String(liveAiSuggestedTh);
    j += "}";
    server.send(200, "application/json", j);
  });
  server.on("/api/settings/automode", HTTP_POST, []() {
  bool en = true;
  if (server.hasArg("en")) en = (server.arg("en").toInt() == 1);

  autoModeEnabled = en;

  if (autoModeEnabled) {
    autoModeShiftSamples = 0;
    autoModeEnabledSinceMs = millis();
    autoModeLastDecisionMs = 0;
    autoModeLastDecision = "WARMUP";
    addLog("AI", "AutoMode=ON (warmup)");
  } else {
    autoModeLastDecision = "OFF";
    addLog("AI", "AutoMode=OFF");
  }

  saveAllSettings();
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
  }); 



  uint32_t magic = eepromReadU32(EEPROM_ADDR_MAGIC_U32);
  if (magic != EEPROM_MAGIC) {
    cutTime = 30;
    currentMode = MODE_RACE;
    antiTheftEnabled = false;
    lcdEnabled = true;
    batteryScale = 4.85f;
    errorCount = 0;
    activeSeconds = 0;
    shiftCooldownMs = 500;
    blockedShifts = 0;
    shiftSensorType = SENSOR_PIEZO;
    EEPROM.write(EEPROM_ADDR_SENSOR_TYPE_U8, (uint8_t)shiftSensorType);

    scoppiEnabled = false;
    bangPulses = 3;
    bangCutMs = 18;
    bangPassMs = 10;
    bangMaxTotalMs = 120;

    piezoTh = 300;

    saveWifiPasswordToEeprom(wifiPassword);
    saveAllSettings();
  } else {
    uint8_t am = EEPROM.read(EEPROM_ADDR_AUTOMODE_U8);
    if (am == 0xFF) autoModeEnabled = false;
    else autoModeEnabled = (am == 1);
    uint8_t st = EEPROM.read(EEPROM_ADDR_SENSOR_TYPE_U8);
    if (st == 0xFF || st > 1) {
    shiftSensorType = SENSOR_PIEZO;
    sensorTypeUnset = true; // prima volta / valore mancante => mostra overlay
    } else {
      shiftSensorType = (ShiftSensorType)st;
      sensorTypeUnset = false;
  }


    uint16_t savedCut = eepromReadU16(EEPROM_ADDR_CUT_U16);
    if (savedCut >= CUT_MIN && savedCut <= CUT_MAX) cutTime = (int)savedCut;

    uint8_t savedMode = EEPROM.read(EEPROM_ADDR_MODE_U8);
    if (savedMode <= 2) currentMode = (Mode)savedMode;

    antiTheftEnabled = (EEPROM.read(EEPROM_ADDR_ANTIFT_U8) == 1);

    uint8_t savedLcd = EEPROM.read(EEPROM_ADDR_LCD_U8);
    if (savedLcd == 0xFF) lcdEnabled = true;
    else lcdEnabled = (savedLcd == 1);

    float savedScale = eepromReadFloat(EEPROM_ADDR_SCALE_F32);
    if (savedScale > 0.1f && savedScale < 50.0f) batteryScale = savedScale;

    errorCount = eepromReadU32(EEPROM_ADDR_ERR_U32);
    if (errorCount == 0xFFFFFFFFUL) errorCount = 0;

    activeSeconds = eepromReadU32(EEPROM_ADDR_ACTS_U32);
    const uint32_t TEN_YEARS_S = 10UL * 365UL * 24UL * 3600UL;
    if (activeSeconds == 0xFFFFFFFFUL || activeSeconds > TEN_YEARS_S) activeSeconds = 0;

    uint16_t cd = eepromReadU16(EEPROM_ADDR_COOLDOWN_U16);
    if (cd == 0xFFFF) cd = 500;
    cd = (uint16_t)constrain((int)cd, (int)COOLDOWN_MIN_MS, (int)COOLDOWN_MAX_MS);
    shiftCooldownMs = cd;

    blockedShifts = eepromReadU32(EEPROM_ADDR_BLOCKED_U32);
    if (blockedShifts == 0xFFFFFFFFUL) blockedShifts = 0;

    scoppiEnabled = (EEPROM.read(EEPROM_ADDR_SCOPPI_U8) == 1);
    uint8_t bp = EEPROM.read(EEPROM_ADDR_BANG_PULSES);
    if (bp == 0xFF) bp = 3;
    bangPulses = (uint8_t)constrain((int)bp, 1, 8);

    uint16_t bcut = eepromReadU16(EEPROM_ADDR_BANG_CUT_U16);
    if (bcut == 0xFFFF) bcut = 18;
    bangCutMs = (uint16_t)constrain((int)bcut, 3, 60);

    uint16_t bpass = eepromReadU16(EEPROM_ADDR_BANG_PASS_U16);
    if (bpass == 0xFFFF) bpass = 10;
    bangPassMs = (uint16_t)constrain((int)bpass, 2, 60);

    uint16_t bmax = eepromReadU16(EEPROM_ADDR_BANG_MAX_U16);
    if (bmax == 0xFFFF) bmax = 120;
    bangMaxTotalMs = (uint16_t)constrain((int)bmax, 10, 250);

    uint16_t pth = eepromReadU16(EEPROM_ADDR_PIEZO_TH_U16);
    if (pth == 0xFFFF) pth = 300;
    piezoTh = (uint16_t)constrain((int)pth, 20, 2500);

    String storedPw = loadWifiPasswordFromEeprom();
    if (storedPw.length() >= 8) wifiPassword = storedPw;

    if (currentMode == MODE_STREET) cutTime = 75;

    saveAllSettings();
  }

  lcdApplyEnabled();
  updateBattery();

  // init piezo baseline
  piezoBase = (float)analogRead(piezoPin);
  piezoEnv = 0.0f;
  lastPiezoProcMs = 0;
  lastPhSampleMs = 0;

  if (lcdEnabled) {
    lcd.clear();
    lcdPrintClean(0, 0, "Batt: " + String(batteryFiltered, 2) + "V");
    lcdPrintClean(0, 1, "Status: OFF");
  }

  addLog("INIT", "Boot " + String(FW_VERSION));
  addLog("INIT", "Mode=" + modeToStr(currentMode) + " Cut=" + String(cutTime) + "ms");
  addLog("INIT", String("AntiTheft=") + (antiTheftEnabled ? "ON" : "OFF"));
  addLog("INIT", "Cooldown=" + String(shiftCooldownMs) + "ms");
  addLog("INIT", String("SCOPPI=") + (scoppiEnabled ? "ON" : "OFF"));
  addLog("INIT", "PiezoTh=" + String(piezoTh));
  // --- AUTO MODE AI (EEPROM) ---



  // -------- WiFi AP ROBUST START --------
  WiFi.mode(WIFI_AP);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  WiFi.softAPdisconnect(true);
  delay(200);

  if (!isValidWifiPassword(wifiPassword)) {
    wifiPassword = "12345678";
    saveWifiPasswordToEeprom(wifiPassword);
    saveAllSettings();
    addLog("WIFI", "Password invalid -> reset to default");
  }

  bool apOk = WiFi.softAP(ssid, wifiPassword.c_str(), 1, 0, 4);
  if (!apOk) {
    addLog("WIFI", "softAP failed, retrying...");
    delay(300);
    apOk = WiFi.softAP(ssid, wifiPassword.c_str(), 1, 0, 4);
  }
  if (!apOk) {
    addLog("WIFI", "softAP failed -> OPEN AP fallback");
    WiFi.softAP(ssid, nullptr, 1, 0, 4);
  }

  delay(150);
  IPAddress ip = WiFi.softAPIP();
  dnsServer.start(53, "*", ip);
  addLog("WIFI", String("AP IP ") + ip.toString());
  addLog("WIFI", String("SSID=") + ssid + " pwLen=" + String(wifiPassword.length()));

  // routes

  server.on("/", []() {
    server.send(200, "text/html", webPage());
  });
  server.on("/api/settings/sensortype/reset_reboot", HTTP_POST, []() {
    addLog("WARN", "SENSOR TYPE reset+reboot");
    Serial.println("[HTTP] /api/settings/sensortype/reset_reboot");

    sensorTypeUnset = true;
    EEPROM.write(EEPROM_ADDR_SENSOR_TYPE_U8, 0xFF);
    EEPROM.commit();

    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true,\"reboot\":true}");

    rebootPending = true;
    rebootAtMs = millis() + 800;   // tempo per far uscire la risposta
  });


  server.on("/toggle", []() {
    applyActivation(!isActivated);
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/settings/sensortype", HTTP_POST, []() {
  if (!server.hasArg("type")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing type\"}");
    return;
  }

  String t = server.arg("type");
  uint8_t v;

  if (t == "piezo") v = 0;
  else if (t == "puntalino") v = 1;
  else {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"bad type\"}");
    return;
  }

  shiftSensorType = (v == 1) ? SENSOR_PUNTALINO : SENSOR_PIEZO;
  sensorTypeUnset = false;

  EEPROM.write(EEPROM_ADDR_SENSOR_TYPE_U8, v); // 0/1, NON 0xFF
  EEPROM.commit();

  server.send(200, "application/json", "{\"ok\":true}");
  });


  server.on("/api/settings/sensortype/reset", HTTP_POST, []() {
    sensorTypeUnset = true;
    EEPROM.write(EEPROM_ADDR_SENSOR_TYPE_U8, 0xFF); // forza overlay prossima apertura
    EEPROM.commit();
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/set", []() {
    if (server.hasArg("cut")) applyCut(server.arg("cut").toInt());
    server.send(200, "text/plain", "OK");
  });

  server.on("/setmode", []() {
    if (server.hasArg("mode")) applyMode(strToMode(server.arg("mode")));
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/ai/warning/clear", HTTP_POST, []() {
    aiClearWarning();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/state", []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", stateJson());
  });

  server.on("/api/logs", []() {
    handleLogs();
  });
  server.on("/api/logs/clear", HTTP_POST, []() {
    clearLogs();
  });
  server.on("/api/history", []() {
    handleHistory();
  });

  // PIEZO HISTORY:
  //  - Mantengo la route originale "nuova"
  server.on("/api/piezo/history", []() {
    handlePiezoHistory();
  });
  //  - ALIAS per la tua WebApp attuale (che chiama /api/piezo_history)
  server.on("/api/piezo_history", []() {
    handlePiezoHistory();
  });

  server.on("/api/test/arm", HTTP_POST, []() {
    armTest();
  });
  server.on("/api/test/pulse", HTTP_POST, []() {
    testPulse();
  });
  server.on("/api/settings/piezo/autocal", HTTP_POST, []() {
    addLog("AI", "AutoCal start");

    uint16_t th = autoCalibratePiezo();
    applyPiezoThreshold(th);

    addLog("AI", "AutoCal done -> th=" + String(th));

    String j = String("{\"ok\":true,\"th\":") + String(th) + "}";
    server.send(200, "application/json", j);
  });

  // Guided 2-step piezo autocal (idle -> rev)
  // step=idle|1  : campiona rumore al minimo e salva temporaneamente
  // step=rev|2   : campiona rumore a giri un po' più alti, calcola soglia consigliata, applica e chiude
  // step=cancel  : annulla (reset stato)
  server.on("/api/settings/piezo/autocal_guided", HTTP_POST, []() {
    if (shiftSensorType != SENSOR_PIEZO) {
      server.send(400, "application/json", "{\"ok\":false,\"err\":\"sensor_not_piezo\"}");
      return;
    }

    String step = server.hasArg("step") ? server.arg("step") : String("");
    step.toLowerCase();

    if (step == "cancel") {
      aiCalHasIdle = false;
      server.send(200, "application/json", "{\"ok\":true,\"canceled\":true}");
      return;
    }

    if (step == "idle" || step == "1") {
      addLog("AI", "AutoCal idle start");
      aiIdle = samplePiezoEnvStats(2500);
      aiCalHasIdle = true;

      String j = "{\"ok\":true,\"step\":\"idle\",";
      j += "\"mean\":" + String(aiIdle.mean, 2) + ",";
      j += "\"sd\":" + String(aiIdle.sd, 2) + ",";
      j += "\"mx\":" + String(aiIdle.mx, 2) + "}";
      server.send(200, "application/json", j);
      return;
    }

    if (step == "rev" || step == "2") {
      if (!aiCalHasIdle) {
        server.send(400, "application/json", "{\"ok\":false,\"err\":\"missing_idle\"}");
        return;
      }

      addLog("AI", "AutoCal rev start");
      PiezoStats rev = samplePiezoEnvStats(2500);

      uint16_t th = computeGuidedThreshold(aiIdle, rev);
      applyPiezoThreshold(th);

      aiCalHasIdle = false;

      String j = "{\"ok\":true,\"step\":\"rev\",\"th\":" + String(th);
      j += ",\"idle\":{\"mean\":" + String(aiIdle.mean, 2) + ",\"sd\":" + String(aiIdle.sd, 2) + ",\"mx\":" + String(aiIdle.mx, 2) + "}";
      j += ",\"rev\":{\"mean\":" + String(rev.mean, 2) + ",\"sd\":" + String(rev.sd, 2) + ",\"mx\":" + String(rev.mx, 2) + "}";
      j += "}";
      server.send(200, "application/json", j);
      return;
    }

    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad_step\"}");
  });


  server.on("/api/antitheft/toggle", HTTP_POST, []() {
    applyAntiTheft(!antiTheftEnabled);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/scoppi/toggle", HTTP_POST, []() {
    applyScoppi(!scoppiEnabled);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/settings/bang", HTTP_POST, []() {
    uint8_t pulses = bangPulses;
    uint16_t cutms = bangCutMs;
    uint16_t passms = bangPassMs;
    uint16_t maxT = bangMaxTotalMs;

    if (server.hasArg("pulses")) pulses = (uint8_t)server.arg("pulses").toInt();
    if (server.hasArg("cut")) cutms = (uint16_t)server.arg("cut").toInt();
    if (server.hasArg("pass")) passms = (uint16_t)server.arg("pass").toInt();
    if (server.hasArg("max")) maxT = (uint16_t)server.arg("max").toInt();

    applyBangParams(pulses, cutms, passms, maxT);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/settings/reset", HTTP_POST, []() {
    resetCounters();
  });
  server.on("/api/settings/scale", HTTP_POST, []() {
    setScale();
  });
  server.on("/api/settings/cooldown", HTTP_POST, []() {
    setCooldownApi();
  });

  server.on("/api/settings/piezo", HTTP_POST, []() {
    setPiezoApi();
  });

  // auto-calibrate (single pass)
  // guided calibration (idle + rev)
  server.on("/api/settings/piezo/ai", HTTP_POST, []() {
    if (!server.hasArg("phase")) {
      server.send(400, "application/json", "{\"error\":\"missing_phase\",\"hint\":\"phase=idle|rev|reset\"}");
      return;
    }
    String phase = server.arg("phase");
    phase.toLowerCase();

    if (phase == "reset") {
      aiCalHasIdle = false;
      server.send(200, "application/json", "{\"ok\":true,\"reset\":true}");
      return;
    }

    if (phase == "idle") {
      PiezoStats st = samplePiezoEnvStats(2500);
      aiIdle = st;
      aiCalHasIdle = true;

      String j = "{";
      j += "\"ok\":true,\"phase\":\"idle\"";
      j += ",\"mean\":" + String(st.mean, 2);
      j += ",\"sd\":" + String(st.sd, 2);
      j += ",\"max\":" + String(st.mx, 2);
      j += ",\"n\":" + String(st.n);
      j += "}";
      server.send(200, "application/json", j);
      return;
    }

    if (phase == "rev") {
      if (!aiCalHasIdle) {
        server.send(409, "application/json", "{\"error\":\"need_idle_first\"}");
        return;
      }
      PiezoStats rev = samplePiezoEnvStats(2500);
      uint16_t th = computeGuidedThreshold(aiIdle, rev);
      applyPiezoThreshold(th);

      String j = "{";
      j += "\"ok\":true,\"phase\":\"rev\"";
      j += ",\"idleMean\":" + String(aiIdle.mean, 2);
      j += ",\"idleSd\":" + String(aiIdle.sd, 2);
      j += ",\"revMean\":" + String(rev.mean, 2);
      j += ",\"revSd\":" + String(rev.sd, 2);
      j += ",\"revMax\":" + String(rev.mx, 2);
      j += ",\"th\":" + String(th);
      j += "}";
      server.send(200, "application/json", j);
      return;
    }

    server.send(400, "application/json", "{\"error\":\"bad_phase\",\"hint\":\"idle|rev|reset\"}");
  });

  server.on("/api/settings/lcd", HTTP_POST, []() {
    bool en = true;
    if (server.hasArg("en")) en = (server.arg("en").toInt() == 1);
    applyLcdEnabled(en);
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/api/settings/wifi", HTTP_GET, []() {
    String j = "{";
    j += "\"ssid\":\"" + String(ssid) + "\",";
    j += "\"pwLen\":" + String(wifiPassword.length());
    j += "}";
    server.send(200, "application/json", j);
  });

  server.on("/api/settings/wifi", HTTP_POST, []() {
    if (!server.hasArg("pw")) {
      server.send(400, "application/json", "{\"error\":\"missing_pw\"}");
      return;
    }
    String pw = server.arg("pw");
    pw.trim();

    if (!isValidWifiPassword(pw)) {
      server.send(400, "application/json", "{\"error\":\"invalid_pw\",\"hint\":\"8-31 ASCII chars\"}");
      return;
    }

    wifiPassword = pw;
    saveWifiPasswordToEeprom(wifiPassword);
    saveAllSettings();

    server.send(200, "application/json", "{\"ok\":true,\"reboot\":true}");
    delay(250);
    ESP.restart();
  });

  server.on("/api/settings/factoryreset", HTTP_POST, []() {
    // fai reset dei dati QUI (senza riavviare)
    factoryReset(); // questa versione NON deve chiamare ESP.restart()

    rebootPending = true;
    rebootAtMs = millis() + 700;

    server.send(200, "application/json", "{\"ok\":true,\"reboot\":true}");
  });


  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });

  server.begin();

  // enforce antifurto at boot
  if (antiTheftEnabled) {
    digitalWrite(mosfetPin, LOW);
    isActivated = false;
    shiftActive = false;
    bangActive = false;
    antiTheftCycleStartMs = millis();
  } else {
    digitalWrite(mosfetPin, HIGH);
  }
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long now = millis();

  dnsServer.processNextRequest();
  server.handleClient();
  // ---------- SERIAL RX ----------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue; // ignore CR
    if (c == '\n') {
      String line = serialLine;
      serialLine = "";
      serialHandleCommandLine(line);
    } else {
      if (serialLine.length() < SERIAL_LINE_MAX) serialLine += c;
    }
  }

  if (rebootPending && (millis() - rebootAtMs) < 0x80000000UL) { // safe wrap not needed; semplice ok
  if (millis() >= rebootAtMs) {
    rebootPending = false;
    ESP.restart();
  }
 }


  updateStatusLed();
  aiBlinkTick();
  autoModeEvaluateAndApply(now);


  // piezo processing always on (for graph + stable trigger)
  updatePiezoProcessing(now);
  liveAiTick(now, piezoEnv, piezoTh);


  static unsigned long lastBatt = 0;
  if (now - lastBatt >= 50) {
    lastBatt = now;
    updateBattery();
  }

  updateActiveTime();
  //aiEvaluateLiveControl(now);


  // Anti theft hard lock
  if (antiTheftEnabled) {
    digitalWrite(mosfetPin, LOW);
    isActivated = false;
    shiftActive = false;
    bangActive = false;
  }

  // ---- Bottone attivazione MOMENTARY ----
  bool pressed = (digitalRead(btnActivate) == LOW);
  if (pressed != isActivated) {
    applyActivation(pressed);
  }

  // local cut adjust buttons
  if (digitalRead(btnInc) == LOW) {
    applyCut(min(CUT_MAX, cutTime + 1));
    delay(130);
  }
  if (digitalRead(btnDec) == LOW) {
    applyCut(max(CUT_MIN, cutTime - 1));
    delay(130);
  }

  // ---- Trigger da piezo (robusto) ----
  bool trig = false;

  if (shiftSensorType == SENSOR_PIEZO) {
    trig = piezoDetect(now);
    if (trig) handleShiftTrigger(now, "PIEZO");
  } else {
    trig = buttonDetect(now);
    if (trig) handleShiftTrigger(now, "PUNTALINO");
  }


  // finish cut / bang
  if (shiftActive) {
    if (bangActive) {
      bangTick(now);
    } else {
      if (now - shiftStartTime >= (unsigned long)cutTime) {
        shiftActive = false;
        if (!antiTheftEnabled) digitalWrite(mosfetPin, HIGH);
        addLog("SHIFT", "Done");
      }
    }
  }

  // LCD SHIFT message
  if (showShiftMessage && lcdEnabled) {
    lcdPrintClean(0, 1, "SHIFT!");
    if (now - shiftMessageStart >= shiftMessageDuration) {
      showShiftMessage = false;
      lcdMode = LCD_STATUS;
      lcdTimer = now;
    }
  }

  // LCD idle screen restore
  if (lcdEnabled && lcdMode != BATTERY && (now - lcdTimer) >= 3000) {
    lcdPrintClean(0, 0, "Batt: " + String(batteryFiltered, 2) + "V");
    lcdPrintClean(0, 1, "St:" + String(isActivated ? "ON " : "OFF") + " " + modeToStr(currentMode));
    lcdMode = BATTERY;
  }
}
