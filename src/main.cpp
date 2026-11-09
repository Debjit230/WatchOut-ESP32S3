#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <NimBLEDevice.h>
#include <LoRa.h>
#include <RF24.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#define RAW_BUFFER_LENGTH 750      // must be defined BEFORE IRremote.hpp: long remotes (AC units) no longer overflow
#include <IRremote.hpp>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <qrcode.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#if __has_include("esp_adc/adc_oneshot.h")
  #include "esp_adc/adc_oneshot.h"
  #define HAVE_ADC_ONESHOT 1
#endif

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// ---------------- USER SETTINGS ----------------
#define WIFI_TX_POWER WIFI_POWER_11dBm
#define DISABLE_BROWNOUT 0

#define DEFAULT_LAT 23.40f
#define DEFAULT_LON 88.49f
#define DEFAULT_TZ_MIN 330
#define CLOCK_24H false
#define RGB_LED_PIN 48

#define TRACKER_SERVER     "http://demo.traccar.org:5055"
#define TRACKER_ID         "watchout01"
#define TRACKER_INTERVAL_S 15

// Tracker options
#define TRACKER_LORA_BEACON 0      // 1 = also broadcast position over LoRa (433 MHz) so another LoRa node can follow you
#define TRACKER_BEACON_S    30

// Custom QR options
#define QR_AP_SSID   "WatchOut-QR"   // hotspot used when no Wi-Fi is available
#define QR_AP_PASS   "watchout123"   // min 8 characters
#define QR_MAX_TEXT  367

// Information QR options
#define QR_OWNER_NAME   "Owner"
#define QR_OWNER_PHONE  ""
#define QR_MUSIC_LINK   "https://youtu.be/Iv7ls3mzPOU?si=00hSNs1WDqglteju"   // <-- put your favourite music / playlist link here
#define QR_SECRET_TEXT  "Hello Cuties😘"

// Animation / display tuning
#define LCD_SPI_HZ   40000000        // try 80000000 for even faster frames if your wiring is short and clean
#define FRAME_MS     16              // ~60 fps target (real rate is limited by the SPI transfer time)
#define SCROLL_K     420.0f          // spring stiffness for list scrolling (higher = snappier)
#define INTRO_SECS   0.30f           // slide/fade-in time when a page or sub menu opens
#define BL_FADE_RATE 0.14f           // backlight fade speed (0..1 per step)

// NRF24 spectrum analyzer tuning
#define NRF_SAMPLES      4           // RPD samples per channel per sweep (more = smoother but slower)
#define NRF_DWELL_US     140         // listen time per sample (the radio needs ~130 us to settle)
#define NRF_SLICE_US     6000        // max time spent scanning per loop pass (keeps the UI responsive)
#define NRF_ATTACK_MS    60.0f       // time constant when a bar rises
#define NRF_RELEASE_MS   380.0f      // time constant when a bar falls
#define NRF_PEAK_FALL    22.0f       // peak marker fall speed, percent per second

#define FW_VERSION "v2.5"

// ---------------- PIN MAP ----------------
#define LCD_MOSI 11
#define LCD_SCLK 13
#define LCD_MISO 12
#define LCD_CS   14
#define LCD_DC   18
#define LCD_RST  7
#define LCD_BL   6

#define JOY_X    15
#define JOY_Y    16
#define JOY_SW   17

#define GPS_RX   1
#define GPS_TX   2
#define GPS_BAUD 9600

#define LORA_CS   10
#define LORA_RST  9
#define LORA_DIO0 8
#define LORA_FREQ 433E6

#define NRF24_CE  21
#define NRF24_CSN 47

#define IR_RX_PIN 4
#define IR_TX_PIN 5

// ---------------- HARDWARE ----------------
Adafruit_ST7789 lcd(&SPI, LCD_CS, LCD_DC, LCD_RST);
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
// (Bluetooth is handled by the NimBLE block below)
RF24 nrf24(NRF24_CE, NRF24_CSN);
bool nrf24Ready = false;
bool nrf24Scanned = false;           // true once the first full sweep has finished

// ---- NRF24 spectrum analyzer state ----
#define NRF_CH        84             // channels 0..83 = 2400..2483 MHz, 1 MHz apart
#define NRF_WF_ROWS   93             // waterfall history (2 px per row)
uint8_t nrf24Activity[NRF_CH] = {};  // latest sweep, 0..100 %
float   nrfAvg[NRF_CH]   = {};       // smoothed activity (fast attack, slow release)
float   nrfPeak[NRF_CH]  = {};       // peak-hold markers
float   nrfShown[NRF_CH] = {};       // what is drawn (glides towards nrfAvg)
uint8_t nrfHits[NRF_CH]  = {};       // hits collected by the sweep in progress
uint8_t nrfWf[NRF_WF_ROWS][NRF_CH];  // waterfall history, newest row at nrfWfHead
int     nrfWfHead = 0;
uint8_t nrfScanCh = 0;               // channel the running sweep is on
uint8_t nrfPass   = 0;               // which of the NRF_SAMPLES passes is running
int     nrfCursor = -1;              // -1 = off, 0..83 = marker channel
bool    nrfHold = false;             // long press = freeze the display
uint8_t nrfView = 0;                 // 0 bars + peak hold, 1 waterfall, 2 Wi-Fi channel load, 3 identified devices
uint32_t nrfSweeps = 0;
float   nrfSweepRate = 0.0f;         // sweeps per second
unsigned long nrfLastRow = 0;
bool    nrfJamActive = false;
int     jamChannel = 76;
uint8_t jamPayload = 0xAA;
unsigned long nrfJamTimer = 0;
const uint32_t NRF_JAM_INTERVAL_MS = 10;
const uint64_t NRF_JAM_PIPE_ADDR = 0xE7E7E7E7E7ULL;

GFXcanvas16 *canvas = nullptr;
bool canvasReady = false;

// One burst transfer of the whole frame buffer (much faster than per-pixel drawing)
void flushCanvas() {
  if (!canvasReady) return;
  lcd.startWrite();
  lcd.setAddrWindow(0, 0, 240, 280);
  lcd.writePixels(canvas->getBuffer(), 240UL * 280UL, true, false);
  lcd.endWrite();
}

// ---------------- COLORS ----------------
static const uint16_t C_CYAN   = 0x07FF;
static const uint16_t C_WHITE  = 0xFFFF;
static const uint16_t C_MUTED  = 0x9CF3;
static const uint16_t C_GREEN  = 0x07E0;
static const uint16_t C_YELLOW = 0xFFE0;
static const uint16_t C_RED    = 0xF800;

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

bool lightMode = false;
bool trackerOn = false;

uint16_t bgColor() {
  return lightMode ? 0xFFFF : 0x0000;
}

uint16_t dimc(uint16_t c, uint8_t br) {
  int r = (c >> 11) & 0x1F;
  int g = (c >> 5) & 0x3F;
  int b = c & 0x1F;

  if (lightMode) {
    r = 31 - ((31 - r) * br) / 255;
    g = 63 - ((63 - g) * br) / 255;
    b = 31 - ((31 - b) * br) / 255;
  } else {
    r = (r * br) / 255;
    g = (g * br) / 255;
    b = (b * br) / 255;
  }
  return (r << 11) | (g << 5) | b;
}

uint16_t themeColor(uint16_t c) {
  if (!lightMode) return c;
  if (c == 0xFFFF) return 0x0000;

  int r = (c >> 11) & 0x1F;
  int g = ((c >> 5) & 0x3F) >> 1;
  int b = c & 0x1F;

  int luma = (r * 3 + g * 6 + b) / 10;
  const int cap = 15;

  if (luma > cap) {
    r = r * cap / luma;
    g = g * cap / luma;
    b = b * cap / luma;
  }
  return (r << 11) | (g << 6) | b;
}

// ---------------- PAGES ----------------
enum Page : uint8_t {
  MENU,
  WIFI_PAGE,
  BLE_PAGE,
  GPS_PAGE,
  LORA_PAGE,
  NRF24_PAGE,
  IR_PAGE,
  CLOCK_PAGE,
  DATE_PAGE,
  WEATHER_PAGE,
  QR_PAGE,
  VAULT_PAGE,
  CONFIG_PAGE,
  FACE_PAGE
};

Page page = MENU;

void enterPage(Page p);
void goBackToMenu();
void saveSettings();
void startLoRa();
void startNRF24();
void nrfSpectrumBegin();
void scanNRF24Spectrum();
void pollNRF24Spectrum();
bool nrfLongClick();
void buildNRF24();
void renderNRFSpectrum();
void buildWifi();
void kbPress();
void kbOpen();
void buildQR();
void renderQR();
void qrBack();
void qrNetStop();
bool vaultKbActive();
void vaultEnter();
void vaultLeave();
void buildVault();
void vaultClick();
bool vaultBack();
bool vaultLongClick();
void vaultPoll();

const char *menuItems[] = {
  "Wi-Fi", "Bluetooth", "GPS", "LoRa", "NRF24", "IR Remote",
  "Clock", "Date", "Weather", "QR Code", "Password", "Settings"
};
const int MENU_COUNT = 12;
int selected = 0;

const int LIST_CENTER_Y = 130;
const int ROW_H         = 60;
const int ICON_CX       = 38;
const int ICON_R        = 22;
const int TEXT_X        = 72;

#define MAX_LINES   49
#define LINE_CHARS  18

const int PAGE_CENTER_Y = 140;
const int PAGE_ROW_H    = 36;
const int TEXT_LEFT_X   = 12;
const int TEXT_RIGHT_X  = 228;

struct Line {
  String l;
  String r;
  uint16_t lc;
  uint16_t rc;
  bool big;
};

Line lines[MAX_LINES];
int lineCount = 0;
int pageSel = 0;
float scrollPos = 0.0f;
float scrollVel = 0.0f;          // spring velocity for smooth scrolling
float introP = 1.0f;             // 0..1 page/sub menu intro progress (1 = finished)
bool uiDirty = true;
unsigned long lastFrame = 0;
unsigned long lastAnimMs = 0;

bool buttonWasDown = false;
bool rightSwipeArmed = true;
bool leftSwipeArmed = true;
unsigned long lastNavigation = 0;
unsigned long lastDraw = 0;
unsigned long lastRightSwipe = 0;
unsigned long lastLeftSwipe = 0;

uint8_t screenBrightness = 220;
const uint8_t brightLevels[] = {40, 90, 150, 220, 255};
const int BRIGHT_COUNT = 5;
int configSelection = 3;

int oriIdx = 0;
int dimIdx = 3;
const uint16_t DIM_SECS[] = {0, 15, 30, 60, 120, 300};
const char *DIM_NAMES[]   = {"Off", "15 sec", "30 sec", "1 min", "2 min", "5 min"};
const int DIM_COUNT = 6;
const uint8_t DIM_LEVEL = 8;
const char *ORI_NAMES[] = {"Normal", "Flip"};
bool dimmed = false;
unsigned long lastActivity = 0;

// Smooth backlight (fades instead of jumping)
float blCur = 220.0f;
uint8_t blTarget = 220;

void backlightUpdate() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 12) return;
  last = now;
  float d = (float)blTarget - blCur;
  if (fabsf(d) < 0.6f) {
    if ((int)lroundf(blCur) != (int)blTarget) {
      blCur = (float)blTarget;
      analogWrite(LCD_BL, blTarget);
    }
    return;
  }
  blCur += d * BL_FADE_RATE;
  analogWrite(LCD_BL, (int)lroundf(blCur));
}

int subView = 0;
int subParentSel = 0;

// ---------------- Wi-Fi page state ----------------
// Wi-Fi sub views: 0 menu, 1 scan, 2 connect AP, 3 radar
#define MAX_WIFI_ENTRIES 20

int apStage = 0;            // connect AP: 0 network list, 1 password keyboard, 2 status page
int apState = 0;            // 0 idle, 1 connecting, 2 connected, 3 failed, 5 waiting for scan to finish
bool apHold = false;        // true = we own the Wi-Fi link, other pages must not drop it
String apPass = "";
bool apShowPass = false;
String apTargetSsid = "";
String apTryPass = "";
String apFailMsg = "";
String apNetMsg = "";
unsigned long apConnStart = 0;
int apListSel = 0;
int apIdx[MAX_WIFI_ENTRIES];
int apIdxCount = 0;
int apActRow[3] = {-1, -1, -1};
String savedSsid = "";      // last network connected through "Connect AP" (stored in flash)
String savedPass = "";
bool radarOn = false;

// ---------------- QR page state ----------------
// QR sub views: 0 menu, 1 information QR list, 2 custom QR (web page)
bool qrShowing = false;             // true = a QR code is drawn full screen
String qrCustomText = "";           // text received from the phone (stored in flash)

inline bool kbWifi() { return page == WIFI_PAGE && subView == 2 && apStage == 1; }
// Joystick keyboard is active on LORA_PAGE sub view 4 (chat) and on the Wi-Fi password screen
inline bool kbActive() { return (page == LORA_PAGE && subView == 4) || kbWifi() || vaultKbActive(); }
inline bool radarActive() { return page == WIFI_PAGE && subView == 3 && radarOn; }
inline bool qrActive() { return page == QR_PAGE && qrShowing; }
void renderKeyboard();
void kbMove(int dx, int dy);
void loraBack();
bool loraLongClick();
bool wifiLongClick();
void wifiBack();

const uint32_t NAV_MS = 120;
const uint32_t PAGE_REFRESH_MS = 1000;

struct WifiEntry {
  String ssid;
  String bssid;
  int rssi;
  int channel;
  wifi_auth_mode_t security;
  unsigned long lastSeen;
};
WifiEntry wifiList[MAX_WIFI_ENTRIES];
int wifiCount = 0;
int wifiSelectedIndex = 0;
bool wifiDetailView = false;
bool wifiStarted = false;
unsigned long lastWiFiStart = 0;
const uint32_t WIFI_RESCAN_MS = 4000;
const uint32_t WIFI_ENTRY_TTL_MS = 20000;

void wifiBeginTo(const String &ssid, const String &pass) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_TX_POWER);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid.c_str(), pass.length() ? pass.c_str() : nullptr);
}

// Uses the network saved by "Connect AP". Returns false if no network has been saved yet
// (there is no built-in Wi-Fi name/password any more).
bool wifiBegin() {
  if (savedSsid.length() == 0) return false;
  wifiBeginTo(savedSsid, savedPass);
  return true;
}

#define MAX_BLE_ENTRIES 20
struct BLEEntry {
  String name;
  String mac;
  int rssi;
  unsigned long lastSeen;
};
BLEEntry bleList[MAX_BLE_ENTRIES];
int bleCount = 0;
int bleSelectedIndex = 0;
bool bleDetailView = false;
bool bleStarted = false;
const uint32_t BLE_ENTRY_TTL_MS = 20000;

bool loraOK = false;
bool loraTried = false;
uint32_t loraPackets = 0;
int loraRSSI = 0;
String loraMessage = "Waiting for packet";
uint32_t loraTx = 0;
float loraSNR = 0;

// ---- LoRa Chat (joystick keyboard) ----
// LoRa sub views: 0 menu, 1 status, 2 chat menu, 3 saved chat, 4 custom chat (keyboard), 5 received
#define LORA_MAX_TEXT 200
#define SAVED_MAX 10
#define RX_MAX 10
#define RX_SHOW_CHARS 54

String savedMsg[SAVED_MAX];
int savedCount = 0;
int savedSentIdx = -1;
unsigned long savedSentAt = 0;

struct RxMsg {
  String text;
  int rssi;
  unsigned long at;
};
RxMsg rxLog[RX_MAX];
int rxCount = 0;
int loraUnread = 0;
int rxClearRow = -1;

int loraChatSel = 0;
String chatText = "";
String chatStatus = "";
unsigned long chatStatusAt = 0;
String vaultInput = "";
String vaultPrompt = "";
int vaultMaxLen = 63;
bool vaultMask = false;
int kbMode = 0;      // 0 abc, 1 ABC (one shot), 2 123/symbols
int kbRow = 0;
int kbCol = 0;

// ---------------- IR STATE ----------------
// IR sub views: 0 = IR menu, 1 = IR Capture, 2 = IR Sender, 3 = Custom IR
bool irCaptured = false;
bool irRawMode = false;          // true = protocol unknown, so only a RAW replay is possible
bool irSendRaw = true;           // IR Sender method: true = RAW replay (works for every remote), false = DECODED
IRData storedIRData;
uint16_t rawIRCode[RAW_BUFFER_LENGTH];   // mark/space timings in microseconds (first entry = mark)
uint16_t rawIRLength = 0;
String irProtocol = "None";
String irMessage = "Listening...";
uint16_t irAddress = 0;
uint16_t irCommand = 0;
uint16_t irBits = 0;
uint32_t irData = 0;
uint32_t irCaptures = 0;
String irSendMessage = "Ready";
uint32_t irSent = 0;
int irRowMethod = -1;            // row index of "Method" in IR Sender (set while building)

// Custom IR builder
const char *CUSTOM_PROTO_NAMES[] = {"NEC", "Samsung", "Sony", "RC5", "RC6", "LG"};
const int CUSTOM_PROTO_COUNT = 6;
int customProto = 0;
uint8_t customAddr = 0x00;
uint8_t customCmd = 0x00;
int customRepeats = 0;
String customMessage = "Ready";
uint32_t customSent = 0;

enum LedMode : uint8_t {
  LED_OFF, LED_AUTO, LED_SOLID, LED_RAINBOW,
  LED_BLINK, LED_BREATHE, LED_MBLINK, LED_MBREATHE
};
const int LED_MODE_COUNT = 8;
const char *LED_MODE_NAMES[] = {
  "Off", "Auto", "Solid", "Rainbow",
  "Blink", "Breathe", "M-Blink", "M-Breathe"
};

struct LedColor {
  const char *name;
  uint8_t r, g, b;
};
const LedColor LED_COLORS[] = {
  {"Red",     255,   0,   0},
  {"Green",     0, 255,   0},
  {"Blue",      0,   0, 255},
  {"Yellow",  255, 200,   0},
  {"Cyan",      0, 255, 255},
  {"Magenta", 255,   0, 255},
  {"White",   255, 255, 255},
  {"Orange",  255,  90,   0},
  {"Purple",  140,   0, 255}
};
const int LED_COLOR_COUNT = 9;
const uint16_t LED_PERIODS[] = {3000, 2000, 1200, 700, 350};
const uint8_t  LED_LEVELS[]  = {10, 40, 90, 160, 255};

uint8_t ledMode = LED_AUTO;
int ledColorIdx = 1;
int ledSpeedIdx = 2;
int ledLevelIdx = 2;
unsigned long ledFlashUntil = 0;
uint8_t flashR = 0, flashG = 0, flashB = 0;
unsigned long lastLedUpdate = 0;

void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t lv = LED_LEVELS[ledLevelIdx];
  rgbLedWrite(RGB_LED_PIN, (r * lv) / 255, (g * lv) / 255, (b * lv) / 255);
}

void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
  flashR = r;
  flashG = g;
  flashB = b;
  ledFlashUntil = millis() + ms;
}

void addLine(const String &l, const String &r, uint16_t lc, uint16_t rc) {
  if (lineCount >= MAX_LINES) return;
  lines[lineCount].l = l;
  lines[lineCount].r = r;
  lines[lineCount].lc = lc;
  lines[lineCount].rc = rc;
  lines[lineCount].big = false;
  lineCount++;
}

void addBigLine(const String &s, uint16_t c) {
  if (lineCount >= MAX_LINES) return;
  lines[lineCount].l = s;
  lines[lineCount].r = "";
  lines[lineCount].lc = c;
  lines[lineCount].rc = c;
  lines[lineCount].big = true;
  lineCount++;
}

void addWrapped(const String &s, uint16_t c) {
  if (s.length() == 0) {
    addLine("-", "", c, c);
    return;
  }
  for (unsigned int i = 0; i < s.length(); i += LINE_CHARS) {
    addLine(s.substring(i, i + LINE_CHARS), "", c, c);
  }
}

void addKV(const String &label, const String &value, uint16_t vc) {
  if ((int)(label.length() + 1 + value.length()) <= LINE_CHARS) {
    addLine(label, value, C_MUTED, vc);
  } else {
    addLine(label, "", C_MUTED, vc);
    addWrapped(value, vc);
  }
}

void startLines() { lineCount = 0; }
void finishLines() {
  if (pageSel >= lineCount) pageSel = max(0, lineCount - 1);
  uiDirty = true;
}

String getSecurityName(wifi_auth_mode_t sec) {
  switch (sec) {
    case WIFI_AUTH_OPEN:          return "OPEN";
    case WIFI_AUTH_WEP:           return "WEP";
    case WIFI_AUTH_WPA_PSK:       return "WPA";
    case WIFI_AUTH_WPA2_PSK:      return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK:      return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    default:                      return "UNKNOWN";
  }
}

int getSignalQuality(int rssi) {
  if (rssi <= -100) return 0;
  if (rssi >= -50) return 100;
  return 2 * (rssi + 100);
}

uint16_t rssiColor(int rssi) {
  if (rssi >= -60) return C_GREEN;
  if (rssi >= -80) return C_YELLOW;
  return C_RED;
}

uint16_t appColor(int i) {
  switch (i) {
    case 0: return rgb565(10, 132, 255);
    case 1: return rgb565(88, 86, 214);
    case 2: return rgb565(48, 209, 88);
    case 3: return rgb565(255, 69, 58);
    case 4: return rgb565(255, 159, 10);
    case 5: return rgb565(191, 90, 242);
    case 6: return rgb565(50, 173, 230);
    case 7: return rgb565(255, 55, 95);
    case 8: return rgb565(255, 159, 10);
    case 9: return rgb565(90, 200, 250);     // QR Code
    case 10: return rgb565(175, 82, 222);    // Passward
    default: return rgb565(142, 142, 147);   // Settings
  }
}

void drawArc(Adafruit_GFX *g, int cx, int cy, float r, float a0, float a1, uint16_t col) {
  if (r < 2) r = 2;
  float step = 28.6f / r;
  for (float a = a0; a <= a1; a += step) {
    float rad = a * 0.0174533f;
    int x = cx + (int)lroundf(cosf(rad) * r);
    int y = cy - (int)lroundf(sinf(rad) * r);
    g->fillCircle(x, y, 1, col);
  }
}

void thickLine(Adafruit_GFX *g, int x0, int y0, int x1, int y1, uint16_t col) {
  g->drawLine(x0, y0, x1, y1, col);
  g->drawLine(x0 + 1, y0 + 1, x1 + 1, y1 + 1, col);
  g->drawLine(x0, y0 + 1, x1, y1 + 1, col);
}

void drawGlyph(Adafruit_GFX *g, int i, int cx, int cy, float k, uint8_t br, uint16_t accent) {
  const uint16_t w = dimc(C_WHITE, br);
  auto S = [&](float v) { return (int)lroundf(v * k); };

  switch (i) {
    case 0: {
      int oy = cy + S(7);
      for (int a = 0; a < 3; a++) drawArc(g, cx, oy, (6 + 5 * a) * k, 50, 130, w);
      g->fillCircle(cx, oy, max(1, S(2)), w);
      break;
    }
    case 1: {
      int px[6] = {-5, 5, 0, 0, 5, -5};
      int py[6] = {-5, 5, 10, -10, -5, 5};
      for (int s = 0; s < 5; s++) thickLine(g, cx + S(px[s]), cy + S(py[s]), cx + S(px[s + 1]), cy + S(py[s + 1]), w);
      break;
    }
    case 2: {
      int oy = cy - S(3);
      g->fillCircle(cx, oy, S(9), w);
      g->fillTriangle(cx - S(8), oy + S(3), cx + S(8), oy + S(3), cx, oy + S(17), w);
      g->fillCircle(cx, oy, S(3.5f), accent);
      break;
    }
    case 3: {
      g->fillCircle(cx, cy, max(1, S(3)), w);
      for (int a = 0; a < 2; a++) {
        float r = (8 + 5 * a) * k;
        drawArc(g, cx, cy, r, -40, 40, w);
        drawArc(g, cx, cy, r, 140, 220, w);
      }
      break;
    }
    case 4: {
      g->fillRoundRect(cx - S(10), cy - S(7), S(20), S(14), 3, w);
      g->drawLine(cx - S(5), cy - S(10), cx - S(5), cy - S(7), w);
      g->drawLine(cx + S(5), cy - S(10), cx + S(5), cy - S(7), w);
      g->fillCircle(cx - S(5), cy, max(1, S(2)), accent);
      g->fillCircle(cx + S(5), cy, max(1, S(2)), accent);
      break;
    }
    case 5: {
      g->fillRoundRect(cx - S(5), cy + S(1), S(10), S(12), 2, w);
      g->fillCircle(cx, cy + S(5), max(1, S(1.5f)), accent);
      g->fillCircle(cx, cy + S(9), max(1, S(1.5f)), accent);
      g->fillCircle(cx, cy - S(2), max(1, S(1.5f)), w);
      for (int a = 0; a < 2; a++) drawArc(g, cx, cy - S(1), (6 + 5 * a) * k, 55, 125, w);
      break;
    }
    case 6: {
      g->fillCircle(cx, cy, S(11), w);
      g->fillCircle(cx, cy, max(1, S(8)), accent);
      thickLine(g, cx, cy, cx, cy - S(6), w);
      thickLine(g, cx, cy, cx + S(4), cy + S(2), w);
      break;
    }
    case 7: {
      g->fillRoundRect(cx - S(10), cy - S(9), S(20), S(21), 3, w);
      g->fillRect(cx - S(10), cy - S(9), S(20), S(6), dimc(accent, 150));
      g->fillRect(cx - S(5), cy - S(12), max(1, S(2)), S(5), w);
      g->fillRect(cx + S(3), cy - S(12), max(1, S(2)), S(5), w);
      for (int r = 0; r < 2; r++) {
        for (int c = 0; c < 3; c++) g->fillRect(cx - S(7) + c * S(5), cy + r * S(5), max(1, S(3)), max(1, S(3)), accent);
      }
      break;
    }
    case 8: {
      int sx = cx - S(4);
      int sy = cy - S(4);
      for (int t = 0; t < 8; t++) {
        float rad = t * 45 * 0.0174533f;
        thickLine(g, sx + (int)lroundf(cosf(rad) * S(7)), sy + (int)lroundf(sinf(rad) * S(7)),
                     sx + (int)lroundf(cosf(rad) * S(10)), sy + (int)lroundf(sinf(rad) * S(10)), w);
      }
      g->fillCircle(sx, sy, S(5), w);
      auto cloud = [&](uint16_t col, int pad) {
        g->fillCircle(cx - S(1), cy + S(7), S(5) + pad, col);
        g->fillCircle(cx + S(5), cy + S(5), S(6) + pad, col);
        g->fillCircle(cx + S(10), cy + S(8), S(4) + pad, col);
        g->fillRect(cx - S(1) - pad, cy + S(8) - pad, S(11) + 2 * pad, S(4) + 2 * pad, col);
      };
      cloud(accent, 2);
      cloud(w, 0);
      break;
    }
    case 9: {                                  // QR code: 3 finder squares + dots
      int s = S(7);
      int in = max(1, s - 2 * max(1, S(2)));
      int off = max(1, S(2));
      int fx[3] = {cx - S(11), cx + S(4), cx - S(11)};
      int fy[3] = {cy - S(11), cy - S(11), cy + S(4)};
      for (int f = 0; f < 3; f++) {
        g->fillRect(fx[f], fy[f], s, s, w);
        g->fillRect(fx[f] + off, fy[f] + off, in, in, accent);
      }
      g->fillRect(cx + S(4), cy + S(4), max(1, S(3)), max(1, S(3)), w);
      g->fillRect(cx + S(8), cy + S(8), max(1, S(3)), max(1, S(3)), w);
      g->fillRect(cx + S(8), cy + S(4), max(1, S(3)), max(1, S(3)), w);
      break;
    }
    case 10: {                                 // Password: closed padlock
      g->drawRoundRect(cx - S(8), cy - S(10), S(16), S(15), S(5), w);
      g->fillRoundRect(cx - S(11), cy - S(2), S(22), S(17), S(3), w);
      g->fillCircle(cx, cy + S(4), max(1, S(2)), accent);
      g->fillRect(cx - max(1, S(1)), cy + S(5), max(1, S(2)), S(4), accent);
      break;
    }
    default: {
      for (int t = 0; t < 8; t++) {
        float rad = t * 45 * 0.0174533f;
        int x = cx + (int)lroundf(cosf(rad) * S(10));
        int y = cy + (int)lroundf(sinf(rad) * S(10));
        g->fillCircle(x, y, max(1, S(2.5f)), w);
      }
      g->fillCircle(cx, cy, S(8), w);
      g->fillCircle(cx, cy, S(3.5f), accent);
      break;
    }
  }
}

// Eased intro factor (0..1) for a row at distance d from the selected row.
// Rows closer to the selection appear first, the rest follow in a short cascade.
float introEase(float d) {
  if (introP >= 1.0f) return 1.0f;
  float p = introP * 1.5f - d * 0.15f;
  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  float q = 1.0f - p;
  return 1.0f - q * q * q;                       // ease-out cubic
}

void drawAppRow(Adafruit_GFX *g, int i, float scroll) {
  float off = (float)i - scroll;
  int cy = LIST_CENTER_Y + (int)lroundf(off * ROW_H);
  if (cy < -40 || cy > 320) return;

  float d = fabsf(off);
  float dNear = fminf(d, 1.0f);
  float k = 1.0f - 0.2f * dNear;

  float e = introEase(d);
  int xo = (int)lroundf((1.0f - e) * 36.0f);

  int brI = 255 - (int)(70.0f * d);
  if (brI < 100) brI = 100;
  uint8_t br = (uint8_t)(brI * e);

  int R = (int)lroundf(ICON_R * k);
  uint16_t accent = dimc(appColor(i), br);

  g->fillCircle(ICON_CX + xo, cy, R, accent);
  drawGlyph(g, i, ICON_CX + xo, cy, k, br, accent);

  uint16_t tc = dimc(themeColor(C_WHITE), br);
  g->setTextSize(2);
  g->setTextWrap(false);
  g->setTextColor(tc);
  g->setCursor(TEXT_X + xo, cy - 7);
  g->print(menuItems[i]);
  g->setCursor(TEXT_X + xo + 1, cy - 7);
  g->print(menuItems[i]);
}

void drawScrollBar(Adafruit_GFX *g, float frac, int thumbH) {
  const int x = 234;
  const int top = 28;
  const int trackH = 224;

  g->fillRoundRect(x, top, 3, trackH, 1, lightMode ? 0xDEFB : 0x2104);
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;

  int ty = top + (int)((trackH - thumbH) * frac);
  g->fillRoundRect(x, ty, 3, thumbH, 1, themeColor(C_MUTED));
}

void renderMenu() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  g->fillScreen(bgColor());
  for (int i = 0; i < MENU_COUNT; i++) drawAppRow(g, i, scrollPos);
  drawScrollBar(g, scrollPos / (float)(MENU_COUNT - 1), 44);
  flushCanvas();
}

void drawMenu() {
  scrollPos = (float)selected;
  scrollVel = 0.0f;
  uiDirty = false;
  renderMenu();
}

void boldText(Adafruit_GFX *g, int x, int y, const String &s, uint16_t col, uint8_t size, int boldOffset) {
  g->setTextSize(size);
  g->setTextWrap(false);
  g->setTextColor(col);
  g->setCursor(x, y);
  g->print(s);
  g->setCursor(x + boldOffset, y);
  g->print(s);
}

void drawLineRow(Adafruit_GFX *g, int i, float scroll) {
  float off = (float)i - scroll;
  int cy = PAGE_CENTER_Y + (int)lroundf(off * PAGE_ROW_H);
  if (cy < -20 || cy > 300) return;

  float d = fabsf(off);
  float e = introEase(d);
  int xo = (int)lroundf((1.0f - e) * 30.0f);

  int brI = 255 - (int)(55.0f * d);
  if (brI < 70) brI = 70;
  uint8_t br = (uint8_t)(brI * e);

  String l = lines[i].l;
  String r = lines[i].r;

  if (r.length() > LINE_CHARS - 1) r = r.substring(0, LINE_CHARS - 1);
  int maxL = LINE_CHARS - (r.length() ? (int)r.length() + 1 : 0);
  if (maxL < 1) maxL = 1;
  if ((int)l.length() > maxL) l = l.substring(0, maxL);

  if (d < 0.5f) {
    int mb = (int)((255 - (int)(d * 510.0f)) * e);
    g->fillRoundRect(3, cy - 11, 3, 22, 1, dimc(themeColor(C_CYAN), (uint8_t)mb));
  }

  if (lines[i].big) {
    int bw = (int)l.length() * 24;
    int bx = (240 - bw) / 2;
    if (bx < 8) bx = 8;
    boldText(g, bx + xo, cy - 14, l, dimc(themeColor(lines[i].lc), br), 4, 2);
    return;
  }

  boldText(g, TEXT_LEFT_X + xo, cy - 7, l, dimc(themeColor(lines[i].lc), br), 2, 1);
  if (r.length()) {
    int rx = TEXT_RIGHT_X - (int)r.length() * 12;
    boldText(g, rx + xo, cy - 7, r, dimc(themeColor(lines[i].rc), br), 2, 1);
  }
}

void renderPage() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  g->fillScreen(bgColor());
  for (int i = 0; i < lineCount; i++) drawLineRow(g, i, scrollPos);
  if (lineCount > 6) {
    int thumbH = 224 * 6 / lineCount;
    if (thumbH < 16) thumbH = 16;
    drawScrollBar(g, scrollPos / (float)(lineCount - 1), thumbH);
  }
  flushCanvas();
}

const char *WEEKDAYS[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char *MONTHS[] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};

bool timeSynced = false;
int64_t baseEpoch = 0;
unsigned long baseMillis = 0;
String timeSource = "NONE";
int tzOffsetMin = DEFAULT_TZ_MIN;
int lastClockSec = -1;

void pumpGPS() {
  while (GPSSerial.available()) gps.encode((char)GPSSerial.read());
}

long daysFromCivil(int y, int m, int d) {
  y -= (m <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097L + (long)doe - 719468L;
}

void civilFromDays(long z, int &y, int &m, int &d) {
  z += 719468;
  long era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = (int)yoe + (int)(era * 400);
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp < 10 ? mp + 3 : mp - 9);
  y += (m <= 2);
}

void syncTimeFromGPS() {
  if (!gps.time.isUpdated()) return;
  if (gps.time.isValid() && gps.date.isValid() && gps.date.year() >= 2024) {
    baseEpoch = (int64_t)daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day()) * 86400LL +
                gps.time.hour() * 3600LL + gps.time.minute() * 60LL + gps.time.second();
    baseMillis = millis();
    timeSynced = true;
    timeSource = "GPS";
  }
}

void syncTimeNTP() {
  if (timeSynced) return;
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  unsigned long t = millis();
  while (millis() - t < 5000) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      baseEpoch = (int64_t)now;
      baseMillis = millis();
      timeSynced = true;
      timeSource = "NTP";
      return;
    }
    pumpGPS();
    delay(100);
  }
}

bool getLocalDateTime(int &Y, int &Mo, int &D, int &h, int &mi, int &s, int &wd) {
  if (!timeSynced) return false;
  int64_t e = baseEpoch + (int64_t)((millis() - baseMillis) / 1000UL) + (int64_t)tzOffsetMin * 60LL;
  int64_t days = e / 86400LL;
  int64_t rem = e % 86400LL;
  if (rem < 0) { rem += 86400LL; days--; }
  civilFromDays((long)days, Y, Mo, D);
  h = (int)(rem / 3600);
  mi = (int)((rem % 3600) / 60);
  s = (int)(rem % 60);
  wd = (int)((days + 4) % 7);
  if (wd < 0) wd += 7;
  return true;
}

String tzLabel() {
  int m = abs(tzOffsetMin);
  String s = String("UTC") + (tzOffsetMin < 0 ? "-" : "+") + String(m / 60);
  if (m % 60) {
    s += ":";
    if (m % 60 < 10) s += "0";
    s += String(m % 60);
  }
  return s;
}

int wxState = 0;
int wxHttp = 0;
float wxTemp = 0, wxFeels = 0, wxHum = 0, wxWind = 0;
int wxCode = 0;

const int   FD_W = 84;
const int   FD_H = 100;
const int   FD_T = 30;
const int   FD_O = 5;
const float DEG2RAD = 0.0174533f;

static Adafruit_GFX *fg = nullptr;
static int fox = 0, foy = 0, fr = 10;
static uint16_t fcol = 0;

static inline void fStamp(float x, float y) {
  fg->fillCircle(fox + (int)lroundf(x), foy + (int)lroundf(y), fr, fcol);
}

static void fLine(float x0, float y0, float x1, float y1) {
  float dx = x1 - x0;
  float dy = y1 - y0;
  int n = (int)(sqrtf(dx * dx + dy * dy) / 2.0f) + 1;
  for (int i = 0; i <= n; i++) {
    float t = (float)i / (float)n;
    fStamp(x0 + dx * t, y0 + dy * t);
  }
}

static void fArc(float cx, float cy, float rx, float ry, float a0, float a1) {
  float span = fabsf(a1 - a0);
  float rmax = rx > ry ? rx : ry;
  int n = (int)(span * DEG2RAD * rmax / 2.0f) + 2;
  for (int i = 0; i <= n; i++) {
    float a = (a0 + (a1 - a0) * ((float)i / (float)n)) * DEG2RAD;
    fStamp(cx + cosf(a) * rx, cy - sinf(a) * ry);
  }
}

static void faceDigitPath(int d) {
  const float a  = FD_T / 2.0f;
  const float L  = a;
  const float R  = FD_W - a;
  const float T  = a;
  const float B  = FD_H - a;
  const float cx = FD_W / 2.0f;
  const float cy = FD_H / 2.0f;
  const float rx = (R - L) / 2.0f;
  const float H  = B - T;

  switch (d) {
    case 0: fArc(cx, cy, rx, H / 2.0f, 0, 360); break;
    case 1: fLine(cx - 20, T + 16, cx + 6, T + 2); fLine(cx + 6, T + 2, cx + 6, B); break;
    case 2: {
      float ry = H * 0.28f;
      float cyT = T + ry;
      fArc(cx, cyT, rx, ry, 165, -35);
      fLine(cx + cosf(-35 * DEG2RAD) * rx, cyT + sinf(35 * DEG2RAD) * ry, L, B);
      fLine(L, B, R, B);
      break;
    }
    case 3: {
      float ryt = H * 0.25f; float ryb = H * 0.27f;
      fArc(cx, T + ryt, rx * 0.92f, ryt, 160, -90);
      fArc(cx, B - ryb, rx, ryb, 90, -160);
      break;
    }
    case 4: {
      float xv = R - 14;
      fLine(xv, T, xv, B);
      fLine(xv, T, L, cy + 12);
      fLine(L, cy + 12, R, cy + 12);
      break;
    }
    case 5: {
      float ryb = (B - (cy - 8)) / 2.0f;
      float cyB = B - ryb;
      float sx = cx + cosf(150 * DEG2RAD) * rx;
      float sy = cyB - sinf(150 * DEG2RAD) * ryb;
      fLine(R - 4, T, L + 4, T);
      fLine(L + 4, T, sx, sy);
      fArc(cx, cyB, rx, ryb, 150, -150);
      break;
    }
    case 6: {
      float ryb = H * 0.30f; float cyB = B - ryb;
      fArc(cx, cyB, rx, ryb, 0, 360);
      fArc(R, cyB, R - L, cyB - T, 180, 80);
      break;
    }
    case 7: fLine(L, T, R, T); fLine(R, T, cx - 10, B); break;
    case 8: {
      float ry1 = H * 0.235f; float ry2 = H * 0.275f;
      fArc(cx, T + ry1, rx * 0.85f, ry1, 0, 360);
      fArc(cx, B - ry2, rx, ry2, 0, 360);
      break;
    }
    default: {
      float ryt = H * 0.30f; float cyT = T + ryt;
      fArc(cx, cyT, rx, ryt, 0, 360);
      fArc(L, cyT, R - L, B - cyT, 0, -100);
      break;
    }
  }
}

static void drawFaceDigit(Adafruit_GFX *g, int d, int x, int y, uint16_t col) {
  fg = g; fox = x; foy = y;
  fr = FD_T / 2 + FD_O; fcol = 0x0000; faceDigitPath(d);
  fr = FD_T / 2; fcol = col; faceDigitPath(d);
}

static void drawCalIcon(Adafruit_GFX *g, int x, int y, uint16_t c) {
  g->drawRoundRect(x, y + 2, 16, 14, 2, c);
  g->fillRect(x, y + 2, 16, 4, c);
  g->fillRect(x + 3, y, 2, 4, c);
  g->fillRect(x + 11, y, 2, 4, c);
}

static void drawSatIcon(Adafruit_GFX *g, int x, int y, uint16_t c) {
  g->fillRect(x,      y + 10, 4, 6,  c);
  g->fillRect(x + 6,  y + 5,  4, 11, c);
  g->fillRect(x + 12, y,      4, 16, c);
}

static void drawThermIcon(Adafruit_GFX *g, int x, int y, uint16_t c) {
  g->fillRoundRect(x + 3, y, 5, 12, 2, c);
  g->fillCircle(x + 5, y + 13, 4, c);
}

uint32_t lastFaceKey = 0xFFFFFFFF;
int faceSats() {
  return gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
}

void faceUpdate() {
  int Y, Mo, D, h, mi, s, wd;
  uint32_t key = 0;
  if (getLocalDateTime(Y, Mo, D, h, mi, s, wd)) {
    key = (uint32_t)(((Mo * 32 + D) * 1440) + h * 60 + mi) + 1;
  }
  key = key * 131u + (uint32_t)faceSats();
  key = key * 997u + (uint32_t)(wxState == 3 ? (int)lroundf(wxTemp) + 100 : 0);
  if (key != lastFaceKey) {
    lastFaceKey = key;
    uiDirty = true;
  }
}

void renderFace() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  g->fillScreen(0x0000);

  const uint16_t cPink   = rgb565(246, 143, 170);
  const uint16_t cBlue   = rgb565(134, 176, 236);
  const uint16_t cWhite  = rgb565(236, 236, 242);
  const uint16_t cPurple = rgb565(196, 134, 236);
  const uint16_t cInfo   = rgb565(140, 180, 255);

  int Y, Mo, D, h, mi, s, wd;
  if (getLocalDateTime(Y, Mo, D, h, mi, s, wd)) {
    int hh = h;
    const char *ap = "";
    if (!CLOCK_24H) {
      ap = (h >= 12) ? "PM" : "AM";
      hh = h % 12;
      if (hh == 0) hh = 12;
    }
    int dig[4] = { hh / 10, hh % 10, mi / 10, mi % 10 };
    const int dx[4] = { 20, 106, 48, 134 };
    const int dy[4] = { 64,  44, 134, 114 };
    const uint16_t dc[4] = { cPink, cBlue, cWhite, cPurple };

    for (int i = 0; i < 4; i++) drawFaceDigit(g, dig[i], dx[i], dy[i], dc[i]);

    String wd3 = String(WEEKDAYS[wd]).substring(0, 3);
    String mo3 = String(MONTHS[Mo - 1]).substring(0, 3);
    wd3.toUpperCase();
    mo3.toUpperCase();
    String ds = wd3 + " " + String(D) + " " + mo3;
    int total = 22 + (int)ds.length() * 12;
    int sx = (240 - total) / 2;

    drawCalIcon(g, sx, 12, C_WHITE);
    boldText(g, sx + 22, 14, ds, cInfo, 2, 1);

    drawSatIcon(g, 12, 254, C_WHITE);
    boldText(g, 34, 254, String(faceSats()), cInfo, 2, 1);

    String mid = CLOCK_24H ? tzLabel() : String(ap);
    int mw = (int)mid.length() * 12;
    boldText(g, (240 - mw) / 2, 254, mid, C_MUTED, 2, 1);

    String t = (wxState == 3) ? String((int)lroundf(wxTemp)) + "C" : String("--");
    int ttw = (int)t.length() * 12;
    boldText(g, 228 - ttw, 254, t, cInfo, 2, 1);
    drawThermIcon(g, 228 - ttw - 18, 252, C_WHITE);
  } else {
    boldText(g, 24, 112, "Waiting for time", C_YELLOW, 2, 1);
    boldText(g, 12, 146, gps.charsProcessed() ? "GPS: NO FIX" : "GPS: NO DATA", C_RED, 2, 1);
    boldText(g, 12, 176, "Sats: " + String(faceSats()), C_WHITE, 2, 1);
    g->setTextSize(1);
    g->setTextColor(C_MUTED);
    g->setCursor(12, 214);
    g->print("Weather app syncs NTP time");
  }

  flushCanvas();
}

// ======================= WI-FI RADAR =======================
// Passive probe-request sniffer (same idea as mode 1 of the ESP8266 code).
// Every phone/laptop that is looking for a Wi-Fi network sends probe requests;
// each unique transmitter MAC becomes a dot on the radar. Closer = stronger RSSI = nearer the centre.
#define RADAR_MAX 10
struct RadarTarget {
  uint8_t mac[6];
  int rssi;
  float angle;
  float dist;                 // smoothed distance from the centre (animated)
  unsigned long lastSeen;
  bool active;
  bool fresh;                 // true = jump straight to the real distance on first draw
};
RadarTarget radarT[RADAR_MAX];
portMUX_TYPE radarMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t radarPkts = 0;
volatile bool radarBlip = false;
uint8_t radarChan = 1;
unsigned long radarLastHop = 0;
unsigned long radarLastPrune = 0;
const unsigned long RADAR_TTL_MS = 6000;

static void radarRegister(const uint8_t *mac, int rssi) {
  unsigned long now = millis();
  portENTER_CRITICAL(&radarMux);
  int idx = -1;
  for (int i = 0; i < RADAR_MAX; i++) {
    if (radarT[i].active && memcmp(radarT[i].mac, mac, 6) == 0) { idx = i; break; }
  }
  if (idx < 0) {
    for (int i = 0; i < RADAR_MAX; i++) {
      if (!radarT[i].active) { idx = i; break; }
    }
  }
  if (idx < 0) {                                  // full: replace the oldest one
    idx = 0;
    for (int i = 1; i < RADAR_MAX; i++) {
      if (radarT[i].lastSeen < radarT[idx].lastSeen) idx = i;
    }
    radarT[idx].active = false;
  }
  memcpy(radarT[idx].mac, mac, 6);
  radarT[idx].rssi = rssi;
  radarT[idx].lastSeen = now;
  if (!radarT[idx].active) {
    radarT[idx].angle = (float)random(360) * DEG2RAD;
    radarT[idx].fresh = true;
  }
  radarT[idx].active = true;
  portEXIT_CRITICAL(&radarMux);
  radarPkts = radarPkts + 1;
  radarBlip = true;
}

// Runs in the Wi-Fi task: keep it short
static void radarSniffer(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
  if (pkt->rx_ctrl.sig_len < 24) return;
  const uint8_t *p = pkt->payload;
  if (p[0] != 0x40) return;                       // 0x40 = probe request
  radarRegister(&p[10], pkt->rx_ctrl.rssi);       // addr2 = transmitter MAC
}

void radarClear() {
  portENTER_CRITICAL(&radarMux);
  for (int i = 0; i < RADAR_MAX; i++) radarT[i].active = false;
  portEXIT_CRITICAL(&radarMux);
}

bool radarStart() {
  if (trackerOn) return false;                    // tracker needs the Wi-Fi link
  if (wifiStarted) { WiFi.scanDelete(); wifiStarted = false; }
  apState = 0;
  apHold = false;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  radarClear();
  radarPkts = 0;
  radarBlip = false;
  radarChan = 1;
  radarLastHop = millis();
  esp_wifi_set_promiscuous_rx_cb(&radarSniffer);
  wifi_promiscuous_filter_t filt = {};
  filt.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT;
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(radarChan, WIFI_SECOND_CHAN_NONE);
  radarOn = true;
  return true;
}

void radarStop() {
  if (!radarOn) return;
  esp_wifi_set_promiscuous(false);
  radarOn = false;
}

void pollRadar() {
  if (!radarOn) return;
  unsigned long now = millis();

  if (now - radarLastHop >= 180) {
    radarLastHop = now;
    radarChan = (radarChan % 13) + 1;
    esp_wifi_set_channel(radarChan, WIFI_SECOND_CHAN_NONE);
  }
  if (radarBlip) {
    radarBlip = false;
    ledFlash(0, 255, 0, 60);
  }
  if (now - radarLastPrune >= 250) {
    radarLastPrune = now;
    portENTER_CRITICAL(&radarMux);
    for (int i = 0; i < RADAR_MAX; i++) {
      if (radarT[i].active && now - radarT[i].lastSeen > RADAR_TTL_MS) radarT[i].active = false;
    }
    portEXIT_CRITICAL(&radarMux);
  }
}

void renderRadar() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  g->fillScreen(bgColor());

  const int cx = 120, cy = 142, R = 98;
  const uint16_t base = themeColor(C_GREEN);
  const uint16_t ring = dimc(base, 85);

  g->drawCircle(cx, cy, R / 3, ring);
  g->drawCircle(cx, cy, (R * 2) / 3, ring);
  g->drawCircle(cx, cy, R, ring);
  g->drawFastHLine(cx - R, cy, 2 * R + 1, ring);
  g->drawFastVLine(cx, cy - R, 2 * R + 1, ring);

  unsigned long now = millis();

  // sweep line with a long, smooth fading trail
  float sw = (float)(now % 2400UL) / 2400.0f * 6.2831853f;
  for (int k = 11; k >= 0; k--) {
    float a = sw - k * 0.06f;
    int br = 255 - k * 20;
    g->drawLine(cx, cy, cx + (int)lroundf(cosf(a) * R), cy + (int)lroundf(sinf(a) * R),
                dimc(base, (uint8_t)br));
  }
  g->fillCircle(cx, cy, 3, base);

  // targets (copied under lock, the Wi-Fi task keeps writing)
  RadarTarget snap[RADAR_MAX];
  portENTER_CRITICAL(&radarMux);
  memcpy(snap, radarT, sizeof(snap));
  portEXIT_CRITICAL(&radarMux);

  int active = 0;
  int best = -100;
  for (int i = 0; i < RADAR_MAX; i++) {
    if (!snap[i].active) continue;
    unsigned long age = now - snap[i].lastSeen;
    if (age > RADAR_TTL_MS) continue;
    active++;
    if (snap[i].rssi > best) best = snap[i].rssi;

    float t = (float)(snap[i].rssi + 95) / 60.0f;       // -95 dBm = edge, -35 dBm = centre
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    float goal = (float)(R - 8) - t * (float)(R - 16);

    // glide towards the real distance instead of jumping
    float cur = (radarT[i].fresh || radarT[i].dist <= 0.0f) ? goal : radarT[i].dist;
    cur += (goal - cur) * 0.18f;
    radarT[i].dist = cur;
    radarT[i].fresh = false;

    int tx = cx + (int)lroundf(cosf(snap[i].angle) * cur);
    int ty = cy + (int)lroundf(sinf(snap[i].angle) * cur);

    int br = 255 - (int)(age * 150UL / RADAR_TTL_MS);
    uint16_t col = dimc(themeColor(rssiColor(snap[i].rssi)), (uint8_t)br);
    g->fillCircle(tx, ty, 4, col);
    if (age < 600) g->drawCircle(tx, ty, 5 + (int)(age / 60), col);
  }

  // header
  boldText(g, 12, 8, "RADAR", themeColor(C_CYAN), 2, 1);
  String ch = "CH " + String(radarChan);
  boldText(g, 228 - (int)ch.length() * 12, 8, ch, themeColor(C_WHITE), 2, 1);

  // footer
  boldText(g, 12, 252, "TGT " + String(active), themeColor(C_WHITE), 2, 1);
  String rs = active ? String(best) + "dBm" : String("SWEEP");
  boldText(g, 228 - (int)rs.length() * 12, 252, rs, themeColor(active ? rssiColor(best) : C_MUTED), 2, 1);

  flushCanvas();
}

void renderUI() {
  if (page == MENU) renderMenu();
  else if (page == FACE_PAGE) renderFace();
  else if (kbActive()) renderKeyboard();
  else if (radarActive()) renderRadar();
  else if (qrActive()) renderQR();
  else if (page == NRF24_PAGE && subView == 2) renderNRFSpectrum();
  else renderPage();
}

// Draws the current screen fully visible (no intro fade). Used for "Scanning..." / "Connecting..."
// status screens that are shown right before a blocking operation, so they never appear blank.
void renderUIStatic() {
  float saved = introP;
  introP = 1.0f;
  renderUI();
  introP = saved;
}

// ---------------- animation engine ----------------
// * critically damped spring for list scrolling (no stepping, no overshoot, follows direction changes smoothly)
// * time based (dt), so the speed is identical no matter how long a frame takes
// * intro cascade whenever a page / sub menu / detail view opens
void animateUI() {
  unsigned long now = millis();
  if (now - lastFrame < FRAME_MS) return;

  float dt = (float)(now - lastAnimMs) / 1000.0f;
  lastAnimMs = now;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;

  // detect a view change (page, sub menu, password screen, detail view) and restart the intro
  static uint32_t lastViewKey = 0xFFFFFFFF;
  uint32_t viewKey = (uint32_t)page * 10000u + (uint32_t)(subView & 15) * 1000u +
                     (uint32_t)(apStage & 3) * 100u + (wifiDetailView ? 10u : 0u) + (bleDetailView ? 20u : 0u) +
                     (qrShowing ? 40u : 0u);
  if (viewKey != lastViewKey) {
    lastViewKey = viewKey;
    introP = 0.0f;
  }

  float target = (page == MENU) ? (float)selected : (float)pageSel;
  float diff = target - scrollPos;

  if (fabsf(diff) > 0.002f || fabsf(scrollVel) > 0.01f) {
    const float K = SCROLL_K;
    const float C = 2.0f * sqrtf(K);               // critical damping
    int steps = (int)(dt / 0.006f) + 1;
    float h = dt / (float)steps;
    for (int i = 0; i < steps; i++) {
      float acc = (target - scrollPos) * K - scrollVel * C;
      scrollVel += acc * h;
      scrollPos += scrollVel * h;
    }
    if (fabsf(target - scrollPos) < 0.002f && fabsf(scrollVel) < 0.02f) {
      scrollPos = target;
      scrollVel = 0.0f;
    }
    uiDirty = true;
  }

  if (introP < 1.0f) {
    introP += dt / INTRO_SECS;
    if (introP > 1.0f) introP = 1.0f;
    uiDirty = true;
  }

  if (radarActive()) uiDirty = true;            // radar sweep animates continuously
  if (uiDirty) {
    uiDirty = false;
    lastFrame = millis();
    renderUI();
  }
}

void mergeWifiResults(int n) {
  for (int i = 0; i < n; i++) {
    String ssid  = WiFi.SSID(i);
    String bssid = WiFi.BSSIDstr(i);
    int rssi     = WiFi.RSSI(i);
    int ch       = WiFi.channel(i);
    wifi_auth_mode_t sec = (wifi_auth_mode_t)WiFi.encryptionType(i);
    bool found = false;

    for (int j = 0; j < wifiCount; j++) {
      if (wifiList[j].bssid == bssid) {
        wifiList[j].ssid = ssid;
        wifiList[j].rssi = rssi;
        wifiList[j].channel = ch;
        wifiList[j].security = sec;
        wifiList[j].lastSeen = millis();
        found = true;
        break;
      }
    }
    if (!found && wifiCount < MAX_WIFI_ENTRIES) {
      wifiList[wifiCount] = {ssid, bssid, rssi, ch, sec, millis()};
      wifiCount++;
    }
  }
  WiFi.scanDelete();

  for (int i = 0; i < wifiCount; i++) {
    if (millis() - wifiList[i].lastSeen > WIFI_ENTRY_TTL_MS) {
      for (int j = i; j < wifiCount - 1; j++) wifiList[j] = wifiList[j + 1];
      wifiCount--;
      i--;
    }
  }
}

void buildWifiList() {
  startLines();
  if (wifiCount == 0) addLine(wifiStarted ? "Scanning..." : "No networks", "", C_YELLOW, C_YELLOW);
  for (int i = 0; i < wifiCount; i++) {
    WifiEntry &w = wifiList[i];
    addLine(w.ssid.length() ? w.ssid : "<hidden>", String(w.rssi), C_WHITE, rssiColor(w.rssi));
  }
  finishLines();
}

void buildWifiDetail() {
  startLines();
  if (wifiSelectedIndex >= wifiCount) {
    addLine("Not available", "", C_YELLOW, C_YELLOW);
    finishLines();
    return;
  }
  WifiEntry &w = wifiList[wifiSelectedIndex];
  addKV("SSID", w.ssid.length() ? w.ssid : "<hidden>", C_WHITE);
  addKV("BSSID", w.bssid, C_CYAN);
  addKV("RSSI", String(w.rssi) + " dBm", rssiColor(w.rssi));
  addKV("Signal", String(getSignalQuality(w.rssi)) + "%", rssiColor(w.rssi));
  addKV("Channel", String(w.channel), C_WHITE);
  addKV("Security", getSecurityName(w.security), C_WHITE);
  addKV("Freq", String(2407 + w.channel * 5) + " MHz", C_WHITE);
  addKV("Hidden", w.ssid.length() ? "No" : "Yes", C_WHITE);
  finishLines();
}

void startWiFiScan() {
  if (wifiStarted) return;
  if (apState == 1 || apState == 5) return;       // don't scan while a connection is being made
  if (!trackerOn && !apHold) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_TX_POWER);
    WiFi.disconnect(false, false);
  }
  int r = WiFi.scanNetworks(true, true);
  wifiStarted = (r == WIFI_SCAN_RUNNING || r >= 0);
  lastWiFiStart = millis();
  if (r >= 0) {
    mergeWifiResults(r);
    wifiStarted = false;
  }
}

// ======================= CONNECT AP =======================
void saveWifiCreds(const String &ssid, const String &pass) {
  Preferences tp;
  tp.begin("watchout", false);
  tp.putString("wssid", ssid);
  tp.putString("wpass", pass);
  tp.end();
  savedSsid = ssid;
  savedPass = pass;
}

void forgetWifiCreds() {
  Preferences tp;
  tp.begin("watchout", false);
  tp.remove("wssid");
  tp.remove("wpass");
  tp.end();
  savedSsid = "";
  savedPass = "";
}

// Non-blocking: pollAP() does the real work
void apConnect(const String &ssid, const String &pass) {
  apTargetSsid = ssid;
  apTryPass = pass;
  apFailMsg = "";
  apNetMsg = "";
  apState = 5;
  apHold = true;
  apConnStart = millis();
  apStage = 2;
  pageSel = 0;
  scrollPos = 0.0f;
  scrollVel = 0.0f;
}

void apFail(const char *msg) {
  apState = 3;
  apFailMsg = msg;
  apHold = false;
  WiFi.disconnect(false, false);
  ledFlash(255, 0, 0, 400);
  lastDraw = 0;
}

void apCancel() {
  WiFi.disconnect(false, false);
  apState = 0;
  apHold = false;
  apStage = 0;
  pageSel = apListSel;
  scrollPos = (float)pageSel;
  scrollVel = 0.0f;
}

void apDisconnect(bool forget) {
  WiFi.disconnect(false, false);
  apHold = false;
  apState = 0;
  apNetMsg = "";
  if (forget) forgetWifiCreds();
}

void pollAP() {
  if (apState != 1 && apState != 5) return;
  unsigned long now = millis();

  if (apState == 5) {                              // wait for a running scan to end, then connect
    bool scanning = wifiStarted && WiFi.scanComplete() == WIFI_SCAN_RUNNING;
    if (scanning && now - apConnStart < 4000) return;
    if (wifiStarted) { WiFi.scanDelete(); wifiStarted = false; }
    WiFi.disconnect(false, false);
    wifiBeginTo(apTargetSsid, apTryPass);
    apState = 1;
    apConnStart = now;
    return;
  }

  wl_status_t st = WiFi.status();
  unsigned long el = now - apConnStart;
  if (st == WL_CONNECTED) {
    apState = 2;
    saveWifiCreds(apTargetSsid, apTryPass);
    ledFlash(0, 255, 0, 400);
    lastDraw = 0;
  } else if (el > 1500 && st == WL_NO_SSID_AVAIL) {
    apFail("Network not found");
  } else if (el > 1500 && st == WL_CONNECT_FAILED) {
    apFail("Wrong password?");
  } else if (el > 15000) {
    apFail("Timed out");
  }
}

// 0 idle, 1 connecting, 2 connected, 3 failed
int apView() {
  if (apState == 1 || apState == 5) return 1;
  if (WiFi.status() == WL_CONNECTED) return 2;
  if (apState == 3) return 3;
  return 0;
}

void apTestInternet() {
  apNetMsg = "Testing...";
  buildWifi();
  uiDirty = false;
  renderUIStatic();

  WiFiClient client;
  HTTPClient http;
  unsigned long t0 = millis();
  if (http.begin(client, "http://connectivitycheck.gstatic.com/generate_204")) {
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    int code = http.GET();
    http.end();
    if (code == 204) apNetMsg = "Online " + String(millis() - t0) + " ms";
    else if (code > 0) apNetMsg = "Captive/blocked " + String(code);
    else apNetMsg = "No internet " + String(code);
  } else {
    apNetMsg = "HTTP init failed";
  }
}

void buildAPList() {
  // one entry per SSID (strongest AP wins), hidden networks are skipped
  apIdxCount = 0;
  for (int i = 0; i < wifiCount; i++) {
    if (wifiList[i].ssid.length() == 0) continue;
    int dup = -1;
    for (int k = 0; k < apIdxCount; k++) {
      if (wifiList[apIdx[k]].ssid == wifiList[i].ssid) { dup = k; break; }
    }
    if (dup >= 0) {
      if (wifiList[i].rssi > wifiList[apIdx[dup]].rssi) apIdx[dup] = i;
    } else if (apIdxCount < MAX_WIFI_ENTRIES) {
      apIdx[apIdxCount++] = i;
    }
  }

  bool linked = WiFi.status() == WL_CONNECTED;
  if (apState == 1 || apState == 5) addLine("Connecting...", "", C_YELLOW, C_YELLOW);
  else if (linked) addLine(WiFi.SSID(), "ONLINE", C_WHITE, C_GREEN);
  else addLine("Not connected", "", C_YELLOW, C_YELLOW);

  if (apIdxCount == 0) addLine(wifiStarted ? "Scanning..." : "No networks", "", C_MUTED, C_MUTED);
  for (int k = 0; k < apIdxCount; k++) {
    WifiEntry &w = wifiList[apIdx[k]];
    addLine(w.ssid, String(w.rssi), C_WHITE, rssiColor(w.rssi));
  }
  addLine("> Rescan", "", C_CYAN, C_CYAN);
}

void buildAPStatus() {
  apActRow[0] = apActRow[1] = apActRow[2] = -1;
  int v = apView();

  if (v == 1) {
    addLine("Connecting...", "", C_YELLOW, C_YELLOW);
    addKV("SSID", apTargetSsid, C_WHITE);
    addLine("Time", String((millis() - apConnStart) / 1000UL) + "s", C_MUTED, C_WHITE);
    apActRow[0] = lineCount;
    addLine("> Cancel", "", C_RED, C_RED);
  } else if (v == 2) {
    addLine("Connected", "", C_GREEN, C_GREEN);
    addKV("SSID", WiFi.SSID(), C_WHITE);
    addKV("IP", WiFi.localIP().toString(), C_CYAN);
    addKV("Gateway", WiFi.gatewayIP().toString(), C_WHITE);
    addKV("RSSI", String(WiFi.RSSI()) + " dBm", rssiColor(WiFi.RSSI()));
    addKV("Channel", String(WiFi.channel()), C_WHITE);
    addKV("BSSID", WiFi.BSSIDstr(), C_CYAN);
    addKV("MAC", WiFi.macAddress(), C_CYAN);
    if (apNetMsg.length()) {
      addLine("Internet", "", C_MUTED, C_MUTED);
      bool ok = apNetMsg.startsWith("Online");
      addWrapped(apNetMsg, ok ? C_GREEN : (apNetMsg == "Testing..." ? C_YELLOW : C_RED));
    }
    apActRow[0] = lineCount;
    addLine("> Test internet", "", C_CYAN, C_CYAN);
    apActRow[1] = lineCount;
    addLine("> Disconnect", "", C_YELLOW, C_YELLOW);
    if (savedSsid.length()) {
      apActRow[2] = lineCount;
      addLine("> Forget saved", "", C_RED, C_RED);
    }
  } else if (v == 3) {
    addLine("Failed", "", C_RED, C_RED);
    addKV("SSID", apTargetSsid, C_WHITE);
    addWrapped(apFailMsg, C_YELLOW);
    apActRow[0] = lineCount;
    addLine("> Retry", "", C_CYAN, C_CYAN);
    apActRow[1] = lineCount;
    addLine("> New password", "", C_CYAN, C_CYAN);
  } else {
    addLine("Not connected", "", C_YELLOW, C_YELLOW);
    addWrapped("Pick a network", C_MUTED);
    if (savedSsid.length()) addKV("Saved", savedSsid, C_CYAN);
  }
}

void buildWifi() {
  if (kbWifi()) { uiDirty = true; return; }          // password keyboard draws itself

  if (subView == 1) {
    if (wifiDetailView) buildWifiDetail();
    else buildWifiList();
    return;
  }

  startLines();
  if (subView == 2) {
    if (apStage == 2) buildAPStatus();
    else buildAPList();
  } else if (subView == 3) {
    if (!radarOn) {                                   // radar is drawn by renderRadar()
      addLine("Tracker is ON", "", C_RED, C_RED);
      addWrapped("Stop it in GPS", C_MUTED);
      addWrapped("to use the radar", C_MUTED);
    }
  } else {
    bool linked = WiFi.status() == WL_CONNECTED;
    String st = (apState == 1 || apState == 5) ? "..." : (linked ? "ONLINE" : "");
    addLine("Wi-Fi Scan", "", C_WHITE, C_WHITE);
    addLine("Connect AP", st, C_WHITE, linked ? C_GREEN : C_YELLOW);
    addLine("Radar", "", C_WHITE, C_WHITE);
  }
  finishLines();
}

void pollWiFi() {
  if (page != WIFI_PAGE) return;
  bool scanView = (subView == 1 && !wifiDetailView) || (subView == 2 && apStage == 0);
  if (!scanView) return;
  if (wifiStarted) {
    int r = WiFi.scanComplete();
    if (r >= 0) {
      mergeWifiResults(r);
      wifiStarted = false;
      buildWifi();
    } else if (r == WIFI_SCAN_FAILED) {
      wifiStarted = false;
    }
    return;
  }
  if (millis() - lastWiFiStart > WIFI_RESCAN_MS) startWiFiScan();
}

// ---- Wi-Fi page navigation ----
void wifiKbBack() {
  apStage = 0;
  pageSel = apListSel;
  scrollPos = (float)pageSel;
  scrollVel = 0.0f;
  buildWifi();
}

void apConnectFromKb() {
  if (apPass.length() == 0) {
    chatStatus = "Empty";
    chatStatusAt = millis();
    return;
  }
  apConnect(apTargetSsid, apPass);
}

void apListClick() {
  if (pageSel == 0) {                               // status line
    apListSel = pageSel;
    apStage = 2;
    pageSel = 0;
    scrollPos = 0.0f;
    scrollVel = 0.0f;
    return;
  }
  int k = pageSel - 1 - (apIdxCount == 0 ? 1 : 0);  // first row is the status, then (maybe) a "Scanning" row
  if (apIdxCount > 0 && k >= 0 && k < apIdxCount) {
    int wi = apIdx[k];
    if (wi >= wifiCount) return;
    apListSel = pageSel;
    apTargetSsid = wifiList[wi].ssid;
    bool open = (wifiList[wi].security == WIFI_AUTH_OPEN);
    if (open) {
      apConnect(apTargetSsid, "");
    } else if (savedSsid.length() && savedSsid == apTargetSsid) {
      apConnect(apTargetSsid, savedPass);           // known network: use the saved password
    } else {
      apPass = "";
      apShowPass = false;
      apStage = 1;
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
      kbOpen();
    }
  } else {
    startWiFiScan();                                // "> Rescan"
  }
}

void apStatusClick() {
  int act = -1;
  for (int j = 0; j < 3; j++) if (apActRow[j] == pageSel) act = j;
  if (act < 0) return;

  int v = apView();
  if (v == 1) {
    if (act == 0) apCancel();
  } else if (v == 2) {
    if (act == 0) {
      apTestInternet();
    } else if (act == 1 || act == 2) {
      apDisconnect(act == 2);
      apStage = 0;
      pageSel = apListSel;
      scrollPos = (float)pageSel;
      scrollVel = 0.0f;
    }
  } else if (v == 3) {
    if (act == 0) {
      apConnect(apTargetSsid, apTryPass);
    } else if (act == 1) {
      apPass = "";
      apShowPass = false;
      apStage = 1;
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
      kbOpen();
    }
  }
}

void wifiEnterView(int v) {
  subView = v;
  pageSel = 0;
  scrollPos = 0.0f;
  scrollVel = 0.0f;
  wifiDetailView = false;
  apStage = 0;
  if (v == 1 || v == 2) startWiFiScan();
  else if (v == 3) radarStart();
}

void wifiClick() {
  if (subView == 0) {
    if (pageSel < 3) {
      subParentSel = pageSel;
      wifiEnterView(pageSel + 1);
    }
  } else if (subView == 1) {
    if (!wifiDetailView) {
      if (wifiCount > 0 && pageSel < wifiCount) {
        wifiSelectedIndex = pageSel;
        wifiDetailView = true;
        pageSel = 0;
        scrollPos = 0.0f;
        scrollVel = 0.0f;
      } else {
        startWiFiScan();
      }
    }
  } else if (subView == 2) {
    if (apStage == 0) apListClick();
    else if (apStage == 1) kbPress();
    else apStatusClick();
  } else if (subView == 3) {
    if (radarOn) radarClear();
  }
  buildWifi();
}

void wifiBack() {
  if (subView == 1 && wifiDetailView) {
    wifiDetailView = false;
    pageSel = wifiSelectedIndex;
    scrollPos = (float)pageSel;
    scrollVel = 0.0f;
    buildWifi();
    return;
  }
  if (subView == 2 && apStage == 1) { wifiKbBack(); return; }
  if (subView == 2 && apStage == 2) {
    apStage = 0;
    pageSel = apListSel;
    scrollPos = (float)pageSel;
    scrollVel = 0.0f;
    buildWifi();
    return;
  }
  if (subView == 3) radarStop();
  subView = 0;
  wifiDetailView = false;
  apStage = 0;
  pageSel = subParentSel;
  scrollPos = (float)pageSel;
  scrollVel = 0.0f;
  buildWifi();
}

// Long press on the joystick button inside the Wi-Fi password keyboard = back
bool wifiLongClick() {
  if (kbWifi()) {
    wifiKbBack();
    return true;
  }
  return false;
}

// ======================= BLE (NimBLE) =======================
// Bluetooth sub views:
//   0 menu | 1 scan | 2 Media Cmd
// Requires NimBLE-Arduino 2.x (Library Manager: "NimBLE-Arduino" by h2zero).
//
// Media Cmd = BLE HID "Consumer Control" remote (play/pause, volume, next/prev, mute, stop,
// camera shutter). What changed compared with v2.2:
//   * action rows are tracked dynamically (bleRowAct[]) - the old hard-coded row numbers
//     pointed at the wrong lines because the hint/status text wraps over a variable number of rows
//   * HID report is now 1 byte (report ID lives in the Report Reference descriptor, not in the payload)
//   * bonding / encrypted characteristics + Device Information (PnP ID) service, so Android, iOS and
//     Windows actually accept the device as a HID remote
//   * advertising now carries the HID service UUID + appearance (the old clearData() removed them)
//   * non-blocking key release (no delay() in the UI loop), subscribe-aware link status,
//     disconnect on stop, Stop / Unpair rows, auto re-advertise after disconnect

NimBLEAdvertising *bleAdv = nullptr;
bool bleNimReady = false;
bool bleScanBusy = false;

// ---- media cmd (BLE HID) ----
NimBLEServer *bleServer = nullptr;
NimBLEService *bleHidSvc = nullptr;
NimBLECharacteristic *bleMediaReport = nullptr;
NimBLECharacteristic *bleKbReport = nullptr;
bool bleHidOn = false;
volatile bool bleHidConnected = false;       // a phone / PC is connected
volatile bool bleHidReady = false;           // ...and it subscribed to the report (keys will really arrive)
volatile uint16_t bleConnHandle = 0xFFFF;
String bleHidMsg = "";
bool bleKeyPending = false;                  // a key is held, release is sent from pollBle()
unsigned long bleKeyReleaseAt = 0;
int bleFirstKeyRow = -1;

const char *BLE_MENU_ITEMS[] = {"BLE Scan", "Media Cmd"};
const int BLE_MENU_COUNT = 2;

// Action codes for the Media Cmd rows
#define BLE_ACT_NONE    0
#define BLE_ACT_STOP    1
#define BLE_ACT_START   2
#define BLE_ACT_UNPAIR  3
#define BLE_ACT_KEY     0x100                // BLE_ACT_KEY | consumer-control bit
int bleRowAct[MAX_LINES];

// Consumer control bits (match HID_REPORT_MAP below)
#define HID_VOL_UP     0x01
#define HID_VOL_DOWN   0x02
#define HID_MUTE       0x04
#define HID_PLAY_PAUSE 0x08
#define HID_NEXT       0x10
#define HID_PREV       0x20
#define HID_STOP       0x40
#define HID_EJECT      0x80

bool bleEnsureInit() {
  if (bleNimReady && bleAdv) return true;
  NimBLEDevice::init("WatchOut Remote");
  NimBLEDevice::setSecurityAuth(true, false, true);                 // bonding, no MITM, secure connections
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);        // "just works" pairing
  bleAdv = NimBLEDevice::getAdvertising();
  if (!bleAdv) return false;
  bleAdv->setMinInterval(160);
  bleAdv->setMaxInterval(160);
  bleNimReady = true;
  return true;
}

void bleAdvStopNow() {
  if (bleAdv && bleAdv->isAdvertising()) bleAdv->stop();
}

class BleServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &info) override {
    bleConnHandle = info.getConnHandle();
    bleHidConnected = true;
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
    bleHidConnected = false;
    bleHidReady = false;
    bleConnHandle = 0xFFFF;
  }
};
static BleServerCb bleSrvCb;

class BleReportCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t subValue) override {
    bleHidReady = (subValue & 1) != 0;              // bit0 = notifications enabled
  }
};
static BleReportCb bleRepCb;

// Stop BLE HID: stop advertising and drop the connection
void bleStopTx() {
  bleHidOn = false;
  bleKeyPending = false;
  bleAdvStopNow();
  if (bleServer && bleHidConnected) bleServer->disconnect(bleConnHandle);
  bleHidConnected = false;
  bleHidReady = false;
}

void bleStopAll() {
  if (!bleNimReady) return;
  bleStopTx();
}

// ---------------- Media Cmd (BLE HID remote) ----------------
// Consumer control report (ID 1) and keyboard report (ID 2):
//   bit0 Vol+  bit1 Vol-  bit2 Mute  bit3 Play/Pause  bit4 Next  bit5 Prev  bit6 Stop  bit7 Eject
static const uint8_t HID_REPORT_MAP[] = {
  0x05, 0x0C,       // Usage Page (Consumer)
  0x09, 0x01,       // Usage (Consumer Control)
  0xA1, 0x01,       // Collection (Application)
  0x85, 0x01,       //   Report ID 1
  0x15, 0x00,       //   Logical Min 0
  0x25, 0x01,       //   Logical Max 1
  0x75, 0x01,       //   Report Size 1
  0x95, 0x08,       //   Report Count 8
  0x09, 0xE9,       //   Volume Up
  0x09, 0xEA,       //   Volume Down
  0x09, 0xE2,       //   Mute
  0x09, 0xCD,       //   Play/Pause
  0x09, 0xB5,       //   Scan Next
  0x09, 0xB6,       //   Scan Previous
  0x09, 0xB7,       //   Stop
  0x09, 0xB8,       //   Eject
  0x81, 0x02,       //   Input (Data, Variable, Absolute)
  0xC0,             // End Consumer Control collection

  0x05, 0x01,       // Usage Page (Generic Desktop)
  0x09, 0x06,       // Usage (Keyboard)
  0xA1, 0x01,       // Collection (Application)
  0x85, 0x02,       //   Report ID 2
  0x05, 0x07,       //   Usage Page (Keyboard)
  0x19, 0xE0,       //   Usage Minimum (Left Control)
  0x29, 0xE7,       //   Usage Maximum (Right GUI)
  0x15, 0x00,       //   Logical Minimum (0)
  0x25, 0x01,       //   Logical Maximum (1)
  0x75, 0x01,       //   Report Size (1)
  0x95, 0x08,       //   Report Count (8)
  0x81, 0x02,       //   Input (Data, Variable, Absolute): modifiers
  0x95, 0x01,       //   Report Count (1)
  0x75, 0x08,       //   Report Size (8)
  0x81, 0x01,       //   Input (Constant): reserved byte
  0x95, 0x05,       //   Report Count (5)
  0x75, 0x01,       //   Report Size (1)
  0x05, 0x08,       //   Usage Page (LEDs)
  0x19, 0x01,       //   Usage Minimum (Num Lock)
  0x29, 0x05,       //   Usage Maximum (Kana)
  0x91, 0x02,       //   Output (Data, Variable, Absolute): LEDs
  0x95, 0x01,       //   Report Count (1)
  0x75, 0x03,       //   Report Size (3)
  0x91, 0x01,       //   Output (Constant): padding
  0x95, 0x06,       //   Report Count (6)
  0x75, 0x08,       //   Report Size (8)
  0x15, 0x00,       //   Logical Minimum (0)
  0x25, 0x65,       //   Logical Maximum (101)
  0x05, 0x07,       //   Usage Page (Keyboard)
  0x19, 0x00,       //   Usage Minimum (Reserved)
  0x29, 0x65,       //   Usage Maximum (Keyboard Application)
  0x81, 0x00,       //   Input (Data, Array)
  0xC0              // End Keyboard collection
};

// Sends one 1-byte HID report (report ID is added by the stack through the Report Reference descriptor)
bool bleMediaSend(uint8_t bits) {
  if (!bleMediaReport || !bleHidConnected) return false;
  bleMediaReport->setValue(&bits, 1);
  return bleMediaReport->notify();
}

// Press now, release is sent ~60 ms later by pollBle() (no blocking delay)
void bleMediaKey(uint8_t bit, const char *name) {
  if (!bleHidConnected) { bleHidMsg = "Not connected"; return; }
  if (bleKeyPending) { bleMediaSend(0); bleKeyPending = false; }   // release the previous key first
  if (bleMediaSend(bit)) {
    bleKeyPending = true;
    bleKeyReleaseAt = millis() + 60;
    bleHidMsg = String("Sent ") + name;
    ledFlash(88, 86, 214, 120);
  } else {
    bleHidMsg = bleHidReady ? "Send failed" : "Not paired yet";
  }
}

void bleMediaStart() {
  bleStopTx();
  if (!bleEnsureInit()) { bleHidMsg = "BLE init failed"; return; }
  if (!bleServer) {
    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(&bleSrvCb);
  }
  if (!bleMediaReport) {
    // --- HID service ---
    bleHidSvc = bleServer->createService("1812");
    NimBLECharacteristic *proto = bleHidSvc->createCharacteristic("2A4E", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
    uint8_t z = 1;                                                // 1 = report protocol
    proto->setValue(&z, 1);
    NimBLECharacteristic *info = bleHidSvc->createCharacteristic("2A4A", NIMBLE_PROPERTY::READ);
    uint8_t hi[4] = {0x11, 0x01, 0x00, 0x02};                     // HID v1.11, country 0, flags: normally connectable
    info->setValue(hi, 4);
    bleHidSvc->createCharacteristic("2A4C", NIMBLE_PROPERTY::WRITE_NR);   // control point
    NimBLECharacteristic *rmap = bleHidSvc->createCharacteristic("2A4B", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC);
    rmap->setValue(HID_REPORT_MAP, sizeof(HID_REPORT_MAP));
    bleMediaReport = bleHidSvc->createCharacteristic("2A4D",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
    uint8_t zero = 0;
    bleMediaReport->setValue(&zero, 1);
    bleMediaReport->setCallbacks(&bleRepCb);
    uint8_t rr[2] = {1, 0x01};                                    // report ID 1, type 1 = input
    NimBLEDescriptor *rd = bleMediaReport->createDescriptor("2908", NIMBLE_PROPERTY::READ);
    rd->setValue(rr, 2);
    bleKbReport = bleHidSvc->createCharacteristic("2A4D",
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::NOTIFY);
    uint8_t kbZero[8] = {};
    bleKbReport->setValue(kbZero, sizeof(kbZero));
    bleKbReport->setCallbacks(&bleRepCb);
    uint8_t kbRef[2] = {2, 0x01};                                // report ID 2, type 1 = input
    NimBLEDescriptor *kbRd = bleKbReport->createDescriptor("2908", NIMBLE_PROPERTY::READ);
    kbRd->setValue(kbRef, sizeof(kbRef));
    // --- Device Information (PnP ID is required by several hosts) ---
    NimBLEService *dis = bleServer->createService("180A");
    NimBLECharacteristic *man = dis->createCharacteristic("2A29", NIMBLE_PROPERTY::READ);
    man->setValue("WatchOut");
    NimBLECharacteristic *pnp = dis->createCharacteristic("2A50", NIMBLE_PROPERTY::READ);
    uint8_t pid[7] = {0x02, 0xE5, 0x02, 0xA1, 0x11, 0x10, 0x02};  // USB vendor source, VID 0x02E5, PID 0x11A1, v2.10
    pnp->setValue(pid, 7);

    // --- Battery ---
    NimBLEService *bat = bleServer->createService("180F");
    NimBLECharacteristic *bl = bat->createCharacteristic("2A19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint8_t bv = 100;
    bl->setValue(&bv, 1);
    // NimBLE 2.x: services are started automatically when the server/advertising starts
  }

  bleAdvStopNow();
  bleAdv->clearData();
  bleAdv->setName("WatchOut Remote");
  bleAdv->addServiceUUID(bleHidSvc->getUUID());     // phones filter HID devices by this UUID
  bleAdv->setAppearance(0x03C0);                    // generic HID
  bool ok = bleAdv->start();
  bleHidOn = ok;
  bleHidMsg = ok ? "Pair WatchOut Remote" : "Advertise failed";
}

// ---------------- BLE scan (NimBLE) ----------------
void startBLEScan() {
  if (!bleEnsureInit()) {
    startLines();
    addLine("BLE init failed", "", C_RED, C_RED);
    finishLines();
    return;
  }
  bleScanBusy = true;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(80);
  NimBLEScanResults res = scan->getResults(2000, false);          // blocking 2 s scan
  int n = res.getCount();
  for (int i = 0; i < n; i++) {
    const NimBLEAdvertisedDevice *d = res.getDevice((uint32_t)i);
    if (!d) continue;
    String mac = String(d->getAddress().toString().c_str());
    mac.toUpperCase();
    String name = d->getName().length() ? String(d->getName().c_str()) : String("Unknown device");
    int rssi = d->getRSSI();
    bool found = false;
    for (int j = 0; j < bleCount; j++) {
      if (bleList[j].mac == mac) {
        bleList[j].name = name;
        bleList[j].rssi = rssi;
        bleList[j].lastSeen = millis();
        found = true;
        break;
      }
    }
    if (!found && bleCount < MAX_BLE_ENTRIES) {
      bleList[bleCount] = {name, mac, rssi, millis()};
      bleCount++;
    }
  }
  scan->clearResults();
  bleScanBusy = false;
  bleStarted = false;

  for (int i = 0; i < bleCount; i++) {
    if (millis() - bleList[i].lastSeen > BLE_ENTRY_TTL_MS) {
      for (int j = i; j < bleCount - 1; j++) bleList[j] = bleList[j + 1];
      bleCount--;
      i--;
    }
  }
}

void buildBleList() {
  startLines();
  for (int i = 0; i < bleCount; i++) {
    BLEEntry &d = bleList[i];
    addLine(d.name, String(d.rssi), C_WHITE, rssiColor(d.rssi));
  }
  addLine("> Scan again", "", C_CYAN, C_CYAN);
  finishLines();
}

void buildBleDetail() {
  startLines();
  if (bleSelectedIndex >= bleCount) {
    addLine("Not available", "", C_YELLOW, C_YELLOW);
    finishLines();
    return;
  }
  BLEEntry &d = bleList[bleSelectedIndex];
  addKV("Name", d.name, C_WHITE);
  addKV("MAC", d.mac, C_CYAN);
  addKV("RSSI", String(d.rssi) + " dBm", rssiColor(d.rssi));
  addKV("Signal", String(getSignalQuality(d.rssi)) + "%", rssiColor(d.rssi));
  addKV("Seen", String((millis() - d.lastSeen) / 1000) + "s ago", C_WHITE);
  finishLines();
}

void bleDoScan() {
  startLines();
  addLine("Scanning...", "", C_YELLOW, C_YELLOW);
  finishLines();
  pageSel = 0;
  scrollPos = 0.0f;
  scrollVel = 0.0f;
  renderUIStatic();
  bleStarted = true;
  startBLEScan();
  buildBleList();
}

String bleSysStatus() {
  if (bleHidConnected) return "HID LINK";
  if (bleHidOn) return "HID ADV";
  return "READY";
}

// Adds one Media Cmd row and remembers what a click on it does
static void bleAddActRow(int act, const char *label, uint16_t col, const char *right = "") {
  if (lineCount >= MAX_LINES) return;
  bleRowAct[lineCount] = act;
  if ((act & BLE_ACT_KEY) && bleFirstKeyRow < 0) bleFirstKeyRow = lineCount;
  addLine(label, right, col, col);
}

void buildBle() {
  startLines();
  for (int i = 0; i < MAX_LINES; i++) bleRowAct[i] = BLE_ACT_NONE;
  bleFirstKeyRow = -1;

  switch (subView) {
    case 1:
      if (bleDetailView) buildBleDetail();
      else buildBleList();
      return;                                     // these builders call finishLines() themselves
    case 2: {
      const char *st = !bleHidOn ? "OFF" : (bleHidReady ? "LINK" : (bleHidConnected ? "PAIR" : "ADV"));
      uint16_t sc = bleHidReady ? C_GREEN : (bleHidOn ? C_YELLOW : C_RED);
      bleRowAct[lineCount] = bleHidOn ? BLE_ACT_STOP : BLE_ACT_START;   // click on the status row toggles
      addLine("HID", st, C_WHITE, sc);
      if (bleHidOn) {
        if (!bleHidConnected) addWrapped("Pair 'WatchOut Remote' in BT settings", C_MUTED);
        if (bleHidMsg.length()) addWrapped(bleHidMsg, C_CYAN);
        bleAddActRow(BLE_ACT_KEY | HID_PLAY_PAUSE, "Play / Pause", C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_VOL_UP,     "Volume +",     C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_VOL_DOWN,   "Volume -",     C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_NEXT,       "Next Track",   C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_PREV,       "Prev Track",   C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_MUTE,       "Mute",         C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_STOP,       "Stop Media",   C_WHITE);
        bleAddActRow(BLE_ACT_KEY | HID_VOL_UP,     "Camera Shutter", C_WHITE);
        bleAddActRow(BLE_ACT_UNPAIR,               "> Unpair all", C_YELLOW);
        bleAddActRow(BLE_ACT_STOP,                 "> Stop",       C_RED);
      } else {
        if (bleHidMsg.length()) addWrapped(bleHidMsg, C_YELLOW);
        addWrapped("Media and camera controls over BLE HID", C_MUTED);
        bleAddActRow(BLE_ACT_START, "> Start", C_GREEN);
      }
      break;
    }
    default:
      for (int i = 0; i < BLE_MENU_COUNT; i++) {
        String st = "";
        if (i == 1 && bleHidOn) st = bleHidConnected ? "LINK" : "ADV";
        addLine(BLE_MENU_ITEMS[i], st, C_WHITE, st.length() ? C_GREEN : C_WHITE);
      }
      break;
  }
  finishLines();
}

void bleClick() {
  switch (subView) {
    case 0:
      if (pageSel < BLE_MENU_COUNT) {
        subParentSel = pageSel;
        subView = pageSel == 0 ? 1 : 2;
        pageSel = 0;
        scrollPos = 0.0f;
        scrollVel = 0.0f;
        if (subView == 1) {
          bleDoScan();
        } else {
          bleMediaStart();
          buildBle();
          if (bleFirstKeyRow > 0) {               // start on the first key instead of the status row
            pageSel = bleFirstKeyRow;
            scrollPos = (float)pageSel;
          }
        }
      }
      break;
    case 1:
      if (!bleDetailView) {
        if (bleCount > 0 && pageSel < bleCount) {
          bleSelectedIndex = pageSel;
          bleDetailView = true;
          pageSel = 0;
          scrollPos = 0.0f;
          scrollVel = 0.0f;
        } else {
          bleDoScan();
        }
      }
      break;
    case 2: {
      int act = (pageSel >= 0 && pageSel < MAX_LINES) ? bleRowAct[pageSel] : BLE_ACT_NONE;
      if (act & BLE_ACT_KEY) {
        uint8_t bit = (uint8_t)(act & 0xFF);
        const char *name = "key";
        switch (bit) {
          case HID_PLAY_PAUSE: name = "Play/Pause"; break;
          case HID_VOL_UP:     name = (lines[pageSel].l == "Camera Shutter") ? "Shutter" : "Vol+"; break;
          case HID_VOL_DOWN:   name = "Vol-"; break;
          case HID_NEXT:       name = "Next"; break;
          case HID_PREV:       name = "Prev"; break;
          case HID_MUTE:       name = "Mute"; break;
          case HID_STOP:       name = "Stop"; break;
        }
        bleMediaKey(bit, name);
      } else if (act == BLE_ACT_STOP) {
        bleStopTx();
        bleHidMsg = "";
        pageSel = 0;
        scrollPos = 0.0f;
        scrollVel = 0.0f;
      } else if (act == BLE_ACT_START) {
        bleMediaStart();
      } else if (act == BLE_ACT_UNPAIR) {
        NimBLEDevice::deleteAllBonds();
        if (bleServer && bleHidConnected) bleServer->disconnect(bleConnHandle);
        bleHidMsg = "Bonds cleared";
      }
      break;
    }
  }
  buildBle();
}

void bleBack() {
  if (subView == 1 && bleDetailView) {
    bleDetailView = false;
    pageSel = bleSelectedIndex;
    scrollPos = (float)pageSel;
    scrollVel = 0.0f;
    buildBle();
    return;
  }
  if (subView != 0) {
    bleStopTx();
    subView = 0;
    pageSel = subParentSel;
    scrollPos = (float)pageSel;
    scrollVel = 0.0f;
    buildBle();
  }
}

void pollBle() {
  if (!bleNimReady) return;
  unsigned long now = millis();

  // non-blocking key release
  if (bleKeyPending && (long)(now - bleKeyReleaseAt) >= 0) {
    bleMediaSend(0);
    bleKeyPending = false;
  }
  // keep advertising while we wait for a phone / PC (advertising stops after a disconnect)
  if (bleHidOn && !bleHidConnected && bleAdv && !bleAdv->isAdvertising()) {
    bleAdv->start();
  }

  // refresh the screen + LED when the link state changes
  static bool lastConn = false;
  static bool lastReady = false;
  bool c = bleHidConnected;
  bool r = bleHidReady;
  if (c != lastConn || r != lastReady) {
    if (r && !lastReady) { ledFlash(0, 255, 0, 300); bleHidMsg = "Linked"; }
    if (!c && lastConn && bleHidOn) bleHidMsg = "Disconnected";
    lastConn = c;
    lastReady = r;
    if (page == BLE_PAGE) lastDraw = 0;            // rebuild on the next loop pass
  }
}

int placeState = 0;
int placeHttp = 0;
String placeName = "";
String placeRegion = "";

void buildGPSPosition() {
  startLines();
  bool fix = gps.location.isValid() && gps.location.age() < 5000;
  bool uart = gps.charsProcessed() > 0;

  addLine("UART", uart ? "OK" : "NO DATA", C_MUTED, uart ? C_GREEN : C_RED);
  addLine("Fix", fix ? "VALID" : "NO FIX", C_MUTED, fix ? C_GREEN : C_YELLOW);

  if (placeState == 3) {
    addWrapped(placeName, C_CYAN);
    if (placeRegion.length()) addWrapped(placeRegion, C_WHITE);
  } else if (placeState == 1) {
    addLine("Looking up...", "", C_YELLOW, C_YELLOW);
  } else if (placeState == 2) {
    addLine("Wi-Fi failed", "", C_RED, C_RED);
  } else if (placeState == 4) {
    addLine("Lookup failed", "", C_RED, C_RED);
    addLine("HTTP", String(placeHttp), C_MUTED, C_WHITE);
  } else {
    addLine("Place", fix ? "CLICK" : "NO FIX", C_MUTED, fix ? C_CYAN : C_YELLOW);
  }

  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
  addLine("HDOP", gps.hdop.isValid() ? String(gps.hdop.hdop(), 1) : String("--"), C_MUTED, C_WHITE);
  addLine("Lat", fix ? String(gps.location.lat(), 6) : String("--"), C_MUTED, fix ? C_WHITE : C_MUTED);
  addLine("Lon", fix ? String(gps.location.lng(), 6) : String("--"), C_MUTED, fix ? C_WHITE : C_MUTED);
  addLine("Alt", gps.altitude.isValid() ? String(gps.altitude.meters(), 1) + " m" : String("--"), C_MUTED, C_WHITE);
  addLine("Speed", gps.speed.isValid() ? String(gps.speed.kmph(), 1) + " km/h" : String("--"), C_MUTED, C_WHITE);
  addLine("Chars", String(gps.charsProcessed()), C_MUTED, C_WHITE);
  finishLines();
}

WebServer webServer(80);
bool trkServerUp = false;
bool trkMdns = false;
bool trkHandlers = false;
volatile uint32_t trkSent = 0;
volatile int trkLastCode = 0;
volatile bool trkSending = false;
unsigned long lastTrkSend = 0;
unsigned long lastTrkWifiTry = 0;

static const char TRACK_HTML[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>WatchOut Tracker</title>
<link rel=stylesheet href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>html,body{margin:0;height:100%;font-family:sans-serif}#m{height:100%}
#i{position:absolute;z-index:999;left:8px;right:8px;top:8px;background:#000c;color:#fff;
padding:8px 12px;border-radius:12px;font-size:14px}#i a{color:#6cf}</style></head><body>
<div id=i>Waiting for GPS...</div><div id=m></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
var map=L.map('m').setView([20,78],4),mk=null,first=true,
tr=L.polyline([],{color:'#0af',weight:4}).addTo(map);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap'}).addTo(map);
function tick(){fetch('/loc').then(r=>r.json()).then(d=>{
var i=document.getElementById('i');
if(!d.fix){i.textContent='No GPS fix ('+d.sats+' sats)';return;}
var p=[d.lat,d.lon];
if(!mk)mk=L.marker(p).addTo(map);else mk.setLatLng(p);
tr.addLatLng(p);
if(first){map.setView(p,17);first=false}else map.panTo(p);
i.innerHTML=d.lat.toFixed(5)+', '+d.lon.toFixed(5)+' &middot; '+d.spd+' km/h &middot; '+d.sats+
' sats &middot; <a href="https://www.google.com/maps?q='+d.lat+','+d.lon+'">Google Maps</a>';
}).catch(function(){})}
tick();setInterval(tick,3000);
</script></body></html>)HTML";

// ---------------- LIVE GPS TRACKER ----------------
unsigned long lastTrkBeacon = 0;
uint32_t trkBeacons = 0;

bool trackerEpoch(uint32_t &ep) {
  if (!(gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2024)) return false;
  int64_t e = (int64_t)daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day()) * 86400LL +
              gps.time.hour() * 3600LL + gps.time.minute() * 60LL + gps.time.second();
  ep = (uint32_t)e;
  return true;
}

void trackerHandleRoot() { webServer.send_P(200, "text/html", TRACK_HTML); }

void trackerHandleLoc() {
  bool fix = gps.location.isValid() && gps.location.age() < 10000;
  String j = "{\"fix\":";
  j += fix ? "true" : "false";
  j += ",\"sats\":" + String(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  if (fix) {
    j += ",\"lat\":" + String(gps.location.lat(), 6);
    j += ",\"lon\":" + String(gps.location.lng(), 6);
    j += ",\"spd\":" + String(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 1);
    j += ",\"alt\":" + String(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 1);
  }
  j += "}";
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(200, "application/json", j);
}

void startTracker() {
  trackerOn = true;
  trkServerUp = false;
  trkLastCode = 0;
  lastTrkSend = 0;
  lastTrkBeacon = 0;
  if (!trkHandlers) {
    webServer.on("/", trackerHandleRoot);
    webServer.on("/loc", trackerHandleLoc);
    trkHandlers = true;
  }
  wifiBegin();
  lastTrkWifiTry = millis();
#if TRACKER_LORA_BEACON
  if (!loraOK) startLoRa();
#endif
}

void stopTracker() {
  trackerOn = false;
  if (trkServerUp) { webServer.stop(); trkServerUp = false; }
  if (trkMdns) { MDNS.end(); trkMdns = false; }
  if (!apHold) WiFi.disconnect(false, false);
}

static void trackerUploadTask(void *arg) {
  String *url = (String *)arg;
  int code = -100;
  if (url->startsWith("https")) {
    WiFiClientSecure secure;
    secure.setInsecure();
    HTTPClient http;
    if (http.begin(secure, *url)) {
      http.setConnectTimeout(4000);
      http.setTimeout(4000);
      code = http.GET();
      http.end();
    }
  } else {
    WiFiClient plain;
    HTTPClient http;
    if (http.begin(plain, *url)) {
      http.setConnectTimeout(4000);
      http.setTimeout(4000);
      code = http.GET();
      http.end();
    }
  }
  trkLastCode = code;
  if (code == 200) trkSent = trkSent + 1;
  delete url;
  trkSending = false;
  vTaskDelete(NULL);
}

bool trackerFire(const String &u) {
  if (trkSending) return false;
  String *url = new String(u);
  trkSending = true;
  if (xTaskCreate(trackerUploadTask, "trkUp", 10240, url, 1, NULL) != pdPASS) {
    delete url;
    trkSending = false;
    trkLastCode = -101;
    return false;
  }
  return true;
}

void trackerSend() {
  if (trkSending) return;
  lastTrkSend = millis();
  trackerFire(
    String(TRACKER_SERVER) + "/?id=" + TRACKER_ID +
    "&lat=" + String(gps.location.lat(), 6) +
    "&lon=" + String(gps.location.lng(), 6) +
    "&speed=" + String(gps.speed.isValid() ? gps.speed.knots() : 0.0, 1) +
    "&bearing=" + String(gps.course.isValid() ? gps.course.deg() : 0.0, 0) +
    "&altitude=" + String(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 1) +
    "&hdop=" + String(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, 1)
  );
}

void trackerBeacon() {
#if TRACKER_LORA_BEACON
  if (!loraOK) return;
  uint32_t ep = 0;
  trackerEpoch(ep);
  LoRa.beginPacket();
  LoRa.print(String("TRK,") + TRACKER_ID + "," + String(gps.location.lat(), 6) + "," +
             String(gps.location.lng(), 6) + "," +
             String(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 1) + "," + String(ep));
  LoRa.endPacket();
  LoRa.receive();
  loraTx++;
  trkBeacons++;
#endif
}

void trackerPoll() {
  if (!trackerOn) return;
  unsigned long now = millis();
  bool fix = gps.location.isValid() && gps.location.age() < 5000;

#if TRACKER_LORA_BEACON
  if (fix && now - lastTrkBeacon >= (unsigned long)TRACKER_BEACON_S * 1000UL) {
    lastTrkBeacon = now;
    trackerBeacon();
  }
#endif

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastTrkWifiTry > 15000 && apState != 1 && apState != 5) {
      lastTrkWifiTry = now;
      wifiBegin();
    }
    return;
  }
  if (!trkServerUp) {
    webServer.begin();
    trkServerUp = true;
    if (!trkMdns && MDNS.begin("watchout")) {
      MDNS.addService("http", "tcp", 80);
      trkMdns = true;
    }
  }
  webServer.handleClient();

  if (!trkSending && fix && strlen(TRACKER_SERVER) > 0 &&
      now - lastTrkSend >= (unsigned long)TRACKER_INTERVAL_S * 1000UL) {
    trackerSend();
  }
}

void buildGPSTracker() {
  startLines();
  bool wifi = WiFi.status() == WL_CONNECTED;
  bool fix = gps.location.isValid() && gps.location.age() < 5000;
  addLine("Tracker", trackerOn ? "ON" : "OFF", C_WHITE, trackerOn ? C_GREEN : C_RED);
  if (trackerOn) {
    addLine("Wi-Fi", wifi ? "ONLINE" : "OFFLINE", C_MUTED, wifi ? C_GREEN : C_YELLOW);
    if (wifi) {
      addKV("Open", WiFi.localIP().toString(), C_CYAN);
      addKV("Or", "watchout.local", C_CYAN);
    }
  }
  addLine("Fix", fix ? "VALID" : "NO FIX", C_MUTED, fix ? C_GREEN : C_YELLOW);
  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
#if TRACKER_LORA_BEACON
  addLine("Beacons", String((unsigned long)trkBeacons), C_MUTED, C_WHITE);
#endif
  if (strlen(TRACKER_SERVER) > 0) {
    int code = trkLastCode;
    addLine("Uploaded", String((unsigned long)trkSent), C_MUTED, C_WHITE);
    addLine("Server", code == 0 ? String("--") : (code == 200 ? String("OK") : String(code)),
            C_MUTED, code == 200 ? C_GREEN : (code == 0 ? C_MUTED : C_RED));
  }
  finishLines();
}

void buildGPS() {
  if (subView == 1) buildGPSPosition();
  else if (subView == 2) buildGPSTracker();
  else {
    startLines();
    addLine("Position", "", C_WHITE, C_WHITE);
    addLine("Tracker", trackerOn ? "ON" : "OFF", C_WHITE, trackerOn ? C_GREEN : C_MUTED);
    finishLines();
  }
}

void startLoRa() {
  digitalWrite(LCD_CS, HIGH);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  loraOK = LoRa.begin(LORA_FREQ);
  loraTried = true;
  if (loraOK) {
    LoRa.receive();
    Serial.println("LoRa ready at 433 MHz");
  } else {
    Serial.println("LoRa init failed: check wiring and 3.3V supply");
  }
}

// ======================= NRF24 SPECTRUM ANALYZER v2 =======================
// * continuous, non-blocking sweep of all 84 channels (2400..2483 MHz), a few ms per loop pass
// * INTERLEAVED sampling: NRF_SAMPLES full passes over all channels per sweep, so a short burst
//   is spread over time instead of being counted 4x or missed 4x
// * time-based smoothing (attack / release / peak fall are in ms, independent of the sweep rate)
// * four views (click to cycle): BARS (+ peak hold), FALL (waterfall history),
//   WIFI (load per Wi-Fi channel 1..13), DEVS (automatic signal identification)
// * DEVS: Wi-Fi, BLE adverts, ZigBee, wideband/microwave, frequency hoppers, continuous carriers,
//   narrow devices. Wi-Fi signals are labelled with real SSIDs from a Wi-Fi scan.
// * colored identification strip drawn over BARS / FALL, the marker readout names the device
// * long-term (2.5 s) average is used for classification so short bursts don't flicker the list
// * stick up/down = move a frequency marker (BARS/FALL) or select a signal (DEVS)
// * long press = HOLD / resume (resume also clears the peak markers)
// * Wi-Fi channel n is centred on 2407 + 5n MHz, i.e. nRF channel 7 + 5n
// NOTE: RPD only reports "signal stronger than -64 dBm on this channel", so the percentage is how
// often the channel was busy, not a calibrated power level, and identification is pattern based
// (width, position, persistence). Labels with a '?' are best guesses.
// While the Wi-Fi scan runs (~2-3 s) the nRF sweep is paused so the ESP32's own
// transmissions are not mistaken for outside signals.

// ---------------- device identification state ----------------
#define NRF_SLOW_MS  2500.0f         // time constant of the long-term average used for classification
#define NRF_MAXDEV   8

enum NrfDevType : uint8_t { DEV_WIFI, DEV_BLE, DEV_ZIGBEE, DEV_WIDE, DEV_HOP, DEV_CARRIER, DEV_NARROW };

struct NrfDev {
  uint8_t type;
  uint8_t lo, hi;                    // channel range (0..83)
  uint8_t level;                     // activity % (long-term)
  char name[16];
  char note[36];
};

float nrfSlow[NRF_CH] = {};          // long-term activity per channel (0..100)
NrfDev nrfDev[NRF_MAXDEV];
int nrfDevCount = 0;
int nrfDevSel = 0;
unsigned long nrfLastClassify = 0;
unsigned long nrfLastWifiScan = 0;

void startNRF24() {
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  nrf24Ready = nrf24.begin() && nrf24.isChipConnected();
  nrf24Scanned = false;
  if (!nrf24Ready) {
    Serial.println("[NRF24] begin() failed - radio not found");
    return;
  }

  Serial.println("[NRF24] begin() succeeded - radio ready");
  nrf24.setAutoAck(false);
  nrf24.disableCRC();
  nrf24.setDataRate(RF24_1MBPS);
  nrf24.setPALevel(RF24_PA_MIN);
  nrf24.setChannel(76);
  nrf24.stopListening();
}

static void nrfJamInit() {
  if (!nrf24Ready) return;
  nrf24.stopListening();
  nrf24.openWritingPipe(NRF_JAM_PIPE_ADDR);
  nrf24.setChannel(jamChannel);
  nrf24.setPALevel(RF24_PA_MAX);
  nrf24.setDataRate(RF24_1MBPS);
  nrf24.disableCRC();
  nrf24.setAutoAck(false);
}

void nrfJamStart() {
  if (!nrf24Ready) startNRF24();
  if (!nrf24Ready) return;
  nrfJamActive = true;
  nrfJamInit();
  nrfJamTimer = millis();
  uiDirty = true;
}

void nrfJamStop() {
  nrfJamActive = false;
  nrf24.stopListening();
  nrf24.setChannel(76);
  uiDirty = true;
}

// Called every time the Spectrum sub view opens: clears all history and restarts the sweep
void nrfSpectrumBegin() {
  memset(nrf24Activity, 0, sizeof(nrf24Activity));
  memset(nrfAvg, 0, sizeof(nrfAvg));
  memset(nrfSlow, 0, sizeof(nrfSlow));
  memset(nrfPeak, 0, sizeof(nrfPeak));
  memset(nrfShown, 0, sizeof(nrfShown));
  memset(nrfHits, 0, sizeof(nrfHits));
  memset(nrfWf, 0, sizeof(nrfWf));
  nrfWfHead = 0;
  nrfScanCh = 0;
  nrfPass = 0;
  nrfCursor = -1;
  nrfHold = false;
  nrfSweeps = 0;
  nrfSweepRate = 0.0f;
  nrfLastRow = 0;
  nrfDevCount = 0;
  nrfDevSel = 0;
  nrfLastClassify = 0;
  nrf24Scanned = false;
  if (!nrf24Ready || !nrf24.isChipConnected()) startNRF24();   // retry if the radio was missing before

  nrfLastWifiScan = millis();                                  // Wi-Fi scan -> SSID labels for detected Wi-Fi signals
  startWiFiScan();
  uiDirty = true;
}

// ---------------- device identification ----------------
static void nrfAddDev(uint8_t type, int lo, int hi, float level, const String &name, const String &note) {
  if (nrfDevCount >= NRF_MAXDEV) return;
  if (lo < 0) lo = 0;
  if (hi > NRF_CH - 1) hi = NRF_CH - 1;
  int lv = (int)lroundf(level);
  if (lv < 0) lv = 0;
  if (lv > 100) lv = 100;
  NrfDev &d = nrfDev[nrfDevCount++];
  d.type = type;
  d.lo = (uint8_t)lo;
  d.hi = (uint8_t)hi;
  d.level = (uint8_t)lv;
  strlcpy(d.name, name.c_str(), sizeof(d.name));
  strlcpy(d.note, note.c_str(), sizeof(d.note));
}

// strongest SSID seen by the Wi-Fi scan on channel n, and how many APs are on it
static String nrfSsidOn(int n, int &aps) {
  aps = 0;
  int best = -200;
  String s = "";
  for (int i = 0; i < wifiCount; i++) {
    if (wifiList[i].channel != n) continue;
    aps++;
    if (wifiList[i].rssi > best) {
      best = wifiList[i].rssi;
      s = wifiList[i].ssid.length() ? wifiList[i].ssid : String("<hidden>");
    }
  }
  return s;
}

static void nrfClassify() {
  nrfDevCount = 0;
  int wifiC[13];
  int wifiN = 0;

  // ---- 1) Wi-Fi: a flat, ~20 MHz wide plateau centred on 2407 + 5n MHz ----
  float wl[15] = {};
  float fr[15] = {};
  for (int n = 1; n <= 13; n++) {
    int c = 7 + 5 * n;
    float sum = 0.0f;
    int cnt = 0, act = 0;
    for (int ch = c - 9; ch <= c + 9; ch++) {
      if (ch < 0 || ch >= NRF_CH) continue;
      sum += nrfSlow[ch];
      cnt++;
      if (nrfSlow[ch] > 4.0f) act++;
    }
    wl[n] = cnt ? sum / cnt : 0.0f;
    fr[n] = cnt ? (float)act / cnt : 0.0f;
  }
  for (int n = 1; n <= 13; n++) {
    if (wl[n] >= 9.0f && fr[n] >= 0.6f && wl[n] > wl[n - 1] && wl[n] >= wl[n + 1]) {
      int c = 7 + 5 * n;
      int aps = 0;
      String ssid = nrfSsidOn(n, aps);
      String note;
      if (ssid.length()) {
        note = ssid.substring(0, 24);
        if (aps > 1) note += " +" + String(aps - 1);
      } else {
        note = wifiCount ? "No AP seen on this ch" : "Run Wi-Fi scan for names";
      }
      nrfAddDev(DEV_WIFI, c - 10, c + 10, wl[n], "Wi-Fi ch " + String(n), note);
      if (wifiN < 13) wifiC[wifiN++] = c;
    }
  }

  // ---- 2) wideband: a long unbroken run of busy channels (too wide for one Wi-Fi channel) ----
  int run = 0, runStart = 0, bestRun = 0, bestStart = 0;
  float runSum = 0.0f, bestSum = 0.0f;
  for (int ch = 0; ch < NRF_CH; ch++) {
    if (nrfSlow[ch] > 6.0f) {
      if (!run) { runStart = ch; runSum = 0.0f; }
      run++;
      runSum += nrfSlow[ch];
      if (run > bestRun) { bestRun = run; bestStart = runStart; bestSum = runSum; }
    } else {
      run = 0;
    }
  }
  bool wide = false;
  if (bestRun >= 30 && bestSum / bestRun >= 14.0f) {
    wide = true;
    nrfAddDev(DEV_WIDE, bestStart, bestStart + bestRun - 1, bestSum / bestRun,
              "Wideband", "Microwave/jammer? " + String(2400 + bestStart) + "-" + String(2400 + bestStart + bestRun - 1));
  }

  // ---- 3) narrow spikes: BLE adverts, ZigBee, carriers, small 2.4G devices ----
  for (int ch = 0; ch < NRF_CH; ch++) {
    float v = nrfSlow[ch];
    if (v < 18.0f) continue;

    bool pk = true;
    for (int d = -2; d <= 2; d++) {
      if (d == 0) continue;
      int k = ch + d;
      if (k < 0 || k >= NRF_CH) continue;
      if (nrfSlow[k] > v || (nrfSlow[k] == v && d < 0)) { pk = false; break; }
    }
    if (!pk) continue;

    float nb = 0.0f;
    int cnt = 0;
    for (int d = 2; d <= 4; d++) {
      if (ch - d >= 0)       { nb += nrfSlow[ch - d]; cnt++; }
      if (ch + d < NRF_CH)   { nb += nrfSlow[ch + d]; cnt++; }
    }
    if (cnt) nb /= cnt;
    if (v - nb < 15.0f) continue;                       // not a spike, part of something wider

    int lo = ch, hi = ch;
    while (lo > 0 && nrfSlow[lo - 1] >= v * 0.5f) lo--;
    while (hi < NRF_CH - 1 && nrfSlow[hi + 1] >= v * 0.5f) hi++;
    if (hi - lo > 5) continue;                          // too wide to be a narrow signal

    String freq = String(2400 + ch) + " MHz";
    if (abs(ch - 2) <= 1 || abs(ch - 26) <= 1 || abs(ch - 80) <= 1) {
      int adv = abs(ch - 2) <= 1 ? 37 : (abs(ch - 26) <= 1 ? 38 : 39);
      nrfAddDev(DEV_BLE, lo, hi, v, "BLE advert", "Ch " + String(adv) + "  " + freq);
    } else if (ch >= 5 && ch <= 80 && (ch - 5) % 5 == 0 && v < 70.0f) {
      nrfAddDev(DEV_ZIGBEE, lo, hi, v, "ZigBee?", "802.15.4 ch " + String(11 + (ch - 5) / 5) + "  " + freq);
    } else if (v >= 70.0f) {
      nrfAddDev(DEV_CARRIER, lo, hi, v, freq, "Continuous: video/baby mon?");
    } else {
      nrfAddDev(DEV_NARROW, lo, hi, v, freq, "Mouse/kbd/toy/remote?");
    }
  }

  // ---- 4) frequency hopper: weak activity spread over many channels outside any Wi-Fi plateau ----
  int spread = 0;
  float hs = 0.0f;
  for (int ch = 0; ch < NRF_CH; ch++) {
    float v = nrfSlow[ch];
    if (v < 2.0f || v > 25.0f) continue;
    bool inWifi = false;
    for (int i = 0; i < wifiN; i++) if (abs(ch - wifiC[i]) <= 10) { inWifi = true; break; }
    if (inWifi) continue;
    spread++;
    hs += v;
  }
  if (!wide && spread >= 22) {
    nrfAddDev(DEV_HOP, 0, NRF_CH - 1, hs / spread, "Hopping", "Bluetooth/FHSS? " + String(spread) + " ch");
  }

  // sort strongest first (insertion sort, max 8 entries)
  for (int i = 1; i < nrfDevCount; i++) {
    NrfDev t = nrfDev[i];
    int j = i - 1;
    while (j >= 0 && nrfDev[j].level < t.level) { nrfDev[j + 1] = nrfDev[j]; j--; }
    nrfDev[j + 1] = t;
  }
  if (nrfDevSel >= nrfDevCount) nrfDevSel = nrfDevCount > 0 ? nrfDevCount - 1 : 0;
}

// A full sweep just finished: update smoothing, peaks, waterfall, classification and the sweep rate
static void nrfFinishSweep() {
  unsigned long now = millis();
  static unsigned long lastT = 0;
  float dt = lastT ? (float)(now - lastT) / 1000.0f : 0.07f;
  if (dt > 0.5f) dt = 0.5f;
  lastT = now;
  const float kA   = 1.0f - expf(-dt * 1000.0f / NRF_ATTACK_MS);
  const float kR   = 1.0f - expf(-dt * 1000.0f / NRF_RELEASE_MS);
  const float kS   = 1.0f - expf(-dt * 1000.0f / NRF_SLOW_MS);
  const float fall = NRF_PEAK_FALL * dt;

  bool pushRow = (now - nrfLastRow >= 80);
  if (pushRow) {
    nrfWfHead = (nrfWfHead + 1) % NRF_WF_ROWS;
    nrfLastRow = now;
  }

  for (int ch = 0; ch < NRF_CH; ch++) {
    float inst = (float)nrfHits[ch] * 100.0f / (float)NRF_SAMPLES;
    nrf24Activity[ch] = (uint8_t)lroundf(inst);

    float a = nrfAvg[ch];
    a += (inst - a) * (inst > a ? kA : kR);
    nrfAvg[ch] = a;

    nrfSlow[ch] += (inst - nrfSlow[ch]) * kS;

    if (a >= nrfPeak[ch]) {
      nrfPeak[ch] = a;
    } else {
      nrfPeak[ch] -= fall;
      if (nrfPeak[ch] < a) nrfPeak[ch] = a;
    }
    if (pushRow) nrfWf[nrfWfHead][ch] = (uint8_t)lroundf(a);
  }

  nrfSweeps++;
  static uint32_t markSweeps = 0;
  static unsigned long markT = 0;
  if (now - markT >= 1000) {
    if (markT != 0) nrfSweepRate = (float)(nrfSweeps - markSweeps) * 1000.0f / (float)(now - markT);
    markSweeps = nrfSweeps;
    markT = now;
  }

  if (now - nrfLastClassify >= 400) {              // classify a few times per second, not every sweep
    nrfLastClassify = now;
    nrfClassify();
  }

  if (!nrf24.isChipConnected()) {                  // radio unplugged / lost power
    nrf24Ready = false;
    return;
  }
  nrf24Scanned = true;
  uiDirty = true;
}

// Scans as many channels as fit into NRF_SLICE_US, then returns so the UI keeps running.
// Paused while a Wi-Fi scan is running (the ESP32's own probe requests would show up as signals).
void scanNRF24Spectrum() {
  if (!nrf24Ready || wifiStarted || nrfJamActive) return;
  unsigned long t0 = micros();
  while (micros() - t0 < (unsigned long)NRF_SLICE_US) {
    uint8_t ch = nrfScanCh;
    nrf24.setChannel(ch);
    nrf24.startListening();
    delayMicroseconds(NRF_DWELL_US);
    nrf24.stopListening();
    if (nrf24.testRPD()) nrfHits[ch]++;            // hits accumulate over the passes

    if (++nrfScanCh >= NRF_CH) {
      nrfScanCh = 0;
      if (++nrfPass >= NRF_SAMPLES) {
        nrfPass = 0;
        nrfFinishSweep();
        memset(nrfHits, 0, sizeof(nrfHits));       // start the next sweep clean
        break;
      }
    }
  }
}

// Collects the Wi-Fi scan result (pollWiFi() only runs on the Wi-Fi page) and refreshes it every 30 s in DEVS view
static void nrfWifiPoll() {
  if (wifiStarted) {
    int r = WiFi.scanComplete();
    if (r >= 0) {
      mergeWifiResults(r);
      wifiStarted = false;
      nrfLastClassify = 0;                         // re-classify right away with the new SSID names
    } else if (r == WIFI_SCAN_FAILED) {
      wifiStarted = false;
    }
  } else if (nrfView == 3 && millis() - nrfLastWifiScan > 30000UL) {
    nrfLastWifiScan = millis();
    startWiFiScan();
  }
}

void pollNRF24Spectrum() {
  if (page != NRF24_PAGE || subView != 2) return;

  if (nrfJamActive) {
    if (!nrf24Ready) return;
    if (millis() - nrfJamTimer >= NRF_JAM_INTERVAL_MS) {
      nrf24.stopListening();
      nrf24.setChannel(jamChannel);
      bool ok = nrf24.write(&jamPayload, 1);
      Serial.printf("[Jammer] wrote %s (payload 0x%02X) on channel %d\n",
                    ok ? "OK" : "FAIL", jamPayload, jamChannel);
      delayMicroseconds(100);
      nrfJamTimer = millis();
    }
    return;
  }

  nrfWifiPoll();
  if (!nrf24Ready || nrfHold) return;
  scanNRF24Spectrum();
}

// Long press inside the spectrum = freeze / resume
bool nrfLongClick() {
  if (page != NRF24_PAGE || subView != 2) return false;
  nrfHold = !nrfHold;
  if (!nrfHold) {                                  // resuming: restart the peak markers
    for (int ch = 0; ch < NRF_CH; ch++) nrfPeak[ch] = nrfAvg[ch];
  }
  ledFlash(255, 159, 10, 200);
  uiDirty = true;
  return true;
}

// Stick up/down: moves the frequency marker (BARS/FALL) or selects a signal (DEVS).
void nrfMoveCursor(int d) {
  if (nrfView == 3) {
    if (nrfDevCount > 0) {
      int s = nrfDevSel + d;
      if (s < 0) s = 0;
      if (s > nrfDevCount - 1) s = nrfDevCount - 1;
      nrfDevSel = s;
      uiDirty = true;
    }
    return;
  }
  int c = nrfCursor + d;
  if (c < -1) c = -1;
  if (c > NRF_CH - 1) c = NRF_CH - 1;
  if (nrfCursor < 0 && c >= 0) {                   // first move: start at the strongest channel
    int pk = 0;
    for (int i = 1; i < NRF_CH; i++) if (nrfShown[i] > nrfShown[pk]) pk = i;
    c = pk;
  }
  nrfCursor = c;
  uiDirty = true;
}

// Readout for the marker: frequency, activity, peak, matching Wi-Fi / BLE channel and detected device
static String nrfCursorInfo(int c) {
  String s = String(2400 + c) + "MHz " + String((int)lroundf(nrfShown[c])) + "% pk" +
             String((int)lroundf(nrfPeak[c]));
  if (c == 2 || c == 26 || c == 80) {
    s += " BLE" + String(c == 2 ? 37 : (c == 26 ? 38 : 39));
  } else {
    int n = (int)lroundf((c - 7) / 5.0f);
    if (n >= 1 && n <= 13 && abs(c - (7 + 5 * n)) <= 2) s += " W" + String(n);
  }
  for (int i = 0; i < nrfDevCount; i++) {
    if (nrfDev[i].type != DEV_HOP && c >= nrfDev[i].lo && c <= nrfDev[i].hi) {
      s += String(" ") + nrfDev[i].name;
      break;
    }
  }
  return s;
}

void buildNRF24Info() {
  addLine("Radio", nrf24Ready ? "CONNECTED" : "NOT FOUND",
          C_MUTED, nrf24Ready ? C_GREEN : C_RED);
  if (!nrf24Ready) {
    addWrapped("Check 3.3V, GND, SPI, CE/CSN", C_YELLOW);
    return;
  }

  addLine("Chip", "nRF24L01+", C_MUTED, C_WHITE);
  addLine("CE / CSN", String(NRF24_CE) + " / " + String(NRF24_CSN), C_MUTED, C_WHITE);
  addLine("SPI", "SCK13 MISO12 MOSI11", C_MUTED, C_WHITE);
  addLine("Channel", String(nrf24.getChannel()), C_MUTED, C_WHITE);
  addLine("Frequency", String(2400 + nrf24.getChannel()) + " MHz", C_MUTED, C_WHITE);
  addLine("Data rate", "1 Mbps", C_MUTED, C_WHITE);
  addLine("Power", "MIN", C_MUTED, C_WHITE);
  addLine("ISM range", "2400-2483 MHz", C_MUTED, C_WHITE);
  addLine("Samples/ch", String(NRF_SAMPLES), C_MUTED, C_WHITE);
  addLine("Sweeps/s", String(nrfSweepRate, 1), C_MUTED, C_WHITE);
  addLine("Signals", String(nrfDevCount), C_MUTED, C_WHITE);
}

void buildNRF24() {
  startLines();
  switch (subView) {
    case 1:
      buildNRF24Info();
      break;
    case 2:
      break;
    default:
      addLine("Information", nrf24Ready ? "READY" : "NO RADIO",
              C_WHITE, nrf24Ready ? C_GREEN : C_RED);
      addLine("Spectrum", nrf24Ready ? "LIVE" : "NO RADIO",
              C_WHITE, nrf24Ready ? C_CYAN : C_MUTED);
      break;
  }
  finishLines();
}

// Heat map colour for 0..100 %: dark blue -> cyan -> yellow -> red
static uint16_t nrfHeat(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 100.0f) v = 100.0f;
  float t = v / 100.0f;
  float r, g, b;
  if (t < 0.33f) {
    float k = t / 0.33f;
    r = 0; g = 200.0f * k; b = 60.0f + 140.0f * k;
  } else if (t < 0.66f) {
    float k = (t - 0.33f) / 0.33f;
    r = 255.0f * k; g = 200.0f + 55.0f * k; b = 200.0f * (1.0f - k);
  } else {
    float k = (t - 0.66f) / 0.34f;
    r = 255; g = 255.0f * (1.0f - k); b = 0;
  }
  return rgb565((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

// Cached heat colours (101 entries, built once) - avoids float math on every bar / pixel
static uint16_t heatC(float v) {
  static uint16_t tab[101];
  static bool init = false;
  if (!init) {
    for (int i = 0; i <= 100; i++) tab[i] = nrfHeat((float)i);
    init = true;
  }
  int i = (int)(v + 0.5f);
  if (i < 0) i = 0;
  if (i > 100) i = 100;
  return tab[i];
}

// Load of Wi-Fi channel n (1..13) in percent: weighted mean over its 22 MHz width blended with the strongest inner channel
static float nrfWifiLoad(int n) {
  int c = 7 + 5 * n;
  float sum = 0.0f, wsum = 0.0f, mx = 0.0f;
  for (int d = -10; d <= 10; d++) {
    int ch = c + d;
    if (ch < 0 || ch >= NRF_CH) continue;
    float w = 1.0f - fabsf((float)d) / 12.0f;
    sum += nrfShown[ch] * w;
    wsum += w;
    if (d >= -4 && d <= 4 && nrfShown[ch] > mx) mx = nrfShown[ch];
  }
  float mean = wsum > 0.0f ? sum / wsum : 0.0f;
  return (mean + mx) * 0.5f;
}

// colour per identified device type
static uint16_t nrfDevColor(uint8_t t) {
  switch (t) {
    case DEV_WIFI:    return themeColor(C_CYAN);
    case DEV_BLE:     return themeColor(rgb565(120, 120, 255));
    case DEV_ZIGBEE:  return themeColor(rgb565(255, 159, 10));
    case DEV_WIDE:    return themeColor(C_RED);
    case DEV_HOP:     return themeColor(rgb565(200, 100, 255));
    case DEV_CARRIER: return themeColor(C_YELLOW);
    default:          return themeColor(C_GREEN);
  }
}

void renderNRFSpectrum() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  g->fillScreen(bgColor());

  static const char *VIEW_NAMES[] = {"BARS", "FALL", "WIFI", "DEVS"};
  const uint16_t cW = themeColor(C_WHITE);
  const uint16_t cM = themeColor(C_MUTED);

  boldText(g, 10, 8, String("NRF24 ") + VIEW_NAMES[nrfView], themeColor(C_CYAN), 2, 1);
  if (nrfHold) boldText(g, 228 - 48, 8, "HOLD", themeColor(C_RED), 2, 1);
  else if (wifiStarted) boldText(g, 228 - 48, 8, "SCAN", themeColor(C_YELLOW), 2, 1);

  if (!nrf24Ready) {
    boldText(g, 10, 90, "RADIO NOT FOUND", themeColor(C_RED), 2, 1);
    boldText(g, 10, 116, "Check wiring and 3.3V", themeColor(C_MUTED), 1, 1);
    flushCanvas();
    return;
  }
  if (!nrf24Scanned) {
    boldText(g, 10, 90, "Scanning...", themeColor(C_YELLOW), 2, 1);
    flushCanvas();
    return;
  }

  const int L = 24, R = 232, T = 55, B = 242, H = B - T;
  const int W = R - L;

  // glide the drawn values towards the smoothed ones; keep redrawing until they settle
  bool moving = false;
  for (int ch = 0; ch < NRF_CH; ch++) {
    float diff = nrfAvg[ch] - nrfShown[ch];
    if (fabsf(diff) > 0.4f) {
      nrfShown[ch] += diff * 0.35f;
      moving = true;
    } else {
      nrfShown[ch] = nrfAvg[ch];
    }
  }
  if (moving) uiDirty = true;

  // strongest channel + average occupancy
  int pk = 0;
  float total = 0.0f;
  for (int ch = 0; ch < NRF_CH; ch++) {
    if (nrfShown[ch] > nrfShown[pk]) pk = ch;
    total += nrfShown[ch];
  }
  int avgBusy = (int)lroundf(total / (float)NRF_CH);

  // wifi load (needed for the WIFI view and the "best channel" hint)
  float wl[13];
  int bestWifi = 0;
  for (int n = 1; n <= 13; n++) {
    wl[n - 1] = nrfWifiLoad(n);
    if (wl[n - 1] < wl[bestWifi]) bestWifi = n - 1;
  }

  // info lines
  String info;
  if (nrfView == 3) info = String(nrfDevCount) + (nrfDevCount == 1 ? " signal found" : " signals found");
  else if (nrfView == 2) info = "BEST CH " + String(bestWifi + 1) + " (" + String((int)lroundf(wl[bestWifi])) + "% busy)";
  else if (nrfCursor >= 0) info = nrfCursorInfo(nrfCursor);
  else info = "PEAK " + String(2400 + pk) + " MHz " + String((int)lroundf(nrfShown[pk])) + "%";
  g->setTextSize(1);
  g->setTextWrap(false);
  g->setTextColor(cM);
  g->setCursor(10, 29);
  g->print(info);
  if (nrfView == 3 && nrfDevCount > 0) {
    String sel = String(nrfDevSel + 1) + "/" + String(nrfDevCount);
    g->setCursor(232 - (int)sel.length() * 6, 29);
    g->print(sel);
  }
  g->setCursor(10, 41);
  g->print(String("AVG ") + String(avgBusy) + "%");
  String rate = String(nrfSweepRate, 1) + " sw/s";
  g->setCursor(232 - (int)rate.length() * 6, 41);
  g->print(rate);

  if (nrfView == 3) {
    // ---------------- IDENTIFIED SIGNALS ----------------
    const int rowH = 40, y0 = 54, rows = 5;
    if (nrfDevCount == 0) {
      boldText(g, 10, 100, "No signals yet", themeColor(C_YELLOW), 2, 1);
      g->setTextSize(1);
      g->setTextColor(cM);
      g->setCursor(10, 128);
      g->print("Listening... needs ~3 s of data");
      g->setCursor(10, 142);
      g->print("Click = next view");
    } else {
      int top = nrfDevSel - 2;
      if (top > nrfDevCount - rows) top = nrfDevCount - rows;
      if (top < 0) top = 0;
      for (int r = 0; r < rows && top + r < nrfDevCount; r++) {
        int i = top + r;
        const NrfDev &d = nrfDev[i];
        int y = y0 + r * rowH;
        bool s = (i == nrfDevSel);
        uint16_t col = nrfDevColor(d.type);
        if (s) g->fillRoundRect(4, y, 232, rowH - 3, 5, lightMode ? 0xDEFB : 0x2104);
        g->fillCircle(16, y + 13, 6, col);
        boldText(g, 30, y + 6, String(d.name), s ? cW : cM, 2, 1);
        String lv = String((int)d.level) + "%";
        boldText(g, 228 - (int)lv.length() * 12, y + 6, lv, col, 2, 1);
        g->setTextSize(1);
        g->setTextColor(s ? cW : cM);
        g->setCursor(30, y + 25);
        g->print(d.note);
      }
    }
    flushCanvas();
    return;
  }

  if (nrfView == 1) {
    // ---------------- WATERFALL ----------------
    const uint16_t dark = rgb565(0, 0, 24);
    g->fillRect(L, T, W, H, dark);
    static uint16_t lut[101];
    for (int v = 0; v <= 100; v++) lut[v] = (v < 3) ? dark : heatC((float)v);
    uint8_t colCh[W];
    for (int x = 0; x < W; x++) colCh[x] = (uint8_t)(x * NRF_CH / W);

    if (canvasReady) {                                // fast path: write straight into the frame buffer
      uint16_t *buf = canvas->getBuffer();
      for (int a = 0; a < NRF_WF_ROWS; a++) {
        const uint8_t *row = nrfWf[(nrfWfHead - a + NRF_WF_ROWS) % NRF_WF_ROWS];
        int y = T + a * 2;
        if (y + 1 >= B + 1) break;
        uint16_t *p0 = buf + (size_t)y * 240 + L;
        uint16_t *p1 = p0 + 240;
        for (int x = 0; x < W; x++) {
          uint16_t c = lut[row[colCh[x]] > 100 ? 100 : row[colCh[x]]];
          p0[x] = c;
          p1[x] = c;
        }
      }
    } else {
      for (int a = 0; a < NRF_WF_ROWS; a++) {
        const uint8_t *row = nrfWf[(nrfWfHead - a + NRF_WF_ROWS) % NRF_WF_ROWS];
        int y = T + a * 2;
        for (int ch = 0; ch < NRF_CH; ch++) {
          if (row[ch] < 3) continue;
          int x0 = L + W * ch / NRF_CH;
          int x1 = L + W * (ch + 1) / NRF_CH;
          g->fillRect(x0, y, max(1, x1 - x0), 2, nrfHeat((float)row[ch]));
        }
      }
    }
    g->drawRect(L - 1, T - 1, W + 2, H + 2, cM);
  } else {
    // grid + axis labels
    for (int level = 0; level <= 100; level += 25) {
      int y = B - (H * level / 100);
      g->drawFastHLine(L, y, W, lightMode ? 0xDEFB : 0x2104);
      g->setTextSize(1);
      g->setTextColor(cW);
      g->setCursor(2, y - 3);
      g->print(level);
    }

    if (nrfView == 0) {
      // ---------------- BARS + PEAK HOLD ----------------
      const int SEG = 4;
      const int segN = H / SEG;
      uint16_t lut[segN];
      for (int k = 0; k < segN; k++) lut[k] = themeColor(heatC(((float)k + 0.5f) * 100.0f / (float)segN));

      for (int ch = 0; ch < NRF_CH; ch++) {
        int x0 = L + W * ch / NRF_CH;
        int x1 = L + W * (ch + 1) / NRF_CH;
        int bw = x1 - x0;
        if (bw >= 3) bw -= 1;                           // thin gap between wide bars
        int bh = (int)(H * nrfShown[ch] / 100.0f);
        if (bh > H) bh = H;

        int full = bh / SEG;
        for (int k = 0; k < full && k < segN; k++) {
          g->fillRect(x0, B - (k + 1) * SEG, bw, SEG, lut[k]);
        }
        int rem = bh - full * SEG;
        if (rem > 0 && full < segN) g->fillRect(x0, B - bh, bw, rem, lut[full]);

        int ph = (int)(H * nrfPeak[ch] / 100.0f);
        if (ph > bh + 1 && ph <= H) g->fillRect(x0, B - ph - 1, bw, 2, cW);
      }
    } else {
      // ---------------- WI-FI CHANNEL LOAD ----------------
      const int bwAll = W / 13;                         // 16 px per channel
      for (int n = 1; n <= 13; n++) {
        int x = L + (n - 1) * bwAll + 1;
        int bw = bwAll - 2;
        int bh = (int)(H * wl[n - 1] / 100.0f);
        if (bh > H) bh = H;
        if (bh > 0) g->fillRoundRect(x, B - bh, bw, bh, 2, themeColor(heatC(wl[n - 1])));
        String pct = String((int)lroundf(wl[n - 1]));
        int ty = B - bh - 10;
        if (ty < T - 4) ty = T - 4;
        g->setTextSize(1);
        g->setTextColor(cW);
        g->setCursor(x + (bw - (int)pct.length() * 6) / 2, ty);
        g->print(pct);
      }
    }

    g->drawFastHLine(L, B, W, cM);
  }

  // identification strip: coloured bar along the top edge of every detected signal (BARS and FALL)
  if (nrfView != 2) {
    for (int i = 0; i < nrfDevCount; i++) {
      const NrfDev &d = nrfDev[i];
      if (d.type == DEV_HOP) continue;                  // hoppers cover everything, no strip
      int x0 = L + W * d.lo / NRF_CH;
      int x1 = L + W * (d.hi + 1) / NRF_CH;
      g->fillRect(x0, T, max(3, x1 - x0), 3, nrfDevColor(d.type));
    }
  }

  // frequency marker (BARS and FALL views)
  if (nrfCursor >= 0 && nrfView != 2) {
    int cx = L + (int)lroundf((float)W * (nrfCursor + 0.5f) / (float)NRF_CH);
    g->drawFastVLine(cx, T, H, themeColor(C_YELLOW));
  }

  // x axis
  if (nrfView == 2) {
    const int bwAll = W / 13;
    for (int n = 1; n <= 13; n++) {
      String s = String(n);
      int x = L + (n - 1) * bwAll + 1 + (bwAll - 2 - (int)s.length() * 6) / 2;
      g->setTextSize(1);
      g->setTextColor(n - 1 == bestWifi ? themeColor(C_GREEN) : cW);
      g->setCursor(x, B + 6);
      g->print(s);

      int aps = 0;                                      // number of access points the Wi-Fi scan saw on this channel
      nrfSsidOn(n, aps);
      if (aps > 0) {
        String a = String(aps);
        g->setTextColor(themeColor(C_CYAN));
        g->setCursor(L + (n - 1) * bwAll + 1 + (bwAll - 2 - (int)a.length() * 6) / 2, B + 17);
        g->print(a);
      }
    }
    g->setTextColor(cM);
    g->setCursor(L, 270);
  } else {
    // Wi-Fi channel ticks: channel n is centred on nRF channel 7 + 5n
    for (int n = 1; n <= 13; n++) {
      int x = L + (int)lroundf((float)W * (7.0f + 5.0f * n + 0.5f) / (float)NRF_CH);
      g->drawFastVLine(x, B + 1, 3, cM);
      if (n == 1 || n == 6 || n == 11 || n == 13) {
        String s = String(n);
        g->setTextSize(1);
        g->setTextColor(n == 13 ? cM : cW);
        g->setCursor(x - (int)s.length() * 3, B + 6);
        g->print(s);
      }
    }
    g->setTextSize(1);
    g->setTextColor(cM);
    g->setCursor(L, 262);
    g->print("2400");
    String mid = "Wi-Fi ch";
    g->setCursor((240 - (int)mid.length() * 6) / 2, 262);
    g->print(mid);
    g->setCursor(R - 24, 262);
    g->print("2483");
  }

  flushCanvas();
}

void buildLoRa();

// ---------- saved messages (stored in flash) ----------
void loraSavedLoad() {
  static const char *DEF[] = {"Hello", "I am OK", "Need help", "On my way",
                              "Where are you?", "Call me", "Yes", "No"};
  Preferences tp;
  tp.begin("watchout", true);
  uint8_t n = tp.getUChar("svn", 255);
  if (n == 255) {
    savedCount = 8;
    for (int i = 0; i < savedCount; i++) savedMsg[i] = DEF[i];
  } else {
    if (n > SAVED_MAX) n = SAVED_MAX;
    savedCount = n;
    for (int i = 0; i < savedCount; i++) {
      char k[8];
      snprintf(k, sizeof(k), "sv%d", i);
      savedMsg[i] = tp.getString(k, "");
    }
  }
  tp.end();
}

void loraSavedStore() {
  Preferences tp;
  tp.begin("watchout", false);
  tp.putUChar("svn", (uint8_t)savedCount);
  for (int i = 0; i < savedCount; i++) {
    char k[8];
    snprintf(k, sizeof(k), "sv%d", i);
    tp.putString(k, savedMsg[i]);
  }
  tp.end();
}

// ---------- send / receive ----------
bool loraSendText(String text) {
  text.trim();
  if (text.length() == 0) { chatStatus = "Empty"; chatStatusAt = millis(); return false; }
  if (text.length() > LORA_MAX_TEXT) text = text.substring(0, LORA_MAX_TEXT);
  if (!loraOK) startLoRa();
  if (!loraOK) { chatStatus = "Radio FAILED"; chatStatusAt = millis(); return false; }
  LoRa.beginPacket();
  LoRa.print(text);
  LoRa.endPacket();
  LoRa.receive();
  loraTx++;
  ledFlash(0, 120, 255, 200);
  chatStatus = "Sent!";
  chatStatusAt = millis();
  lastDraw = 0;
  return true;
}

void rxAdd(const String &raw, int rssi) {
  String t = "";
  for (unsigned int i = 0; i < raw.length(); i++) {
    char c = raw[i];
    if (c >= 32 && c < 127) t += c;
  }
  if (t.length() == 0) t = "(binary data)";
  if (rxCount >= RX_MAX) {
    for (int i = 0; i < RX_MAX - 1; i++) rxLog[i] = rxLog[i + 1];
    rxCount = RX_MAX - 1;
  }
  rxLog[rxCount].text = t;
  rxLog[rxCount].rssi = rssi;
  rxLog[rxCount].at = millis();
  rxCount++;
}

// Receives in the background as soon as the radio has been started once
void pollLoRa() {
  if (!loraOK) return;
  static unsigned long lastPoll = 0;
  unsigned long now = millis();
  if (now - lastPoll < 25) return;
  lastPoll = now;

  int n = LoRa.parsePacket();
  if (n <= 0) return;
  String msg = "";
  while (LoRa.available()) msg += (char)LoRa.read();
  loraMessage = msg;
  loraPackets++;
  loraRSSI = LoRa.packetRssi();
  loraSNR = LoRa.packetSnr();
  rxAdd(msg, loraRSSI);
  if (!(page == LORA_PAGE && subView == 5)) loraUnread++;
  ledFlash(0, 255, 0, 250);
  lastDraw = 0;
  Serial.printf("LoRa packet #%lu RSSI=%d: %s\n", (unsigned long)loraPackets, loraRSSI, msg.c_str());
}

// ---------- on-screen keyboard (LoRa chat + Wi-Fi password) ----------
static const char *KB_ROWS[3][3] = {
  {"qwertyuiop", "asdfghjkl",  "zxcvbnm,.?"},
  {"QWERTYUIOP", "ASDFGHJKL",  "ZXCVBNM,.?"},
  {"1234567890", "@#&-+=()/%", "!.,:;'\"*_~"}
};

int kbRowLen(int r) { return r == 3 ? 6 : (int)strlen(KB_ROWS[kbMode][r]); }

// The key highlight glides between keys (smooth cursor)
float kbCurX = -1.0f, kbCurY = -1.0f;

void kbMove(int dx, int dy) {
  int len = kbRowLen(kbRow);
  if (dx != 0) {
    kbCol = (kbCol + dx + len) % len;
  } else if (dy != 0) {
    int nr = kbRow + dy;
    if (nr < 0) nr = 3;
    if (nr > 3) nr = 0;
    int nl = kbRowLen(nr);
    kbCol = (kbCol * nl) / len;
    if (kbCol >= nl) kbCol = nl - 1;
    kbRow = nr;
  }
  uiDirty = true;
}

void kbOpen() {
  kbMode = 0;
  kbRow = 0;
  kbCol = 0;
  kbCurX = -1.0f;
  chatStatus = "";
}

void loraSaveCurrent() {
  String t = chatText;
  t.trim();
  if (t.length() == 0) { chatStatus = "Empty"; chatStatusAt = millis(); return; }
  for (int i = 0; i < savedCount; i++) {
    if (savedMsg[i] == t) { chatStatus = "Already saved"; chatStatusAt = millis(); return; }
  }
  if (savedCount >= SAVED_MAX) {
    for (int i = 0; i < SAVED_MAX - 1; i++) savedMsg[i] = savedMsg[i + 1];
    savedCount = SAVED_MAX - 1;
  }
  savedMsg[savedCount++] = t;
  loraSavedStore();
  chatStatus = "Saved";
  chatStatusAt = millis();
}

void kbPress() {
  bool w = kbWifi();
  String &buf = w ? apPass : chatText;
  const int maxLen = w ? 63 : LORA_MAX_TEXT;

  if (kbRow < 3) {
    char ch = KB_ROWS[kbMode][kbRow][kbCol];
    if ((int)buf.length() < maxLen) buf += ch;
    if (kbMode == 1) kbMode = 0;               // shift is one-shot
  } else {
    switch (kbCol) {
      case 0: kbMode = (kbMode + 1) % 3; { int l = kbRowLen(3); if (kbCol >= l) kbCol = l - 1; } break;
      case 1: if ((int)buf.length() < maxLen) buf += ' '; break;
      case 2: if (buf.length() > 0) buf.remove(buf.length() - 1); break;
      case 3:
        if (w) { apConnectFromKb(); return; }
        if (loraSendText(chatText)) chatText = "";
        break;
      case 4:
        if (w) apShowPass = !apShowPass;
        else loraSaveCurrent();
        break;
      case 5:
        if (w) wifiKbBack();
        else loraBack();
        return;
    }
  }
  uiDirty = true;
}

void renderKeyboard() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  g->fillScreen(bgColor());

  const bool w = kbWifi();
  const bool v = page == VAULT_PAGE && vaultKbActive();
  const String &buf = v ? vaultInput : (w ? apPass : chatText);
  const int maxLen = v ? vaultMaxLen : (w ? 63 : LORA_MAX_TEXT);

  const uint16_t fg     = themeColor(C_WHITE);
  const uint16_t muted  = themeColor(C_MUTED);
  const uint16_t accent = themeColor(C_CYAN);
  const uint16_t keyBg  = lightMode ? 0xDEFB : 0x2104;
  const uint16_t selFg  = bgColor();

  // header
  String hd = v ? vaultPrompt : (w ? (String("Pass: ") + apTargetSsid) : String("Custom chat"));
  if (hd.length() > 24) hd = hd.substring(0, 24);
  g->setTextSize(1);
  g->setTextWrap(false);
  g->setTextColor(muted);
  g->setCursor(8, 8);
  g->print(hd);
  String cnt = String(buf.length()) + "/" + String(maxLen);
  g->setCursor(232 - (int)cnt.length() * 6, 8);
  g->print(cnt);

  // text box (shows the last 4 lines). Wi-Fi password is masked except the last typed key.
  g->drawRoundRect(4, 20, 232, 82, 6, accent);
  String shown = buf;
  if ((w && !apShowPass) || (v && vaultMask)) {
    shown = "";
    for (unsigned int i = 0; i < buf.length(); i++) {
      if (!(v && vaultMask) && i + 1 == buf.length()) shown += buf[i];
      else shown += '*';
    }
  }
  // blinking text cursor
  String t = shown + (((millis() / 500) & 1) ? " " : "_");
  const int per = 17;
  int total = ((int)t.length() + per - 1) / per;
  int first = total > 4 ? total - 4 : 0;
  for (int l = first; l < total; l++) {
    int from = l * per;
    int to = min((int)t.length(), from + per);
    boldText(g, 10, 26 + (l - first) * 19, t.substring(from, to), fg, 2, 1);
  }

  // status (Sent! / Saved / Empty ...)
  if (chatStatus.length() && millis() - chatStatusAt < 2500) {
    bool bad = chatStatus.indexOf("FAILED") >= 0 || chatStatus == "Empty";
    g->setTextSize(1);
    g->setTextColor(bad ? themeColor(C_RED) : themeColor(C_GREEN));
    g->setCursor(8, 108);
    g->print(chatStatus);
  }

  // keys
  const char *modeLabel = kbMode == 0 ? "ABC" : (kbMode == 1 ? "123" : "abc");
  const char *specLabel[6] = {modeLabel, "SPACE", "DEL", v ? "OK" : (w ? "JOIN" : "SEND"),
                              v ? (vaultMask ? "HIDE" : "SHOW") : (w ? (apShowPass ? "HIDE" : "SHOW") : "SAVE"), "BACK"};
  const uint16_t specColor[6] = {fg, fg, themeColor(C_RED), themeColor(C_GREEN),
                                 (w || v) ? themeColor(C_CYAN) : themeColor(C_YELLOW), themeColor(C_RED)};

  // geometry of the selected key (used for the gliding highlight)
  int selN = kbRowLen(kbRow);
  int selStep = (kbRow == 3) ? 38 : 22;
  float selX = (float)((240 - selN * selStep) / 2 + 1 + kbCol * selStep);
  float selY = (float)(124 + kbRow * 30);
  float selW = (float)(selStep - 2);
  if (kbCurX < 0.0f) { kbCurX = selX; kbCurY = selY; }
  kbCurX += (selX - kbCurX) * 0.45f;
  kbCurY += (selY - kbCurY) * 0.45f;
  bool kbMoving = fabsf(selX - kbCurX) > 0.6f || fabsf(selY - kbCurY) > 0.6f;
  if (!kbMoving) { kbCurX = selX; kbCurY = selY; }

  for (int r = 0; r < 4; r++) {
    int n = kbRowLen(r);
    int step = (r == 3) ? 38 : 22;
    int kw = step - 2;
    int x0 = (240 - n * step) / 2 + 1;
    int y = 124 + r * 30;
    for (int c = 0; c < n; c++) {
      int x = x0 + c * step;
      g->fillRoundRect(x, y, kw, 26, 4, keyBg);
    }
  }

  // gliding highlight
  g->fillRoundRect((int)lroundf(kbCurX), (int)lroundf(kbCurY), (int)selW, 26, 4, accent);

  for (int r = 0; r < 4; r++) {
    int n = kbRowLen(r);
    int step = (r == 3) ? 38 : 22;
    int kw = step - 2;
    int x0 = (240 - n * step) / 2 + 1;
    int y = 124 + r * 30;
    for (int c = 0; c < n; c++) {
      int x = x0 + c * step;
      bool sel = (r == kbRow && c == kbCol);
      if (r < 3) {
        char ch[2] = {KB_ROWS[kbMode][r][c], 0};
        g->setTextSize(2);
        g->setTextColor(sel ? selFg : fg);
        g->setCursor(x + (kw - 12) / 2 + 1, y + 5);
        g->print(ch);
      } else {
        const char *lab = specLabel[c];
        g->setTextSize(1);
        g->setTextColor(sel ? selFg : specColor[c]);
        g->setCursor(x + (kw - (int)strlen(lab) * 6) / 2, y + 9);
        g->print(lab);
      }
    }
  }

  uiDirty = true;                                  // keeps the cursor blink + glide running
  flushCanvas();
}

// ---------- screens ----------
String ageStr(unsigned long ms) {
  unsigned long s = ms / 1000UL;
  if (s < 5) return "now";
  if (s < 60) return String(s) + "s ago";
  if (s < 3600) return String(s / 60) + "m ago";
  return String(s / 3600) + "h ago";
}

void buildLoRaStatus() {
  addLine("Radio", loraOK ? "READY" : "FAILED", C_MUTED, loraOK ? C_GREEN : C_RED);
  addLine("Freq", String((int)(LORA_FREQ / 1E6)) + " MHz", C_MUTED, C_WHITE);
  addLine("Received", String(loraPackets), C_MUTED, C_WHITE);
  addLine("Sent", String(loraTx), C_MUTED, C_WHITE);
  addLine("RSSI", String(loraRSSI) + " dBm", C_MUTED, C_WHITE);
  addLine("SNR", String(loraSNR, 1) + " dB", C_MUTED, C_WHITE);
  addLine("Last packet", "", C_MUTED, C_MUTED);
  String m = loraMessage;
  if (m.length() > 90) m = m.substring(0, 90);
  addWrapped(m, C_CYAN);
}

void buildLoRaChatMenu() {
  addLine("Saved chat", String(savedCount), C_WHITE, C_MUTED);
  addLine("Custom chat", "", C_WHITE, C_WHITE);
  if (loraUnread > 0) addLine("Received msg", String(loraUnread) + " new", C_WHITE, C_GREEN);
  else addLine("Received msg", rxCount ? String(rxCount) : String(""), C_WHITE, C_MUTED);
}

void buildLoRaSaved() {
  for (int i = 0; i < savedCount; i++) {
    bool justSent = (i == savedSentIdx && millis() - savedSentAt < 1500);
    addLine(savedMsg[i], justSent ? "SENT" : "", C_WHITE, C_GREEN);
  }
  addLine("+ New message", "", C_CYAN, C_CYAN);
}

void buildLoRaRx() {
  rxClearRow = -1;
  if (rxCount == 0) {
    addWrapped("No messages yet", C_MUTED);
    return;
  }
  for (int i = rxCount - 1; i >= 0; i--) {
    addLine(ageStr(millis() - rxLog[i].at), String(rxLog[i].rssi), C_YELLOW, rssiColor(rxLog[i].rssi));
    String t = rxLog[i].text;
    if (t.length() > RX_SHOW_CHARS) t = t.substring(0, RX_SHOW_CHARS);
    addWrapped(t, C_CYAN);
  }
  rxClearRow = lineCount;
  addLine("> Clear all", "", C_RED, C_RED);
}

void buildLoRa() {
  startLines();
  switch (subView) {
    case 1: buildLoRaStatus(); break;
    case 2: buildLoRaChatMenu(); break;
    case 3: buildLoRaSaved(); break;
    case 4: uiDirty = true; break;            // keyboard is drawn by renderKeyboard()
    case 5: buildLoRaRx(); break;
    default:
      addLine("LoRa Status", loraOK ? "READY" : "FAILED", C_WHITE, loraOK ? C_GREEN : C_RED);
      addLine("Chat", loraUnread > 0 ? String(loraUnread) + " new" : String(""), C_WHITE, C_GREEN);
      break;
  }
  finishLines();
}

// ---------- navigation ----------
void loraBack() {
  if (subView >= 3) {
    subView = 2;
    pageSel = loraChatSel;
  } else {
    subView = 0;
    pageSel = subParentSel;
  }
  scrollPos = (float)pageSel;
  scrollVel = 0.0f;
  buildLoRa();
}

void loraEnterView(int v) {
  subView = v;
  pageSel = 0;
  scrollPos = 0.0f;
  scrollVel = 0.0f;
  if (v == 3) loraSavedLoad();
  if (v == 4) kbOpen();
  if (v == 5) loraUnread = 0;
}

void loraClick() {
  if (subView == 0) {
    subParentSel = pageSel;
    if (pageSel == 0) {
      loraEnterView(1);
    } else {
      if (!loraOK) startLoRa();
      loraSavedLoad();
      loraEnterView(2);
    }
  } else if (subView == 1) {
    startLoRa();
  } else if (subView == 2) {
    loraChatSel = pageSel;
    loraEnterView(3 + pageSel);
  } else if (subView == 3) {
    if (pageSel < savedCount) {
      if (loraSendText(savedMsg[pageSel])) {
        savedSentIdx = pageSel;
        savedSentAt = millis();
      }
    } else if (pageSel == savedCount) {
      loraEnterView(4);
    }
  } else if (subView == 4) {
    kbPress();
  } else if (subView == 5) {
    if (rxClearRow >= 0 && pageSel == rxClearRow) {
      rxCount = 0;
      loraUnread = 0;
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
    }
  }
  buildLoRa();
}

// Long press on the joystick button: delete saved message / leave keyboard
bool loraLongClick() {
  if (page != LORA_PAGE) return false;
  if (subView == 3) {
    if (pageSel < savedCount) {
      for (int i = pageSel; i < savedCount - 1; i++) savedMsg[i] = savedMsg[i + 1];
      savedCount--;
      loraSavedStore();
      ledFlash(255, 0, 0, 200);
      buildLoRa();
    }
    return true;
  }
  if (subView == 4) {
    loraBack();
    return true;
  }
  return false;
}

// ======================= QR CODE =======================
// QR sub views: 0 menu (Information QR / Custom QR), 1 information QR list, 2 custom QR (phone web page)
// A QR code is drawn black on white (always, so phone cameras can read it) by renderQR().

QRCode qrCode;
uint8_t qrBuf[700];                 // enough for version 12 (65x65 modules)
bool qrValid = false;
String qrCachedText = "\x01";       // text the current qrCode was made from

bool qrFitsVersion(const String &text, uint8_t version) {
  static const uint16_t lowEccDataBits[] = {
    152, 272, 440, 640, 864, 1088, 1248, 1552, 1856, 2192, 2592, 2960
  };
  if (version < 1 || version > sizeof(lowEccDataBits) / sizeof(lowEccDataBits[0])) return false;

  bool numeric = true;
  bool alphanumeric = true;
  for (unsigned int i = 0; i < text.length(); i++) {
    char c = text[i];
    if (c < '0' || c > '9') numeric = false;
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
          c == ' ' || c == '$' || c == '%' || c == '*' || c == '+' ||
          c == '-' || c == '.' || c == '/' || c == ':')) {
      alphanumeric = false;
    }
  }

  uint32_t length = text.length();
  uint32_t countBits;
  uint32_t dataBits;
  if (numeric) {
    countBits = version <= 9 ? 10 : 12;
    dataBits = (length / 3) * 10 + (length % 3 ? (length % 3) * 3 + 1 : 0);
  } else if (alphanumeric) {
    countBits = version <= 9 ? 9 : 11;
    dataBits = (length / 2) * 11 + (length % 2 ? 6 : 0);
  } else {
    countBits = version <= 9 ? 8 : 16;
    dataBits = length * 8;
  }

  return 4 + countBits + dataBits <= lowEccDataBits[version - 1];
}

// Smallest version (1..12) that fits the text, low error correction = most data
bool qrMake(const String &t) {
  qrValid = false;
  if (t.length() == 0) return false;
  for (uint8_t v = 1; v <= 12; v++) {
    if (!qrFitsVersion(t, v)) continue;
    if (qrcode_getBufferSize(v) > sizeof(qrBuf)) break;
    if (qrcode_initText(&qrCode, qrBuf, v, ECC_LOW, t.c_str()) == 0) {
      qrValid = true;
      return true;
    }
  }
  return false;
}

// ---- Information QR codes ----
// Order on the watch: Contact, GitHub, Music, Free Wi-Fi, Google, Secret Msg, My Wi-Fi
const int FAKE_COUNT = 7;
const char *FAKE_NAMES[FAKE_COUNT] = {
  "Contact", "GitHub", "Music", "Free Wi-Fi",
  "Google", "Secret Msg", "My Wi-Fi"
};

String qrEsc(const String &s) {     // escape for the WIFI:... QR format
  String o = "";
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\' || c == ';' || c == ',' || c == ':' || c == '"') o += '\\';
    o += c;
  }
  return o;
}

String fakePayload(int i) {
  switch (i) {
    case 0: {                                      // Contact (vCard): scanning offers "Add contact"
      String contact = String("BEGIN:VCARD\nVERSION:3.0\nFN:") + QR_OWNER_NAME;
      if (QR_OWNER_PHONE[0] != '\0')
        contact += String("\nTEL;TYPE=CELL:") + QR_OWNER_PHONE;
      return contact + "\nEND:VCARD";
    }
    case 1: return "https://github.com/Debjit230";
    case 2: return QR_MUSIC_LINK;
    case 3: return "WIFI:T:nopass;S:Free Wi-Fi;;";
    case 4: return "https://www.google.com";
    case 5: return QR_SECRET_TEXT;
    default: {                                     // My Wi-Fi: share the last network connected through Connect AP
      if (savedSsid.length() == 0) return "No saved Wi-Fi. Use Wi-Fi > Connect AP first.";
      if (savedPass.length() == 0) return "WIFI:T:nopass;S:" + qrEsc(savedSsid) + ";;";
      return "WIFI:T:WPA;S:" + qrEsc(savedSsid) + ";P:" + qrEsc(savedPass) + ";;";
    }
  }
}

// ---- Custom QR (web page on the watch) ----
WebServer qrServer(80);
bool qrServerUp = false;
bool qrHandlers = false;
bool qrAP = false;                  // true = watch runs its own hotspot
bool qrOwnsWifi = false;            // true = we started the link, we must stop it
bool qrMdns = false;
String qrNetMsg = "";
int qrActRow[3] = {-1, -1, -1};

static const char QR_HTML[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>WatchOut QR</title>
<style>
body{font-family:sans-serif;margin:0;padding:16px;background:#111;color:#fff}
h2{margin:0 0 12px}h3{margin:22px 0 6px}
textarea{width:100%;height:120px;box-sizing:border-box;font-size:16px;padding:10px;border-radius:10px;border:0}
button{margin-top:10px;width:100%;padding:14px;font-size:17px;border:0;border-radius:10px;background:#0a84ff;color:#fff}
input[type=file]{width:100%;margin-top:6px;font-size:15px}
p{color:#aaa;font-size:14px;margin:4px 0}
#s{margin-top:14px;font-size:16px;color:#6cf;min-height:20px}
</style></head><body>
<h2>WatchOut QR</h2>
<textarea id=t maxlength=367 placeholder="Type text or a link"></textarea>
<button onclick="send()">Send to watch</button>
<h3>Or send a picture</h3>
<p>Pick a picture that contains a QR code. The watch will draw it again.</p>
<input type=file id=f accept="image/*" onchange="pic(this.files[0])">
<canvas id=c hidden></canvas>
<div id=s></div>
<script>
function st(m){document.getElementById('s').textContent=m}
function send(){
  var v=document.getElementById('t').value.trim();
  if(!v){st('Type something first');return}
  fetch('/set',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'t='+encodeURIComponent(v)})
  .then(function(r){if(r.ok){st('Sent to watch!')}else{r.text().then(function(m){st(m||('Error '+r.status))})}})
  .catch(function(){st('Connection lost')})
}
async function pic(file){
  if(!file)return;
  st('Reading picture...');
  try{
    var bmp=await createImageBitmap(file);
    var sc=Math.min(1,1000/Math.max(bmp.width,bmp.height));
    var c=document.getElementById('c');
    c.width=Math.round(bmp.width*sc);c.height=Math.round(bmp.height*sc);
    var x=c.getContext('2d');
    x.drawImage(bmp,0,0,c.width,c.height);
    var txt=null;
    if('BarcodeDetector' in window){
      try{
        var r=await new BarcodeDetector({formats:['qr_code']}).detect(c);
        if(r.length)txt=r[0].rawValue;
      }catch(e){}
    }
    if(!txt){
      if(!window.jsQR){
        await new Promise(function(ok,no){
          var s=document.createElement('script');
          s.src='https://cdn.jsdelivr.net/npm/jsqr@1.4.0/dist/jsQR.js';
          s.onload=ok;s.onerror=no;document.head.appendChild(s);
        });
      }
      var d=x.getImageData(0,0,c.width,c.height);
      var q=jsQR(d.data,d.width,d.height);
      if(q)txt=q.data;
    }
    if(!txt){st('No QR code found in that picture');return}
    document.getElementById('t').value=txt;
    send();
  }catch(e){st('Could not read picture (the decoder needs internet on this phone)')}
}
</script></body></html>)HTML";

void qrSaveText() {
  Preferences tp;
  tp.begin("watchout", false);
  tp.putString("qrtxt", qrCustomText);
  tp.end();
}

void qrHandleRoot() { qrServer.send_P(200, "text/html", QR_HTML); }

void qrHandleSet() {
  String t = qrServer.arg("t");
  t.trim();
  if (t.length() == 0) { qrServer.send(400, "text/plain", "Empty"); return; }
  if (t.length() > QR_MAX_TEXT) {
    qrServer.send(413, "text/plain", "Text too long (maximum 367 bytes)");
    return;
  }
  qrCustomText = t;
  qrSaveText();
  qrServer.send(200, "text/plain", "OK");
  ledFlash(0, 255, 0, 300);
  if (page == QR_PAGE && subView == 2) {           // show it right away
    qrShowing = true;
    uiDirty = true;
  }
}

void qrNetStop() {
  if (qrServerUp) { qrServer.stop(); qrServerUp = false; }
  if (qrMdns) { MDNS.end(); qrMdns = false; }
  if (qrAP) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
  } else if (qrOwnsWifi && !apHold && !trackerOn) {
    WiFi.disconnect(false, false);
  }
  qrAP = false;
  qrOwnsWifi = false;
  qrNetMsg = "";
}

// Start the web page: saved Wi-Fi first, own hotspot as fallback (or when forced)
void qrNetStart(bool forceAP) {
  if (trackerOn) { qrNetMsg = "Tracker is ON"; return; }
  qrNetStop();
  qrNetMsg = forceAP ? "Starting hotspot..." : "Connecting Wi-Fi...";
  buildQR();
  uiDirty = false;
  renderUIStatic();

  bool ok = false;
  if (!forceAP) {
    if (WiFi.status() != WL_CONNECTED) {
      if (wifiStarted) { WiFi.scanDelete(); wifiStarted = false; }
      if (wifiBegin()) {                           // only if a network was saved before
        unsigned long t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
          pumpGPS();
          delay(100);
        }
        if (WiFi.status() == WL_CONNECTED) qrOwnsWifi = true;
      }
    }
    ok = (WiFi.status() == WL_CONNECTED);
  }

  if (!ok) {                                       // hotspot
    if (wifiStarted) { WiFi.scanDelete(); wifiStarted = false; }
    apHold = false;
    apState = 0;
    WiFi.disconnect(false, false);
    WiFi.mode(WIFI_AP);
    ok = WiFi.softAP(QR_AP_SSID, QR_AP_PASS);
    qrAP = ok;
    qrOwnsWifi = ok;
  }

  if (!ok) { qrNetMsg = "Network failed"; return; }

  if (!qrHandlers) {
    qrServer.on("/", qrHandleRoot);
    qrServer.on("/set", HTTP_POST, qrHandleSet);
    qrHandlers = true;
  }
  qrServer.begin();
  qrServerUp = true;
  qrNetMsg = "";
  if (!qrAP && MDNS.begin("watchout")) {
    MDNS.addService("http", "tcp", 80);
    qrMdns = true;
  }
  ledFlash(0, 120, 255, 300);
}

void qrPoll() {
  if (qrServerUp) qrServer.handleClient();
}

// ---- screens ----
void buildQRMenu() {
  addLine("Information QR", "", C_WHITE, C_WHITE);
  addLine("Custom QR", qrCustomText.length() ? "SAVED" : "", C_WHITE, C_GREEN);
}

void buildQRFake() {
  for (int i = 0; i < FAKE_COUNT; i++) addLine(FAKE_NAMES[i], "", C_WHITE, C_WHITE);
}

void buildQRCustom() {
  qrActRow[0] = qrActRow[1] = qrActRow[2] = -1;

  if (trackerOn) {
    addLine("Tracker is ON", "", C_RED, C_RED);
    addWrapped("Stop it in GPS", C_MUTED);
    addWrapped("to use Custom QR", C_MUTED);
    return;
  }

  if (!qrServerUp) {
    addLine("Server", "OFF", C_MUTED, C_RED);
    if (qrNetMsg.length()) addWrapped(qrNetMsg, C_YELLOW);
  } else {
    addLine("Server", "ON", C_MUTED, C_GREEN);
    if (qrAP) {
      addLine("Mode", "HOTSPOT", C_MUTED, C_YELLOW);
      addKV("SSID", QR_AP_SSID, C_WHITE);
      addKV("Pass", QR_AP_PASS, C_WHITE);
      addKV("Open", WiFi.softAPIP().toString(), C_CYAN);
    } else {
      addLine("Mode", "WI-FI", C_MUTED, C_GREEN);
      addKV("Open", WiFi.localIP().toString(), C_CYAN);
      addKV("Or", "watchout.local", C_CYAN);
    }
  }

  addLine("Text", "", C_MUTED, C_MUTED);
  if (qrCustomText.length()) {
    String t = qrCustomText;
    if (t.length() > 54) t = t.substring(0, 54);
    addWrapped(t, C_CYAN);
  } else {
    addWrapped("(none yet)", C_MUTED);
  }

  if (qrCustomText.length()) {
    qrActRow[0] = lineCount;
    addLine("> Show QR", "", C_GREEN, C_GREEN);
  }
  qrActRow[1] = lineCount;
  addLine(qrAP ? "> Use Wi-Fi" : "> Use hotspot", "", C_CYAN, C_CYAN);
  if (qrCustomText.length()) {
    qrActRow[2] = lineCount;
    addLine("> Clear text", "", C_RED, C_RED);
  }
}

void buildQR() {
  if (qrShowing) return;                            // QR is drawn by renderQR()
  startLines();
  if (subView == 1) buildQRFake();
  else if (subView == 2) buildQRCustom();
  else buildQRMenu();
  finishLines();
}

String qrCurrentText(String &title) {
  if (subView == 1) {
    int i = pageSel;
    if (i < 0) i = 0;
    if (i >= FAKE_COUNT) i = FAKE_COUNT - 1;
    title = FAKE_NAMES[i];
    if (i == FAKE_COUNT - 1 && savedSsid.length()) title = String("Wi-Fi: ") + savedSsid;
    return fakePayload(i);
  }
  title = "Custom QR";
  return qrCustomText;
}

void renderQR() {
  Adafruit_GFX *g = canvasReady ? (Adafruit_GFX *)canvas : (Adafruit_GFX *)&lcd;
  String title;
  String text = qrCurrentText(title);
  if (text != qrCachedText) {
    qrCachedText = text;
    qrMake(text);
  }

  g->fillScreen(0xFFFF);

  if (!qrValid) {
    g->setTextSize(2);
    g->setTextWrap(false);
    g->setTextColor(C_RED);
    g->setCursor(12, 110);
    g->print(text.length() ? "Text too long" : "No text");
  } else {
    int n = qrCode.size;
    int scale = 240 / (n + 8);                      // reserve the required 4-module quiet zone
    if (scale < 1) scale = 1;
    int px = n * scale;
    int ox = (240 - px) / 2;
    int oy = (252 - px) / 2;
    if (oy < 0) oy = 0;
    for (int y = 0; y < n; y++) {
      for (int x = 0; x < n; x++) {
        if (qrcode_getModule(&qrCode, x, y)) g->fillRect(ox + x * scale, oy + y * scale, scale, scale, 0x0000);
      }
    }
  }

  if (title.length() > 36) title = title.substring(0, 36);
  g->setTextSize(1);
  g->setTextWrap(false);
  g->setTextColor(0x0000);
  g->setCursor(max(2, (240 - (int)title.length() * 6) / 2), 262);
  g->print(title);

  flushCanvas();
}

// ---- navigation ----
void qrBack() {
  if (qrShowing) {
    qrShowing = false;
    buildQR();
    uiDirty = true;
    return;
  }
  if (subView == 2) qrNetStop();
  subView = 0;
  pageSel = subParentSel;
  scrollPos = (float)pageSel;
  scrollVel = 0.0f;
  buildQR();
}

void qrClick() {
  if (qrShowing) { qrBack(); return; }

  if (subView == 0) {
    if (pageSel < 2) {
      subParentSel = pageSel;
      subView = pageSel + 1;
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
      if (subView == 2) {
        qrNetStart(false);
        pageSel = 0;
        scrollPos = 0.0f;
        scrollVel = 0.0f;
      }
    }
  } else if (subView == 1) {
    if (pageSel < FAKE_COUNT) {
      qrShowing = true;
      uiDirty = true;
      return;
    }
  } else if (subView == 2) {
    int act = -1;
    for (int j = 0; j < 3; j++) if (qrActRow[j] == pageSel) act = j;
    if (act == 0) {
      if (qrCustomText.length()) { qrShowing = true; uiDirty = true; return; }
    } else if (act == 1) {
      qrNetStart(!qrAP);
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
    } else if (act == 2) {
      qrCustomText = "";
      qrSaveText();
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
    }
  }
  buildQR();
}

// ======================= IR =======================
// Capture stores BOTH the decoded values (protocol / address / command) and the raw
// mark/space timings in microseconds. IR Sender replays the raw timings by default, which
// works for every remote (even unknown protocols and long air-conditioner frames).

// Only decode while inside IR Capture (subView 1)
void pollIR() {
  if (page != IR_PAGE || subView != 1 || !IrReceiver.decode()) return;
  auto &d = IrReceiver.decodedIRData;
  bool repeat = (d.flags & IRDATA_FLAGS_IS_REPEAT);
  bool overflow = (d.flags & IRDATA_FLAGS_WAS_OVERFLOW);
  uint16_t rawlen = IrReceiver.irparams.rawlen;      // index 0 = gap before the frame, 1.. = mark,space,mark...

  if (overflow) {
    irMessage = "Too long, retry";
    lastDraw = 0;
    uiDirty = true;
  } else if (!repeat && rawlen >= 6) {
    // 1) decoded details
    storedIRData = d;
    storedIRData.flags = 0;
    bool known = !(d.protocol == UNKNOWN || d.protocol == PULSE_WIDTH || d.protocol == PULSE_DISTANCE);
    irRawMode = !known;
    irProtocol = String(getProtocolString(d.protocol));
    irAddress = d.address;
    irCommand = d.command;
    irBits = (uint16_t)d.numberOfBits;
    irData = (uint32_t)d.decodedRawData;

    // 2) raw timings in microseconds (compensated for the receiver's mark/space distortion)
    uint16_t n = rawlen - 1;
    if (n > RAW_BUFFER_LENGTH) n = RAW_BUFFER_LENGTH;
    for (uint16_t i = 0; i < n; i++) {
      uint32_t us = (uint32_t)IrReceiver.irparams.rawbuf[i + 1] * MICROS_PER_TICK;
      if ((i & 1) == 0) {                            // mark
        us = (us > MARK_EXCESS_MICROS) ? us - MARK_EXCESS_MICROS : us;
      } else {                                       // space
        us += MARK_EXCESS_MICROS;
      }
      rawIRCode[i] = (us > 65535UL) ? 65535 : (uint16_t)us;
    }
    rawIRLength = n;

    irCaptured = true;
    irCaptures++;
    irSendRaw = true;                                // default replay method
    ledFlash(191, 90, 242, 300);
    irMessage = known ? "Captured" : "Captured (raw)";
    irSendMessage = "Ready";
    lastDraw = 0;
    uiDirty = true;
  }
  IrReceiver.resume();
}

// make sure the IR LED pin / PWM carrier is (re)initialised right before every transmission
void irTxPrepare() {
  IrSender.begin(IR_TX_PIN);
}

// Send the last captured code (IR Sender)
void replayIR() {
  if (!irCaptured || rawIRLength < 4) {
    irSendMessage = "Capture first";
    lastDraw = 0;
    return;
  }
  // receiver is stopped while in the Sender view, so just transmit
  irTxPrepare();
  bool decoded = (!irSendRaw && !irRawMode);
  if (decoded) {
    IrSender.write(&storedIRData);
    irSendMessage = "Sent decoded";
  } else {
    IrSender.sendRaw(rawIRCode, rawIRLength, 38);
    irSendMessage = (!irSendRaw && irRawMode) ? "Sent raw (unknown)" : "Sent raw";
  }
  irSent++;
  ledFlash(255, 255, 255, 200);
  lastDraw = 0;
}

// Send the code built in Custom IR
void sendCustomIR() {
  irTxPrepare();
  int rep = customRepeats;
  switch (customProto) {
    case 0: IrSender.sendNEC(customAddr, customCmd, rep); break;
    case 1: IrSender.sendSamsung(customAddr, customCmd, rep); break;
    case 2: IrSender.sendSony(customAddr, customCmd, rep < 2 ? 2 : rep, SIRCS_12_PROTOCOL); break;
    case 3: IrSender.sendRC5(customAddr, customCmd, rep); break;
    case 4: IrSender.sendRC6(customAddr, customCmd, rep); break;
    case 5: IrSender.sendLG(customAddr, customCmd, rep); break;
    default: break;
  }
  customSent++;
  customMessage = "Code sent";
  ledFlash(255, 255, 255, 200);
  lastDraw = 0;
}

String hex2(uint8_t v) {
  String s = String(v, HEX);
  s.toUpperCase();
  if (s.length() < 2) s = "0" + s;
  return "0x" + s;
}

String hexStr(uint32_t v) {
  String s = String(v, HEX);
  s.toUpperCase();
  return "0x" + s;
}

void addIRDecodedDetails() {
  if (irProtocol == "UNKNOWN") {
    addLine("Hash", hexStr(irData), C_MUTED, C_WHITE);
    addLine("Bits", String(irBits), C_MUTED, C_WHITE);
    addWrapped("No protocol decode", C_YELLOW);
  } else if (irProtocol == "PULSE_WIDTH" || irProtocol == "PULSE_DISTANCE") {
    addLine("Data", hexStr(irData), C_MUTED, C_WHITE);
    addLine("Bits", String(irBits), C_MUTED, C_WHITE);
  } else {
    addLine("Address", hexStr(irAddress), C_MUTED, C_WHITE);
    addLine("Command", hexStr(irCommand), C_MUTED, C_WHITE);
    addLine("Bits", String(irBits), C_MUTED, C_WHITE);
    addLine("Data", hexStr(irData), C_MUTED, C_WHITE);
  }
}

void buildIRCapture() {
  addLine("Status", irCaptured ? "CAPTURED" : "LISTENING", C_MUTED, irCaptured ? C_GREEN : C_YELLOW);
  addKV("Protocol", irProtocol, C_WHITE);
  if (irCaptured) addIRDecodedDetails();
  addLine("Pulses", String(rawIRLength), C_MUTED, C_WHITE);
  addLine("Captures", String(irCaptures), C_MUTED, C_WHITE);
  addWrapped(irMessage, C_CYAN);
}

void buildIRSender() {
  irRowMethod = -1;
  addLine("Saved", irCaptured ? "YES" : "EMPTY", C_MUTED, irCaptured ? C_GREEN : C_RED);
  if (irCaptured) {
    addKV("Protocol", irProtocol, C_WHITE);
    addIRDecodedDetails();
    addLine("Pulses", String(rawIRLength), C_MUTED, C_WHITE);
    irRowMethod = lineCount;
    bool decoded = (!irSendRaw && !irRawMode);
    addLine("Method", decoded ? "DECODED" : "RAW", C_WHITE, C_CYAN);
    addLine("> SEND", "", C_GREEN, C_GREEN);
  } else {
    addWrapped("Use IR Capture", C_MUTED);
  }
  addLine("Sent", String(irSent), C_MUTED, C_WHITE);
  addWrapped(irSendMessage, C_CYAN);
}

void buildIRCustom() {
  addLine("Protocol", CUSTOM_PROTO_NAMES[customProto], C_WHITE, C_CYAN);
  addLine("Address +16", hex2(customAddr), C_WHITE, C_CYAN);
  addLine("Address +1", hex2(customAddr), C_WHITE, C_CYAN);
  addLine("Command +16", hex2(customCmd), C_WHITE, C_CYAN);
  addLine("Command +1", hex2(customCmd), C_WHITE, C_CYAN);
  addLine("Repeats", String(customRepeats), C_WHITE, C_CYAN);
  addLine("> SEND", "", C_GREEN, C_GREEN);
  addLine("Sent", String(customSent), C_MUTED, C_WHITE);
  addWrapped(customMessage, C_CYAN);
}

void buildIR() {
  startLines();
  switch (subView) {
    case 1: buildIRCapture(); break;
    case 2: buildIRSender(); break;
    case 3: buildIRCustom(); break;
    default:
      addLine("IR Capture", "", C_WHITE, C_WHITE);
      addLine("IR Sender", irCaptured ? "READY" : "EMPTY", C_WHITE, irCaptured ? C_GREEN : C_MUTED);
      addLine("Custom IR", "", C_WHITE, C_WHITE);
      break;
  }
  finishLines();
}

String fmtUptime() {
  unsigned long s = millis() / 1000UL;
  unsigned long m = s / 60, h = m / 60, d = h / 24;
  if (d > 0) return String(d) + "d " + String(h % 24) + "h";
  if (h > 0) return String(h) + "h " + String(m % 60) + "m";
  return String(m) + "m " + String(s % 60) + "s";
}

void buildSystem() {
  bool wifiConn = (WiFi.status() == WL_CONNECTED);
  bool fix = gps.location.isValid() && gps.location.age() < 5000;
  bool uart = gps.charsProcessed() > 0;
  addLine("Wi-Fi", wifiConn ? "CONNECTED" : "READY", C_MUTED, wifiConn ? C_GREEN : C_WHITE);
  addLine("BLE", bleSysStatus(), C_MUTED, bleSysStatus() == "READY" ? C_WHITE : C_GREEN);
  addLine("GPS", uart ? "UART OK" : "NO DATA", C_MUTED, uart ? C_GREEN : C_RED);
  addLine("GPS Fix", fix ? "VALID" : "NO FIX", C_MUTED, fix ? C_GREEN : C_YELLOW);
  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
  addLine("LoRa", loraTried ? (loraOK ? "READY" : "FAILED") : "IDLE", C_MUTED, loraTried ? (loraOK ? C_GREEN : C_RED) : C_WHITE);
  addLine("IR", "READY", C_MUTED, C_GREEN);
  addLine("Display", canvasReady ? "BUFFERED" : "DIRECT", C_MUTED, canvasReady ? C_GREEN : C_YELLOW);
  addLine("Time", timeSynced ? timeSource : String("NONE"), C_MUTED, timeSynced ? C_GREEN : C_YELLOW);
  addLine("Tracker", trackerOn ? "ON" : "OFF", C_MUTED, trackerOn ? C_GREEN : C_MUTED);
  addLine("Heap", String(ESP.getFreeHeap() / 1024) + " KB", C_MUTED, C_WHITE);
  addLine("PSRAM", psramFound() ? String(ESP.getFreePsram() / 1024) + " KB" : String("NONE"), C_MUTED, psramFound() ? C_WHITE : C_RED);
  addLine("Uptime", fmtUptime(), C_MUTED, C_WHITE);
}

void buildAbout() {
  addKV("Device", "WatchOut", C_WHITE);
  addKV("Firmware", FW_VERSION, C_WHITE);
  addKV("Chip", String(ESP.getChipModel()) + " r" + String(ESP.getChipRevision()), C_WHITE);
  addKV("CPU", String(ESP.getChipCores()) + "x " + String(ESP.getCpuFreqMHz()) + " MHz", C_WHITE);
  addKV("Flash", String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB", C_WHITE);
  addKV("PSRAM", String(ESP.getPsramSize() / (1024 * 1024)) + " MB", C_WHITE);
  addKV("SDK", String(ESP.getSdkVersion()), C_WHITE);
  addKV("MAC", WiFi.macAddress(), C_CYAN);
  addKV("Display", "ST7789 240x280", C_WHITE);
  addKV("GPS", "NEO6M " + String(GPS_BAUD) + " baud", C_WHITE);
  addKV("LoRa", "SX1278 " + String((int)(LORA_FREQ / 1E6)) + " MHz", C_WHITE);
  addKV("IR", "RX " + String(IR_RX_PIN) + " TX " + String(IR_TX_PIN), C_WHITE);
  addKV("RGB LED", "WS2812 GPIO" + String(RGB_LED_PIN), C_WHITE);
}

void buildConfig() {
  startLines();
  switch (subView) {
    case 1:
      addLine("Brightness", String(configSelection + 1) + " / " + String(BRIGHT_COUNT), C_WHITE, C_CYAN);
      addLine("Orientation", ORI_NAMES[oriIdx], C_WHITE, C_CYAN);
      addLine("Dim Time", DIM_NAMES[dimIdx], C_WHITE, C_CYAN);
      addLine("Mode", lightMode ? "Light" : "Dark", C_WHITE, C_YELLOW);
      break;
    case 2: {
      const LedColor &c = LED_COLORS[ledColorIdx];
      addLine("LED Mode", LED_MODE_NAMES[ledMode], C_WHITE, C_GREEN);
      addLine("LED Color", c.name, C_WHITE, rgb565(c.r, c.g, c.b));
      addLine("LED Speed", String(ledSpeedIdx + 1) + " / 5", C_WHITE, C_CYAN);
      addLine("LED Level", String(ledLevelIdx + 1) + " / 5", C_WHITE, C_CYAN);
      break;
    }
    case 3: buildSystem(); break;
    case 4: buildAbout(); break;
    default:
      addLine("Display", "", C_WHITE, C_WHITE);
      addLine("LED Matrix", "", C_WHITE, C_WHITE);
      addLine("System", "", C_WHITE, C_WHITE);
      addLine("About", "", C_WHITE, C_WHITE);
      break;
  }
  finishLines();
}

void addNoTimeLines() {
  addLine("Waiting for time", "", C_YELLOW, C_YELLOW);
  addLine("GPS", gps.charsProcessed() ? "NO FIX" : "NO DATA", C_MUTED, C_RED);
  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
}

void buildClock() {
  int Y, Mo, D, h, mi, s, wd;
  startLines();
  if (!getLocalDateTime(Y, Mo, D, h, mi, s, wd)) {
    lastClockSec = -1;
    addBigLine("--:--:--", C_MUTED);
    addNoTimeLines();
    finishLines();
    return;
  }
  if (s == lastClockSec && lineCount > 0) return;
  lastClockSec = s;

  char buf[16];
  const char *ap = "";
  if (CLOCK_24H) {
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", h, mi, s);
  } else {
    ap = (h >= 12) ? "PM" : "AM";
    int hh = h % 12;
    if (hh == 0) hh = 12;
    snprintf(buf, sizeof(buf), "%d:%02d:%02d", hh, mi, s);
  }
  addBigLine(String(buf), C_WHITE);
  if (CLOCK_24H) addLine(tzLabel(), "", C_CYAN, C_CYAN);
  else addLine(String(ap), tzLabel(), C_CYAN, C_MUTED);
  addLine("Source", timeSource, C_MUTED, C_GREEN);
  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
  finishLines();
}

void buildDate() {
  int Y, Mo, D, h, mi, s, wd;
  startLines();
  if (!getLocalDateTime(Y, Mo, D, h, mi, s, wd)) {
    addBigLine("--", C_MUTED);
    addNoTimeLines();
    finishLines();
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d/%02d/%04d", D, Mo, Y);
  addBigLine(String(D), C_WHITE);
  addLine(String(WEEKDAYS[wd]), "", C_CYAN, C_CYAN);
  addLine(String(MONTHS[Mo - 1]), "", C_WHITE, C_WHITE);
  addLine(String(Y), "", C_MUTED, C_MUTED);
  addLine("Date", String(buf), C_MUTED, C_WHITE);
  finishLines();
}

const uint32_t WEATHER_REFRESH_MS = 600000;
float wxLat = DEFAULT_LAT;
float wxLon = DEFAULT_LON;
bool wxFromGPS = false;
unsigned long wxLastOk = 0;
unsigned long lastWeatherFetch = 0;

const char *wxText(int c) {
  if (c == 0) return "Clear sky";
  if (c == 1) return "Mainly clear";
  if (c == 2) return "Partly cloudy";
  if (c == 3) return "Overcast";
  if (c == 45 || c == 48) return "Fog";
  if (c >= 51 && c <= 55) return "Drizzle";
  if (c >= 61 && c <= 65) return "Rain";
  if (c >= 71 && c <= 77) return "Snow";
  if (c >= 80 && c <= 82) return "Rain showers";
  if (c >= 95) return "Thunderstorm";
  return "Unknown";
}

bool jsonNum(const String &s, const char *key, int from, float &out) {
  String k = String("\"") + key + "\":";
  int p = s.indexOf(k, from);
  if (p < 0) return false;
  out = s.substring(p + k.length()).toFloat();
  return true;
}

void buildWeather() {
  startLines();
  if (wxState == 1) {
    addLine("Connecting...", "", C_YELLOW, C_YELLOW);
  } else if (wxState == 2) {
    addLine("Wi-Fi failed", "", C_RED, C_RED);
    addWrapped("Use Wi-Fi app", C_MUTED);
    addWrapped("> Connect AP", C_MUTED);
  } else if (wxState == 4) {
    addLine("Fetch failed", "", C_RED, C_RED);
    addLine("HTTP", String(wxHttp), C_MUTED, C_WHITE);
  } else if (wxState == 3) {
    addBigLine(String(wxTemp, 1) + " C", C_WHITE);
    addWrapped(String(wxText(wxCode)), C_CYAN);
    addLine("Feels", String(wxFeels, 1) + " C", C_MUTED, C_WHITE);
    addLine("Humidity", String((int)wxHum) + "%", C_MUTED, C_WHITE);
    addLine("Wind", String(wxWind, 1) + " km/h", C_MUTED, C_WHITE);
    addLine("Place", wxFromGPS ? "GPS" : "DEFAULT", C_MUTED, wxFromGPS ? C_GREEN : C_YELLOW);
    unsigned long mins = (millis() - wxLastOk) / 60000UL;
    addLine("Updated", mins == 0 ? "now" : String(mins) + " min ago", C_MUTED, C_WHITE);
  } else {
    addLine("Loading...", "", C_YELLOW, C_YELLOW);
  }
  finishLines();
}

void fetchWeather() {
  lastWeatherFetch = millis();
  if (gps.location.isValid()) {
    wxLat = (float)gps.location.lat();
    wxLon = (float)gps.location.lng();
    wxFromGPS = true;
  } else {
    wxLat = DEFAULT_LAT;
    wxLon = DEFAULT_LON;
    wxFromGPS = false;
  }

  wxState = 1;
  buildWeather();
  uiDirty = false;
  renderUIStatic();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiBegin()) {                               // only if a network was saved before
      unsigned long t = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) {
        pumpGPS();
        delay(100);
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED) { wxState = 2; return; }
  syncTimeNTP();

  String url = String("https://api.open-meteo.com/v1/forecast?latitude=") +
               String(wxLat, 4) + "&longitude=" + String(wxLon, 4) +
               "&current=temperature_2m,relative_humidity_2m,apparent_temperature,weather_code,wind_speed_10m&timezone=auto";

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  wxHttp = 0;
  wxState = 4;

  if (http.begin(client, url)) {
    http.setTimeout(8000);
    wxHttp = http.GET();
    if (wxHttp == 200) {
      String body = http.getString();
      int cp = body.indexOf("\"current\":{");
      float v;
      if (cp >= 0 && jsonNum(body, "temperature_2m", cp, v)) {
        wxTemp = v;
        if (jsonNum(body, "apparent_temperature", cp, v)) wxFeels = v;
        if (jsonNum(body, "relative_humidity_2m", cp, v)) wxHum = v;
        if (jsonNum(body, "wind_speed_10m", cp, v)) wxWind = v;
        if (jsonNum(body, "weather_code", cp, v)) wxCode = (int)v;
        if (jsonNum(body, "utc_offset_seconds", 0, v)) tzOffsetMin = (int)(v / 60.0f);
        wxLastOk = millis();
        wxState = 3;
      }
    }
    http.end();
  }
  pumpGPS();
}

String cleanAscii(const String &s) {
  String o = "";
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c >= 32 && c < 127) o += c;
  }
  return o;
}

bool jsonStr(const String &s, const char *key, String &out) {
  String k = String("\"") + key + "\"";
  int p = s.indexOf(k);
  while (p >= 0) {
    int c = p + k.length();
    while (c < (int)s.length() && s[c] == ' ') c++;
    if (c < (int)s.length() && s[c] == ':') {
      c++;
      while (c < (int)s.length() && s[c] == ' ') c++;
      if (c < (int)s.length() && s[c] == '"') {
        int e = s.indexOf('"', c + 1);
        if (e < 0) return false;
        out = cleanAscii(s.substring(c + 1, e));
        return true;
      }
    }
    p = s.indexOf(k, p + 1);
  }
  return false;
}

int httpGetBody(const String &url, String &body) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setUserAgent("WatchOut-ESP32/1.0");
  http.setTimeout(10000);
  if (!http.begin(client, url)) return -100;
  int code = http.GET();
  if (code == 200) body = http.getString();
  http.end();
  return code;
}

void fetchPlace() {
  if (!(gps.location.isValid() && gps.location.age() < 5000)) {
    placeState = 0;
    return;
  }
  float lat = (float)gps.location.lat();
  float lon = (float)gps.location.lng();

  placeState = 1;
  buildGPS();
  uiDirty = false;
  renderUIStatic();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiBegin()) {                               // only if a network was saved before
      unsigned long t = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - t < 12000) {
        pumpGPS();
        delay(100);
      }
    }
  }

  if (WiFi.status() != WL_CONNECTED) { placeState = 2; return; }

  placeState = 4;
  placeName = "";
  placeRegion = "";
  String body = "";

  String url1 = String("https://api.bigdatacloud.net/data/reverse-geocode-client?latitude=") +
                String(lat, 5) + "&longitude=" + String(lon, 5) + "&localityLanguage=en";

  placeHttp = httpGetBody(url1, body);
  if (placeHttp == 200) {
    String locality = "", city = "", sub = "", country = "";
    jsonStr(body, "locality", locality);
    jsonStr(body, "city", city);
    jsonStr(body, "principalSubdivision", sub);
    jsonStr(body, "countryName", country);

    placeName = locality.length() ? locality : city;
    if (placeName.length() == 0) placeName = sub;
    if (sub.length() && sub != placeName) placeRegion = sub;
    if (country.length()) {
      if (placeRegion.length()) placeRegion += ", ";
      placeRegion += country;
    }
  }
  pumpGPS();

  if (placeName.length() == 0) {
    body = "";
    placeRegion = "";
    String url2 = String("https://nominatim.openstreetmap.org/reverse?format=jsonv2&zoom=14&accept-language=en&lat=") +
                  String(lat, 5) + "&lon=" + String(lon, 5);
    placeHttp = httpGetBody(url2, body);
    if (placeHttp == 200) {
      String n = "", st = "", country = "";
      const char *keys[] = {"city", "town", "village", "suburb", "county"};
      for (int i = 0; i < 5 && n.length() == 0; i++) jsonStr(body, keys[i], n);
      jsonStr(body, "state", st);
      jsonStr(body, "country", country);
      placeName = n.length() ? n : st;
      if (st.length() && st != placeName) placeRegion = st;
      if (country.length()) {
        if (placeRegion.length()) placeRegion += ", ";
        placeRegion += country;
      }
    }
  }
  if (placeName.length()) placeState = 3;
  pumpGPS();
}

void pollWeather() {
  if (page != WEATHER_PAGE) return;
  if (millis() - lastWeatherFetch > WEATHER_REFRESH_MS) {
    fetchWeather();
    buildWeather();
  }
}

void c565to888(uint16_t c, uint8_t &r, uint8_t &g, uint8_t &b) {
  r = ((c >> 11) & 0x1F) * 255 / 31;
  g = ((c >> 5) & 0x3F) * 255 / 63;
  b = (c & 0x1F) * 255 / 31;
}

void hsvToRgb(float h, uint8_t &r, uint8_t &g, uint8_t &b) {
  float x = 1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f);
  float rr = 0, gg = 0, bb = 0;
  if      (h < 60)  { rr = 1; gg = x; }
  else if (h < 120) { rr = x; gg = 1; }
  else if (h < 180) { gg = 1; bb = x; }
  else if (h < 240) { gg = x; bb = 1; }
  else if (h < 300) { rr = x; bb = 1; }
  else              { rr = 1; bb = x; }
  r = (uint8_t)(rr * 255);
  g = (uint8_t)(gg * 255);
  b = (uint8_t)(bb * 255);
}

void ledApply(uint8_t style, uint8_t r, uint8_t g, uint8_t b, uint32_t period, bool multi) {
  unsigned long now = millis();
  uint32_t cycle = now / period;
  float phase = (float)(now % period) / (float)period;

  if (multi) {
    const LedColor &c = LED_COLORS[cycle % LED_COLOR_COUNT];
    r = c.r; g = c.g; b = c.b;
  }
  float v = 1.0f;
  if (style == 1) v = (phase < 0.5f) ? 1.0f : 0.0f;
  else if (style == 2) {
    v = (1.0f - cosf(phase * 6.2831853f)) * 0.5f;
    v = v * v;
  }
  ledWrite((uint8_t)(r * v), (uint8_t)(g * v), (uint8_t)(b * v));
}

void ledAuto() {
  uint8_t r = 255, g = 255, b = 255;
  uint8_t style = 2;
  uint32_t period = 2500;

  switch (page) {
    case MENU: c565to888(appColor(selected), r, g, b); break;
    case WIFI_PAGE:
      if (radarOn) { r = 0; g = 255; b = 0; style = 2; period = 1200; }
      else if (apState == 1 || apState == 5) { r = 255; g = 160; b = 0; style = 1; period = 500; }
      else { c565to888(appColor(0), r, g, b); style = wifiStarted ? 2 : 0; period = 800; }
      break;
    case BLE_PAGE: c565to888(appColor(1), r, g, b); period = 1500; break;
    case GPS_PAGE:
      if (gps.location.isValid() && gps.location.age() < 5000) { r = 0; g = 255; b = 0; style = 0; }
      else if (gps.charsProcessed() > 0) { r = 255; g = 160; b = 0; style = 1; period = 1000; }
      else { r = 255; g = 0; b = 0; style = 1; period = 600; }
      break;
    case LORA_PAGE:
      if (loraOK) { r = 0; g = 255; b = 0; period = 3000; }
      else { r = 255; g = 0; b = 0; style = 1; period = 600; }
      break;
    case NRF24_PAGE: c565to888(appColor(4), r, g, b); style = (subView == 2 && nrf24Ready && !nrfHold) ? 2 : 0; period = 1500; break;
    case IR_PAGE: c565to888(appColor(5), r, g, b); style = (subView == 1 && !irCaptured) ? 2 : 0; period = 1500; break;
    case CLOCK_PAGE: c565to888(appColor(6), r, g, b); period = 4000; break;
    case DATE_PAGE: c565to888(appColor(7), r, g, b); style = 0; break;
    case WEATHER_PAGE:
      if (wxState == 1) { r = 255; g = 160; b = 0; style = 1; period = 500; }
      else if (wxState == 3) { c565to888(appColor(8), r, g, b); period = 3000; }
      else { r = 255; g = 0; b = 0; style = 1; period = 700; }
      break;
    case QR_PAGE: c565to888(appColor(9), r, g, b); style = qrServerUp ? 2 : 0; period = 2500; break;
    case FACE_PAGE: r = 196; g = 134; b = 236; period = 4000; break;
    default: r = 140; g = 140; b = 150; style = 0; break;
  }
  ledApply(style, r, g, b, period, false);
}

void ledUpdate() {
  unsigned long now = millis();
  if (now - lastLedUpdate < 20) return;
  lastLedUpdate = now;

  if (ledMode == LED_OFF) {
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
    return;
  }
  if (ledMode == LED_AUTO && now < ledFlashUntil) {
    ledWrite(flashR, flashG, flashB);
    return;
  }

  uint32_t period = LED_PERIODS[ledSpeedIdx];
  const LedColor &c = LED_COLORS[ledColorIdx];

  switch (ledMode) {
    case LED_AUTO: ledAuto(); break;
    case LED_SOLID: ledWrite(c.r, c.g, c.b); break;
    case LED_RAINBOW: {
      uint8_t r, g, b;
      uint32_t full = period * 3;
      hsvToRgb((float)(now % full) * 360.0f / full, r, g, b);
      ledWrite(r, g, b);
      break;
    }
    case LED_BLINK: ledApply(1, c.r, c.g, c.b, period, false); break;
    case LED_BREATHE: ledApply(2, c.r, c.g, c.b, period, false); break;
    case LED_MBLINK: ledApply(1, 0, 0, 0, period, true); break;
    case LED_MBREATHE: ledApply(2, 0, 0, 0, period, true); break;
  }
}

bool irRunning = false;
void irStart() { if (!irRunning) { IrReceiver.start(); irRunning = true; } }
void irStop() { if (irRunning) { IrReceiver.stop(); irRunning = false; } }

void enterPage(Page p) {
  page = p;
  lastDraw = 0;
  pageSel = 0;
  scrollPos = 0.0f;
  scrollVel = 0.0f;
  introP = 0.0f;                  // cascade-in animation for the new page
  lastClockSec = -1;
  subView = 0;
  qrShowing = false;

  switch (page) {
    case MENU: drawMenu(); break;
    case WIFI_PAGE:
      wifiDetailView = false;
      wifiSelectedIndex = 0;
      apStage = 0;
      buildWifi();                // starts on the Wi-Fi sub menu (Scan / Connect AP / Radar)
      break;
    case BLE_PAGE:
      bleDetailView = false;
      bleSelectedIndex = 0;
      subView = 0;
      buildBle();
      break;
    case GPS_PAGE: buildGPS(); break;
    case LORA_PAGE: startLoRa(); buildLoRa(); break;
    case NRF24_PAGE: startNRF24(); buildNRF24(); break;
    case IR_PAGE: irStop(); buildIR(); break;   // starts on the IR submenu, receiver off
    case CLOCK_PAGE: buildClock(); break;
    case DATE_PAGE: buildDate(); break;
    case WEATHER_PAGE: wxState = 0; fetchWeather(); buildWeather(); break;
    case QR_PAGE: buildQR(); break;             // starts on the QR sub menu (Information / Custom)
    case CONFIG_PAGE: buildConfig(); break;
    case VAULT_PAGE: vaultEnter(); buildVault(); break;
    case FACE_PAGE: startLines(); lastFaceKey = 0xFFFFFFFF; break;
  }
  introP = 0.0f;                  // restart after any blocking work (scan / fetch) so the intro is actually seen
  lastAnimMs = millis();
  if (page != MENU) { uiDirty = false; renderUI(); }
  lastDraw = millis();
}

Preferences prefs;
void loadSettings() {
  prefs.begin("watchout", true);
  configSelection = prefs.getUChar("br", 3);
  oriIdx          = prefs.getUChar("ori", 0);
  dimIdx          = prefs.getUChar("dim", 3);
  lightMode       = prefs.getUChar("light", 0) != 0;
  ledMode         = prefs.getUChar("lm", LED_AUTO);
  ledColorIdx     = prefs.getUChar("lc", 1);
  ledSpeedIdx     = prefs.getUChar("ls", 2);
  ledLevelIdx     = prefs.getUChar("ll", 2);
  customProto     = prefs.getUChar("cp", 0);
  customAddr      = prefs.getUChar("ca", 0);
  customCmd       = prefs.getUChar("cc", 0);
  customRepeats   = prefs.getUChar("cr", 0);
  savedSsid       = prefs.getString("wssid", "");
  savedPass       = prefs.getString("wpass", "");
  qrCustomText    = prefs.getString("qrtxt", "");
  nrfView         = prefs.getUChar("nrfv", 0);
  prefs.end();

  if (qrCustomText.length() > QR_MAX_TEXT) qrCustomText = qrCustomText.substring(0, QR_MAX_TEXT);
  if (configSelection < 0 || configSelection >= BRIGHT_COUNT) configSelection = 3;
  if (oriIdx < 0 || oriIdx > 1) oriIdx = 0;
  if (dimIdx < 0 || dimIdx >= DIM_COUNT) dimIdx = 3;
  if (ledMode >= LED_MODE_COUNT) ledMode = LED_AUTO;
  if (ledColorIdx < 0 || ledColorIdx >= LED_COLOR_COUNT) ledColorIdx = 1;
  if (ledSpeedIdx < 0 || ledSpeedIdx > 4) ledSpeedIdx = 2;
  if (ledLevelIdx < 0 || ledLevelIdx > 4) ledLevelIdx = 2;
  if (customProto < 0 || customProto >= CUSTOM_PROTO_COUNT) customProto = 0;
  if (customRepeats < 0 || customRepeats > 3) customRepeats = 0;
  if (nrfView > 3) nrfView = 0;
  screenBrightness = brightLevels[configSelection];
}

void saveSettings() {
  prefs.begin("watchout", false);
  prefs.putUChar("br", configSelection);
  prefs.putUChar("ori", oriIdx);
  prefs.putUChar("dim", dimIdx);
  prefs.putUChar("light", lightMode ? 1 : 0);
  prefs.putUChar("lm", ledMode);
  prefs.putUChar("lc", ledColorIdx);
  prefs.putUChar("ls", ledSpeedIdx);
  prefs.putUChar("ll", ledLevelIdx);
  prefs.putUChar("cp", customProto);
  prefs.putUChar("ca", customAddr);
  prefs.putUChar("cc", customCmd);
  prefs.putUChar("cr", customRepeats);
  prefs.putUChar("nrfv", nrfView);
  prefs.end();
}

// Backlight changes go through backlightUpdate(), which fades smoothly
void markActivity() {
  lastActivity = millis();
  if (dimmed) {
    dimmed = false;
    blTarget = screenBrightness;
  }
}

void dimUpdate() {
  if (dimIdx == 0 || dimmed) return;
  if (millis() - lastActivity > (unsigned long)DIM_SECS[dimIdx] * 1000UL) {
    dimmed = true;
    blTarget = DIM_LEVEL;
  }
}

void gpsClick() {
  if (subView == 0) {
    subParentSel = pageSel;
    subView = (pageSel == 0) ? 1 : 2;
    pageSel = 0;
    scrollPos = 0.0f;
    scrollVel = 0.0f;
  } else if (subView == 1) {
    fetchPlace();
  } else {
    if (trackerOn) stopTracker();
    else startTracker();
  }
  buildGPS();
}

// IR page click handling
// subView 0 = menu, 1 = IR Capture, 2 = IR Sender, 3 = Custom IR
void irClick() {
  if (subView == 0) {
    if (pageSel < 3) {
      subParentSel = pageSel;
      subView = pageSel + 1;
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
      if (subView == 1) {
        irMessage = irCaptured ? "Captured" : "Listening...";
        irStart();                 // receiver only runs inside IR Capture
        IrReceiver.resume();
      } else {
        irStop();
      }
    }
  } else if (subView == 1) {
    // clear last capture and listen again
    irCaptured = false;
    irRawMode = false;
    irProtocol = "None";
    irAddress = 0;
    irCommand = 0;
    irBits = 0;
    irData = 0;
    rawIRLength = 0;
    irMessage = "Listening...";
    IrReceiver.resume();
  } else if (subView == 2) {
    if (irCaptured && irRowMethod >= 0 && pageSel == irRowMethod) {
      // toggle RAW <-> DECODED
      if (irRawMode) {
        irSendRaw = true;
        irSendMessage = "Unknown: RAW only";
      } else {
        irSendRaw = !irSendRaw;
        irSendMessage = "Ready";
      }
    } else {
      replayIR();
    }
  } else if (subView == 3) {
    switch (pageSel) {
      case 0: customProto = (customProto + 1) % CUSTOM_PROTO_COUNT; break;
      case 1: customAddr = (uint8_t)(customAddr + 0x10); break;
      case 2: customAddr = (uint8_t)((customAddr & 0xF0) | ((customAddr + 1) & 0x0F)); break;
      case 3: customCmd = (uint8_t)(customCmd + 0x10); break;
      case 4: customCmd = (uint8_t)((customCmd & 0xF0) | ((customCmd + 1) & 0x0F)); break;
      case 5: customRepeats = (customRepeats + 1) % 4; break;
      case 6: sendCustomIR(); break;
      default: break;
    }
    if (pageSel <= 5) saveSettings();
  }
  buildIR();
}

void nrf24Click() {
  if (subView == 0) {
    subParentSel = pageSel;
    subView = pageSel + 1;
    pageSel = 0;
    scrollPos = 0.0f;
    scrollVel = 0.0f;
    if (subView == 2) nrfSpectrumBegin();            // fresh history + restart the sweep
  } else if (subView == 1) {
    nrf24Ready = nrf24.isChipConnected();
  } else if (subView == 2) {
    nrfView = (nrfView + 1) % 4;                     // BARS -> FALL -> WIFI -> DEVS
    saveSettings();
    uiDirty = true;
  }
  buildNRF24();
}

void configClick() {
  if (subView == 0) {
    if (pageSel < 4) {
      subParentSel = pageSel;
      subView = pageSel + 1;
      pageSel = 0;
      scrollPos = 0.0f;
      scrollVel = 0.0f;
    }
  } else if (subView == 1) {
    switch (pageSel) {
      case 0:
        configSelection = (configSelection + 1) % BRIGHT_COUNT;
        screenBrightness = brightLevels[configSelection];
        blTarget = screenBrightness;               // fades to the new level
        break;
      case 1:
        oriIdx = (oriIdx + 1) % 2;
        lcd.setRotation(oriIdx == 0 ? 2 : 0);
        break;
      case 2: dimIdx = (dimIdx + 1) % DIM_COUNT; break;
      case 3: lightMode = !lightMode; break;
    }
    saveSettings();
  } else if (subView == 2) {
    switch (pageSel) {
      case 0: ledMode     = (ledMode + 1) % LED_MODE_COUNT;      break;
      case 1: ledColorIdx = (ledColorIdx + 1) % LED_COLOR_COUNT; break;
      case 2: ledSpeedIdx = (ledSpeedIdx + 1) % 5;               break;
      case 3: ledLevelIdx = (ledLevelIdx + 1) % 5;               break;
    }
    saveSettings();
  }
  buildConfig();
}

void singleClick() {
  if (page == MENU) {
    enterPage((Page)(selected + 1));
    return;
  }

  switch (page) {
    case WIFI_PAGE: wifiClick(); break;

    case BLE_PAGE: bleClick(); break;

    case GPS_PAGE: gpsClick(); break;
    case LORA_PAGE: loraClick(); break;
    case NRF24_PAGE: nrf24Click(); break;
    case IR_PAGE: irClick(); break;
    case CLOCK_PAGE:
    case DATE_PAGE: break;
    case WEATHER_PAGE: fetchWeather(); buildWeather(); break;
    case QR_PAGE: qrClick(); break;
    case CONFIG_PAGE: configClick(); break;
    case VAULT_PAGE: vaultClick(); break;
    case FACE_PAGE: goBackToMenu(); break;
    default: break;
  }
  lastDraw = millis();
}

void leavePageCleanup() {
  if (page == WIFI_PAGE) {
    radarStop();
    if (wifiStarted) WiFi.scanDelete();
    wifiStarted = false;
    wifiDetailView = false;
    apStage = 0;
  }
  if (page == BLE_PAGE) { bleDetailView = false; bleStopAll(); }
  if (page == NRF24_PAGE) {
    if (nrfJamActive) nrfJamStop();
    if (nrf24Ready) nrf24.stopListening();
    if (wifiStarted) { WiFi.scanDelete(); wifiStarted = false; }   // drop a DEVS-view Wi-Fi scan that is still running
  }
  if (page == IR_PAGE) irStop();
  if (page == VAULT_PAGE) vaultLeave();
  if (page == QR_PAGE) {
    qrShowing = false;
    qrNetStop();                // web server / hotspot only live inside Custom QR
  }
  // a link made with "Connect AP" (apHold) stays up so Weather / NTP / GPS place lookup can use it
  if ((page == WEATHER_PAGE || page == GPS_PAGE) && !trackerOn && !apHold) {
    WiFi.disconnect(false, false);
  }
}

void goBackToMenu() {
  if (page == MENU) return;
  leavePageCleanup();
  enterPage(MENU);
}

void openWatchFace() {
  if (page == FACE_PAGE) return;
  leavePageCleanup();
  enterPage(FACE_PAGE);
}

#ifdef HAVE_ADC_ONESHOT
adc_oneshot_unit_handle_t joyAdc = nullptr;
adc_channel_t joyChX = ADC_CHANNEL_0;
adc_channel_t joyChY = ADC_CHANNEL_0;
#endif

bool joyAdcOK = false;
int joyX = 2048;
int joyY = 2048;
unsigned long joyLastGood = 0;

void joyAdcInit() {
#ifdef HAVE_ADC_ONESHOT
  adc_unit_t unitX, unitY;
  if (adc_oneshot_io_to_channel(JOY_X, &unitX, &joyChX) != ESP_OK) return;
  if (adc_oneshot_io_to_channel(JOY_Y, &unitY, &joyChY) != ESP_OK) return;
  if (unitX != unitY) return;

  adc_oneshot_unit_init_cfg_t ucfg = {};
  ucfg.unit_id = unitX;
  ucfg.ulp_mode = ADC_ULP_MODE_DISABLE;

  if (adc_oneshot_new_unit(&ucfg, &joyAdc) != ESP_OK) {
    joyAdc = nullptr;
    return;
  }

  adc_oneshot_chan_cfg_t ccfg = {};
  ccfg.atten = (adc_atten_t)3;
  ccfg.bitwidth = ADC_BITWIDTH_12;

  if (adc_oneshot_config_channel(joyAdc, joyChX, &ccfg) != ESP_OK ||
      adc_oneshot_config_channel(joyAdc, joyChY, &ccfg) != ESP_OK) {
    adc_oneshot_del_unit(joyAdc);
    joyAdc = nullptr;
    return;
  }
  joyAdcOK = true;
#endif
}

bool joyReadRaw(int &x, int &y) {
#ifdef HAVE_ADC_ONESHOT
  if (joyAdcOK) {
    for (int attempt = 0; attempt < 3; attempt++) {
      int vx = 0, vy = 0;
      if (adc_oneshot_read(joyAdc, joyChX, &vx) == ESP_OK &&
          adc_oneshot_read(joyAdc, joyChY, &vy) == ESP_OK) {
        x = vx; y = vy;
        return true;
      }
    }
    return false;
  }
#endif
  int vx = analogRead(JOY_X);
  int vy = analogRead(JOY_Y);
  if (vx == 0 && vy == 0) return false;
  x = vx; y = vy;
  return true;
}

void pollJoystick() {
  unsigned long now = millis();
  int rx, ry;

  if (joyReadRaw(rx, ry)) {
    joyX = rx; joyY = ry;
    joyLastGood = now;
  } else if (now - joyLastGood > 120) {
    joyX = 2048; joyY = 2048;
  }

  int x = joyX;
  int y = joyY;

  static bool swRaw = false;
  static bool swStable = false;
  static unsigned long swChange = 0;

  bool rawDown = (digitalRead(JOY_SW) == LOW);
  if (rawDown != swRaw) { swRaw = rawDown; swChange = now; }
  if (now - swChange >= 25) swStable = swRaw;

  static unsigned long leftSince = 0;
  static unsigned long rightSince = 0;
  static unsigned long ySince = 0;
  static int yPrev = 0;

  if (x < 1000) { if (!leftSince) leftSince = now ? now : 1; }
  else leftSince = 0;

  if (x > 3000) { if (!rightSince) rightSince = now ? now : 1; }
  else rightSince = 0;

  bool leftHeld  = leftSince  && (now - leftSince  >= 50);
  bool rightHeld = rightSince && (now - rightSince >= 50);

  int yd = 0;
  if (y > 3000) yd = -1;
  else if (y < 1000) yd = 1;

  if (yd != yPrev) { yPrev = yd; ySince = now; }
  bool yHeld = (yd != 0) && (now - ySince >= 30);

  if (leftHeld || rightHeld || yHeld || swStable) markActivity();
  if (x < 2500) rightSwipeArmed = true;
  if (x > 1500) leftSwipeArmed = true;

  if (page != MENU && !kbActive() && rightSwipeArmed && rightHeld && now - lastRightSwipe > 350) {
    rightSwipeArmed = false;
    lastRightSwipe = now;
    if (page == WIFI_PAGE && subView != 0) {
      wifiBack();
    } else if (page == LORA_PAGE && subView != 0) {
      loraBack();
    } else if (page == QR_PAGE && (subView != 0 || qrShowing)) {
      qrBack();
    } else if (page == NRF24_PAGE && subView != 0) {
      subView = 0;
      pageSel = subParentSel;
      scrollPos = (float)pageSel;
      scrollVel = 0.0f;
      buildNRF24();
    } else if ((page == GPS_PAGE || page == CONFIG_PAGE || page == IR_PAGE) && subView != 0) {
      subView = 0;
      pageSel = subParentSel;
      scrollPos = (float)pageSel;
      scrollVel = 0.0f;
      if (page == GPS_PAGE) buildGPS();
      else if (page == IR_PAGE) { irStop(); buildIR(); }
      else buildConfig();
    } else if (page == BLE_PAGE && subView != 0) {
      bleBack();
    } else if (page == VAULT_PAGE) {
      vaultBack();
    } else {
      goBackToMenu();
    }
    return;
  }

  if (page != FACE_PAGE && !kbActive() && leftSwipeArmed && leftHeld && now - lastLeftSwipe > 350) {
    leftSwipeArmed = false;
    lastLeftSwipe = now;
    openWatchFace();
    return;
  }

  // on-screen keyboard: stick moves the key cursor (repeats while held)
  if (kbActive()) {
    static int kbDirPrev = 0;
    static unsigned long kbNext = 0;
    int dx = leftHeld ? -1 : (rightHeld ? 1 : 0);
    int dy = yHeld ? yd : 0;                 // -1 = up, +1 = down
    int dir = dx != 0 ? dx : dy * 2;
    if (dir == 0) {
      kbDirPrev = 0;
    } else if (dir != kbDirPrev) {
      kbDirPrev = dir;
      kbNext = now + 380;
      kbMove(dx, dx != 0 ? 0 : dy);
    } else if (now >= kbNext) {
      kbNext = now + 140;
      kbMove(dx, dx != 0 ? 0 : dy);
    }
  }

  // NRF24 spectrum: stick up/down moves the frequency marker (down = towards 2483 MHz) or selects a signal in DEVS
  if (page == NRF24_PAGE && subView == 2 && yHeld && now - lastNavigation > 70) {
    lastNavigation = now;
    nrfMoveCursor(yd);
  }

  if (!kbActive() && yHeld && now - lastNavigation > NAV_MS) {
    int dir = yd;
    if (page == MENU) {
      int ns = selected + dir;
      if (ns >= 0 && ns < MENU_COUNT) {
        selected = ns;
        lastNavigation = now;
        uiDirty = true;
      }
    } else {
      int ns = pageSel + dir;
      if (ns >= 0 && ns < lineCount) {
        pageSel = ns;
        lastNavigation = now;
        uiDirty = true;
      }
    }
  }

  static unsigned long btnDownAt = 0;
  bool down = swStable;
  if (down && !buttonWasDown) {
    buttonWasDown = true;
    btnDownAt = now;
  } else if (!down && buttonWasDown) {
    buttonWasDown = false;
    if (now - btnDownAt >= 700 && (loraLongClick() || wifiLongClick() || nrfLongClick() || vaultLongClick())) {
      lastDraw = millis();
    } else {
      singleClick();
    }
  }
}

void setup() {
#if DISABLE_BROWNOUT
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- WATCHOUT SYSTEM BOOTING ---");

  WiFi.persistent(false);
  loadSettings();

  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(LCD_BL, OUTPUT);
  blCur = 0.0f;                                   // start dark, then fade the backlight in
  blTarget = screenBrightness;
  analogWrite(LCD_BL, 0);

  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

  joyAdcInit();
  if (!joyAdcOK) analogReadResolution(12);

  Serial.printf("Joystick ADC: %s\n", joyAdcOK ? "IDF driver (Wi-Fi safe)" : "analogRead fallback");

  SPI.begin(LCD_SCLK, LCD_MISO, LCD_MOSI, LCD_CS);
  lcd.init(240, 280);
  lcd.setRotation(oriIdx == 0 ? 2 : 0);
  lcd.setSPISpeed(LCD_SPI_HZ);
  lcd.fillScreen(ST77XX_BLACK);

  canvas = new GFXcanvas16(240, 280);
  canvasReady = (canvas != nullptr && canvas->getBuffer() != nullptr);

  Serial.printf("Canvas buffer: %s\n", canvasReady ? "OK" : "FAILED (drawing directly)");

  GPSSerial.setRxBufferSize(2048);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
  IrReceiver.stop();
  irRunning = false;
  IrSender.begin(IR_TX_PIN);

  Serial.println("ESP32-S3 WATCHOUT MULTI-TESTER READY");
  lastActivity = millis();
  lastAnimMs = millis();
  enterPage(MENU);
}

void loop() {
  pollIR();

  while (GPSSerial.available()) gps.encode((char)GPSSerial.read());

  syncTimeFromGPS();
  pollJoystick();
  animateUI();
  backlightUpdate();
  ledUpdate();
  pollWeather();
  pollWiFi();
  pollAP();
  pollRadar();
  pollLoRa();
  pollBle();
  pollNRF24Spectrum();
  trackerPoll();
  qrPoll();
  vaultPoll();
  dimUpdate();

  if (page != MENU && millis() - lastDraw >= (page == CLOCK_PAGE ? (uint32_t)250 : PAGE_REFRESH_MS)) {
    lastDraw = millis();
    switch (page) {
      case WIFI_PAGE: buildWifi(); break;
      case BLE_PAGE: buildBle(); break;
      case GPS_PAGE: buildGPS(); break;
      case LORA_PAGE: buildLoRa(); break;
      case NRF24_PAGE: buildNRF24(); break;
      case IR_PAGE: buildIR(); break;
      case CLOCK_PAGE: buildClock(); break;
      case DATE_PAGE: buildDate(); break;
      case WEATHER_PAGE: buildWeather(); break;
      case QR_PAGE: buildQR(); break;
      case CONFIG_PAGE: buildConfig(); break;
      case VAULT_PAGE: buildVault(); break;
      case FACE_PAGE: faceUpdate(); break;
      default: break;
    }
  }
}
#include "vault.h"
/*git add -A
git commit -m "Update project files and changes"
git push*/
