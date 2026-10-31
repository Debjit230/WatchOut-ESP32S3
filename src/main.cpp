

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <BLEDevice.h>
#include <BLEScan.h>
#include <SD.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
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

// Offline tracker options
#define TRACKER_USE_WIFI    1      // 1 = upload/live-view when Wi-Fi is available, 0 = pure offline SD logger
#define TRACKER_LORA_BEACON 0      // 1 = also broadcast position over LoRa (433 MHz) so another LoRa node can follow you
#define TRACKER_BEACON_S    30
#define TRACKER_FILE        "/track.csv"

// Custom QR options
#define QR_AP_SSID   "WatchOut-QR"   // hotspot used when no Wi-Fi is available
#define QR_AP_PASS   "watchout123"   // min 8 characters
#define QR_MAX_TEXT  300

#define FW_VERSION "v1.8"

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

#define SD_CS     19
#define IR_RX_PIN 4
#define IR_TX_PIN 5

// ---------------- HARDWARE ----------------
Adafruit_ST7789 lcd(&SPI, LCD_CS, LCD_DC, LCD_RST);
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
BLEScan *bleScan = nullptr;

GFXcanvas16 *canvas = nullptr;
bool canvasReady = false;

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
  SD_PAGE,
  IR_PAGE,
  CLOCK_PAGE,
  DATE_PAGE,
  WEATHER_PAGE,
  QR_PAGE,
  CONFIG_PAGE,
  FACE_PAGE
};

Page page = MENU;

void enterPage(Page p);
void goBackToMenu();
void saveSettings();
void startLoRa();
void buildWifi();
void kbPress();
void kbOpen();
void buildQR();
void renderQR();
void qrBack();
void qrNetStop();

const char *menuItems[] = {
  "Wi-Fi", "Bluetooth", "GPS", "LoRa", "SD Card",
  "IR Remote", "Clock", "Date", "Weather", "QR Code", "Settings"
};
const int MENU_COUNT = 11;
int selected = 0;

const int LIST_CENTER_Y = 130;
const int ROW_H         = 60;
const int ICON_CX       = 38;
const int ICON_R        = 22;
const int TEXT_X        = 72;

#define MAX_LINES   48
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
bool uiDirty = true;
unsigned long lastFrame = 0;

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
// QR sub views: 0 menu, 1 fake QR list, 2 custom QR (web page)
bool qrShowing = false;             // true = a QR code is drawn full screen
String qrCustomText = "";           // text received from the phone (stored in flash)

inline bool kbWifi() { return page == WIFI_PAGE && subView == 2 && apStage == 1; }
// Joystick keyboard is active on LORA_PAGE sub view 4 (chat) and on the Wi-Fi password screen
inline bool kbActive() { return (page == LORA_PAGE && subView == 4) || kbWifi(); }
inline bool radarActive() { return page == WIFI_PAGE && subView == 3 && radarOn; }
inline bool qrActive() { return page == QR_PAGE && qrShowing; }
void renderKeyboard();
void kbMove(int dx, int dy);
void loraBack();
bool loraLongClick();
bool wifiLongClick();
void wifiBack();

const uint32_t NAV_MS = 150;
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
int kbMode = 0;      // 0 abc, 1 ABC (one shot), 2 123/symbols
int kbRow = 0;
int kbCol = 0;

bool sdOK = false;
bool sdTried = false;
bool sdTestOK = false;
String sdMessage = "Not tested";

// ---------------- IR STATE ----------------
// IR sub views: 0 = IR menu, 1 = IR Capture, 2 = IR Sender, 3 = Custom IR
bool irCaptured = false;
bool irRawMode = false;
IRData storedIRData;
uint8_t rawIRCode[RAW_BUFFER_LENGTH];
uint16_t rawIRLength = 0;
String irProtocol = "None";
String irMessage = "Listening...";
uint16_t irAddress = 0;
uint16_t irCommand = 0;
uint32_t irCaptures = 0;
String irSendMessage = "Ready";
uint32_t irSent = 0;

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
      g->fillRect(cx - S(8), cy - S(11), S(16), S(22), w);
      g->fillTriangle(cx + S(2), cy - S(11), cx + S(8) + 1, cy - S(11), cx + S(8) + 1, cy - S(5), accent);
      for (int s = 0; s < 3; s++) g->fillRect(cx - S(5) + s * S(3.5f), cy - S(8), max(1, S(2)), S(6), accent);
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

void drawAppRow(Adafruit_GFX *g, int i, float scroll) {
  float off = (float)i - scroll;
  int cy = LIST_CENTER_Y + (int)lroundf(off * ROW_H);
  if (cy < -40 || cy > 320) return;

  float d = fabsf(off);
  float dNear = fminf(d, 1.0f);
  float k = 1.0f - 0.2f * dNear;

  int brI = 255 - (int)(70.0f * d);
  if (brI < 100) brI = 100;
  uint8_t br = (uint8_t)brI;

  int R = (int)lroundf(ICON_R * k);
  uint16_t accent = dimc(appColor(i), br);

  g->fillCircle(ICON_CX, cy, R, accent);
  drawGlyph(g, i, ICON_CX, cy, k, br, accent);

  uint16_t tc = dimc(themeColor(C_WHITE), br);
  g->setTextSize(2);
  g->setTextWrap(false);
  g->setTextColor(tc);
  g->setCursor(TEXT_X, cy - 7);
  g->print(menuItems[i]);
  g->setCursor(TEXT_X + 1, cy - 7);
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
  if (canvasReady) lcd.drawRGBBitmap(0, 0, canvas->getBuffer(), 240, 280);
}

void drawMenu() {
  scrollPos = (float)selected;
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
  int brI = 255 - (int)(55.0f * d);
  if (brI < 70) brI = 70;
  uint8_t br = (uint8_t)brI;

  String l = lines[i].l;
  String r = lines[i].r;

  if (r.length() > LINE_CHARS - 1) r = r.substring(0, LINE_CHARS - 1);
  int maxL = LINE_CHARS - (r.length() ? (int)r.length() + 1 : 0);
  if (maxL < 1) maxL = 1;
  if ((int)l.length() > maxL) l = l.substring(0, maxL);

  if (d < 0.5f) {
    int mb = 255 - (int)(d * 510.0f);
    g->fillRoundRect(3, cy - 11, 3, 22, 1, dimc(themeColor(C_CYAN), (uint8_t)mb));
  }

  if (lines[i].big) {
    int bw = (int)l.length() * 24;
    int bx = (240 - bw) / 2;
    if (bx < 8) bx = 8;
    boldText(g, bx, cy - 14, l, dimc(themeColor(lines[i].lc), br), 4, 2);
    return;
  }

  boldText(g, TEXT_LEFT_X, cy - 7, l, dimc(themeColor(lines[i].lc), br), 2, 1);
  if (r.length()) {
    int rx = TEXT_RIGHT_X - (int)r.length() * 12;
    boldText(g, rx, cy - 7, r, dimc(themeColor(lines[i].rc), br), 2, 1);
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
  if (canvasReady) lcd.drawRGBBitmap(0, 0, canvas->getBuffer(), 240, 280);
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

  if (canvasReady) lcd.drawRGBBitmap(0, 0, canvas->getBuffer(), 240, 280);
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
  unsigned long lastSeen;
  bool active;
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
  if (!radarT[idx].active) radarT[idx].angle = (float)random(360) * DEG2RAD;
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

  // sweep line with a fading trail
  float sw = (float)(now % 2400UL) / 2400.0f * 6.2831853f;
  for (int k = 6; k >= 0; k--) {
    float a = sw - k * 0.09f;
    g->drawLine(cx, cy, cx + (int)lroundf(cosf(a) * R), cy + (int)lroundf(sinf(a) * R),
                dimc(base, (uint8_t)(255 - k * 34)));
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
    float d = (float)(R - 8) - t * (float)(R - 16);
    int tx = cx + (int)lroundf(cosf(snap[i].angle) * d);
    int ty = cy + (int)lroundf(sinf(snap[i].angle) * d);

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

  if (canvasReady) lcd.drawRGBBitmap(0, 0, canvas->getBuffer(), 240, 280);
}

void renderUI() {
  if (page == MENU) renderMenu();
  else if (page == FACE_PAGE) renderFace();
  else if (kbActive()) renderKeyboard();
  else if (radarActive()) renderRadar();
  else if (qrActive()) renderQR();
  else renderPage();
}

void animateUI() {
  unsigned long now = millis();
  if (now - lastFrame < 25) return;
  float target = (page == MENU) ? (float)selected : (float)pageSel;
  float diff = target - scrollPos;
  if (fabsf(diff) > 0.004f) {
    scrollPos += diff * 0.38f;
    if (fabsf(target - scrollPos) < 0.01f) scrollPos = target;
    uiDirty = true;
  }
  if (radarActive()) uiDirty = true;            // radar sweep animates continuously
  if (uiDirty) {
    uiDirty = false;
    lastFrame = now;
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
  renderUI();

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
      kbOpen();
    }
  }
}

void wifiEnterView(int v) {
  subView = v;
  pageSel = 0;
  scrollPos = 0.0f;
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
    buildWifi();
    return;
  }
  if (subView == 2 && apStage == 1) { wifiKbBack(); return; }
  if (subView == 2 && apStage == 2) {
    apStage = 0;
    pageSel = apListSel;
    scrollPos = (float)pageSel;
    buildWifi();
    return;
  }
  if (subView == 3) radarStop();
  subView = 0;
  wifiDetailView = false;
  apStage = 0;
  pageSel = subParentSel;
  scrollPos = (float)pageSel;
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

void startBLEScan() {
  if (!bleScan) {
    BLEDevice::init("");
    bleScan = BLEDevice::getScan();
    bleScan->setActiveScan(true);
    bleScan->setInterval(100);
    bleScan->setWindow(80);
  }
  bleStarted = true;
  BLEScanResults *results = bleScan->start(2, false);
  if (results != nullptr) {
    int n = results->getCount();
    for (int i = 0; i < n; i++) {
      BLEAdvertisedDevice d = results->getDevice(i);
      String mac = String(d.getAddress().toString().c_str());
      String name = d.haveName() ? String(d.getName().c_str()) : String("Unknown device");
      int rssi = d.getRSSI();
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
  }
  bleScan->clearResults();
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

// ---------------- OFFLINE TRACKER ----------------
// Every fix is appended to /track.csv on the SD card:  epoch,lat,lon,speed_knots,bearing,alt,hdop
// If Wi-Fi is up, points not yet uploaded are sent to the Traccar server (with their real timestamps).
bool trkSdReady = false;
uint32_t trkOff = 0;              // byte offset in the log of the next point to upload
uint32_t trkLogSize = 0;
uint32_t trkLogged = 0;           // points logged this session
uint32_t trkPending = 0;          // points logged but not uploaded yet
uint32_t trkPendingOff = 0;
uint8_t trkAckSinceSave = 0;
volatile bool trkAwaiting = false;
unsigned long lastTrkLog = 0;
unsigned long trkRetryAt = 0;
unsigned long lastTrkSdTry = 0;
unsigned long lastTrkBeacon = 0;
uint32_t trkBeacons = 0;

void trkSaveOff() {
  Preferences tp;
  tp.begin("watchout", false);
  tp.putUInt("tkoff", trkOff);
  tp.end();
  trkAckSinceSave = 0;
}

bool trackerSdMount() {
  if (sdOK) return true;
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(LCD_CS, HIGH);
  sdOK = SD.begin(SD_CS, SPI, 4000000);
  sdTried = true;
  return sdOK;
}

void trackerPrepareLog() {
  trkSdReady = trackerSdMount();
  trkPending = 0;
  if (!trkSdReady) return;

  Preferences tp;
  tp.begin("watchout", true);
  trkOff = tp.getUInt("tkoff", 0);
  tp.end();

  File f = SD.open(TRACKER_FILE, FILE_READ);
  if (!f) { trkLogSize = 0; trkOff = 0; return; }
  trkLogSize = f.size();
  if (trkOff > trkLogSize) trkOff = 0;      // card was replaced / file cleared
  f.seek(trkOff);
  uint8_t buf[512];
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    for (int i = 0; i < n; i++) if (buf[i] == '\n') trkPending++;
    pumpGPS();
  }
  f.close();
}

bool trackerEpoch(uint32_t &ep) {
  if (!(gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2024)) return false;
  int64_t e = (int64_t)daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day()) * 86400LL +
              gps.time.hour() * 3600LL + gps.time.minute() * 60LL + gps.time.second();
  ep = (uint32_t)e;
  return true;
}

void trackerLogPoint() {
  if (!trkSdReady) return;
  uint32_t ep;
  if (!trackerEpoch(ep)) return;
  char buf[128];
  snprintf(buf, sizeof(buf), "%lu,%.6f,%.6f,%.1f,%.0f,%.1f,%.1f\n",
           (unsigned long)ep, gps.location.lat(), gps.location.lng(),
           gps.speed.isValid() ? gps.speed.knots() : 0.0,
           gps.course.isValid() ? gps.course.deg() : 0.0,
           gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
           gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
  File f = SD.open(TRACKER_FILE, FILE_APPEND);
  if (!f) { trkSdReady = false; sdOK = false; return; }   // card lost, retry mount later
  f.print(buf);
  trkLogSize = f.size();
  f.close();
  trkLogged++;
  trkPending++;
}

#if TRACKER_USE_WIFI
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

// Download the whole SD log:  http://watchout.local/log.csv
void trackerHandleLog() {
  if (!trkSdReady) { webServer.send(404, "text/plain", "No SD log"); return; }
  File f = SD.open(TRACKER_FILE, FILE_READ);
  if (!f) { webServer.send(404, "text/plain", "No log file yet"); return; }
  webServer.sendHeader("Content-Disposition", "attachment; filename=track.csv");
  webServer.streamFile(f, "text/csv");
  f.close();
}
#endif

void startTracker() {
  trackerOn = true;
  trkServerUp = false;
  trkLastCode = 0;
  lastTrkSend = 0;
  lastTrkLog = 0;
  lastTrkBeacon = 0;
  trkAwaiting = false;
  trkRetryAt = 0;
  trkLogged = 0;
  trkAckSinceSave = 0;
  trackerPrepareLog();
#if TRACKER_USE_WIFI
  if (!trkHandlers) {
    webServer.on("/", trackerHandleRoot);
    webServer.on("/loc", trackerHandleLoc);
    webServer.on("/log.csv", trackerHandleLog);
    trkHandlers = true;
  }
  wifiBegin();
  lastTrkWifiTry = millis();
#endif
#if TRACKER_LORA_BEACON
  if (!loraOK) startLoRa();
#endif
}

void stopTracker() {
  trackerOn = false;
  if (trkSdReady) trkSaveOff();
  if (trkServerUp) { webServer.stop(); trkServerUp = false; }
  if (trkMdns) { MDNS.end(); trkMdns = false; }
#if TRACKER_USE_WIFI
  if (!apHold) WiFi.disconnect(false, false);
#endif
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

// Fallback when there is no SD card: send the live position only (old behaviour)
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

// Upload the next un-sent point from the SD log (with its original timestamp)
void trackerSendNext() {
  File f = SD.open(TRACKER_FILE, FILE_READ);
  if (!f) { trkSdReady = false; sdOK = false; return; }
  if (trkOff >= f.size()) { trkPending = 0; f.close(); return; }
  f.seek(trkOff);
  String line = f.readStringUntil('\n');
  uint32_t next = f.position();
  f.close();
  line.trim();

  unsigned long ep = 0;
  double lat = 0, lon = 0;
  float spd = 0, brg = 0, alt = 0, hd = 0;
  int n = sscanf(line.c_str(), "%lu,%lf,%lf,%f,%f,%f,%f", &ep, &lat, &lon, &spd, &brg, &alt, &hd);
  if (n != 7) {                       // damaged line: skip it
    trkOff = next;
    if (trkPending > 0) trkPending--;
    return;
  }
  trkPendingOff = next;
  String url = String(TRACKER_SERVER) + "/?id=" + TRACKER_ID +
               "&timestamp=" + String(ep) +
               "&lat=" + String(lat, 6) + "&lon=" + String(lon, 6) +
               "&speed=" + String(spd, 1) + "&bearing=" + String(brg, 0) +
               "&altitude=" + String(alt, 1) + "&hdop=" + String(hd, 1);
  if (trackerFire(url)) trkAwaiting = true;
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

  // 1) always log to SD, with or without Wi-Fi
  if (fix && now - lastTrkLog >= (unsigned long)TRACKER_INTERVAL_S * 1000UL) {
    lastTrkLog = now;
    trackerLogPoint();
  }

  // 2) retry SD mount if the card was missing / lost
  if (!trkSdReady && now - lastTrkSdTry > 30000) {
    lastTrkSdTry = now;
    trackerPrepareLog();
  }

#if TRACKER_LORA_BEACON
  if (fix && now - lastTrkBeacon >= (unsigned long)TRACKER_BEACON_S * 1000UL) {
    lastTrkBeacon = now;
    trackerBeacon();
  }
#endif

#if TRACKER_USE_WIFI
  // 3) Wi-Fi is optional: only used to upload the backlog and serve the live map
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

  if (trkAwaiting && !trkSending) {
    if (trkLastCode == 200) {
      trkOff = trkPendingOff;
      if (trkPending > 0) trkPending--;
      if (++trkAckSinceSave >= 10) trkSaveOff();
    } else {
      trkRetryAt = now + 5000;        // server unreachable: wait and try again
    }
    trkAwaiting = false;
  }

  if (!trkSending && !trkAwaiting && strlen(TRACKER_SERVER) > 0) {
    if (trkSdReady) {
      if (trkPending > 0 && now >= trkRetryAt) trackerSendNext();
    } else if (fix && now - lastTrkSend >= (unsigned long)TRACKER_INTERVAL_S * 1000UL) {
      trackerSend();                  // no SD card: live upload only
    }
  }
#endif
}

void buildGPSTracker() {
  startLines();
  bool wifi = WiFi.status() == WL_CONNECTED;
  bool fix = gps.location.isValid() && gps.location.age() < 5000;
  addLine("Tracker", trackerOn ? "ON" : "OFF", C_WHITE, trackerOn ? C_GREEN : C_RED);
  if (trackerOn) {
#if TRACKER_USE_WIFI
    addLine("Wi-Fi", wifi ? "ONLINE" : "OFFLINE", C_MUTED, wifi ? C_GREEN : C_YELLOW);
    if (wifi) {
      addKV("Open", WiFi.localIP().toString(), C_CYAN);
      addKV("Or", "watchout.local", C_CYAN);
    }
#else
    addLine("Mode", "OFFLINE", C_MUTED, C_YELLOW);
#endif
    addLine("SD Log", trkSdReady ? "LOGGING" : "NO SD", C_MUTED, trkSdReady ? C_GREEN : C_RED);
  }
  addLine("Fix", fix ? "VALID" : "NO FIX", C_MUTED, fix ? C_GREEN : C_YELLOW);
  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
  if (trackerOn && trkSdReady) {
    addLine("Logged", String((unsigned long)trkLogged), C_MUTED, C_WHITE);
    addLine("Pending", String((unsigned long)trkPending), C_MUTED, trkPending ? C_YELLOW : C_GREEN);
    addLine("Log size", String((unsigned long)(trkLogSize / 1024)) + " KB", C_MUTED, C_WHITE);
  }
#if TRACKER_LORA_BEACON
  addLine("Beacons", String((unsigned long)trkBeacons), C_MUTED, C_WHITE);
#endif
#if TRACKER_USE_WIFI
  if (strlen(TRACKER_SERVER) > 0) {
    int code = trkLastCode;
    addLine("Uploaded", String((unsigned long)trkSent), C_MUTED, C_WHITE);
    addLine("Server", code == 0 ? String("--") : (code == 200 ? String("OK") : String(code)),
            C_MUTED, code == 200 ? C_GREEN : (code == 0 ? C_MUTED : C_RED));
  }
#endif
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

void runSDTest() {
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(LCD_CS, HIGH);
  sdOK = SD.begin(SD_CS, SPI, 4000000);
  sdTried = true;
  sdTestOK = false;
  if (!sdOK) { sdMessage = "Mount failed: check SPI/CS"; return; }
  File f = SD.open("/module_test.txt", FILE_WRITE);
  if (!f) { sdMessage = "File write failed"; return; }
  f.println("ESP32-S3 SD test passed");
  f.close();
  f = SD.open("/module_test.txt", FILE_READ);
  if (!f) { sdMessage = "File read failed"; return; }
  String data = f.readStringUntil('\n');
  f.close();
  sdTestOK = data.indexOf("test passed") >= 0;
  sdMessage = sdTestOK ? "Read/write PASS" : "Data mismatch";
}

void buildSD() {
  startLines();
  addLine("Card", sdOK ? "OK" : "FAILED", C_MUTED, sdOK ? C_GREEN : C_RED);
  addLine("Test", "", C_MUTED, C_MUTED);
  addWrapped(sdMessage, sdTestOK ? C_GREEN : C_YELLOW);
  if (sdOK) {
    uint64_t mb = SD.cardSize() / (1024ULL * 1024ULL);
    addLine("Size", String((unsigned long)mb) + " MB", C_MUTED, C_WHITE);
  } else {
    addWrapped("Check wiring", C_MUTED);
    addWrapped("FAT32, 3.3V", C_MUTED);
  }
  finishLines();
}

void startLoRa() {
  digitalWrite(SD_CS, HIGH);
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
  const String &buf = w ? apPass : chatText;
  const int maxLen = w ? 63 : LORA_MAX_TEXT;

  const uint16_t fg     = themeColor(C_WHITE);
  const uint16_t muted  = themeColor(C_MUTED);
  const uint16_t accent = themeColor(C_CYAN);
  const uint16_t keyBg  = lightMode ? 0xDEFB : 0x2104;
  const uint16_t selFg  = bgColor();

  // header
  String hd = w ? (String("Pass: ") + apTargetSsid) : String("Custom chat");
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
  if (w && !apShowPass) {
    shown = "";
    for (unsigned int i = 0; i + 1 < buf.length(); i++) shown += '*';
    if (buf.length()) shown += buf[buf.length() - 1];
  }
  String t = shown + "_";
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
  const char *specLabel[6] = {modeLabel, "SPACE", "DEL", w ? "JOIN" : "SEND",
                              w ? (apShowPass ? "HIDE" : "SHOW") : "SAVE", "BACK"};
  const uint16_t specColor[6] = {fg, fg, themeColor(C_RED), themeColor(C_GREEN),
                                 w ? themeColor(C_CYAN) : themeColor(C_YELLOW), themeColor(C_RED)};

  for (int r = 0; r < 4; r++) {
    int n = kbRowLen(r);
    int step = (r == 3) ? 38 : 22;
    int kw = step - 2;
    int x0 = (240 - n * step) / 2 + 1;
    int y = 124 + r * 30;
    for (int c = 0; c < n; c++) {
      int x = x0 + c * step;
      bool sel = (r == kbRow && c == kbCol);
      g->fillRoundRect(x, y, kw, 26, 4, sel ? accent : keyBg);
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

  if (canvasReady) lcd.drawRGBBitmap(0, 0, canvas->getBuffer(), 240, 280);
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
  buildLoRa();
}

void loraEnterView(int v) {
  subView = v;
  pageSel = 0;
  scrollPos = 0.0f;
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
// QR sub views: 0 menu (Fake QR Code / Custom QR), 1 fake QR list, 2 custom QR (phone web page)
// A QR code is drawn black on white (always, so phone cameras can read it) by renderQR().

QRCode qrCode;
uint8_t qrBuf[700];                 // enough for version 12 (65x65 modules)
bool qrValid = false;
String qrCachedText = "\x01";       // text the current qrCode was made from

// Smallest version (1..12) that fits the text, low error correction = most data
bool qrMake(const String &t) {
  qrValid = false;
  if (t.length() == 0) return false;
  for (uint8_t v = 1; v <= 12; v++) {
    if (qrcode_getBufferSize(v) > sizeof(qrBuf)) break;
    if (qrcode_initText(&qrCode, qrBuf, v, ECC_LOW, t.c_str()) == 0) {
      qrValid = true;
      return true;
    }
  }
  return false;
}

// ---- Fake QR codes ----
const int FAKE_COUNT = 8;
const char *FAKE_NAMES[FAKE_COUNT] = {
  "Rickroll", "Free Wi-Fi", "Hello World", "Google",
  "GitHub", "Fake Contact", "Secret Msg", "My Wi-Fi"
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
    case 0: return "https://youtu.be/dQw4w9WgXcQ";
    case 1: return "WIFI:T:nopass;S:Free Wi-Fi;;";
    case 2: return "Hello World from WATCHOUT!";
    case 3: return "https://www.google.com";
    case 4: return "https://github.com";
    case 5: return "BEGIN:VCARD\nVERSION:3.0\nFN:John Doe\nTEL:+10000000000\nEMAIL:john@example.com\nEND:VCARD";
    case 6: return "You found the secret message!";
    default: {                                     // join the Wi-Fi saved by Connect AP
      if (savedSsid.length() == 0) return "WIFI:T:WPA;S:Home Wi-Fi;P:password123;;";   // placeholder
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
<textarea id=t maxlength=300 placeholder="Type text or a link"></textarea>
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
  .then(function(r){st(r.ok?'Sent to watch!':'Error '+r.status)})
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
  if (t.length() > QR_MAX_TEXT) t = t.substring(0, QR_MAX_TEXT);
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
  renderUI();

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
  addLine("Fake QR Code", "", C_WHITE, C_WHITE);
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
    int scale = 240 / (n + 4);                      // leaves a 2 module quiet zone
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

  if (canvasReady) lcd.drawRGBBitmap(0, 0, canvas->getBuffer(), 240, 280);
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
      if (subView == 2) {
        qrNetStart(false);
        pageSel = 0;
        scrollPos = 0.0f;
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
    } else if (act == 2) {
      qrCustomText = "";
      qrSaveText();
      pageSel = 0;
      scrollPos = 0.0f;
    }
  }
  buildQR();
}

// ======================= IR =======================

// Only decode while inside IR Capture (subView 1)
void pollIR() {
  if (page != IR_PAGE || subView != 1 || !IrReceiver.decode()) return;
  auto &d = IrReceiver.decodedIRData;
  bool repeat = (d.flags & IRDATA_FLAGS_IS_REPEAT);
  bool overflow = (d.flags & IRDATA_FLAGS_WAS_OVERFLOW);

  if (!repeat && !overflow && IrReceiver.irparams.rawlen > 3) {
    storedIRData = d;
    storedIRData.flags = 0;
    irRawMode = (d.protocol == UNKNOWN || d.protocol == PULSE_WIDTH || d.protocol == PULSE_DISTANCE);
    if (irRawMode) {
      rawIRLength = IrReceiver.irparams.rawlen - 1;
      if (rawIRLength > RAW_BUFFER_LENGTH) rawIRLength = RAW_BUFFER_LENGTH;
      IrReceiver.compensateAndStoreIRResultInArray(rawIRCode);
    }
    irCaptured = true;
    irCaptures++;
    ledFlash(191, 90, 242, 300);
    irProtocol = String(getProtocolString(d.protocol));
    irAddress = d.address;
    irCommand = d.command;
    irMessage = "Captured";
    irSendMessage = "Ready";
    lastDraw = 0;
  }
  IrReceiver.resume();
}

// Send the last captured code (IR Sender)
void replayIR() {
  if (!irCaptured) {
    irSendMessage = "Capture first";
    lastDraw = 0;
    return;
  }
  // receiver is stopped while in the Sender view, so just transmit
  if (irRawMode) IrSender.sendRaw(rawIRCode, rawIRLength, 38);
  else IrSender.write(&storedIRData);
  irSent++;
  irSendMessage = "Code sent";
  ledFlash(255, 255, 255, 200);
  lastDraw = 0;
}

// Send the code built in Custom IR
void sendCustomIR() {
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

void buildIRCapture() {
  addLine("Status", irCaptured ? "CAPTURED" : "LISTENING", C_MUTED, irCaptured ? C_GREEN : C_YELLOW);
  addKV("Protocol", irProtocol, C_WHITE);
  String a = String(irAddress, HEX); a.toUpperCase();
  String c = String(irCommand, HEX); c.toUpperCase();
  addLine("Address", "0x" + a, C_MUTED, C_WHITE);
  addLine("Command", "0x" + c, C_MUTED, C_WHITE);
  addLine("Captures", String(irCaptures), C_MUTED, C_WHITE);
  addWrapped(irMessage, C_CYAN);
}

void buildIRSender() {
  addLine("Saved", irCaptured ? "YES" : "EMPTY", C_MUTED, irCaptured ? C_GREEN : C_RED);
  if (irCaptured) {
    addKV("Protocol", irProtocol, C_WHITE);
    String a = String(irAddress, HEX); a.toUpperCase();
    String c = String(irCommand, HEX); c.toUpperCase();
    addLine("Address", "0x" + a, C_MUTED, C_WHITE);
    addLine("Command", "0x" + c, C_MUTED, C_WHITE);
    addLine("Mode", irRawMode ? "RAW" : "DECODED", C_MUTED, C_WHITE);
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
  addLine("BLE", bleScan ? "READY" : "IDLE", C_MUTED, bleScan ? C_GREEN : C_WHITE);
  addLine("GPS", uart ? "UART OK" : "NO DATA", C_MUTED, uart ? C_GREEN : C_RED);
  addLine("GPS Fix", fix ? "VALID" : "NO FIX", C_MUTED, fix ? C_GREEN : C_YELLOW);
  addLine("Sats", String(gps.satellites.isValid() ? gps.satellites.value() : 0), C_MUTED, C_WHITE);
  addLine("LoRa", loraTried ? (loraOK ? "READY" : "FAILED") : "IDLE", C_MUTED, loraTried ? (loraOK ? C_GREEN : C_RED) : C_WHITE);
  addLine("SD Card", sdTried ? (sdOK ? "OK" : "FAILED") : "IDLE", C_MUTED, sdTried ? (sdOK ? C_GREEN : C_RED) : C_WHITE);
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
  addKV("SD", "SPI, CS " + String(SD_CS), C_WHITE);
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
  renderUI();

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
  renderUI();

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
    case SD_PAGE:
      if (sdTestOK) { r = 0; g = 255; b = 0; style = 0; }
      else { r = 255; g = 0; b = 0; style = 1; period = 600; }
      break;
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
      startLines();
      addLine("Scanning...", "", C_YELLOW, C_YELLOW);
      finishLines();
      renderUI();
      startBLEScan();
      buildBleList();
      break;
    case GPS_PAGE: buildGPS(); break;
    case LORA_PAGE: startLoRa(); buildLoRa(); break;
    case SD_PAGE: runSDTest(); buildSD(); break;
    case IR_PAGE: irStop(); buildIR(); break;   // starts on the IR submenu, receiver off
    case CLOCK_PAGE: buildClock(); break;
    case DATE_PAGE: buildDate(); break;
    case WEATHER_PAGE: wxState = 0; fetchWeather(); buildWeather(); break;
    case QR_PAGE: buildQR(); break;             // starts on the QR sub menu (Fake / Custom)
    case CONFIG_PAGE: buildConfig(); break;
    case FACE_PAGE: startLines(); lastFaceKey = 0xFFFFFFFF; break;
  }
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
  prefs.end();
}

void markActivity() {
  lastActivity = millis();
  if (dimmed) {
    dimmed = false;
    analogWrite(LCD_BL, screenBrightness);
  }
}

void dimUpdate() {
  if (dimIdx == 0 || dimmed) return;
  if (millis() - lastActivity > (unsigned long)DIM_SECS[dimIdx] * 1000UL) {
    dimmed = true;
    analogWrite(LCD_BL, DIM_LEVEL);
  }
}

void gpsClick() {
  if (subView == 0) {
    subParentSel = pageSel;
    subView = (pageSel == 0) ? 1 : 2;
    pageSel = 0;
    scrollPos = 0.0f;
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
      if (subView == 1) {
        irMessage = irCaptured ? "Captured" : "Listening...";
        irStart();                 // receiver only runs inside IR Capture
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
    irMessage = "Listening...";
    IrReceiver.resume();
  } else if (subView == 2) {
    replayIR();
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

void configClick() {
  if (subView == 0) {
    if (pageSel < 4) {
      subParentSel = pageSel;
      subView = pageSel + 1;
      pageSel = 0;
      scrollPos = 0.0f;
    }
  } else if (subView == 1) {
    switch (pageSel) {
      case 0:
        configSelection = (configSelection + 1) % BRIGHT_COUNT;
        screenBrightness = brightLevels[configSelection];
        analogWrite(LCD_BL, screenBrightness);
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

    case BLE_PAGE:
      if (!bleDetailView) {
        if (pageSel < bleCount) {
          bleSelectedIndex = pageSel;
          bleDetailView = true;
          pageSel = 0;
          scrollPos = 0.0f;
          buildBleDetail();
        } else {
          startLines();
          addLine("Scanning...", "", C_YELLOW, C_YELLOW);
          finishLines();
          pageSel = 0;
          scrollPos = 0.0f;
          renderUI();
          startBLEScan();
          buildBleList();
        }
      }
      break;

    case GPS_PAGE: gpsClick(); break;
    case LORA_PAGE: loraClick(); break;
    case SD_PAGE: runSDTest(); buildSD(); break;
    case IR_PAGE: irClick(); break;
    case CLOCK_PAGE:
    case DATE_PAGE: break;
    case WEATHER_PAGE: fetchWeather(); buildWeather(); break;
    case QR_PAGE: qrClick(); break;
    case CONFIG_PAGE: configClick(); break;
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
  if (page == BLE_PAGE) bleDetailView = false;
  if (page == IR_PAGE) irStop();
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
    } else if ((page == GPS_PAGE || page == CONFIG_PAGE || page == IR_PAGE) && subView != 0) {
      subView = 0;
      pageSel = subParentSel;
      scrollPos = (float)pageSel;
      if (page == GPS_PAGE) buildGPS();
      else if (page == IR_PAGE) { irStop(); buildIR(); }
      else buildConfig();
    } else if (page == BLE_PAGE && bleDetailView) {
      bleDetailView = false;
      pageSel = bleSelectedIndex;
      scrollPos = (float)pageSel;
      buildBleList();
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
    if (now - btnDownAt >= 700 && (loraLongClick() || wifiLongClick())) {
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
  analogWrite(LCD_BL, screenBrightness);

  pinMode(LCD_CS, OUTPUT);
  digitalWrite(LCD_CS, HIGH);
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

  joyAdcInit();
  if (!joyAdcOK) analogReadResolution(12);

  Serial.printf("Joystick ADC: %s\n", joyAdcOK ? "IDF driver (Wi-Fi safe)" : "analogRead fallback");

  SPI.begin(LCD_SCLK, LCD_MISO, LCD_MOSI, LCD_CS);
  lcd.init(240, 280);
  lcd.setRotation(oriIdx == 0 ? 2 : 0);
  lcd.setSPISpeed(40000000);
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
  enterPage(MENU);
}

void loop() {
  while (GPSSerial.available()) gps.encode((char)GPSSerial.read());

  syncTimeFromGPS();
  pollJoystick();
  animateUI();
  ledUpdate();
  pollWeather();
  pollWiFi();
  pollAP();
  pollRadar();
  pollLoRa();
  pollIR();
  trackerPoll();
  qrPoll();
  dimUpdate();

  if (page != MENU && millis() - lastDraw >= (page == CLOCK_PAGE ? (uint32_t)250 : PAGE_REFRESH_MS)) {
    lastDraw = millis();
    switch (page) {
      case WIFI_PAGE: buildWifi(); break;
      case GPS_PAGE: buildGPS(); break;
      case LORA_PAGE: buildLoRa(); break;
      case SD_PAGE: buildSD(); break;
      case IR_PAGE: buildIR(); break;
      case CLOCK_PAGE: buildClock(); break;
      case DATE_PAGE: buildDate(); break;
      case WEATHER_PAGE: buildWeather(); break;
      case QR_PAGE: buildQR(); break;
      case CONFIG_PAGE: buildConfig(); break;
      case FACE_PAGE: faceUpdate(); break;
      default: break;
    }
  }
}


/*git add -A
git commit -m "Update project files and changes"
git push*/