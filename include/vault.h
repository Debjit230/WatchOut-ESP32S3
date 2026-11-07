#pragma once
// ============================================================================
//  vault.h - WATCHOUT encrypted password vault
//
//  * Entries (name / username / password) are stored on internal flash:
//      /watchout.vlt   current vault
//      /watchout.bak   previous version (kept on every save)
//    The file is AES-256-GCM encrypted. The key comes from your PIN through
//    PBKDF2-HMAC-SHA256 (VAULT_KDF_ITER rounds, random salt stored in the file header).
//    A wrong PIN = GCM authentication failure, nothing is ever stored in plain text.
//  * The key only lives in RAM while the vault is unlocked. Leaving the page, "Lock",
//    long press in the list, or the configured idle timeout wipes it and
//    drops Bluetooth.
//  * "Type ..." sends the text over BLE HID as a keyboard (US layout, report ID 2).
//  * Wrong-PIN lockout counter is kept in flash (NVS key "vfail").
//
//  This file is included from the main sketch AFTER everything it uses
//  (page enum, line builder, BLE HID block, on-screen keyboard) is defined.
// ============================================================================

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <esp_random.h>
#include "esp_partition.h"
#include "mbedtls/md.h"
#include "mbedtls/gcm.h"

// ---------------- settings ----------------
#define VAULT_KEY_MS        20         // time for each key press / release; raise it if characters get lost
#define VAULT_TYPE_DELAY_MS 500        // pause between clicking "Type" and the first character
#define VAULT_KDF_ITER      20000      // PBKDF2 rounds (stored in the file, so you may change it later)
#define VAULT_PIN_MIN       8          // minimum PIN / passphrase length
#define VAULT_PIN_MAX       63
#define VAULT_MAX_ENTRIES   40         // list plus actions and memory status fits within MAX_LINES
#define VAULT_NAME_MAX      24
#define VAULT_USER_MAX      63
#define VAULT_PASS_MAX      63
#define VAULT_GEN_LEN       16         // length of generated passwords
#define VAULT_FILE "/watchout.vlt"
#define VAULT_BAK  "/watchout.bak"
#define VAULT_TMP  "/watchout.new"

// file layout: "WVLT"(4) ver(1) rsv(3) iter(4) salt(16) iv(12) tag(16) clen(4) ciphertext(clen)
// AES-GCM additional data = the first 28 bytes (magic .. salt)
#define VLT_HDR      60
#define VLT_AAD_LEN  28
#define VLT_FILE_MAX 40000

// ---------------- data ----------------
struct VaultEntry {
  String name;
  String user;
  String pass;
};
static VaultEntry vltEntries[VAULT_MAX_ENTRIES];
static int vltCount = 0;

enum VltState : uint8_t {
  VLT_NOFS = 0,     // internal flash filesystem unavailable
  VLT_NEWPIN1,      // keyboard: create PIN (also "change PIN")
  VLT_NEWPIN2,      // keyboard: repeat PIN
  VLT_LOCKED,       // vault exists, locked
  VLT_UNLOCK,       // keyboard: enter PIN
  VLT_LIST,         // unlocked: entry list
  VLT_DETAIL,       // unlocked: one entry
  VLT_ADD_NAME,     // keyboard: add entry, name
  VLT_ADD_USER,     // keyboard: add entry, username
  VLT_ADD_PASS,     // keyboard: add entry, password
  VLT_GEN_NAME,     // keyboard: generate, name
  VLT_GEN_USER,     // keyboard: generate, username
  VLT_WEB,          // temporary hotspot and web entry form
  VLT_BUSY          // key derivation running
};
static VltState vltState = VLT_LOCKED;

// row actions (what a click on a line does)
#define VA_NONE    0
#define VA_BT      1
#define VA_ADD     2
#define VA_GEN     3
#define VA_CHGPIN  4
#define VA_LOCK    5
#define VA_UNLOCK  6
#define VA_RETRY   7
#define VA_T_LOGIN 8
#define VA_T_PASS  9
#define VA_T_USER  10
#define VA_SHOW    11
#define VA_DEL     12
#define VA_UPLOAD  13
#define VA_WEBSTOP 14
#define VA_TIMEOUT 15
#define VA_ENTRY   0x100          // VA_ENTRY + entry index
static int vltRowAct[MAX_LINES];

static const uint32_t vltIdleOptions[] = {
  30000UL, 60000UL, 120000UL, 300000UL, 600000UL, 900000UL
};
static const char *const vltIdleLabels[] = {
  "30 sec", "1 min", "2 min", "5 min", "10 min", "15 min"
};
static uint8_t vltIdleIndex = 2;
static bool vltUnlocked = false;
static bool vltFsOk = false;
static bool vltChgPin = false;
static bool vltDelPending = false;
static bool vltShowPass = false;
static bool vltBootInit = false;
static bool vltWebUp = false;
static bool vltWebStarting = false;
static bool vltWebOwnsWifi = false;
static bool vltWebMdns = false;
static bool vltWebWifiWasConnected = false;
static unsigned long vltWebStartAt = 0;
static String vltWebToken = "";
static String vltWebStatus = "";
static WebServer vltWebServer(80);
static bool vltWebHandlers = false;
static VaultEntry vltImportBackup[VAULT_MAX_ENTRIES];

static uint8_t  vltKey[32];
static uint8_t  vltSalt[16];
static uint32_t vltIter = VAULT_KDF_ITER;

static String vltMsg = "";
static unsigned long vltMsgAt = 0;
static uint8_t vltFails = 0;
static unsigned long vltLockUntil = 0;
static String vltTmpName = "";
static String vltTmpUser = "";
static String vltPin1 = "";
static int vltSel = 0;                       // entry shown in the detail view

static bool vltSaveIdleSetting() {
  Preferences settings;
  if (!settings.begin("watchout", false)) return false;
  size_t written = settings.putUChar("vltIdle", vltIdleIndex);
  settings.end();
  return written > 0;
}

static void vltLoadIdleSetting() {
  Preferences settings;
  settings.begin("watchout", true);
  uint8_t stored = settings.getUChar("vltIdle", 2);
  settings.end();
  vltIdleIndex = stored < (sizeof(vltIdleOptions) / sizeof(vltIdleOptions[0])) ? stored : 2;
}

// ---- typing machine ----
static char vltTypeBuf[200];
static int  vltTypeLen = 0;
static int  vltTypeIdx = 0;
static uint8_t vltTypePhase = 0;             // 0 = press next key, 1 = release
static uint8_t vltTypeFails = 0;
static unsigned long vltTypeAt = 0;
static bool vltTyping = false;

// ---------------- small helpers ----------------
static void vltWipe(void *p, size_t n) {
  volatile uint8_t *q = (volatile uint8_t *)p;
  while (n--) *q++ = 0;
}

static void vltWipeStr(String &s) {
  if (s.length()) vltWipe((void *)s.c_str(), s.length());
  s = "";
}

static void vltWipeEntry(VaultEntry &e) {
  vltWipeStr(e.name);
  vltWipeStr(e.user);
  vltWipeStr(e.pass);
}

static void vltPut32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint32_t vltGet32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// case-insensitive compare for the sorted list
static int vltCmp(const String &a, const String &b) {
  int la = (int)a.length(), lb = (int)b.length();
  int n = la < lb ? la : lb;
  for (int i = 0; i < n; i++) {
    char x = tolower(a[i]);
    char y = tolower(b[i]);
    if (x != y) return x < y ? -1 : 1;
  }
  return la < lb ? -1 : (la > lb ? 1 : 0);
}

static void vltSay(const String &m) {
  vltMsg = m;
  vltMsgAt = millis();
  lastDraw = 0;                              // rebuild on the next loop pass
}

static bool vltMsgOn() {
  return vltMsg.length() > 0 && millis() - vltMsgAt < 4000;
}

static uint16_t vltMsgColor() {
  if (vltMsg.startsWith("FAILED") || vltMsg.startsWith("Wrong")) return C_RED;
  if (vltMsg.startsWith("Typing")) return C_YELLOW;
  return C_GREEN;
}

static void vltSetState(VltState s) {
  vltState = s;
  subView = (int)s;                          // lets animateUI() restart the intro on every view change
}

static void vltSelect(int row) {
  if (row < 0) row = 0;
  pageSel = row;
  scrollPos = (float)row;
  scrollVel = 0.0f;
  uiDirty = true;
}

static void vltSelectAct(int act) {
  for (int i = 0; i < lineCount && i < MAX_LINES; i++) {
    if (vltRowAct[i] == act) { vltSelect(i); return; }
  }
}

// ---------------- internal flash filesystem ----------------
// Initialize a new partition only; never erase a non-empty filesystem that fails to mount.
static bool vltFormatIfBlank() {
  const esp_partition_t *part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "spiffs");
  if (!part) return false;

  uint8_t block[256];
  for (size_t offset = 0; offset < part->size; offset += sizeof(block)) {
    size_t n = part->size - offset;
    if (n > sizeof(block)) n = sizeof(block);
    if (esp_partition_read(part, offset, block, n) != ESP_OK) return false;
    for (size_t i = 0; i < n; i++) {
      if (block[i] != 0xFF) return false;
    }
    if ((offset & 0xFFFF) == 0xFF00) yield();
  }
  return LittleFS.format();
}

static bool vltMount() {
  if (vltFsOk) return true;
  vltFsOk = LittleFS.begin(false);
  if (!vltFsOk && vltFormatIfBlank()) vltFsOk = LittleFS.begin(false);
  return vltFsOk;
}

static void vltUnmount() {
  if (vltFsOk) {
    LittleFS.end();
    vltFsOk = false;
  }
}

// ---------------- crypto ----------------
// PBKDF2-HMAC-SHA256, one 32 byte output block
static bool vltKdf(const char *pw, size_t plen, const uint8_t *salt, uint32_t iter, uint8_t *out) {
  const mbedtls_md_info_t *mi = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (!mi) return false;
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  if (mbedtls_md_setup(&ctx, mi, 1) != 0) {
    mbedtls_md_free(&ctx);
    return false;
  }
  uint8_t u[32], t[32], blk[20];
  bool ok = (mbedtls_md_hmac_starts(&ctx, (const uint8_t *)pw, plen) == 0);
  if (ok) {
    memcpy(blk, salt, 16);
    blk[16] = 0; blk[17] = 0; blk[18] = 0; blk[19] = 1;          // INT(1)
    ok = (mbedtls_md_hmac_update(&ctx, blk, 20) == 0) && (mbedtls_md_hmac_finish(&ctx, u) == 0);
  }
  if (ok) {
    memcpy(t, u, 32);
    for (uint32_t i = 1; i < iter && ok; i++) {
      ok = (mbedtls_md_hmac_reset(&ctx) == 0) &&
           (mbedtls_md_hmac_update(&ctx, u, 32) == 0) &&
           (mbedtls_md_hmac_finish(&ctx, u) == 0);
      for (int k = 0; k < 32; k++) t[k] ^= u[k];
      if ((i & 0x7FF) == 0) yield();
    }
  }
  if (ok) memcpy(out, t, 32);
  vltWipe(u, sizeof(u));
  vltWipe(t, sizeof(t));
  vltWipe(blk, sizeof(blk));
  mbedtls_md_free(&ctx);
  return ok;
}

// ---------------- encrypted vault file ----------------
static uint8_t *vltSerialize(size_t &len) {
  size_t n = 2;
  for (int i = 0; i < vltCount; i++) {
    n += 3 + vltEntries[i].name.length() + vltEntries[i].user.length() + vltEntries[i].pass.length();
  }
  uint8_t *b = (uint8_t *)malloc(n);
  if (!b) return nullptr;
  b[0] = (uint8_t)(vltCount & 0xFF);
  b[1] = (uint8_t)(vltCount >> 8);
  size_t pos = 2;
  for (int i = 0; i < vltCount; i++) {
    const String *f[3] = {&vltEntries[i].name, &vltEntries[i].user, &vltEntries[i].pass};
    for (int k = 0; k < 3; k++) {
      size_t l = f[k]->length();
      if (l > 255) l = 255;
      b[pos++] = (uint8_t)l;
      memcpy(b + pos, f[k]->c_str(), l);
      pos += l;
    }
  }
  len = pos;
  return b;
}

static bool vltParse(const uint8_t *p, size_t len) {
  for (int i = 0; i < VAULT_MAX_ENTRIES; i++) vltWipeEntry(vltEntries[i]);
  vltCount = 0;
  if (len < 2) return false;
  int n = p[0] | (p[1] << 8);
  if (n > VAULT_MAX_ENTRIES) return false;
  size_t pos = 2;
  bool ok = true;
  for (int i = 0; i < n && ok; i++) {
    String *f[3] = {&vltEntries[i].name, &vltEntries[i].user, &vltEntries[i].pass};
    for (int k = 0; k < 3 && ok; k++) {
      if (pos >= len) { ok = false; break; }
      size_t l = p[pos++];
      if (pos + l > len) { ok = false; break; }
      char tmp[256];
      memcpy(tmp, p + pos, l);
      tmp[l] = 0;
      *f[k] = String(tmp);
      vltWipe(tmp, sizeof(tmp));
      pos += l;
    }
  }
  if (!ok) {
    for (int i = 0; i < VAULT_MAX_ENTRIES; i++) vltWipeEntry(vltEntries[i]);
    return false;
  }
  vltCount = n;
  return true;
}

static uint8_t *vltReadFile(const char *path, size_t &len) {
  len = 0;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return nullptr;
  size_t sz = f.size();
  if (f.isDirectory() || sz < VLT_HDR || sz > VLT_FILE_MAX) { f.close(); return nullptr; }
  uint8_t *b = (uint8_t *)malloc(sz);
  if (!b) { f.close(); return nullptr; }
  size_t n = f.read(b, sz);
  f.close();
  if (n != sz) { free(b); return nullptr; }
  len = sz;
  return b;
}

// 0 = unlocked, 1 = wrong PIN, 2 = no readable vault file, 3 = file damaged
static int vltLoad(const String &pin) {
  const char *paths[2] = {VAULT_FILE, VAULT_BAK};
  for (int p = 0; p < 2; p++) {
    if (!LittleFS.exists(paths[p])) continue;
    size_t len = 0;
    uint8_t *b = vltReadFile(paths[p], len);
    if (!b) continue;
    uint32_t iter = vltGet32(b + 8);
    uint32_t clen = vltGet32(b + 56);
    bool valid = memcmp(b, "WVLT", 4) == 0 && b[4] == 1 &&
                 iter >= 1000 && iter <= 2000000UL &&
                 clen >= 2 && (size_t)clen + VLT_HDR == len;
    if (!valid) { free(b); continue; }                 // damaged / foreign file: try the backup

    uint8_t key[32];
    if (!vltKdf(pin.c_str(), pin.length(), b + 12, iter, key)) { free(b); return 2; }
    uint8_t *pt = (uint8_t *)malloc(clen);
    if (!pt) { vltWipe(key, sizeof(key)); free(b); return 2; }

    mbedtls_gcm_context g;
    mbedtls_gcm_init(&g);
    int r = mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, key, 256);
    if (r == 0) {
      r = mbedtls_gcm_auth_decrypt(&g, clen, b + 28, 12, b, VLT_AAD_LEN, b + 40, 16, b + VLT_HDR, pt);
    }
    mbedtls_gcm_free(&g);
    if (r != 0) {                                      // authentication failed = wrong PIN
      vltWipe(key, sizeof(key));
      vltWipe(pt, clen);
      free(pt);
      free(b);
      return 1;
    }
    bool ok = vltParse(pt, clen);
    vltWipe(pt, clen);
    free(pt);
    if (!ok) { vltWipe(key, sizeof(key)); free(b); return 3; }
    memcpy(vltKey, key, 32);
    memcpy(vltSalt, b + 12, 16);
    vltIter = iter;
    vltWipe(key, sizeof(key));
    free(b);
    return 0;
  }
  return 2;
}

// Encrypts the entries in RAM and replaces the vault file (previous version becomes the .bak)
static bool vltSave() {
  if (!vltMount()) return false;
  size_t plen = 0;
  uint8_t *pt = vltSerialize(plen);
  if (!pt) return false;
  size_t total = VLT_HDR + plen;
  uint8_t *out = (uint8_t *)malloc(total);
  if (!out) { vltWipe(pt, plen); free(pt); return false; }

  memcpy(out, "WVLT", 4);
  out[4] = 1; out[5] = 0; out[6] = 0; out[7] = 0;
  vltPut32(out + 8, vltIter);
  memcpy(out + 12, vltSalt, 16);
  esp_fill_random(out + 28, 12);                       // fresh IV for every save
  vltPut32(out + 56, (uint32_t)plen);

  mbedtls_gcm_context g;
  mbedtls_gcm_init(&g);
  int r = mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, vltKey, 256);
  if (r == 0) {
    r = mbedtls_gcm_crypt_and_tag(&g, MBEDTLS_GCM_ENCRYPT, plen, out + 28, 12, out, VLT_AAD_LEN,
                                  pt, out + VLT_HDR, 16, out + 40);
  }
  mbedtls_gcm_free(&g);
  vltWipe(pt, plen);
  free(pt);
  if (r != 0) { free(out); return false; }

  bool ok = false;
  LittleFS.remove(VAULT_TMP);
  File f = LittleFS.open(VAULT_TMP, FILE_WRITE);
  if (f) {
    size_t w = f.write(out, total);
    f.flush();
    f.close();
    ok = (w == total);
  }
  free(out);
  if (ok) {                                            // verify what really landed in flash
    File v = LittleFS.open(VAULT_TMP, FILE_READ);
    ok = v && ((size_t)v.size() == total);
    if (v) v.close();
  }
  if (!ok) { LittleFS.remove(VAULT_TMP); return false; }

  if (LittleFS.exists(VAULT_BAK)) LittleFS.remove(VAULT_BAK);
  if (LittleFS.exists(VAULT_FILE)) {
    if (!LittleFS.rename(VAULT_FILE, VAULT_BAK)) { LittleFS.remove(VAULT_TMP); return false; }
  }
  if (!LittleFS.rename(VAULT_TMP, VAULT_FILE)) return false; // load() falls back to the .bak in this rare case
  return true;
}

static void vltWebStop();

static void vltAddMemoryStatus() {
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  size_t free = total > used ? total - used : 0;
  addLine("Flash", String((unsigned long)(used / 1024)) + "/" +
          String((unsigned long)(total / 1024)) + "K", C_MUTED, C_CYAN);
  addLine("Free/RAM", String((unsigned long)(free / 1024)) + "K/" +
          String((unsigned long)(ESP.getFreeHeap() / 1024)) + "K", C_MUTED, C_GREEN);
}

static bool vltWebAuthorized() {
  String supplied = vltWebServer.header("X-Vault-Token");
  size_t expectedLen = vltWebToken.length();
  size_t suppliedLen = supplied.length();
  size_t diff = expectedLen ^ suppliedLen;
  for (size_t i = 0; i < expectedLen; i++) {
    char c = i < suppliedLen ? supplied[i] : 0;
    diff |= (size_t)(uint8_t)(vltWebToken[i] ^ c);
  }
  vltWipeStr(supplied);
  return vltWebUp && expectedLen > 0 && diff == 0;
}

static int vltWebAddEntry() {
  if (!vltWebAuthorized()) {
    vltWebServer.send(401, "text/plain", "Invalid or expired token");
    return 401;
  }
  if (vltWebServer.args() != 3) {
    vltWebStatus = "Expected name, user and pass fields";
    vltWebServer.send(400, "text/plain", vltWebStatus);
    return 400;
  }

  String name = vltWebServer.arg("name");
  String user = vltWebServer.arg("user");
  String pass = vltWebServer.arg("pass");
  name.trim();
  user.trim();
  if (name.length() == 0 || name.length() > VAULT_NAME_MAX ||
      user.length() > VAULT_USER_MAX || pass.length() == 0 ||
      pass.length() > VAULT_PASS_MAX) {
    vltWipeStr(name);
    vltWipeStr(user);
    vltWipeStr(pass);
    vltWebStatus = "Check required fields and lengths";
    vltWebServer.send(400, "text/plain", vltWebStatus);
    return 400;
  }

  int oldCount = vltCount;
  int match = 0;
  while (match < oldCount && vltCmp(vltEntries[match].name, name) != 0) match++;
  bool replacing = match < oldCount;
  if (!replacing && oldCount >= VAULT_MAX_ENTRIES) {
    vltWipeStr(name);
    vltWipeStr(user);
    vltWipeStr(pass);
    vltWebStatus = "Vault limit is 40 entries";
    vltWebServer.send(413, "text/plain", vltWebStatus);
    return 413;
  }

  for (int i = 0; i < oldCount; i++) vltImportBackup[i] = vltEntries[i];
  if (replacing) {
    vltWipeEntry(vltEntries[match]);
  } else {
    match = vltCount++;
  }
  vltEntries[match].name = name;
  vltEntries[match].user = user;
  vltEntries[match].pass = pass;
  vltWipeStr(name);
  vltWipeStr(user);
  vltWipeStr(pass);

  for (int i = 1; i < vltCount; i++) {
    VaultEntry entry = vltEntries[i];
    int j = i;
    while (j > 0 && vltCmp(vltEntries[j - 1].name, entry.name) > 0) {
      vltEntries[j] = vltEntries[j - 1];
      j--;
    }
    vltEntries[j] = entry;
    vltWipeEntry(entry);
  }

  if (!vltSave()) {
    for (int i = 0; i < VAULT_MAX_ENTRIES; i++) vltWipeEntry(vltEntries[i]);
    for (int i = 0; i < oldCount; i++) vltEntries[i] = vltImportBackup[i];
    vltCount = oldCount;
    vltWebStatus = "Flash write failed";
    vltWebServer.send(500, "text/plain", vltWebStatus);
    for (int i = 0; i < VAULT_MAX_ENTRIES; i++) vltWipeEntry(vltImportBackup[i]);
    return 500;
  }

  for (int i = 0; i < VAULT_MAX_ENTRIES; i++) vltWipeEntry(vltImportBackup[i]);
  vltWebStatus = String(replacing ? "Updated entry; " : "Added entry; ") +
                 String(vltCount) + " total";
  lastActivity = millis();
  uiDirty = true;
  lastDraw = 0;
  vltWebServer.send(200, "text/plain", vltWebStatus);
  return 200;
}

static const char VLT_WEB_HTML[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>WatchOut password vault</title>
<style>
body{font-family:sans-serif;max-width:560px;margin:32px auto;padding:0 16px;background:#111;color:#fff}
input,button{box-sizing:border-box;width:100%;margin:8px 0;padding:13px;font-size:16px;border:0;border-radius:8px}
button{background:#a64de0;color:#fff;font-weight:bold}p{color:#bbb;line-height:1.45}
#s{min-height:24px;color:#7df}
</style></head><body>
<h2>Password vault</h2>
<p>Enter the same details as <b>Add entry</b> on the device. A matching name updates that entry.</p>
<form id=f>
<label>Name<input name=name maxlength=24 required autocomplete=off></label>
<label>Username / email<input name=user maxlength=63 autocomplete=username></label>
<label>Password<input name=pass type=password maxlength=63 required autocomplete=new-password></label>
<label>Import token<input id=t type=password autocomplete=off required></label>
<button>Add entry</button></form><p id=s></p>
<script>
document.getElementById('f').addEventListener('submit',async function(e){
 e.preventDefault();var s=document.getElementById('s');s.textContent='Saving entry...';
 try{var r=await fetch('/add',{method:'POST',headers:{'X-Vault-Token':document.getElementById('t').value,
 'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(new FormData(this))});
 s.textContent=await r.text();if(r.ok){this.elements.name.value='';this.elements.user.value='';
 this.elements.pass.value='';this.elements.name.focus();}}
 catch(_){s.textContent='Could not save entry; check the connection.'}
});
</script></body></html>)HTML";

static void vltWebRoot() {
  if (!vltWebUp) { vltWebServer.send(503, "text/plain", "Import server is closed"); return; }
  vltWebServer.send_P(200, "text/html", VLT_WEB_HTML);
}

static void vltWebBeginServer() {
  vltWebServer.begin();
  vltWebUp = true;
  vltWebStarting = false;
  vltWebWifiWasConnected = true;
  vltWebStatus = "Ready to add entries";
  if (MDNS.begin("watchout")) {
    MDNS.addService("http", "tcp", 80);
    vltWebMdns = true;
  }
}

static bool vltWebStart() {
  if (trackerOn || trkServerUp || qrServerUp) {
    vltWebStatus = "Stop tracker or QR server first";
    return false;
  }
  if (vltWebUp || vltWebStarting) return true;
  if (WiFi.status() != WL_CONNECTED && savedSsid.length() == 0) {
    vltWebStatus = "No saved Wi-Fi; connect in Wi-Fi > Connect AP";
    return false;
  }
  if (wifiStarted) {
    WiFi.scanDelete();
    wifiStarted = false;
  }

  vltWebToken = "";
  uint8_t randomBytes[8];
  esp_fill_random(randomBytes, sizeof(randomBytes));
  for (int i = 0; i < sizeof(randomBytes); i++) {
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", randomBytes[i]);
    vltWebToken += hex;
  }
  vltWipe(randomBytes, sizeof(randomBytes));

  if (!vltWebHandlers) {
    static const char *headers[] = {"X-Vault-Token"};
    vltWebServer.collectHeaders(headers, 1);
    vltWebServer.on("/", HTTP_GET, vltWebRoot);
    vltWebServer.on("/add", HTTP_POST, vltWebAddEntry);
    vltWebHandlers = true;
  }
  if (WiFi.status() == WL_CONNECTED) {
    vltWebBeginServer();
  } else {
    if (!wifiBegin()) {
      vltWebStatus = "Could not start Wi-Fi";
      vltWipeStr(vltWebToken);
      return false;
    }
    vltWebStarting = true;
    vltWebOwnsWifi = true;
    vltWebStartAt = millis();
    vltWebStatus = "Connecting to saved Wi-Fi...";
  }
  return true;
}

static void vltWebStop() {
  if (vltWebUp) vltWebServer.stop();
  if (vltWebMdns) MDNS.end();
  vltWebUp = false;
  vltWebStarting = false;
  if (vltWebOwnsWifi && !apHold && !trackerOn) WiFi.disconnect(false, false);
  vltWebOwnsWifi = false;
  vltWebMdns = false;
  vltWebWifiWasConnected = false;
  vltWipeStr(vltWebToken);
}

// New vault, or new PIN for the entries already in RAM. Fresh salt every time.
static bool vltCreate(const String &pin) {
  uint8_t oldKey[32], oldSalt[16];
  uint32_t oldIter = vltIter;
  memcpy(oldKey, vltKey, 32);
  memcpy(oldSalt, vltSalt, 16);

  esp_fill_random(vltSalt, 16);
  vltIter = VAULT_KDF_ITER;
  bool ok = vltKdf(pin.c_str(), pin.length(), vltSalt, vltIter, vltKey) && vltSave();
  if (!ok) {                                           // keep the old key if anything failed
    memcpy(vltKey, oldKey, 32);
    memcpy(vltSalt, oldSalt, 16);
    vltIter = oldIter;
  }
  vltWipe(oldKey, sizeof(oldKey));
  vltWipe(oldSalt, sizeof(oldSalt));
  return ok;
}

// ---------------- wrong PIN counter (flash) ----------------
static void vltSaveFails() {
  Preferences tp;
  tp.begin("watchout", false);
  tp.putUChar("vfail", vltFails);
  tp.end();
}

static uint32_t vltLockSecs(uint8_t f) {
  if (f < 3) return 0;
  uint32_t s = 15UL << ((f - 3) > 8 ? 8 : (f - 3));    // 15 s, 30 s, 60 s ... capped at 1 hour
  return s > 3600UL ? 3600UL : s;
}

// ---------------- random password ----------------
static String vltGenPass() {
  static const char UP[]  = "ABCDEFGHJKLMNPQRSTUVWXYZ";
  static const char LOWER[] = "abcdefghijkmnpqrstuvwxyz";
  static const char DIG[] = "23456789";
  static const char SYM[] = "!@#$%&*-_+=?";
  char all[80];
  snprintf(all, sizeof(all), "%s%s%s%s", UP, LOWER, DIG, SYM);
  const int n = (int)strlen(all);
  const int limit = 256 - (256 % n);                   // rejection sampling: no modulo bias
  for (;;) {
    char buf[VAULT_GEN_LEN + 1];
    bool hu = false, hl = false, hd = false, hs = false;
    for (int i = 0; i < VAULT_GEN_LEN; i++) {
      uint8_t r;
      do { esp_fill_random(&r, 1); } while (r >= limit);
      char c = all[r % n];
      buf[i] = c;
      if (strchr(UP, c)) hu = true;
      else if (strchr(LOWER, c)) hl = true;
      else if (strchr(DIG, c)) hd = true;
      else hs = true;
    }
    buf[VAULT_GEN_LEN] = 0;
    if (hu && hl && hd && hs) {
      String s(buf);
      vltWipe(buf, sizeof(buf));
      return s;
    }
  }
}

// ---------------- BLE keyboard ----------------
// US layout. Returns false for characters that cannot be typed.
static bool vltHid(char c, uint8_t &mod, uint8_t &key) {
  mod = 0;
  if (c >= 'a' && c <= 'z') { key = 0x04 + (c - 'a'); return true; }
  if (c >= 'A' && c <= 'Z') { mod = 0x02; key = 0x04 + (c - 'A'); return true; }
  if (c >= '1' && c <= '9') { key = 0x1E + (c - '1'); return true; }
  if (c == '0') { key = 0x27; return true; }
  switch (c) {
    case ' ':  key = 0x2C; return true;
    case '\t': key = 0x2B; return true;
    case '\n': key = 0x28; return true;
    case '-':  key = 0x2D; return true;
    case '_':  mod = 2; key = 0x2D; return true;
    case '=':  key = 0x2E; return true;
    case '+':  mod = 2; key = 0x2E; return true;
    case '[':  key = 0x2F; return true;
    case '{':  mod = 2; key = 0x2F; return true;
    case ']':  key = 0x30; return true;
    case '}':  mod = 2; key = 0x30; return true;
    case '\\': key = 0x31; return true;
    case '|':  mod = 2; key = 0x31; return true;
    case ';':  key = 0x33; return true;
    case ':':  mod = 2; key = 0x33; return true;
    case '\'': key = 0x34; return true;
    case '"':  mod = 2; key = 0x34; return true;
    case '`':  key = 0x35; return true;
    case '~':  mod = 2; key = 0x35; return true;
    case ',':  key = 0x36; return true;
    case '<':  mod = 2; key = 0x36; return true;
    case '.':  key = 0x37; return true;
    case '>':  mod = 2; key = 0x37; return true;
    case '/':  key = 0x38; return true;
    case '?':  mod = 2; key = 0x38; return true;
    case '!':  mod = 2; key = 0x1E; return true;
    case '@':  mod = 2; key = 0x1F; return true;
    case '#':  mod = 2; key = 0x20; return true;
    case '$':  mod = 2; key = 0x21; return true;
    case '%':  mod = 2; key = 0x22; return true;
    case '^':  mod = 2; key = 0x23; return true;
    case '&':  mod = 2; key = 0x24; return true;
    case '*':  mod = 2; key = 0x25; return true;
    case '(':  mod = 2; key = 0x26; return true;
    case ')':  mod = 2; key = 0x27; return true;
  }
  return false;
}

// One 8 byte keyboard report: modifiers, reserved, 6 key slots (report ID 2 comes from the descriptor)
static bool vltSendKey(uint8_t mod, uint8_t key) {
  if (!bleKbReport || !bleHidConnected) return false;
  uint8_t r[8] = {mod, 0, key, 0, 0, 0, 0, 0};
  bleKbReport->setValue(r, 8);
  return bleKbReport->notify();
}

static void vltTypeWipe() {
  vltWipe(vltTypeBuf, sizeof(vltTypeBuf));
  vltTypeLen = 0;
  vltTypeIdx = 0;
  vltTypePhase = 0;
  vltTyping = false;
}

static void vltTypeEnd(const char *msg) {
  if (vltTyping) vltSendKey(0, 0);                     // make sure no key stays pressed
  vltTypeWipe();
  vltSay(msg);
}

static void vltTypeBegin(const String &seq) {
  size_t n = seq.length();
  if (n >= sizeof(vltTypeBuf)) n = sizeof(vltTypeBuf) - 1;
  memcpy(vltTypeBuf, seq.c_str(), n);
  vltTypeBuf[n] = 0;
  vltTypeLen = (int)n;
  vltTypeIdx = 0;
  vltTypePhase = 0;
  vltTypeFails = 0;
  vltTypeAt = millis() + VAULT_TYPE_DELAY_MS;
  vltTyping = true;
  vltSay("Typing...");
  ledFlash(255, 204, 0, 200);
}

// Non-blocking: one step (press or release) per VAULT_KEY_MS
static void vltTypeStep() {
  unsigned long now = millis();
  if ((long)(now - vltTypeAt) < 0) return;
  if (!bleKbReport || !bleHidConnected) { vltTypeEnd("FAILED: no link"); return; }

  if (vltTypePhase == 0) {
    if (vltTypeIdx >= vltTypeLen) { vltTypeEnd("Typed"); return; }
    uint8_t mod, key;
    if (!vltHid(vltTypeBuf[vltTypeIdx], mod, key)) { vltTypeIdx++; return; }   // skip untypeable char
    if (vltSendKey(mod, key)) {
      vltTypePhase = 1;
      vltTypeFails = 0;
      vltTypeAt = now + VAULT_KEY_MS;
    } else if (++vltTypeFails > 25) {
      vltTypeEnd("FAILED: send");
    } else {
      vltTypeAt = now + VAULT_KEY_MS;
    }
  } else {
    if (vltSendKey(0, 0)) {
      vltTypeIdx++;
      vltTypePhase = 0;
      vltTypeFails = 0;
      vltTypeAt = now + VAULT_KEY_MS;
    } else if (++vltTypeFails > 25) {
      vltTypeEnd("FAILED: send");
    } else {
      vltTypeAt = now + VAULT_KEY_MS;
    }
  }
}

// ---------------- wipe / lock ----------------
static void vltWipeAll() {
  vltTypeWipe();
  vltWipe(vltKey, sizeof(vltKey));
  vltWipe(vltSalt, sizeof(vltSalt));
  for (int i = 0; i < VAULT_MAX_ENTRIES; i++) vltWipeEntry(vltEntries[i]);
  vltCount = 0;
  vltWipeStr(vltTmpName);
  vltWipeStr(vltTmpUser);
  vltWipeStr(vltPin1);
  vltWipeStr(vaultInput);
  vaultPrompt = "";
  for (int i = 0; i < MAX_LINES; i++) {                // the line list may still hold entry names / a shown password
    vltWipeStr(lines[i].l);
    vltWipeStr(lines[i].r);
  }
  lineCount = 0;
}

static void vltLockNow(const char *why) {
  vltWebStop();
  vltWipeAll();
  bleStopTx();                                         // stop advertising and drop the connection
  bleHidMsg = "";
  vltUnlocked = false;
  vltChgPin = false;
  vltDelPending = false;
  vltShowPass = false;
  vltSetState(vltFsOk ? VLT_LOCKED : VLT_NOFS);
  vltMsg = "";
  if (why) vltSay(why);
  vltSelect(0);
  lastDraw = 0;
}

static void vltStartBle() {
  bleMediaStart();                                     // HID service + advertising as "WatchOut Remote"
}

// ---------------- screens ----------------
static void vltAct(int act, const char *label, uint16_t col) {
  if (lineCount >= MAX_LINES) return;
  vltRowAct[lineCount] = act;
  addLine(label, "", col, col);
}

static const char *vltBtWord() {
  return bleHidReady ? "LINK" : (bleHidConnected ? "PAIR" : (bleHidOn ? "ADV" : "OFF"));
}

static uint16_t vltBtColor() {
  return bleHidReady ? C_GREEN : (bleHidOn ? C_YELLOW : C_RED);
}

// First row of the list / detail view: Bluetooth status, replaced by a short message for a few seconds
// (the row count never changes, so the selection cannot jump)
static void vltBtRow() {
  if (lineCount >= MAX_LINES) return;
  vltRowAct[lineCount] = VA_BT;
  if (vltMsgOn()) {
    uint16_t c = vltMsgColor();
    addLine(vltMsg, "", c, c);
  } else {
    addLine("Bluetooth", vltBtWord(), C_WHITE, vltBtColor());
  }
}

void buildVault() {
  if (page != VAULT_PAGE) return;
  if (vaultKbActive()) { uiDirty = true; return; }     // the keyboard draws itself

  startLines();
  for (int i = 0; i < MAX_LINES; i++) vltRowAct[i] = VA_NONE;

  switch (vltState) {
    case VLT_NOFS:
      addLine("Internal storage", "ERROR", C_WHITE, C_RED);
      addWrapped("Flash filesystem unavailable", C_MUTED);
      if (vltMsg.length()) addWrapped(vltMsg, vltMsgColor());
      addLine("> Retry", "", C_CYAN, C_CYAN);
      for (int i = 0; i < lineCount; i++) vltRowAct[i] = VA_RETRY;
      break;

    case VLT_BUSY:
      addLine("Please wait", "", C_YELLOW, C_YELLOW);
      addWrapped(vltMsg, C_CYAN);
      break;

    case VLT_LOCKED: {
      addLine("Vault", "LOCKED", C_WHITE, C_RED);
      long rem = vltLockUntil ? (long)(vltLockUntil - millis()) : 0;
      if (rem > 0) addLine("Locked out", String((rem + 999) / 1000) + "s", C_MUTED, C_YELLOW);
      if (vltFails > 0) addLine("Wrong PINs", String((int)vltFails), C_MUTED, C_YELLOW);
      if (vltMsg.length()) addWrapped(vltMsg, vltMsgColor());
      addLine("> Unlock", "", C_GREEN, C_GREEN);
      for (int i = 0; i < lineCount; i++) vltRowAct[i] = VA_UNLOCK;   // any click starts unlocking
      break;
    }

    case VLT_LIST:
      vltBtRow();
      for (int i = 0; i < vltCount && lineCount < MAX_LINES; i++) {
        vltRowAct[lineCount] = VA_ENTRY + i;
        addLine(vltEntries[i].name, "", C_WHITE, C_WHITE);
      }
      vltAct(VA_ADD,    "+ Add entry",   C_CYAN);
      vltAct(VA_GEN,    "+ Generate",    C_CYAN);
      vltAct(VA_UPLOAD, "> Web import",  C_CYAN);
      vltAct(VA_CHGPIN, "> Change PIN",  C_YELLOW);
      vltRowAct[lineCount] = VA_TIMEOUT;
      addLine("Auto-lock", vltIdleLabels[vltIdleIndex], C_YELLOW, C_CYAN);
      vltAct(VA_LOCK,    "> Lock",        C_RED);
      break;

    case VLT_WEB:
      addLine("Web entry", vltWebUp ? "ON" : (vltWebStarting ? "WAIT" : "OFF"),
              C_WHITE, vltWebUp ? C_GREEN : (vltWebStarting ? C_YELLOW : C_RED));
      if (vltWebUp) {
        addKV("Open", WiFi.localIP().toString(), C_CYAN);
        if (vltWebMdns) addKV("Or", "watchout.local", C_CYAN);
        addKV("Token", vltWebToken, C_GREEN);
      }
      addWrapped(vltWebStatus, vltWebUp ? C_CYAN : (vltWebStarting ? C_YELLOW : C_RED));
      addWrapped("Join the same Wi-Fi, then open the device IP", C_MUTED);
      addWrapped("Add with Name, Username/email and Password", C_MUTED);
      vltAct(VA_WEBSTOP, "> Close import", C_RED);
      break;

    case VLT_DETAIL: {
      if (vltSel < 0 || vltSel >= vltCount) {
        addLine("Not available", "", C_YELLOW, C_YELLOW);
        break;
      }
      VaultEntry &e = vltEntries[vltSel];
      vltBtRow();
      addKV("Name", e.name, C_WHITE);
      addKV("User", e.user.length() ? e.user : String("(none)"), C_CYAN);
      addKV("Pass", vltShowPass ? e.pass : String("********"), vltShowPass ? C_YELLOW : C_MUTED);
      vltAct(VA_T_LOGIN, "> Type login",    C_GREEN);
      vltAct(VA_T_PASS,  "> Type password", C_GREEN);
      vltAct(VA_T_USER,  "> Type username", C_GREEN);
      vltAct(VA_SHOW,    vltShowPass ? "> Hide password" : "> Show password", C_CYAN);
      vltAct(VA_DEL,     vltDelPending ? "> CONFIRM DELETE" : "> Delete", C_RED);
      break;
    }

    default:
      break;
  }
  if (vltState != VLT_NOFS && vltState != VLT_BUSY) vltAddMemoryStatus();
  finishLines();
}

// ---------------- navigation helpers ----------------
static void vltToList(int row) {
  vltSetState(VLT_LIST);
  buildVault();
  vltSelect(row);
}

static void vltShowDetail(int idx, int selAct) {
  vltSel = idx;
  vltShowPass = false;
  vltDelPending = false;
  vltSetState(VLT_DETAIL);
  buildVault();
  vltSelectAct(selAct);
}

static void vltOpenKb(VltState s, const char *prompt, int maxLen, bool mask) {
  vltWipeStr(vaultInput);
  vaultPrompt = prompt;
  vaultMaxLen = maxLen;
  vaultMask = mask;
  vltSetState(s);
  kbOpen();                                            // resets mode / cursor and clears chatStatus
  uiDirty = true;
}

// Shows a "please wait" screen right now (used before the slow key derivation)
static void vltBusy(const char *m) {
  vltSetState(VLT_BUSY);
  vltMsg = m;
  vltMsgAt = millis();
  buildVault();
  uiDirty = false;
  renderUIStatic();
}

static void vltRemoveAt(int idx) {
  for (int i = idx; i < vltCount - 1; i++) vltEntries[i] = vltEntries[i + 1];
  vltWipeEntry(vltEntries[vltCount - 1]);
  vltCount--;
}

static void vltCommitEntry(const String &name, const String &user, const String &pass, int selAct) {
  if (vltCount >= VAULT_MAX_ENTRIES) {
    vltSay("Vault full");
    vltToList(1);
    return;
  }
  int pos = 0;
  while (pos < vltCount && vltCmp(vltEntries[pos].name, name) <= 0) pos++;
  for (int i = vltCount; i > pos; i--) vltEntries[i] = vltEntries[i - 1];
  vltEntries[pos].name = name;
  vltEntries[pos].user = user;
  vltEntries[pos].pass = pass;
  vltCount++;
  if (!vltSave()) {
    vltRemoveAt(pos);
    vltSay("FAILED: flash write");
    vltToList(1);
    return;
  }
  vltSay("Saved");
  vltShowDetail(pos, selAct);
}

static void vltDeleteEntry(int idx) {
  VaultEntry bak = vltEntries[idx];
  vltRemoveAt(idx);
  if (!vltSave()) {                                    // put it back
    for (int i = vltCount; i > idx; i--) vltEntries[i] = vltEntries[i - 1];
    vltEntries[idx] = bak;
    vltCount++;
    vltSay("FAILED: flash write");
  } else {
    vltSay("Deleted");
  }
  vltWipeEntry(bak);
  int row = 1 + idx;
  if (row > vltCount) row = vltCount > 0 ? vltCount : 1;
  vltToList(row);
}

static void vltDoType(int act) {
  if (vltSel < 0 || vltSel >= vltCount) return;
  if (vltTyping) { vltSay("Busy typing"); return; }
  if (!bleHidConnected) { vltSay("FAILED: not linked"); return; }
  VaultEntry &e = vltEntries[vltSel];
  String seq;
  if (act == VA_T_LOGIN) {                             // username, Tab, password, Enter
    seq = e.user;
    seq += '\t';
    seq += e.pass;
    seq += '\n';
  } else if (act == VA_T_PASS) {
    seq = e.pass;
  } else {
    seq = e.user;
  }
  if (seq.length() == 0) { vltSay("FAILED: empty"); return; }
  vltTypeBegin(seq);
  vltWipeStr(seq);
}

// ---------------- PIN flows ----------------
static void vltDoUnlock(String pin) {
  vltWipeStr(vaultInput);
  vaultPrompt = "";
  vltBusy("Unlocking...");
  int r = vltLoad(pin);
  vltWipeStr(pin);
  if (r == 0) {
    vltUnlocked = true;
    vltFails = 0;
    vltLockUntil = 0;
    vltSaveFails();
    vltMsg = "";
    vltStartBle();
    vltToList(1);
  } else if (r == 1) {
    if (vltFails < 250) vltFails++;
    vltSaveFails();
    uint32_t s = vltLockSecs(vltFails);
    vltLockUntil = s ? millis() + s * 1000UL : 0;
    vltSetState(VLT_LOCKED);
    vltSay("Wrong PIN");
  } else {
    vltSetState(VLT_LOCKED);
    vltSay(r == 3 ? "FAILED: vault file damaged" : "FAILED: cannot read vault");
  }
}

static void vltFinishPin(String pin) {
  bool chg = vltChgPin;
  vltWipeStr(vaultInput);
  vaultPrompt = "";
  vltBusy(chg ? "Re-encrypting..." : "Creating vault...");
  bool ok = vltCreate(pin);
  vltWipeStr(pin);
  vltWipeStr(vltPin1);
  if (chg) {
    vltChgPin = false;
    vltSay(ok ? "PIN changed" : "FAILED: flash write");
    vltToList(1);
  } else if (ok) {
    vltUnlocked = true;
    vltFails = 0;
    vltLockUntil = 0;
    vltSaveFails();
    vltMsg = "";
    vltStartBle();
    vltToList(1);
    vltSay("Vault created");
  } else {
    vltSetState(VLT_NOFS);
    vltSay("FAILED: flash write");
  }
}

static void vltKbCancel() {
  vltWipeStr(vaultInput);
  vaultPrompt = "";
  switch (vltState) {
    case VLT_NEWPIN1:
    case VLT_NEWPIN2:
      vltWipeStr(vltPin1);
      if (vltChgPin) {
        vltChgPin = false;
        vltToList(1);
      } else {
        goBackToMenu();                                // first-time setup cancelled
      }
      break;
    case VLT_UNLOCK:
      vltSetState(VLT_LOCKED);
      vltMsg = "";
      vltSelect(0);
      break;
    default:                                           // add / generate flows
      vltWipeStr(vltTmpName);
      vltWipeStr(vltTmpUser);
      vltToList(1);
      break;
  }
}

static void vltKbFail(const char *msg) {
  chatStatus = msg;                                    // drawn by renderKeyboard (red when it contains FAILED)
  chatStatusAt = millis();
}

// OK key of the keyboard
static void vltKbOk() {
  String in = vaultInput;
  switch (vltState) {
    case VLT_NEWPIN1:
      if (in.length() < VAULT_PIN_MIN) { vltKbFail("FAILED: min 8 chars"); break; }
      vltPin1 = in;
      vltOpenKb(VLT_NEWPIN2, "Repeat PIN", VAULT_PIN_MAX, true);
      break;

    case VLT_NEWPIN2:
      if (in != vltPin1) {
        vltWipeStr(vltPin1);
        vltOpenKb(VLT_NEWPIN1, "New PIN (8-63 chars)", VAULT_PIN_MAX, true);
        vltKbFail("FAILED: no match");
      } else {
        vltFinishPin(in);
      }
      break;

    case VLT_UNLOCK:
      if (in.length() == 0) { vltKbFail("Empty"); break; }
      vltDoUnlock(in);
      break;

    case VLT_ADD_NAME:
    case VLT_GEN_NAME:
      in.trim();
      if (in.length() == 0) { vltKbFail("Empty"); break; }
      vltTmpName = in;
      vltOpenKb(vltState == VLT_ADD_NAME ? VLT_ADD_USER : VLT_GEN_USER, "Username / email", VAULT_USER_MAX, false);
      break;

    case VLT_ADD_USER:
      in.trim();
      vltTmpUser = in;
      vltOpenKb(VLT_ADD_PASS, "Password", VAULT_PASS_MAX, true);
      break;

    case VLT_ADD_PASS: {
      if (in.length() == 0) { vltKbFail("Empty"); break; }
      String n = vltTmpName, u = vltTmpUser;
      vltWipeStr(vltTmpName);
      vltWipeStr(vltTmpUser);
      vltWipeStr(vaultInput);
      vaultPrompt = "";
      vltCommitEntry(n, u, in, VA_T_LOGIN);
      vltWipeStr(n);
      vltWipeStr(u);
      break;
    }

    case VLT_GEN_USER: {
      if (!bleHidOn) {                                 // the hardware RNG is only fully random while the radio runs
        vltSay("FAILED: start BT");
        vltKbCancel();
        break;
      }
      in.trim();
      String n = vltTmpName;
      String p = vltGenPass();
      vltWipeStr(vltTmpName);
      vltWipeStr(vaultInput);
      vaultPrompt = "";
      vltCommitEntry(n, in, p, VA_T_PASS);             // ready to click "Type password" in the sign-up form
      vltWipeStr(n);
      vltWipeStr(p);
      break;
    }

    default:
      break;
  }
  vltWipeStr(in);
}

// Joystick click while the keyboard is open (same layout / keys as the LoRa chat keyboard)
static void vltKbPress() {
  String &buf = vaultInput;
  const int maxLen = vaultMaxLen;

  if (kbRow < 3) {
    char ch = KB_ROWS[kbMode][kbRow][kbCol];
    if ((int)buf.length() < maxLen) buf += ch;
    if (kbMode == 1) kbMode = 0;                       // shift is one-shot
  } else {
    switch (kbCol) {
      case 0:
        kbMode = (kbMode + 1) % 3;
        { int l = kbRowLen(3); if (kbCol >= l) kbCol = l - 1; }
        break;
      case 1: if ((int)buf.length() < maxLen) buf += ' '; break;
      case 2: if (buf.length() > 0) buf.remove(buf.length() - 1); break;
      case 3: vltKbOk(); return;
      case 4: vaultMask = !vaultMask; break;
      case 5: vltKbCancel(); return;
    }
  }
  uiDirty = true;
}

// ============================================================================
//  public interface (declared at the top of the main sketch)
// ============================================================================
bool vaultKbActive() {
  if (page != VAULT_PAGE) return false;
  switch (vltState) {
    case VLT_NEWPIN1:
    case VLT_NEWPIN2:
    case VLT_UNLOCK:
    case VLT_ADD_NAME:
    case VLT_ADD_USER:
    case VLT_ADD_PASS:
    case VLT_GEN_NAME:
    case VLT_GEN_USER:
      return true;
    default:
      return false;
  }
}

// Called by enterPage(VAULT_PAGE)
void vaultEnter() {
  if (!vltBootInit) {                                  // wrong-PIN penalty survives a reboot
    vltBootInit = true;
    Preferences tp;
    tp.begin("watchout", true);
    vltFails = tp.getUChar("vfail", 0);
    tp.end();
    vltLoadIdleSetting();
    uint32_t s = vltLockSecs(vltFails);
    vltLockUntil = s ? millis() + s * 1000UL : 0;
  }
  vltWipeAll();
  vltUnlocked = false;
  vltChgPin = false;
  vltDelPending = false;
  vltShowPass = false;
  vltMsg = "";
  vltSelect(0);

  if (!vltMount()) {
    vltSetState(VLT_NOFS);
    return;
  }
  bool has = LittleFS.exists(VAULT_FILE) || LittleFS.exists(VAULT_BAK);
  if (has) {
    vltSetState(VLT_LOCKED);
  } else {                                             // first run: create the PIN
    vltOpenKb(VLT_NEWPIN1, "New PIN (8-63 chars)", VAULT_PIN_MAX, true);
  }
}

// Called by leavePageCleanup(): wipe the key, drop Bluetooth, unmount flash storage
void vaultLeave() {
  vltLockNow(nullptr);
  vltUnmount();
  vltMsg = "";
}

// Joystick click
void vaultClick() {
  if (vaultKbActive()) {
    vltKbPress();
    buildVault();
    return;
  }

  int act = (pageSel >= 0 && pageSel < MAX_LINES) ? vltRowAct[pageSel] : VA_NONE;
  bool wasPending = vltDelPending;
  if (act != VA_DEL) vltDelPending = false;

  if (act >= VA_ENTRY) {
    int idx = act - VA_ENTRY;
    if (idx < vltCount) vltShowDetail(idx, VA_T_LOGIN);
  } else {
    switch (act) {
      case VA_RETRY:
        vltUnmount();
        vaultEnter();
        break;

      case VA_UNLOCK: {
        long rem = vltLockUntil ? (long)(vltLockUntil - millis()) : 0;
        if (rem > 0) {
          vltSay(String("Wait ") + String((rem + 999) / 1000) + "s");
        } else {
          vltMsg = "";
          vltOpenKb(VLT_UNLOCK, "Enter PIN", VAULT_PIN_MAX, true);
        }
        break;
      }

      case VA_BT:
        if (!vltTyping) {
          vltStartBle();                               // restart advertising (also useful after "Unpair" on the host)
          vltSay("BT restarted");
        }
        break;

      case VA_UPLOAD:
        if (vltWebStart()) {
          vltSetState(VLT_WEB);
          lastActivity = millis();
        } else {
          vltSay(vltWebStatus);
        }
        break;

      case VA_WEBSTOP:
        vltWebStop();
        vltSetState(VLT_LIST);
        vltSay("Import server stopped");
        break;

      case VA_ADD:
        if (vltCount >= VAULT_MAX_ENTRIES) vltSay("Vault full");
        else vltOpenKb(VLT_ADD_NAME, "Name (e.g. Gmail)", VAULT_NAME_MAX, false);
        break;

      case VA_GEN:
        if (vltCount >= VAULT_MAX_ENTRIES) vltSay("Vault full");
        else if (!bleHidOn) vltSay("FAILED: start BT");
        else vltOpenKb(VLT_GEN_NAME, "Name (e.g. Gmail)", VAULT_NAME_MAX, false);
        break;

      case VA_CHGPIN:
        vltChgPin = true;
        vltOpenKb(VLT_NEWPIN1, "New PIN (8-63 chars)", VAULT_PIN_MAX, true);
        break;

      case VA_TIMEOUT: {
        uint8_t previous = vltIdleIndex;
        vltIdleIndex = (vltIdleIndex + 1) %
                       (sizeof(vltIdleOptions) / sizeof(vltIdleOptions[0]));
        if (!vltSaveIdleSetting()) {
          vltIdleIndex = previous;
          vltSay("FAILED: save auto-lock");
        } else {
          lastActivity = millis();
          buildVault();
          vltSelectAct(VA_TIMEOUT);
        }
        break;
      }

      case VA_LOCK:
        vltLockNow("Locked");
        break;

      case VA_T_LOGIN:
      case VA_T_PASS:
      case VA_T_USER:
        vltDoType(act);
        break;

      case VA_SHOW:
        vltShowPass = !vltShowPass;
        buildVault();
        vltSelectAct(VA_SHOW);                         // the password may wrap over several rows, keep the cursor on this row
        break;

      case VA_DEL:
        if (!wasPending) {
          vltDelPending = true;
          vltSay("Click again: DEL");
        } else {
          vltDelPending = false;
          vltDeleteEntry(vltSel);
        }
        break;

      default:
        break;
    }
  }
  buildVault();
}

// Back swipe: returns true when the vault handled it itself (otherwise the sketch goes back to the menu)
bool vaultBack() {
  if (page != VAULT_PAGE) return false;
  if (vltState == VLT_DETAIL) {
    vltDelPending = false;
    vltShowPass = false;
    vltToList(1 + vltSel);
    return true;
  }
  if (vltState == VLT_WEB) {
    vltWebStop();
    vltToList(1);
    return true;
  }
  goBackToMenu();
  return true;
}

// Long press on the joystick button
bool vaultLongClick() {
  if (page != VAULT_PAGE) return false;
  if (vaultKbActive()) {                               // leave the keyboard
    vltKbCancel();
    buildVault();
    return true;
  }
  if (vltState == VLT_DETAIL) {
    vaultBack();
    return true;
  }
  if (vltUnlocked && vltState == VLT_LIST) {           // lock right now
    vltLockNow("Locked");
    buildVault();
    return true;
  }
  return false;
}

// Called from loop()
void vaultPoll() {
  if (vltTyping) vltTypeStep();
  if (page != VAULT_PAGE) return;

  unsigned long now = millis();
  if (vltWebStarting) {
    if (WiFi.status() == WL_CONNECTED) {
      vltWebBeginServer();
      lastActivity = now;
      uiDirty = true;
      lastDraw = 0;
    } else if (now - vltWebStartAt >= 20000UL) {
      vltWebStarting = false;
      vltWipeStr(vltWebToken);
      if (vltWebOwnsWifi && !apHold && !trackerOn) WiFi.disconnect(false, false);
      vltWebOwnsWifi = false;
      vltWebStatus = "Wi-Fi connection timed out";
      uiDirty = true;
      lastDraw = 0;
    }
  }
  if (vltWebUp) {
    bool connected = WiFi.status() == WL_CONNECTED;
    if (!connected && vltWebWifiWasConnected) {
      vltWebStatus = "Wi-Fi disconnected; waiting to reconnect";
      vltWebWifiWasConnected = false;
      uiDirty = true;
    } else if (connected && !vltWebWifiWasConnected) {
      vltWebStatus = "Ready to add entries";
      vltWebWifiWasConnected = true;
      uiDirty = true;
    }
    if (connected) vltWebServer.handleClient();
  }

  if (now - lastActivity > vltIdleOptions[vltIdleIndex] && !vltTyping) {
    if (vltUnlocked) {
      vltLockNow("Auto-locked");
      buildVault();
      return;
    }
    if (vltState == VLT_NEWPIN1 || vltState == VLT_NEWPIN2 || vltState == VLT_UNLOCK) {
      vltKbCancel();
      buildVault();
      return;
    }
  }

  // refresh the Bluetooth row as soon as the link state changes
  static uint8_t lastBt = 255;
  uint8_t bt = (bleHidReady ? 4 : 0) | (bleHidConnected ? 2 : 0) | (bleHidOn ? 1 : 0);
  if (bt != lastBt) {
    lastBt = bt;
    if (page == VAULT_PAGE && !vaultKbActive()) lastDraw = 0;
  }
}