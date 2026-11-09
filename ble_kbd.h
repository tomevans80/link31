// ble_kbd.h - Bluetooth LE keyboard input for the ESP32 (M5Stack Core2).
//
// Acts as a BLE HID-over-GATT *host*: scans for a keyboard, pairs (passkey shown on our screen),
// bonds, subscribes to its input reports and turns key presses into ASCII for the sketch.
//
// Needs the "NimBLE-Arduino" library (Library Manager, tested against 2.5.2). If the library is
// not installed this file compiles to a stub, so the sketch still builds (without keyboard support).
//
// Works with BLE keyboards only (Logitech K380/MX Keys, Keychron, most multi-device and tablet
// keyboards, ...). Bluetooth *Classic* keyboards and 2.4 GHz dongle keyboards are not supported.
//
// Everything slow (scan, connect, pair, service discovery) runs in its own FreeRTOS task, so the
// UI never blocks. The sketch only calls the small API below from loop().
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

namespace blekbd {

enum State { kOff, kIdle, kScanning, kConnecting, kPairing, kReady };

}  // namespace blekbd

#if __has_include(<NimBLEDevice.h>)
#define BLEKBD_AVAILABLE 1
#include <Arduino.h>
#include <NimBLEDevice.h>

namespace blekbd {
namespace detail {

struct Cand {
  bool have = false;
  NimBLEAddress addr;
  int rssi = -127;
  uint32_t firstMs = 0;
  char name[24] = "";
};

struct Ctx {
  volatile State state = kOff;
  volatile bool inited = false, enabled = false, pairMode = false, forgetReq = false, cancelReq = false;
  volatile bool uk = true;
  volatile bool hold = false;     // sketch asks us to stay off the air (radio link is connecting)
  volatile uint32_t pairUntil = 0;
  volatile bool passShown = false;
  volatile uint32_t passkey = 0;
  char name[24] = "";
  char msg[48] = "";
  // key queue: single producer (BLE host task) / single consumer (loop)
  volatile uint8_t qh = 0, qt = 0;
  uint8_t q[64];
  uint8_t prev[6] = {0, 0, 0, 0, 0, 0};
  volatile bool caps = false;
  Cand cand;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};
static Ctx c;

static void setMsg(const char* m) { strncpy(c.msg, m, sizeof c.msg - 1); c.msg[sizeof c.msg - 1] = 0; }

static void push(uint8_t ch) {
  uint8_t nh = (uint8_t)((c.qh + 1) & 63);
  if (nh != c.qt) { c.q[c.qh] = ch; c.qh = nh; }
}

// HID usage -> ASCII. Returns 0 for keys we do not use. Control keys: \r enter, \b backspace, 27 esc, \t tab,
// 0x80 left arrow, 0x81 right arrow, 0x85 up arrow, 0x86 down arrow, 0x82 Ctrl+Tab, 0x83 Ctrl+T, 0x84 Ctrl+M.
static uint8_t translate(uint8_t k, bool sh, bool uk, bool caps) {
  if (k >= 0x04 && k <= 0x1D) {
    char ch = (char)('a' + (k - 0x04));
    return (uint8_t)((sh != caps) ? ch - 32 : ch);
  }
  if (k >= 0x1E && k <= 0x27) {
    static const char num[10] = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
    static const char us[10]  = {'!', '@', '#', '$', '%', '^', '&', '*', '(', ')'};
    static const char gb[10]  = {'!', '"', 0, '$', '%', '^', '&', '*', '(', ')'};   // shift+3 is a pound sign: not ASCII
    int i = k - 0x1E;
    return (uint8_t)(sh ? (uk ? gb[i] : us[i]) : num[i]);
  }
  switch (k) {
    case 0x28: return '\r';
    case 0x29: return 27;
    case 0x2A: return '\b';
    case 0x2B: return '\t';
    case 0x2C: return ' ';
    case 0x2D: return sh ? '_' : '-';
    case 0x2E: return sh ? '+' : '=';
    case 0x2F: return sh ? '{' : '[';
    case 0x30: return sh ? '}' : ']';
    case 0x31: return sh ? '|' : '\\';
    case 0x32: return uk ? (sh ? '~' : '#') : (sh ? '|' : '\\');
    case 0x33: return sh ? ':' : ';';
    case 0x34: return uk ? (sh ? '@' : '\'') : (sh ? '"' : '\'');
    case 0x35: return uk ? (sh ? 0 : '`') : (sh ? '~' : '`');
    case 0x36: return sh ? '<' : ',';
    case 0x37: return sh ? '>' : '.';
    case 0x38: return sh ? '?' : '/';
    case 0x4C: return '\b';             // Delete
    case 0x4F: return 0x81;             // right arrow
    case 0x50: return 0x80;             // left arrow
    case 0x52: return 0x85;             // up arrow
    case 0x51: return 0x86;             // down arrow
    case 0x64: return sh ? '|' : '\\';  // ISO key next to left shift
    case 0x54: return '/';              // keypad
    case 0x55: return '*';
    case 0x56: return '-';
    case 0x57: return '+';
    case 0x58: return '\r';
    case 0x63: return '.';
  }
  if (k >= 0x59 && k <= 0x61) return (uint8_t)('1' + (k - 0x59));   // keypad 1..9
  if (k == 0x62) return '0';
  return 0;
}

// 8-byte boot-format keyboard report: [modifiers][reserved][key1..key6]
static void onReport(const uint8_t* d, size_t n) {
  static int logged = 0;                          // log the first reports: shows what the keyboard really sends
  if (logged < 40) {
    logged++;
    char hex[64];
    int p = 0;
    for (size_t i = 0; i < n && i < 20; i++) p += snprintf(hex + p, sizeof hex - p, "%02x ", d[i]);
    hex[p] = 0;
    Serial.printf("[kbd] report len %u: %s\n", (unsigned)n, hex);
  }
  if (n == 9 && d[0] <= 4) { d++; n--; }          // some keyboards put the report id in front
  if (n != 8) return;
  uint8_t mod = d[0];
  bool shift = (mod & 0x22) != 0;
  bool ctrl = (mod & 0x11) != 0;
  bool other = (mod & (0x04 | 0x40 | 0x08 | 0x80)) != 0;   // alt, AltGr, GUI: ignore the key
  for (int i = 2; i < 8; i++) {
    uint8_t k = d[i];
    if (k < 0x04) continue;                       // none / rollover error
    bool was = false;
    for (int j = 0; j < 6; j++) if (c.prev[j] == k) was = true;
    if (was) continue;                            // still held from the last report
    if (k == 0x39) { c.caps = !c.caps; continue; }   // Caps Lock
    if (other) continue;
    if (ctrl) {                                   // Ctrl shortcuts: Tab = TYPE, T = TUNE, M = menu; any other Ctrl+key is ignored
      uint8_t sc = (k == 0x2B) ? 0x82 : (k == 0x17) ? 0x83 : (k == 0x10) ? 0x84 : 0;
      if (sc) push(sc);
      continue;
    }
    uint8_t ch = translate(k, shift, c.uk, c.caps);
    if (ch) push(ch);
  }
  for (int j = 0; j < 6; j++) c.prev[j] = d[2 + j];
}

static void notifyCb(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { onReport(data, len); }

class ScanCb : public NimBLEScanCallbacks {
 public:
  void onResult(const NimBLEAdvertisedDevice* d) override {
    if (c.state != kScanning || !d->isConnectable()) return;
    bool bonded = NimBLEDevice::isBonded(d->getAddress());
    if (c.pairMode) {
      if (!d->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) return;         // HID service
      uint16_t ap = d->getAppearance();
      if (ap != 0 && ap != 0x03C0 && ap != 0x03C1) return;                        // not a keyboard (mouse, gamepad...)
    } else if (!bonded) {
      return;                                                                     // reconnect only to known keyboards
    }                                                                             // (a bonded keyboard may use directed adverts: no UUID list)
    portENTER_CRITICAL(&c.mux);
    if (!c.cand.have || d->getRSSI() > c.cand.rssi) {       // keep the strongest (= nearest) keyboard
      if (!c.cand.have) c.cand.firstMs = millis();
      c.cand.addr = d->getAddress();
      c.cand.rssi = d->getRSSI();
      std::string n = d->haveName() ? d->getName() : std::string();
      strncpy(c.cand.name, n.c_str(), sizeof c.cand.name - 1);
      c.cand.name[sizeof c.cand.name - 1] = 0;
      c.cand.have = true;
    }
    portEXIT_CRITICAL(&c.mux);
  }
};

class CliCb : public NimBLEClientCallbacks {
 public:
  // Passkey entry is shown through NimBLEDevice::setSecurityPasskey() (set before pairing starts).
  void onConfirmPasskey(NimBLEConnInfo& ci, uint32_t) override { NimBLEDevice::injectConfirmPasskey(ci, true); }
};

static ScanCb scanCb;
static CliCb cliCb;

static void clearCand() {
  portENTER_CRITICAL(&c.mux);
  c.cand.have = false;
  c.cand.rssi = -127;
  portEXIT_CRITICAL(&c.mux);
}

static bool takeCand(uint32_t now, NimBLEAddress& a, char* name, size_t cap) {
  bool ok = false;
  portENTER_CRITICAL(&c.mux);
  if (c.cand.have && (!c.pairMode || now - c.cand.firstMs >= 2500)) {   // pairing: wait 2.5 s and take the strongest
    a = c.cand.addr;
    strncpy(name, c.cand.name, cap - 1);
    name[cap - 1] = 0;
    c.cand.have = false;
    c.cand.rssi = -127;
    ok = true;
  }
  portEXIT_CRITICAL(&c.mux);
  return ok;
}

static void dropClient(NimBLEClient*& cl) {
  if (!cl) return;
  if (cl->isConnected()) {
    cl->disconnect();
    for (int i = 0; i < 20 && cl->isConnected(); i++) vTaskDelay(pdMS_TO_TICKS(50));
  }
  NimBLEDevice::deleteClient(cl);
  cl = nullptr;
  memset(c.prev, 0, sizeof c.prev);
}

// Subscribe to the keyboard's input reports. Returns true if at least one subscription worked.
// Preferred: switch the keyboard to "boot protocol" and use the Boot Keyboard Input Report, whose 8-byte
// layout is fixed by the HID spec. Otherwise subscribe to the normal input reports (8-byte layout assumed).
static bool setupHid(NimBLEClient* cl) {
  NimBLERemoteService* s = cl->getService(NimBLEUUID((uint16_t)0x1812));
  if (!s) { Serial.println("[ble] no HID service"); return false; }
  const std::vector<NimBLERemoteCharacteristic*>& chars = s->getCharacteristics(true);
  Serial.printf("[ble] HID service: %u characteristics\n", (unsigned)chars.size());

  NimBLERemoteCharacteristic* boot = s->getCharacteristic(NimBLEUUID((uint16_t)0x2A22));   // Boot Keyboard Input Report
  NimBLERemoteCharacteristic* mode = s->getCharacteristic(NimBLEUUID((uint16_t)0x2A4E));   // Protocol Mode
  if (boot && mode && boot->canNotify()) {
    uint8_t zero = 0;
    bool w = mode->writeValue(&zero, 1, false);                                            // 0 = boot protocol
    bool sub = boot->subscribe(true, notifyCb);
    Serial.printf("[ble] boot protocol: mode write %s, subscribe %s\n", w ? "ok" : "failed", sub ? "ok" : "failed");
    if (sub) return true;
  }

  int subs = 0;
  for (NimBLERemoteCharacteristic* ch : chars) {
    if (!(ch->getUUID() == NimBLEUUID((uint16_t)0x2A4D)) || !ch->canNotify()) continue;   // Report
    NimBLERemoteDescriptor* rd = ch->getDescriptor(NimBLEUUID((uint16_t)0x2908));         // Report Reference
    int id = -1, type = -1;
    if (rd) {
      NimBLEAttValue v = rd->readValue();
      if (v.size() >= 2) { id = v.data()[0]; type = v.data()[1]; }
    }
    if (type != -1 && type != 1) continue;                                                // 1 = input report
    bool sub = ch->subscribe(true, notifyCb);
    Serial.printf("[ble] input report id %d: subscribe %s\n", id, sub ? "ok" : "failed");
    if (sub) subs++;
  }
  return subs > 0;
}

static bool connectAndSetup(NimBLEClient*& cl, const NimBLEAddress& a) {
  c.state = kConnecting;
  setMsg("connecting...");
  cl = NimBLEDevice::createClient();
  cl->setClientCallbacks(&cliCb, false);
  cl->setConnectTimeout(8000);
  cl->setConnectionParams(48, 96, 0, 400);   // 60..120 ms interval (was 30..60): fewer Bluetooth time slots taken from Wi-Fi; typing is unaffected
  if (!cl->connect(a)) { setMsg("connect failed"); return false; }

  c.state = kPairing;
  uint32_t pk = esp_random() % 1000000;
  if (pk == 123456) pk = 654321;            // 123456 is the library's "ask the callback" marker
  NimBLEDevice::setSecurityPasskey(pk);
  c.passkey = pk;
  c.passShown = true;
  setMsg("pairing...");
  if (!cl->secureConnection(true)) { c.passShown = false; setMsg("pairing could not start"); return false; }
  uint32_t t0 = millis();
  while (cl->isConnected() && !cl->getConnInfo().isEncrypted() && millis() - t0 < 60000 && c.enabled && !c.cancelReq) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  c.passShown = false;
  if (c.cancelReq) { setMsg("cancelled"); return false; }
  if (!cl->isConnected()) { setMsg("keyboard disconnected while pairing"); return false; }
  if (!cl->getConnInfo().isEncrypted()) { setMsg("pairing timed out"); return false; }

  if (!setupHid(cl)) { setMsg("no HID input reports found"); return false; }
  return true;
}

static void worker(void*) {
  NimBLEClient* cl = nullptr;
  uint32_t retryAt = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(50));
    uint32_t now = millis();
    NimBLEScan* scan = NimBLEDevice::getScan();
    static int lastSt = -1;
    if ((int)c.state != lastSt) {
      lastSt = (int)c.state;
      static const char* nm[] = {"off", "idle", "scanning", "connecting", "pairing", "ready"};
      Serial.printf("[ble] %s%s%s\n", nm[lastSt], c.msg[0] ? " - " : "", c.msg);
    }

    if (c.forgetReq) {
      c.forgetReq = false;
      if (scan->isScanning()) scan->stop();
      dropClient(cl);
      NimBLEDevice::deleteAllBonds();
      c.pairMode = false;
      setMsg("all keyboards forgotten");
    }
    if (!c.enabled) {
      if (scan->isScanning()) scan->stop();
      dropClient(cl);
      c.state = kOff;
      c.passShown = false;
      continue;
    }
    if (cl && cl->isConnected()) { c.state = kReady; continue; }
    if (cl) {                                    // we had a link and lost it
      dropClient(cl);
      c.name[0] = 0;
      setMsg("keyboard disconnected");
      retryAt = now + 1000;
    }
    c.cancelReq = false;
    if (c.hold && !c.pairMode) {                 // radio link is connecting: stay quiet, let Wi-Fi have the air
      if (scan->isScanning()) scan->stop();
      c.state = kIdle;
      continue;
    }
    if (c.pairMode && (int32_t)(now - c.pairUntil) >= 0) { c.pairMode = false; setMsg("pairing timed out"); }

    bool wantScan = c.pairMode || NimBLEDevice::getNumBonds() > 0;
    if (!wantScan) {
      if (scan->isScanning()) scan->stop();
      c.state = kIdle;
      continue;
    }
    if ((int32_t)(now - retryAt) < 0) { c.state = kIdle; continue; }
    if (!scan->isScanning()) {
      clearCand();
      c.state = kScanning;
      if (c.pairMode) { scan->setInterval(100); scan->setWindow(60); scan->start(0, false, true); }       // pairing: look hard
      else            { scan->setInterval(320); scan->setWindow(30); scan->start(10000, false, true); }  // reconnect: ~10 % duty, gentle on Wi-Fi
    }
    c.state = kScanning;

    NimBLEAddress addr;
    char nm[24];
    if (!takeCand(now, addr, nm, sizeof nm)) continue;
    scan->stop();
    if (connectAndSetup(cl, addr)) {
      strncpy(c.name, nm[0] ? nm : "keyboard", sizeof c.name - 1);
      c.name[sizeof c.name - 1] = 0;
      c.pairMode = false;
      c.state = kReady;
      setMsg("connected");
    } else {
      dropClient(cl);
      retryAt = millis() + (c.pairMode ? 1500 : 4000);
      c.state = kIdle;
    }
  }
}

}  // namespace detail

// ---- public API -------------------------------------------------------------------------------------

// Start the Bluetooth stack and the worker task (once). Safe to call repeatedly.
inline void begin() {
  using namespace detail;
  if (c.inited) return;
  Serial.println("[ble] init...");
#ifdef __EXCEPTIONS
  try {
#endif
    if (!NimBLEDevice::init("IC705 Core2")) { Serial.println("[ble] NimBLEDevice::init returned false"); setMsg("Bluetooth init failed"); return; }
    Serial.printf("[ble] stack up, heap %u (largest %u)\n", (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());
    NimBLEDevice::setSecurityAuth(true, true, true);          // bonding, MITM (passkey), secure connections
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);   // we show the passkey, the keyboard types it
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCb, false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(40);
    c.inited = true;
    xTaskCreatePinnedToCore(worker, "blekbd", 8192, nullptr, 1, nullptr, 1);
    Serial.println("[ble] ready");
#ifdef __EXCEPTIONS
  } catch (const std::exception& ex) {
    Serial.printf("[ble] EXCEPTION during init: %s\n", ex.what());
    setMsg("Bluetooth init exception (see serial)");
  } catch (...) {
    Serial.println("[ble] unknown EXCEPTION during init");
    setMsg("Bluetooth init exception (see serial)");
  }
#endif
}

// Turn the keyboard feature on/off. On: reconnects to a paired keyboard whenever it is awake.
inline void enable(bool on) {
  if (on) begin();
  detail::c.enabled = on && detail::c.inited;
  if (!on) { detail::c.pairMode = false; detail::c.msg[0] = 0; detail::c.name[0] = 0; }   // no stale "connected: ..." text
}
inline bool enabled() { return detail::c.enabled; }
// Ask the module to stay off the air (no scanning / connecting) while the radio link is being set up.
inline void setHold(bool h) { detail::c.hold = h; }

// Look for a NEW keyboard for 60 s (put it in pairing mode first).
inline void pair() {
  if (!detail::c.enabled) return;
  detail::c.cancelReq = false;
  detail::setMsg("searching: put keyboard in pairing mode");
  detail::c.pairUntil = millis() + 60000;
  detail::c.pairMode = true;
}
inline void cancel() { detail::c.pairMode = false; detail::c.cancelReq = true; }
inline void forget() { if (detail::c.inited) detail::c.forgetReq = true; }

inline bool getKey(uint8_t& k) {
  using namespace detail;
  if (c.qt == c.qh) return false;
  k = c.q[c.qt];
  c.qt = (uint8_t)((c.qt + 1) & 63);
  return true;
}

inline State state() { return detail::c.state; }
inline bool ready() { return detail::c.state == kReady; }
inline bool pairSearching() { return detail::c.pairMode; }
inline bool passkeyShown() { return detail::c.passShown; }
inline uint32_t passkey() { return detail::c.passkey; }
inline const char* name() { return detail::c.name; }
inline const char* message() { return detail::c.msg; }
inline int bondCount() { return detail::c.inited ? NimBLEDevice::getNumBonds() : 0; }
inline void setUkLayout(bool uk) { detail::c.uk = uk; }
inline bool ukLayout() { return detail::c.uk; }
inline bool available() { return true; }

}  // namespace blekbd

#else  // ---------------------------------- stub: NimBLE-Arduino not installed ---------------------

namespace blekbd {
inline void begin() {}
inline void enable(bool) {}
inline bool enabled() { return false; }
inline void setHold(bool) {}
inline void pair() {}
inline void cancel() {}
inline void forget() {}
inline bool getKey(uint8_t&) { return false; }
inline State state() { return kOff; }
inline bool ready() { return false; }
inline bool pairSearching() { return false; }
inline bool passkeyShown() { return false; }
inline uint32_t passkey() { return 0; }
inline const char* name() { return ""; }
inline const char* message() { return "NimBLE-Arduino library not installed"; }
inline int bondCount() { return 0; }
inline void setUkLayout(bool) {}
inline bool ukLayout() { return true; }
inline bool available() { return false; }
}  // namespace blekbd

#endif
