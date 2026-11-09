// IC-705 CW / PSK31 terminal for the M5Stack Core2.
//
//  - Wi-Fi station -> IC-705 network remote protocol (control + CI-V + audio, see icom.h)
//  - RX: radio audio -> CW decoder or PSK31 decoder -> text, plus a waterfall (tap to tune)
//  - TX: on-screen touch keyboard -> CW via the radio's keyer (CI-V 0x17),
//        PSK31 via generated audio sent over the network audio stream + CI-V PTT
//
// Architecture: Arduino loop() (core 1) = UI + DSP.  netTask (core 0) = UDP protocol,
// CI-V polling, CW pacing, PSK31 audio generation.  Shared state is kept small and
// guarded by a mutex or single-producer/single-consumer rings.

#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <ctype.h>
#include "icom.h"
#include "civ.h"
#include "dsp.h"
#include <NimBLEDevice.h>   // NimBLE-Arduino (Library Manager). This plain include is what makes the Arduino IDE add the library to the build.
#include "ble_kbd.h"   // Bluetooth LE keyboard
#include <new>

// Declared up here (not next to the UI code) because the Arduino IDE auto-generates function
// prototypes at the top of an .ino file, and some of them take a Screen parameter.
enum Screen { SCR_MAIN, SCR_KBD, SCR_SET, SCR_BT };

// ============================================================================
// Big buffers live in PSRAM, which leaves the small internal RAM for Wi-Fi and Bluetooth. They are
// allocated at the start of setup() (allocBigBuffers), once PSRAM is available to the heap.
// Arduino IDE: Tools > PSRAM must be "Enabled" (default for the M5Stack-Core2 board). If it is not,
// the buffers fall back to internal RAM.
#if __has_include("esp_rom_sys.h")
#include "esp_rom_sys.h"
#else
#include "rom/ets_sys.h"
#endif
#include "esp_heap_caps.h"

// ============================================================================
static void* bigAlloc(size_t n) {
  void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_8BIT);
  if (!p) {   // report before aborting
#if __has_include("esp_rom_sys.h")
    esp_rom_printf("bigAlloc(%u) failed: largest internal block %u, psram free %u\n", (unsigned)n,
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
#else
    ets_printf("bigAlloc(%u) failed: largest internal block %u\n", (unsigned)n, (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
#endif
    abort();
  }
  memset(p, 0, n);
  return p;
}

// Free-memory log on the serial port (115200) to see where internal RAM goes.
static void memLog(const char* tag) {
  Serial.printf("[mem] %-9s heap %6u (largest block %6u)  psram %7u\n", tag, (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap(), (unsigned)ESP.getFreePsram());
}

// ============================================================================
// Configuration constants
// ============================================================================
static const uint32_t kAudioRate = 8000;   // radio network audio rate (8000/16000/24000/48000)
static const int      kWfRows    = 42;
static const int      kWfTop     = 130;
static const int      kWfDotH    = 5;     // rows above the waterfall used by the tuning dot (the receive text ends at y 124)
static const float    kWfMaxHz   = 3000.0f;
static const int      kRxCols    = 26;   // text size 2: 12 px per character, 26 x 12 = 312 px
static const int      kRxRows    = 5;    // 18 px line pitch, 5 x 18 = 90 px (same box as before)
static const int      kRxLineH   = 18;
static const int      kRxTop     = 36;
static const bool     kTrace     = true;   // hex trace of the radio protocol on the serial port (115200 baud)

// ============================================================================
// Settings (persisted in NVS)
// ============================================================================
struct Settings {
  char ssid[33] = "";
  char wpass[65] = "";
  char ip[16] = "";          // empty = use the Wi-Fi gateway (radio in access-point mode)
  char user[17] = "";
  char pass[17] = "";
  char civ[3] = "A4";
  int  wpm = 20;
  int  pskLevel = 25;        // % of full scale
  bool allowTx = false;      // transmit (CW, PSK31, RTTY, TUNE) must be armed explicitly
  bool autoPitch = true;
  int  mode = 0;             // 0 = CW, 1 = PSK31, 2 = RTTY
  int  pitch = 600;          // CW pitch (Hz) used by the decoder
  int  pskHz = 1000;         // PSK31 carrier (Hz)
  int  cwGate = 28;          // CW output squelch x0.1 (lower = more sensitive, more junk from noise)
  int  rttyHz = 2210;        // RTTY centre between mark and space (Hz)
  int  rttyBaud = 0;         // 0 = 45.45, 1 = 50, 2 = 75
  int  rttyShift = 170;      // Hz
  bool rttyRev = true;       // true: mark is the HIGHER audio tone (default: standard RTTY polarity on USB); false: mark is the lower tone
  bool btOn = false;         // Bluetooth LE keyboard enabled
  bool btUk = true;          // keyboard layout: true = UK, false = US
} S;

static Preferences gPrefs;

static void loadSettings() {
  gPrefs.begin("ic705", true);
  gPrefs.getString("ssid", S.ssid, sizeof S.ssid);
  gPrefs.getString("wpass", S.wpass, sizeof S.wpass);
  gPrefs.getString("ip", S.ip, sizeof S.ip);
  gPrefs.getString("user", S.user, sizeof S.user);
  gPrefs.getString("pass", S.pass, sizeof S.pass);
  char t[3];
  if (gPrefs.getString("civ", t, sizeof t) > 0) strcpy(S.civ, t);
  S.wpm = gPrefs.getInt("wpm", 20);
  S.pskLevel = gPrefs.getInt("lvl", 25);
  S.allowTx = gPrefs.getBool("allow", false);
  S.autoPitch = gPrefs.getBool("apitch", true);
  S.mode = constrain(gPrefs.getInt("mode", 0), 0, 2);
  S.pitch = gPrefs.getInt("pitch", 600);
  S.pskHz = gPrefs.getInt("pskhz", 1000);
  S.cwGate = gPrefs.getInt("gate", 28);
  S.rttyHz = gPrefs.getInt("rtyhz", 2210);
  S.rttyBaud = constrain(gPrefs.getInt("rtybd", 0), 0, 2);
  S.rttyShift = gPrefs.getInt("rtysh", 170);
  S.rttyRev = gPrefs.getBool("rtyrev", true);
  S.btOn = gPrefs.getBool("btkb", false);
  S.btUk = gPrefs.getBool("btuk", true);
  gPrefs.end();
}

static void saveSettings() {
  gPrefs.begin("ic705", false);
  gPrefs.putString("ssid", S.ssid);
  gPrefs.putString("wpass", S.wpass);
  gPrefs.putString("ip", S.ip);
  gPrefs.putString("user", S.user);
  gPrefs.putString("pass", S.pass);
  gPrefs.putString("civ", S.civ);
  gPrefs.putInt("wpm", S.wpm);
  gPrefs.putInt("lvl", S.pskLevel);
  gPrefs.putBool("allow", S.allowTx);
  gPrefs.putBool("apitch", S.autoPitch);
  gPrefs.putInt("mode", S.mode);
  gPrefs.putInt("pitch", S.pitch);
  gPrefs.putInt("pskhz", S.pskHz);
  gPrefs.putInt("gate", S.cwGate);
  gPrefs.putInt("rtyhz", S.rttyHz);
  gPrefs.putInt("rtybd", S.rttyBaud);
  gPrefs.putInt("rtysh", S.rttyShift);
  gPrefs.putBool("rtyrev", S.rttyRev);
  gPrefs.putBool("btkb", S.btOn);
  gPrefs.putBool("btuk", S.btUk);
  gPrefs.end();
}

static uint8_t civAddr() { return (uint8_t)strtoul(S.civ, nullptr, 16); }

// ============================================================================
// Platform glue for the Icom link (WiFiUDP)
// ============================================================================
// Packet trace for debugging the radio link. Credential bytes in login/stream-request packets are
// masked (xx), so a pasted log does not reveal them. Frequent keep-alives and audio are rate limited.
static void tracePacket(const char* dir, int s, const uint8_t* d, size_t n) {
  if (!kTrace || n < 16) return;
  static uint16_t cnt[3][2];
  int di = (dir[0] == 'T') ? 0 : 1;
  bool noisy = (n == 21 && d[4] == 7) || (n == 16 && d[4] == 0 && d[5] == 0);
  uint16_t limit = (s == 0) ? (noisy ? 4 : 60) : 10;
  if (cnt[s][di] >= limit) return;
  cnt[s][di]++;
  char line[220];
  int pos = snprintf(line, sizeof line, "[%s %s %u] ", dir, s == 0 ? "ctrl" : s == 1 ? "civ" : "audio", (unsigned)n);
  size_t show = n < 48 ? n : 48;
  for (size_t i = 0; i < show && pos < (int)sizeof line - 4; i++) {
    bool mask = di == 0 && s == 0 && ((n == 128 && i >= 64 && i < 96) || (n == 144 && i >= 96 && i < 112));
    if (mask) pos += snprintf(line + pos, sizeof line - pos, "xx ");
    else pos += snprintf(line + pos, sizeof line - pos, "%02x ", d[i]);
  }
  Serial.println(line);
}

class EspIo : public icom::Io {
 public:
  IPAddress remote;
  uint32_t millis() override { return ::millis(); }
  bool open(int s, uint16_t remotePort) override {
    rport_[s] = remotePort;
    udp_[s].stop();
    return udp_[s].begin(lport_[s]) != 0;
  }
  void close(int s) override { udp_[s].stop(); }
  bool send(int s, const uint8_t* d, size_t n) override {
    if (!udp_[s].beginPacket(remote, rport_[s])) return false;
    udp_[s].write(d, n);
    tracePacket("TX", s, d, n);
    return udp_[s].endPacket() != 0;
  }
  int recv(int s, uint8_t* buf, size_t cap) override {
    int sz = udp_[s].parsePacket();
    if (sz <= 0) return 0;
    int n = udp_[s].read(buf, cap);
    if (n > 0) tracePacket("RX", s, buf, (size_t)n);
    return n > 0 ? n : 0;
  }
  uint32_t localSid(int s) override {
    IPAddress ip = WiFi.localIP();
    uint32_t low16 = ((uint32_t)ip[2] << 8) | ip[3];
    return (low16 << 16) | lport_[s];
  }
  uint32_t random32() override { return esp_random(); }
  void log(const char* m) override { Serial.printf("[icom] %s\n", m); }

 private:
  WiFiUDP udp_[3];
  uint16_t rport_[3] = {0, 0, 0};
  uint16_t lport_[3] = {50101, 50102, 50103};
};

static EspIo gIo;
static icom::Link* gLinkP = nullptr;   // ~22 KB, allocated in allocBigBuffers() (PSRAM)
#define gLink (*gLinkP)
static civ::Rig gRig;
static volatile bool gReconnect = true;   // (re)start the radio link
static uint32_t gAudioSeen = 0;             // last time an audio frame arrived (stall watchdog)
static volatile bool gWifiWanted = true;

// ============================================================================
// Shared buffers between net task and UI task
// ============================================================================
static const uint32_t kRxRing = 8192;   // power of two
static int16_t* gRx = nullptr;   // 16 KB, allocated in allocBigBuffers() (PSRAM)
static volatile uint32_t gRxW = 0, gRxR = 0;
static volatile uint32_t gRxDrop = 0;   // samples dropped because the DSP side fell behind (ring full)

static void rxPush(const int16_t* p, size_t n) {
  for (size_t i = 0; i < n; i++) {
    uint32_t nw = (gRxW + 1) & (kRxRing - 1);
    if (nw == gRxR) { gRxDrop += (uint32_t)(n - i); return; }   // overflow: drop
    gRx[gRxW] = p[i];
    gRxW = nw;
  }
}

struct CivCmd { uint8_t n; uint8_t d[40]; };
static QueueHandle_t gCivQ;
static SemaphoreHandle_t gMux;
#define LOCK()   xSemaphoreTake(gMux, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(gMux)

static void civSend(const uint8_t* f, size_t n) {
  CivCmd c;
  if (n > sizeof c.d) return;
  c.n = (uint8_t)n;
  memcpy(c.d, f, n);
  xQueueSend(gCivQ, &c, 0);
}

// Live transmit. The typed text (gTxBuf) IS the transmit queue: characters 0..gTxSent-1 have been handed to the
// radio's keyer (CW) or to the PSK31 modulator and are locked; the rest are still editable. The net task advances
// gTxSent just in time (a character at a time); the UI appends/deletes at the end. gMux protects gTxBuf, gTxSent.
static char gTxBuf[121] = "";
static int gTxSent = 0;
static volatile uint32_t gTxRev = 0;          // bumped on every change of gTxBuf / gTxSent (redraw trigger)
static const uint32_t kCwLeadMs = 250;        // CW: next character goes to the keyer when it has <= this much left to key
static const uint32_t kPskHangMs = 3000;      // PSK31: carrier idles this long after the last character, then PTT drops
static uint32_t gCwBusyUntil = 0;             // estimated time the keyer finishes what it has been given
static volatile bool gCwActive = false;
static volatile bool gTxFinishing = false;    // PSK31: end of over requested, no more characters accepted

// PSK31 / RTTY transmit (both are audio sent to the radio; S.mode picks the modulator)
static dsp::Psk31Modulator gMod;
static dsp::RttyModulator gRMod;
static float rttyBaudVal() { static const float b[3] = {45.45f, 50.0f, 75.0f}; return b[constrain(S.rttyBaud, 0, 2)]; }
static float rttyMarkHz() { float h = S.rttyShift * 0.5f; return S.rttyRev ? S.rttyHz + h : S.rttyHz - h; }
static void digBegin() {
  if (S.mode == 2) gRMod.begin((float)S.rttyHz, S.pskLevel / 100.0f, rttyBaudVal(), (float)S.rttyShift, S.rttyRev);
  else gMod.begin((float)S.pskHz, S.pskLevel / 100.0f);
}
static bool digAccepting() { return S.mode == 2 ? gRMod.accepting() : gMod.accepting(); }
static bool digQueueEmpty() { return S.mode == 2 ? gRMod.queueEmpty() : gMod.queueEmpty(); }
static bool digQueueChar(char c) { return S.mode == 2 ? gRMod.queueChar(c) : gMod.queueChar(c); }
static bool digDone() { return S.mode == 2 ? gRMod.done() : gMod.done(); }
static void digFinish() { if (S.mode == 2) gRMod.finish(); else gMod.finish(); }
static void digGenerate(int16_t* b) { if (S.mode == 2) gRMod.generate(b, 160); else gMod.generate(b, 160); }
enum TxState { TX_OFF, TX_ON };
static volatile TxState gTxState = TX_OFF;
static volatile bool gTxReq = false, gTxAbort = false;
static uint32_t gTxNext = 0, gTxEmptySince = 0, gTxEndAt = 0;

// Antenna TUNE: PSK31 mode = steady 1500 Hz tone through the network audio stream (radio in USB-D);
// CW mode = PTT only (a plain carrier, no audio). The UI sets gTuneWant; the net task does the rest.
static const uint32_t kTuneMaxMs = 20000;   // safety: TUNE switches itself off after this long
static float           gTuneHz    = 1500.0f;      // tune tone: 1500 Hz, or the RTTY mark tone
static volatile bool gTuneWant = false, gTuneAudio = false;
static volatile bool gTuning = false;
static float gTunePh = 0;
static int gTuneRamp = 0;
static uint32_t gTuneStart = 0;

// SWR is held at its peak for 1.5 s: with CW keying the meter reads zero between elements.
static volatile int gSwrPeak = -1;
static uint32_t gSwrPeakMs = 0;
static void onCivRx(void*, const uint8_t* f, size_t n) {
  uint32_t now = millis();
  if (civ::parse(f, n, gRig, now, civAddr()) == civ::kSwr) {
    if (gRig.swrRaw >= gSwrPeak || now - gSwrPeakMs > 1500) { gSwrPeak = gRig.swrRaw; gSwrPeakMs = now; }
  }
}
static void onAudioRx(void*, const int16_t* pcm, size_t n) { rxPush(pcm, n); }

// ============================================================================
// CW helpers
// ============================================================================
static int cwUnits(const char* s, size_t n) {
  int u = 0;
  for (size_t i = 0; i < n; i++) {
    if (s[i] == ' ') { u += 7; continue; }
    const char* p = dsp::morsePattern(s[i]);
    if (!p) continue;
    size_t l = strlen(p);
    for (size_t k = 0; k < l; k++) u += (p[k] == '.') ? 1 : 3;
    u += (int)(l - 1) + 3;
  }
  return u;
}

// ============================================================================
// Net task (core 0)
// ============================================================================
static uint32_t gPollFreq = 0, gPollMode = 0, gPollS = 0, gPollTx = 0, gPollSwr = 0;

static void startLink() {
  IPAddress ip;
  if (S.ip[0]) ip.fromString(S.ip);
  else ip = WiFi.gatewayIP();
  gIo.remote = ip;
  icom::Config cfg;
  cfg.user = S.user;
  cfg.pass = S.pass;
  cfg.audioRate = kAudioRate;
  gLink.begin(&gIo, cfg);
  gLink.setCallbacks(nullptr, onCivRx, onAudioRx);
  gLink.connect();
  Serial.printf("[net] Core2 IP %s, gateway %s, connecting to radio at %s (user '%s', %u-char password)\n",
                WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(),
                ip.toString().c_str(), S.user, (unsigned)strlen(S.pass));
}

static void pttOff() {
  uint8_t f[16];
  size_t n = civ::ptt(f, false, civAddr());
  gLink.sendCiv(f, n);
}

static void tuneFinish() {
  pttOff();
  gTuning = false;
  gTuneWant = false;
  gTxState = TX_OFF;
}

static void tuneService(uint32_t now) {
  uint8_t ca = civAddr();
  if (!gTuning) {                       // start: PTT on, audio follows after 150 ms
    uint8_t fr[16];
    gLink.sendCiv(fr, civ::ptt(fr, true, ca));
    gTuning = true;
    gTuneStart = now;
    gTunePh = 0;
    gTuneRamp = 0;
    gTxState = TX_ON;
    gTxNext = now + 150;
    return;
  }
  bool stop = !gTuneWant || (now - gTuneStart > kTuneMaxMs);
  if (!gTuneAudio) {                    // CW carrier: nothing to stream
    if (stop) tuneFinish();
    return;
  }
  const float dph = 6.2831853f * gTuneHz / 8000.0f;
  const float level = S.pskLevel / 100.0f;
  int guard = 0;
  while ((int32_t)(now - gTxNext) >= 0 && guard++ < 4) {
    int16_t buf[160];
    for (int i = 0; i < 160; i++) {
      float g = 1.0f;
      if (gTuneRamp < 64) g = (float)(gTuneRamp++) / 64.0f;          // soft start
      if (stop) g *= 1.0f - (float)(i + 1) / 160.0f;                  // soft stop in the last frame
      gTunePh += dph;
      if (gTunePh > 6.2831853f) gTunePh -= 6.2831853f;
      buf[i] = (int16_t)(32767.0f * level * g * sinf(gTunePh));
    }
    gLink.sendAudio(buf, 160);
    gTxNext += 20;
    if (stop) { tuneFinish(); return; }
  }
}

static void netService(uint32_t now) {
  // diagnostics: log Wi-Fi and radio-link state changes on the serial port
  static int lastWs = -99, lastLs = -1;
  int ws = (int)WiFi.status();
  int ls = (int)gLink.state();
  if (ws != lastWs) { lastWs = ws; Serial.printf("[wifi] status %d%s\n", ws, ws == WL_CONNECTED ? " (connected)" : ""); }
  if (ls != lastLs) {
    lastLs = ls;
    Serial.printf("[link] %s%s%s", gLink.stateName(), gLink.error()[0] ? ": " : "", gLink.error());
    if (ls == (int)icom::Link::kFailed)       // which streams had gone quiet (ms since the last packet)
      Serial.printf("  (silent for: ctrl %u, civ %u, audio frames %u ms, wifi %d dBm)", (unsigned)gLink.streamAgeMs(icom::kCtrl),
                    (unsigned)gLink.streamAgeMs(icom::kCiv), (unsigned)gLink.audioAgeMs(), (int)WiFi.RSSI());
    Serial.println();
  }

  // Wi-Fi and Bluetooth share one radio on the ESP32, so they take turns:
  //  - while the radio link is connecting (and for 2 s after), Bluetooth stays off the air;
  //  - while a NEW keyboard is being paired, the radio link is paused (it restarts by itself afterwards).
  static uint32_t runSince = 0;
  bool hs = ws == WL_CONNECTED && ls >= (int)icom::Link::kCtrlHello && ls <= (int)icom::Link::kStreamsHello;
  if (gLink.running()) { if (!runSince) runSince = now; } else runSince = 0;
  blekbd::setHold(hs || (runSince && now - runSince < 2000));
  if (blekbd::pairSearching() && gTxState == TX_OFF) {
    if (ls != (int)icom::Link::kIdle) gLink.disconnect();
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (gLink.state() != icom::Link::kIdle && gLink.state() != icom::Link::kFailed) gLink.disconnect();
    return;
  }
  if (gReconnect) {
    gReconnect = false;
    if (gLink.state() != icom::Link::kIdle) gLink.disconnect();
    startLink();
  } else if (gLink.state() == icom::Link::kIdle) {
    startLink();
  } else if (gLink.state() == icom::Link::kFailed && now - gLink.failedAtMs() > 8000) {
    startLink();
  }

  gLink.poll();
  if (!gLink.running()) {
    gTxState = TX_OFF;
    gTxReq = false;
    gTxFinishing = false;
    gCwActive = false;
    gCwBusyUntil = 0;
    LOCK();                                  // text not yet sent must not go out by itself after a reconnect
    if (gTxSent < (int)strlen(gTxBuf)) { gTxBuf[gTxSent] = 0; gTxRev++; }
    UNLOCK();
    gTuning = false;
    gTuneWant = false;
    gAudioSeen = now;
    return;
  }
  // Watchdog: the radio normally streams audio at 50 frames/s. If the frames stop while the control link still
  // answers, nothing else would notice and the display would simply freeze, so restart the link.
  {
    static uint32_t lastPk = 0;
    uint32_t pk = gLink.audioPackets();
    if (pk != lastPk) { lastPk = pk; gAudioSeen = now; }
    if (now - gAudioSeen > 4000 && gTxState != TX_ON && !gCwActive) {
      Serial.printf("[link] audio stalled for %u ms (ctrl %u, civ %u ms since last packet), restarting\n", (unsigned)(now - gAudioSeen),
                    (unsigned)gLink.streamAgeMs(icom::kCtrl), (unsigned)gLink.streamAgeMs(icom::kCiv));
      gAudioSeen = now;
      gReconnect = true;
    }
  }

  // commands from the UI
  CivCmd c;
  while (xQueueReceive(gCivQ, &c, 0) == pdTRUE) gLink.sendCiv(c.d, c.n);

  // poll the radio
  uint8_t f[16];
  uint8_t ca = civAddr();
  if (now - gPollS > 150)  { gPollS = now;  gLink.sendCiv(f, civ::readS(f, ca)); }
  if (now - gPollFreq > 500) { gPollFreq = now; gLink.sendCiv(f, civ::readFreq(f, ca)); }
  if (now - gPollMode > 1500) { gPollMode = now; gLink.sendCiv(f, civ::readMode(f, ca)); }
  if (now - gPollTx > 700) { gPollTx = now; gLink.sendCiv(f, civ::readTx(f, ca)); }
  if (gTxState == TX_ON || gRig.tx || gCwActive) {     // SWR only matters (and is only polled) while transmitting
    if (now - gPollSwr > 120) { gPollSwr = now; gLink.sendCiv(f, civ::readSwr(f, ca)); }
  } else {
    gSwrPeak = -1;
  }

  // CW: live send. The text typed so far is the queue; one character at a time goes to the radio's keyer, and only
  // when the keyer is about to run dry (kCwLeadMs), so characters that have not been handed over can still be edited.
  if (S.mode == 0 && !gTuning && !gTuneWant && gLink.running()) {
    bool sentOne = false;
    if ((int32_t)(gCwBusyUntil - now) < (int32_t)kCwLeadMs && S.allowTx) {
      char ch = 0;
      LOCK();
      while (!ch && gTxSent < (int)strlen(gTxBuf)) { ch = civ::cwSanitize(gTxBuf[gTxSent]); gTxSent++; gTxRev++; }
      UNLOCK();
      if (ch) {
        uint8_t fr[16];
        gLink.sendCiv(fr, civ::cwText(fr, &ch, 1, ca));
        int wpm = S.wpm < 5 ? 5 : S.wpm;
        uint32_t base = ((int32_t)(gCwBusyUntil - now) > 0) ? gCwBusyUntil : now;
        gCwBusyUntil = base + (uint32_t)cwUnits(&ch, 1) * 1200u / (uint32_t)wpm;
        gCwActive = true;
        sentOne = true;
      }
    }
    if (!sentOne && gCwActive && (int32_t)(now - gCwBusyUntil) >= 0) gCwActive = false;
  }

  // PSK31 transmit
  if (gTxAbort) {
    gTxAbort = false;
    gTxReq = false;
    gTuneWant = false;
    gTuning = false;
    if (gTxState == TX_ON) pttOff();
    gTxState = TX_OFF;
    gTxFinishing = false;
  }
  if (gTuning || (gTuneWant && gTxState == TX_OFF && !gTxReq)) { tuneService(now); return; }
  // PSK31 live send: the first typed character starts an over (PTT, preamble), more characters follow as typed.
  if (S.mode != 0 && S.allowTx && kAudioRate == 8000 && gLink.running() && gTxState == TX_OFF && !gTxReq) {
    LOCK();
    bool pending = gTxSent < (int)strlen(gTxBuf);
    if (pending) digBegin();
    UNLOCK();
    if (pending) { gTxFinishing = false; gTxReq = true; }
  }
  if (gTxReq && gTxState == TX_OFF) {
    uint8_t fr[16];
    gLink.sendCiv(fr, civ::ptt(fr, true, ca));
    gTxState = TX_ON;
    gTxNext = now + 150;      // let PTT settle before audio starts
    gTxEmptySince = 0;
    gTxEndAt = 0;
  }
  if (gTxState == TX_ON) {
    int guard = 0;
    while ((int32_t)(now - gTxNext) >= 0 && guard++ < 4) {
      int16_t buf[160];
      LOCK();
      bool pend = gTxSent < (int)strlen(gTxBuf);
      // one character at a time, only once the preamble is over and the modulator has nothing left (editable until then)
      if (pend && !gTxFinishing && digAccepting() && digQueueEmpty()) {
        digQueueChar(gTxBuf[gTxSent]);
        gTxSent++; gTxRev++;
        pend = gTxSent < (int)strlen(gTxBuf);
      }
      digGenerate(buf);
      bool done = digDone();
      bool empty = digQueueEmpty();
      bool run = digAccepting();
      UNLOCK();
      gLink.sendAudio(buf, 160);
      gTxNext += 20;
      if (run && empty && !pend && !done) {
        if (!gTxEmptySince) gTxEmptySince = now;
        if (now - gTxEmptySince > kPskHangMs) { LOCK(); digFinish(); UNLOCK(); gTxFinishing = true; }   // auto end of over
      } else {
        gTxEmptySince = 0;
      }
      if (done) {
        if (!gTxEndAt) gTxEndAt = now + 150;   // let the fade-out drain
        else if (now >= gTxEndAt) {
          pttOff();
          gTxState = TX_OFF;
          gTxReq = false;
          gTxFinishing = false;
          break;
        }
      }
    }
  }
}

static void netTask(void*) {
  for (;;) {
    netService(millis());
    vTaskDelay(1);
  }
}

// ============================================================================
// DSP objects (UI task)
// ============================================================================
static dsp::CwDecoder gCw;
static dsp::Psk31Decoder gPsk;
static dsp::RttyDecoder gRtty;
static void rttyApply() {
  gRtty.setBaud(rttyBaudVal());
  gRtty.setShift((float)S.rttyShift);
  gRtty.setReverse(S.rttyRev);
  gRtty.setFreq((float)S.rttyHz);
}
static dsp::Spectrum* gSpecP = nullptr;   // 12 KB, allocated in allocBigBuffers() (PSRAM)
#define gSpec (*gSpecP)
static dsp::Decimator gDec(kAudioRate / dsp::kRate);

// ============================================================================
// Colours
// ============================================================================
static uint32_t C_BG, C_BR, C_FG, C_DIM, C_ACC, C_WARN, C_OK, C_OKD, C_KEY, C_KEYHI, C_BTN, C_SENT;
static uint32_t gPal[256];

static void initColors() {
  auto& d = M5.Display;
  C_BG = d.color565(0, 0, 0);
  C_BR = d.color565(6, 6, 15);
  C_FG = d.color565(235, 235, 235);
  C_DIM = d.color565(120, 120, 120);
  C_ACC = d.color565(120, 200, 255);
  C_WARN = d.color565(255, 80, 60);
  C_OK = d.color565(80, 220, 100);
  C_OKD = d.color565(30, 120, 50);
  C_KEY = d.color565(50, 50, 60);
  C_KEYHI = d.color565(90, 90, 120);
  C_BTN = d.color565(40, 60, 90);
  C_SENT = d.color565(255, 150, 0);   // transmitted text (locked); not yet sent text is C_OK green
  for (int i = 0; i < 256; i++) {
    int r, g, b;
    if (i < 64) { r = 0; g = 0; b = i * 3; }
    else if (i < 128) { r = 0; g = (i - 64) * 4; b = 192 - (i - 64) * 2; }
    else if (i < 192) { r = (i - 128) * 4; g = 255; b = 64 - (i - 128); }
    else { r = 255; g = 255 - (i - 192) * 2; b = (i - 192) * 4; }
    gPal[i] = d.color565(r > 255 ? 255 : r, g < 0 ? 0 : g, b > 255 ? 255 : b);
  }
}

// ============================================================================
// Receive text window
// ============================================================================
static char gRxLines[kRxRows][kRxCols + 1];
static int gRxCol = 0;
static int gRxRow = 0;     // line being written: starts at the top, scrolling only once the window is full
static bool gRxDirty = true;

static void rxClear() {
  for (int r = 0; r < kRxRows; r++) gRxLines[r][0] = 0;
  gRxCol = 0;
  gRxRow = 0;
  gRxDirty = true;
}
static void rxNewline() {
  if (gRxRow < kRxRows - 1) {
    gRxRow++;
  } else {
    for (int r = 0; r < kRxRows - 1; r++) memcpy(gRxLines[r], gRxLines[r + 1], kRxCols + 1);
  }
  gRxLines[gRxRow][0] = 0;
  gRxCol = 0;
}
static void rxPut(char c) {
  if (c == '\n') { rxNewline(); gRxDirty = true; return; }
  if (c == '\b') {
    if (gRxCol > 0) { gRxCol--; gRxLines[gRxRow][gRxCol] = 0; gRxDirty = true; }
    return;
  }
  if (gRxCol >= kRxCols) {                       // line full: wrap at the last space, if there is one
    if (c == ' ') { rxNewline(); gRxDirty = true; return; }   // never start a wrapped line with a space
    char tail[kRxCols + 1];
    tail[0] = 0;
    char* ln = gRxLines[gRxRow];
    for (int i = kRxCols - 1; i >= 0; i--) {
      if (ln[i] == ' ') {                        // carry the unfinished word to the next line
        strcpy(tail, ln + i + 1);
        ln[i] = 0;
        break;
      }
    }                                            // no space in the whole line: plain hard break
    rxNewline();
    if (tail[0]) {
      strcpy(gRxLines[gRxRow], tail);
      gRxCol = (int)strlen(tail);
    }
  }
  gRxLines[gRxRow][gRxCol++] = c;
  gRxLines[gRxRow][gRxCol] = 0;
  gRxDirty = true;
}
static void rxPutStr(const char* s) { while (*s) rxPut(*s++); }

// ============================================================================
// Waterfall
// ============================================================================
static uint8_t (*gWf)[320] = nullptr;   // 13 KB, allocated in allocBigBuffers() (PSRAM)
static int gWfHead = 0;
static float gWfLo = -100, gWfHi = -50;
static bool gWfDirty = true;
static M5Canvas gWfCanvas(&M5.Display);
static bool gWfReady = false;
static float gLastPeakHz = 0, gLastPeakDb = -120;
static int gPitchVotes = 0;

static void wfAddRow(const float* db) {
  const int nb = (int)(kWfMaxHz / dsp::Spectrum::binHz());
  float lo, hi;
  if (S.mode == 0) {
    // CW: the radio's narrow filter leaves most of the 0-3 kHz span almost empty. Scaling from the whole-span mean
    // and maximum then makes the passband noise look as strong as the signal (everything in the passband saturates).
    // So the scale comes from the bins around the CW pitch only: the median there is the noise floor, the colours
    // run from just under it to at least 26 dB above it, and the empty stopband simply shows black.
    float bh = dsp::Spectrum::binHz();
    int c = (int)(gCw.freq() / bh);
    int w = (int)(250.0f / bh);
    int b0 = c - w < 2 ? 2 : c - w, b1 = c + w >= nb ? nb - 1 : c + w;
    float v[64];
    int n = 0;
    float mx = -200;
    for (int b = b0; b <= b1 && n < 64; b++) {
      float x = db[b];
      if (x > mx) mx = x;
      int i = n++;
      while (i > 0 && v[i - 1] > x) { v[i] = v[i - 1]; i--; }   // insertion sort
      v[i] = x;
    }
    float med = n > 0 ? v[n / 2] : -100.0f;
    lo = med - 4.0f;
    hi = mx > med + 26.0f ? mx : med + 26.0f;
  } else {
    float mean = 0, mx = -200;
    for (int b = 2; b < nb; b++) { mean += db[b]; if (db[b] > mx) mx = db[b]; }
    mean /= (nb - 2);
    lo = mean - 6.0f;
    hi = mx;
  }
  gWfLo += (lo - gWfLo) * 0.05f;
  gWfHi += (hi - gWfHi) * 0.05f;
  if (gWfHi - gWfLo < 30.0f) gWfHi = gWfLo + 30.0f;
  gWfHead = (gWfHead + kWfRows - 1) % kWfRows;
  for (int x = 0; x < 320; x++) {
    int b = (int)((float)x * nb / 320.0f);
    float v = (db[b] - gWfLo) / (gWfHi - gWfLo);
    if (v < 0) v = 0;
    if (v > 1) v = 1;
    gWf[gWfHead][x] = (uint8_t)(v * 255.0f);
  }
  gWfDirty = true;
}

static int cursorHz() { return S.mode == 0 ? (int)gCw.freq() : S.mode == 1 ? (int)gPsk.freq() : (int)gRtty.freq(); }

static void drawWaterfall() {
  if (!gWfReady) {
    gWfCanvas.setColorDepth(16);
    gWfCanvas.setPsram(true);                 // 27 KB sprite: keep it out of internal RAM
    gWfCanvas.createSprite(320, kWfRows + kWfDotH);   // a few extra rows on top for the tuning dot
    gWfReady = true;
  }
  gWfCanvas.fillRect(0, 0, 320, kWfDotH, C_BG);
  for (int r = 0; r < kWfRows; r++) {
    const uint8_t* row = gWf[(gWfHead + r) % kWfRows];
    for (int x = 0; x < 320; x++) gWfCanvas.drawPixel(x, r + kWfDotH, gPal[row[x]]);
  }
  int cx = (int)(cursorHz() * 320.0f / kWfMaxHz);
  gWfCanvas.drawFastVLine(cx, kWfDotH, kWfRows, C_WARN);
  gWfCanvas.fillCircle(cx, kWfDotH, 4, C_WARN);        // dot on top of the tuning line, reaching above the waterfall
  if (S.mode == 1) {   // PSK31 occupies about +-16 Hz
    int w = (int)(16 * 320.0f / kWfMaxHz);
    gWfCanvas.drawFastVLine(cx - w, kWfDotH, 4, C_FG);
    gWfCanvas.drawFastVLine(cx + w, kWfDotH, 4, C_FG);
  }
  if (S.mode == 2) {   // RTTY: ticks at the mark (green) and space (white) tones either side of the centre line
    float h = S.rttyShift * 0.5f;
    int xl = (int)((gRtty.freq() - h) * 320.0f / kWfMaxHz), xh = (int)((gRtty.freq() + h) * 320.0f / kWfMaxHz);
    gWfCanvas.drawFastVLine(S.rttyRev ? xh : xl, kWfDotH, 6, C_OK);
    gWfCanvas.drawFastVLine(S.rttyRev ? xl : xh, kWfDotH, 6, C_FG);
  }
  gWfCanvas.pushSprite(0, kWfTop - kWfDotH);
  gWfDirty = false;
}

// ============================================================================
// UI state
// ============================================================================
static Screen gScr = SCR_MAIN;
static bool gFullRedraw = true;
static char gNotice[40] = "";
static uint32_t gNoticeUntil = 0;

static void notice(const char* m) {
  strncpy(gNotice, m, sizeof gNotice - 1);
  gNotice[sizeof gNotice - 1] = 0;
  gNoticeUntil = millis() + 3000;
}

// ============================================================================
// Live typing: every typed character goes straight into the transmit queue (gTxBuf); the net task sends it
// ============================================================================
static bool txBusy() { return gCwActive || gTxState != TX_OFF || gTxReq; }

// Copy of the transmit text for drawing (the net task advances gTxSent at any time). '\n' is shown as '|'.
static void txSnapshot(char* out, size_t cap, int* sent) {
  LOCK();
  size_t n = strlen(gTxBuf);
  if (n >= cap) n = cap - 1;
  memcpy(out, gTxBuf, n);
  out[n] = 0;
  *sent = gTxSent;
  UNLOCK();
  for (size_t i = 0; i < n; i++) if (out[i] == '\n') out[i] = '|';
}

// Draw s[0..n) at (x, y) in the current text size, the first nSent characters in cSent, the others in cRest,
// padded with spaces to 'cols' cells with an opaque background (no clear step, so no flicker). cols <= 58.
static void drawSentText(int x, int y, int cw, const char* s, int n, int nSent, int cols, uint32_t cSent, uint32_t cRest) {
  auto& d = M5.Display;
  char seg[60];
  if (n > cols) n = cols;
  if (nSent > n) nSent = n;
  if (nSent < 0) nSent = 0;
  if (nSent > 0) {
    memcpy(seg, s, nSent);
    seg[nSent] = 0;
    d.setTextColor(cSent, C_BG);
    d.drawString(seg, x, y);
  }
  int rest = n - nSent, pad = cols - nSent;
  memcpy(seg, s + nSent, rest);
  memset(seg + rest, ' ', pad - rest);
  seg[pad] = 0;
  d.setTextColor(cRest, C_BG);
  d.drawString(seg, x + nSent * cw, y);
}

// Type one character into the live transmit line (it starts sending by itself). Returns false if refused.
static bool txType(char ch) {
  if (!gLink.running()) { notice("radio not connected"); return false; }
  if (gTuning || gTuneWant) { notice("stop TUNE first"); return false; }
  if (!S.allowTx) { notice("TX is off: enable in MENU"); return false; }
  if (S.mode != 0) {
    if (kAudioRate != 8000) { notice("digital TX needs kAudioRate = 8000"); return false; }
    if (ch == '\r') ch = '\n';
    if (S.mode == 2) {                 // RTTY: upper case, only what ITA2 can send
      if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
      if (ch != '\n' && !dsp::rttyCanSend(ch)) return false;
    } else if (ch != '\n' && (ch < 32 || ch > 126)) return false;
  } else {
    ch = civ::cwSanitize(ch);          // upper case, only what the radio's keyer accepts (Enter becomes a space)
    if (!ch) return false;
  }
  bool ok = true;
  LOCK();
  int len = (int)strlen(gTxBuf);
  if (len > 0 && gTxSent >= len && !txBusy()) { gTxBuf[0] = 0; gTxSent = 0; len = 0; }   // previous over finished: new line
  if (len + 1 >= (int)sizeof gTxBuf) {                    // full: forget the oldest sent characters
    int drop = gTxSent < 60 ? gTxSent : 60;
    if (drop > 0) { memmove(gTxBuf, gTxBuf + drop, len - drop + 1); gTxSent -= drop; len -= drop; }
  }
  if (len + 1 < (int)sizeof gTxBuf) { gTxBuf[len] = ch; gTxBuf[len + 1] = 0; gTxRev++; }
  else ok = false;
  UNLOCK();
  if (!ok) notice("line full: wait for it to send");
  return ok;
}

// Backspace removes only characters that have not been sent yet.
static void txBackspace() {
  bool ok = false;
  LOCK();
  int len = (int)strlen(gTxBuf);
  if (len > gTxSent) { gTxBuf[len - 1] = 0; gTxRev++; ok = true; }
  UNLOCK();
  if (!ok) notice("already sent: cannot delete");
}

// Throw away what has not been sent (STOP, X, Esc). If nothing is being sent, the whole line is cleared.
static void txDropUnsent() {
  LOCK();
  gTxBuf[gTxSent] = 0;
  if (!txBusy()) { gTxBuf[0] = 0; gTxSent = 0; }
  gTxRev++;
  UNLOCK();
}

// CW send speed (5..48 wpm): store it and set the radio's keyer to match.
static void setCwSpeed(int wpm) {
  S.wpm = constrain(wpm, 5, 48);
  saveSettings();
  uint8_t f[16];
  civSend(f, civ::keySpeed(f, S.wpm, civAddr()));
}

static void setAppMode(int m) {
  S.mode = m;
  gCw.reset();
  gCw.setFreq((float)S.pitch);
  gPsk.setFreq((float)S.pskHz);
  rttyApply();
  rxClear();
  saveSettings();
  uint8_t f[16];
  uint8_t ca = civAddr();
  if (m == 0) {
    civSend(f, civ::setDataMode(f, false, 1, ca));
    civSend(f, civ::setMode(f, 0x03, 1, ca));   // CW
    civSend(f, civ::keySpeed(f, S.wpm, ca));
  } else {
    civSend(f, civ::setMode(f, 0x01, 1, ca));   // USB
    civSend(f, civ::setDataMode(f, true, 1, ca));  // USB-D
  }
  gFullRedraw = true;
}

// ============================================================================
// Keyboard
// ============================================================================
struct Kbd {
  char* buf = nullptr;
  size_t cap = 0;
  const char* title = "";
  bool upper = false;
  bool password = false;
  bool shift = false;
  bool sym = false;
  const char* okLabel = "OK";
  bool live = false;      // TYPE keyboard: edits gTxBuf through txType()/txBackspace(), which transmit as you type
  Screen ret = SCR_MAIN;
  void (*done)(bool ok) = nullptr;
} gK;

static const char* kRowL0 = "QWERTYUIOP";
static const char* kRowL1 = "ASDFGHJKL";
static const char* kRowL2 = "ZXCVBNM.,";
static const char* kRowS0 = "1234567890";
static const char* kRowS1 = "/?:'()=+-";
static const char* kRowS2 = "@!#$%&*;_\"";
enum { K_SHIFT = 1, K_BKSP = 2, K_SYM = 3, K_SPACE = 4, K_ENTER = 5, K_CANCEL = 6 };
static char gKbdBackup[130];

static void openKeyboard(const char* title, char* buf, size_t cap, bool upper, bool password,
                         Screen ret, void (*done)(bool)) {
  gK.title = title; gK.buf = buf; gK.cap = cap; gK.upper = upper; gK.password = password;
  gK.shift = false; gK.sym = false; gK.ret = ret; gK.done = done; gK.okLabel = "OK"; gK.live = false;
  strncpy(gKbdBackup, buf, sizeof gKbdBackup - 1);
  gKbdBackup[sizeof gKbdBackup - 1] = 0;
  gScr = SCR_KBD;
  gFullRedraw = true;
}

static const int kKbY0 = 82, kKbRowH = 38, kKbW = 32;

// returns key code for (row, col); ASCII for characters
static int keyAt(int row, int col) {
  if (row == 3) {
    if (col <= 1) return K_SYM;
    if (col <= 6) return K_SPACE;
    if (col <= 8) return K_ENTER;
    return K_CANCEL;
  }
  if (!gK.sym) {
    if (row == 0) return kRowL0[col];
    if (row == 1) return col < 9 ? (int)kRowL1[col] : (int)K_BKSP;
    return col == 0 ? K_SHIFT : (col <= 9 ? kRowL2[col - 1] : 0);
  }
  if (row == 0) return kRowS0[col];
  if (row == 1) return col < 9 ? (int)kRowS1[col] : (int)K_BKSP;
  return kRowS2[col];
}

static char caseOf(char c) {
  if (c >= 'A' && c <= 'Z' && !(gK.upper || gK.shift)) return (char)(c - 'A' + 'a');
  return c;
}

static void drawKey(int x, int y, int w, int h, const char* label, uint32_t bg) {
  auto& d = M5.Display;
  d.fillRoundRect(x, y, w, h, 4, bg);
  d.setTextColor(C_FG, bg);
  d.setTextSize(1.5f);
  int tw = d.textWidth(label), th = d.fontHeight();
  d.drawString(label, x + (w - tw) / 2, y + (h - th) / 2);
}

// Title/notice line and the text field (3 lines of 26 characters at size 2, the tail of long text). Redrawn on its own,
// opaque and padded, when the live transmit text changes.
static void drawKbdText() {
  auto& d = M5.Display;
  d.setTextSize(1);
  bool note = gK.live && (int32_t)(gNoticeUntil - millis()) > 0;
  d.setTextColor(note ? C_WARN : C_ACC, C_BG);
  char t[54];
  snprintf(t, sizeof t, "%-53s", note ? gNotice : gK.title);
  d.drawString(t, 4, 2);
  char vis[130];
  int sent = 0;
  if (gK.live) txSnapshot(vis, sizeof vis, &sent);
  else {
    size_t n = strlen(gK.buf);
    if (n > sizeof vis - 1) n = sizeof vis - 1;
    size_t last = n ? n - 1 : 0;
    for (size_t i = 0; i < n; i++) vis[i] = (gK.password && i != last) ? '*' : gK.buf[i];
    vis[n] = 0;
  }
  d.setTextSize(2);
  const int per = 26;
  int len = (int)strlen(vis);
  int start = len > per * 3 ? len - per * 3 : 0;
  for (int r = 0; r < 3; r++) {
    int s0 = start + r * per;
    int n = s0 < len ? (len - s0 < per ? len - s0 : per) : 0;
    drawSentText(4, 14 + r * 20, 12, vis + (s0 < len ? s0 : len), n, sent - s0, per, gK.live ? C_SENT : C_FG, gK.live ? C_OK : C_FG);
  }
}

static void drawKeyboard() {
  auto& d = M5.Display;
  d.fillScreen(C_BG);
  drawKbdText();
  d.drawFastHLine(0, 76, 320, C_DIM);
  // keys
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 10; col++) {
      int code = keyAt(row, col);
      int x = col * kKbW + 1, y = kKbY0 + row * kKbRowH + 1;
      if (row == 3) {
        if (col == 0) drawKey(x, y, 2 * kKbW - 2, kKbRowH - 2, gK.sym ? "ABC" : "?123", C_KEY);
        else if (col == 2) drawKey(x, y, 5 * kKbW - 2, kKbRowH - 2, "SPACE", C_KEY);
        else if (col == 7) drawKey(x, y, 2 * kKbW - 2, kKbRowH - 2, gK.okLabel, C_OKD);
        else if (col == 9) drawKey(x, y, kKbW - 2, kKbRowH - 2, "X", C_WARN);
        continue;
      }
      if (code == 0) continue;
      char lab[2] = {0, 0};
      const char* label = lab;
      uint32_t bg = C_KEY;
      if (code == K_SHIFT) { label = "^"; bg = (gK.shift ? C_KEYHI : C_KEY); }
      else if (code == K_BKSP) { label = "<"; }
      else { lab[0] = caseOf((char)code); }
      drawKey(x, y, kKbW - 2, kKbRowH - 2, label, bg);
    }
  }
}

static void kbdFinish(bool ok) {
  if (!ok) {
    if (gK.live) txDropUnsent();            // live keyboard: what was sent stays sent, only the unsent text goes
    else { strncpy(gK.buf, gKbdBackup, gK.cap - 1); gK.buf[gK.cap - 1] = 0; }
  }
  Screen r = gK.ret;
  void (*cb)(bool) = gK.done;
  gScr = r;
  gFullRedraw = true;
  if (cb) cb(ok);
}

static void kbdTap(int x, int y) {
  if (y < kKbY0) return;
  int row = (y - kKbY0) / kKbRowH, col = x / kKbW;
  if (row > 3 || col > 9) return;
  int code = keyAt(row, col);
  if (code == 0) return;
  size_t len = strlen(gK.buf);
  switch (code) {
    case K_SHIFT: gK.shift = !gK.shift; drawKeyboard(); return;
    case K_SYM: gK.sym = !gK.sym; drawKeyboard(); return;
    case K_BKSP:
      if (gK.live) { txBackspace(); drawKbdText(); return; }
      if (len) gK.buf[len - 1] = 0;
      drawKeyboard();
      return;
    case K_SPACE: code = ' '; break;
    case K_ENTER: kbdFinish(true); return;
    case K_CANCEL: kbdFinish(false); return;
  }
  char typed = gK.upper ? (char)toupper(caseOf((char)code)) : caseOf((char)code);
  if (code == ' ') typed = ' ';
  if (gK.live) {
    txType(typed);
  } else if (len + 1 < gK.cap) {
    gK.buf[len] = typed;
    gK.buf[len + 1] = 0;
  }
  bool wasShift = gK.shift;
  if (gK.shift && !gK.upper) gK.shift = false;
  if (gK.live && wasShift == gK.shift) drawKbdText(); else drawKeyboard();
}

// ============================================================================
// Settings screen
// ============================================================================
static char gEdit[70];
static int gEditRow = -1;
// Settings navigation with the Bluetooth keyboard: items 0..kSetRows-1 are the rows, then the BT KBD / CONNECT / BACK buttons.
static int gSetSel = 0;
static bool gSetKeys = false;     // selection is drawn only after a key was used (touch users see no highlight)
static const int kSetRows = 14, kRowY0 = 14, kRowH = 14;   // 14 rows of 14 px; buttons below

static void setRowLabel(int r, char* label, char* value, size_t cap) {
  switch (r) {
    case 0: strcpy(label, "Wi-Fi SSID"); snprintf(value, cap, "%s", S.ssid); break;
    case 1: strcpy(label, "Wi-Fi password"); snprintf(value, cap, "%s", S.wpass[0] ? "********" : ""); break;
    case 2: strcpy(label, "Radio IP"); snprintf(value, cap, "%s", S.ip[0] ? S.ip : "(gateway)"); break;
    case 3: strcpy(label, "Radio user"); snprintf(value, cap, "%s", S.user); break;
    case 4: strcpy(label, "Radio password"); snprintf(value, cap, "%s", S.pass[0] ? "********" : ""); break;
    case 5: strcpy(label, "CI-V address (hex)"); snprintf(value, cap, "%s", S.civ); break;
    case 6: strcpy(label, "CW TX speed (wpm)"); snprintf(value, cap, "%d", S.wpm); break;
    case 7: strcpy(label, "PSK31/RTTY TX level (%)"); snprintf(value, cap, "%d", S.pskLevel); break;
    case 8: strcpy(label, "Allow TX"); snprintf(value, cap, "%s", S.allowTx ? "YES" : "no"); break;
    case 9: strcpy(label, "CW auto pitch"); snprintf(value, cap, "%s", S.autoPitch ? "on" : "off"); break;
    case 10: strcpy(label, "CW squelch (x0.1)"); snprintf(value, cap, "%d", S.cwGate); break;
    case 11: strcpy(label, "RTTY baud"); snprintf(value, cap, "%s", S.rttyBaud == 0 ? "45.45" : S.rttyBaud == 1 ? "50" : "75"); break;
    case 12: strcpy(label, "RTTY shift (Hz)"); snprintf(value, cap, "%d", S.rttyShift); break;
    case 13: strcpy(label, "RTTY mark tone"); snprintf(value, cap, "%s", S.rttyRev ? "high (normal)" : "low (reversed)"); break;
  }
}

static void drawSettings() {
  auto& d = M5.Display;
  d.fillScreen(C_BG);
  d.setTextSize(1);
  d.setTextColor(C_ACC, C_BG);
  d.drawString("SETTINGS", 4, 4);
  if (gSetKeys) {
    d.setTextColor(C_DIM, C_BG);
    d.drawString("Up/Down select   Enter change   Esc back", 70, 4);
  }
  char label[32], value[40];
  for (int r = 0; r < kSetRows; r++) {
    int y = kRowY0 + r * kRowH;
    setRowLabel(r, label, value, sizeof value);
    uint32_t bg = (gSetKeys && r == gSetSel) ? (uint32_t)d.color565(30, 55, 130) : (r & 1) ? C_BR : C_BG;
    if (bg != C_BG) d.fillRect(0, y, 320, kRowH, bg);
    d.setTextColor(C_FG, bg);
    d.drawString(label, 4, y + 3);
    d.setTextColor(C_ACC, bg);
    d.drawString(value, 150, y + 3);
  }
  static const char* const names[3] = {"BT KBD", "CONNECT", "BACK"};
  d.setTextSize(2);
  for (int i = 0; i < 3; i++) {
    int x = 4 + i * 106;
    bool sel = gSetKeys && gSetSel == kSetRows + i;
    uint32_t bg = sel ? C_KEYHI : C_BTN;
    d.fillRoundRect(x, 212, 100, 26, 5, bg);
    if (sel) { d.drawRoundRect(x, 212, 100, 26, 5, C_WARN); d.drawRoundRect(x + 1, 213, 98, 24, 4, C_WARN); }
    d.setTextColor(C_FG, bg);
    d.drawString(names[i], x + (100 - d.textWidth(names[i])) / 2, 217);
  }
}

static void settingEdited(bool ok) {
  if (ok) {
    switch (gEditRow) {
      case 0: strncpy(S.ssid, gEdit, sizeof S.ssid - 1); break;
      case 1: strncpy(S.wpass, gEdit, sizeof S.wpass - 1); break;
      case 2: strncpy(S.ip, gEdit, sizeof S.ip - 1); break;
      case 3: strncpy(S.user, gEdit, sizeof S.user - 1); break;
      case 4: strncpy(S.pass, gEdit, sizeof S.pass - 1); break;
      case 5: if (strlen(gEdit) > 0) { strncpy(S.civ, gEdit, 2); S.civ[2] = 0; } break;
      case 6: { int v = atoi(gEdit); if (v >= 5 && v <= 48) setCwSpeed(v); break; }
      case 7: { int v = atoi(gEdit); if (v >= 1 && v <= 80) S.pskLevel = v; break; }
      case 10: { int v = atoi(gEdit); if (v >= 15 && v <= 60) { S.cwGate = v; gCw.setSquelchRatio(v / 10.0f); } break; }
    }
    saveSettings();
  }
  gScr = SCR_SET;
}

// The three buttons below the rows: 0 = BT KBD, 1 = CONNECT, 2 = BACK (touch and keyboard both use these)
static void settingsButton(int i) {
  if (i == 0) { gScr = SCR_BT; gFullRedraw = true; return; }                          // BT KBD
  if (i == 2) { gScr = SCR_MAIN; gFullRedraw = true; saveSettings(); return; }        // BACK
  // CONNECT: (re)join Wi-Fi with the stored credentials and restart the radio link
  if (!S.ssid[0]) { notice("enter the Wi-Fi SSID first"); return; }
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);                       // Wi-Fi is left off until it is configured (see setup())
  WiFi.setSleep(S.btOn || blekbd::enabled()); // Wi-Fi power save must be on whenever Bluetooth is running
  WiFi.setAutoReconnect(true);
  WiFi.begin(S.ssid, S.wpass);
  gReconnect = true;
  gScr = SCR_MAIN;
  gFullRedraw = true;
  saveSettings();
  notice("connecting...");
}

// Change / edit settings row r: toggles and cycles act at once, the others open the on-screen keyboard.
static void settingsRow(int r) {
  if (r < 0 || r >= kSetRows) return;
  if (r == 11 || r == 12 || r == 13) {         // RTTY options: tap to cycle
    if (r == 11) S.rttyBaud = (S.rttyBaud + 1) % 3;
    else if (r == 12) { static const int sh[4] = {170, 200, 425, 850}; int i = 0; while (i < 3 && sh[i] != S.rttyShift) i++; S.rttyShift = sh[(i + 1) % 4]; }
    else S.rttyRev = !S.rttyRev;
    rttyApply();
    saveSettings();
    drawSettings();
    return;
  }
  if (r == 8) { S.allowTx = !S.allowTx; saveSettings(); drawSettings(); return; }
  if (r == 9) { S.autoPitch = !S.autoPitch; saveSettings(); drawSettings(); return; }
  char label[32], value[40];
  setRowLabel(r, label, value, sizeof value);
  gEditRow = r;
  switch (r) {
    case 0: strncpy(gEdit, S.ssid, sizeof gEdit - 1); break;
    case 1: strncpy(gEdit, S.wpass, sizeof gEdit - 1); break;
    case 2: strncpy(gEdit, S.ip, sizeof gEdit - 1); break;
    case 3: strncpy(gEdit, S.user, sizeof gEdit - 1); break;
    case 4: strncpy(gEdit, S.pass, sizeof gEdit - 1); break;
    case 5: strncpy(gEdit, S.civ, sizeof gEdit - 1); break;
    default: snprintf(gEdit, sizeof gEdit, "%s", value); break;
  }
  gEdit[sizeof gEdit - 1] = 0;
  size_t cap = (r == 0) ? 33 : (r == 1) ? 65 : (r == 2) ? 16 : (r == 5) ? 3 : (r >= 6) ? 4 : 17;
  openKeyboard(label, gEdit, cap, false, r == 1 || r == 4, SCR_SET, settingEdited);
}

static void settingsTap(int x, int y) {
  gSetKeys = false;                                  // touch: no keyboard selection highlight
  if (y >= 211) { settingsButton(x < 107 ? 0 : x >= 213 ? 2 : 1); return; }
  int r = (y - kRowY0) / kRowH;
  if (r < 0 || r >= kSetRows) return;
  gSetSel = r;
  settingsRow(r);
}

// ============================================================================
// Main screen
// ============================================================================
static const int kBtnY = 196, kBtnH = 42;
static const int kBtnCount = 5, kBtnPitch = 63, kBtnW = 59;   // 5-slot layout, still used by the Bluetooth screen
// Main screen buttons. Slot 0 = mode switch (label shows the current mode).
static const int kMainBtnCount = 5, kMainBtnPitch = 63, kMainBtnW = 59;
static const char* kMainBtnLabel[kMainBtnCount] = {"", "TYPE", "TUNE", "STOP", "MENU"};

static void drawButtons() {
  auto& d = M5.Display;
  for (int i = 0; i < kMainBtnCount; i++) {
    int x = 4 + i * kMainBtnPitch;
    uint32_t bg = (i == 3) ? d.color565(110, 30, 30) : (i == 0 ? d.color565(20, 70, 110) : C_BTN);
    if (i == 2 && (gTuning || gTuneWant)) bg = d.color565(170, 90, 0);   // TUNE lit while active
    d.fillRoundRect(x, kBtnY, kMainBtnW, kBtnH, 6, bg);
    d.setTextColor(C_FG, bg);
    d.setTextSize(2);
    const char* lab = (i == 0) ? (S.mode == 0 ? "CW" : S.mode == 1 ? "PSK" : "RTTY") : kMainBtnLabel[i];
    int tw = d.textWidth(lab);
    d.drawString(lab, x + (kMainBtnW - tw) / 2, kBtnY + 13);
  }
}

// The header is drawn into an off-screen sprite (in PSRAM, like the waterfall) and pushed in one go,
// so the top of the screen is never seen half-cleared.
static M5Canvas gHdrCanvas(&M5.Display);
static bool gHdrReady = false;

static void drawHeader() {
  if (!gHdrReady) {
    gHdrCanvas.setColorDepth(16);
    gHdrCanvas.setPsram(true);
    gHdrCanvas.createSprite(320, 35);
    gHdrReady = true;
  }
  auto& d = gHdrCanvas;
  d.fillRect(0, 0, 320, 35, C_BG);
  char b[48];
  uint32_t f = gRig.freq;
  snprintf(b, sizeof b, "%lu.%03lu.%03lu", (unsigned long)(f / 1000000), (unsigned long)((f / 1000) % 1000),
           (unsigned long)(f % 1000));
  bool tx = gRig.tx || gTxState == TX_ON;
  d.setTextSize(2);
  d.setTextColor(tx ? C_WARN : C_FG, C_BG);
  d.drawString(f ? b : "---.---.---", 4, 2);
  d.setTextColor(C_ACC, C_BG);
  d.drawString(gRig.mode >= 0 ? civ::modeName(gRig.mode) : "--", 134, 2);
  char s[12];
  civ::sText(gRig.sRaw, s, sizeof s);
  // While transmitting, the S-meter slot shows the radio's SWR (peak-held); "TX" until a reading above zero arrives.
  int swr = gSwrPeak;
  bool swrOk = tx && swr > 0 && millis() - gSwrPeakMs < 2000;
  uint32_t swrCol = swr < 80 ? C_OK : C_WARN;      // under SWR 2.0 green, otherwise amber
  int tend;   // x where the S / SWR / TX text ends: the bar starts just after it
  if (swrOk) {
    char sw[12];
    civ::swrText(swr, sw, sizeof sw);
    d.setTextSize(1);
    d.setTextColor(C_DIM, C_BG);
    d.drawString("SWR", 178, 2);
    d.setTextSize(2);
    d.setTextColor(swrCol, C_BG);
    d.drawString(sw, 202, 2);
    tend = 202 + d.textWidth(sw);
  } else {
    const char* str = tx ? "TX" : s;
    d.setTextColor(C_OK, C_BG);
    d.setTextSize(2);
    if (d.textWidth(str) > 38) d.setTextSize(1.5f);   // "S9+60" would otherwise run into the bar
    d.drawString(str, 200, 2);
    tend = 200 + d.textWidth(str);
  }
  // meter bar: S-meter on receive, SWR (full scale = 4.0) while a reading is available
  int bx = tend + 5;
  if (bx < 232) bx = 232;
  int bw = 284 - bx;                                  // bar ends at 284, the battery icon sits at 290..314
  if (bw < 24) bw = 24;
  int inner = bw - 2;
  int w = swrOk ? (swr > 220 ? inner : (swr * inner) / 220) : (gRig.sRaw < 0 ? 0 : (gRig.sRaw * inner) / 241);
  d.drawRect(bx, 4, bw, 10, C_DIM);
  d.fillRect(bx + 1, 5, w, 8, swrOk ? swrCol : (tx ? C_WARN : C_OK));
  // battery icon (top right): level fill, green / amber / red, cyan while charging
  {
    static uint32_t lastBat = 0;
    static int lvl = -1;
    static bool charging = false;
    if (lastBat == 0 || millis() - lastBat > 3000) {
      lastBat = millis() | 1;
      lvl = M5.Power.getBatteryLevel();
      charging = (M5.Power.isCharging() == m5::Power_Class::is_charging);
    }
    if (lvl >= 0) {
      if (lvl > 100) lvl = 100;
      uint32_t bc = charging ? C_ACC : lvl <= 15 ? (uint32_t)d.color565(220, 40, 40) : lvl <= 30 ? C_WARN : C_OK;
      d.drawRect(290, 3, 22, 11, C_DIM);
      d.fillRect(312, 6, 2, 5, C_DIM);
      d.fillRect(291, 4, 20, 9, C_BG);
      d.fillRect(292, 5, (lvl * 18 + 50) / 100, 7, bc);
    }
  }
  // second line
  d.setTextSize(1);
  d.setTextColor(C_DIM, C_BG);
  if (gNoticeUntil && millis() < gNoticeUntil) {
    d.setTextColor(C_WARN, C_BG);
    d.drawString(gNotice, 4, 22);
  } else {
    if (S.mode == 0) {
      snprintf(b, sizeof b, "CW %.0fHz %.0fwpm BW%.0f x%.1f%s %s", gCw.freq(), gCw.wpm(), gCw.bandwidthHz(),
               gCw.signalToNoise(), S.autoPitch ? " AUTO" : "", gCw.signalValid() ? "SIG" : "");
    } else if (S.mode == 1) {
      snprintf(b, sizeof b, "PSK31 %.1fHz Q%.2f %s", gPsk.freq(), gPsk.quality(), gPsk.squelchOpen() ? "LOCK" : "sq");
    } else {
      snprintf(b, sizeof b, "RTTY %.0fHz %d %s Q%.2f %s", gRtty.freq(), S.rttyShift, S.rttyBaud == 0 ? "45" : S.rttyBaud == 1 ? "50" : "75",
               gRtty.quality(), gRtty.squelchOpen() ? "LOCK" : "sq");
    }
    d.drawString(b, 4, 22);
    const char* st = WiFi.status() != WL_CONNECTED ? "WiFi down" : gLink.running() ? "linked" : gLink.stateName();
    uint32_t col = gLink.running() ? C_OK : C_WARN;
    d.setTextColor(col, C_BG);
    d.drawString(st, 316 - d.textWidth(st), 22);
    if (blekbd::enabled()) {
      d.setTextColor(blekbd::ready() ? C_OK : C_DIM, C_BG);
      d.drawString("KB", 240, 22);
    }
    if (gLink.state() == icom::Link::kFailed) {
      d.setTextColor(C_WARN, C_BG);
      d.drawString(gLink.error(), 150, 22);
    }
  }
  gHdrCanvas.pushSprite(0, 0);
}

static void drawRx() {
  auto& d = M5.Display;
  // No clear-then-redraw (that flickers): every line is padded with spaces and drawn with an opaque
  // background, so each character cell is overwritten in one step.
  d.setTextSize(2);
  d.setTextColor(C_FG, C_BG);
  char line[kRxCols + 1];
  for (int r = 0; r < kRxRows; r++) {
    int n = (int)strlen(gRxLines[r]);
    memcpy(line, gRxLines[r], n);
    memset(line + n, ' ', kRxCols - n);
    line[kRxCols] = 0;
    d.drawString(line, 4, kRxTop + r * kRxLineH);
  }
  gRxDirty = false;
}

static void drawTxLine() {
  auto& d = M5.Display;
  d.setTextSize(1);
  d.setTextColor(C_WARN, C_BG);
  d.drawString("TX>", 4, 180);
  char snap[121];
  int sent;
  txSnapshot(snap, sizeof snap, &sent);
  int len = (int)strlen(snap);
  int start = len > 48 ? len - 48 : 0;      // tail of the line; orange = sent (locked), green = not sent yet
  drawSentText(26, 180, 6, snap + start, len - start, sent - start, 48, C_SENT, C_OK);
}

static void drawMain() {
  M5.Display.fillScreen(C_BG);
  drawHeader();
  drawRx();
  gWfDirty = true;
  drawWaterfall();
  drawTxLine();
  drawButtons();
  gFullRedraw = false;
}

// Switch between CW and PSK31 (receive decoder, TX path and radio mode all follow).
static void toggleMode() {
  if (txBusy()) { notice("stop TX before switching mode"); return; }
  LOCK(); gTxBuf[0] = 0; gTxSent = 0; gTxRev++; UNLOCK();     // the line was typed for the other mode
  setAppMode((S.mode + 1) % 3);
  const char* nm = S.mode == 0 ? "CW" : S.mode == 1 ? "PSK31" : "RTTY";
  char b[40];
  if (S.allowTx) snprintf(b, sizeof b, "mode: %s", nm);
  else snprintf(b, sizeof b, "mode: %s (TX not armed: MENU)", nm);
  notice(b);
}

static void typeDone(bool ok);

// TYPE: open the touch keyboard. It types into the live transmit line: each key is sent at once, orange = sent
// (locked), green = not sent yet (backspace works on these). DONE goes back and leaves everything as it is;
// X goes back and drops the text that has not been sent.
static void doType() {
  openKeyboard(S.mode == 0 ? "CW: sends as you type. Orange = sent. X drops unsent" : S.mode == 1 ? "PSK31: sends as you type. Orange = sent. X drops unsent" : "RTTY: sends as you type. Orange = sent. X drops unsent",
               gTxBuf, sizeof gTxBuf, S.mode != 1, false, SCR_MAIN, typeDone);
  gK.okLabel = "DONE";
  gK.live = true;
}

static void doTune() {
  if (gTuning || gTuneWant) { gTuneWant = false; notice("tune off"); drawButtons(); return; }   // second tap stops
  if (!gLink.running()) { notice("radio not connected"); return; }
  if (gTxState == TX_ON || gTxReq || gCwActive) { notice("busy: STOP first"); return; }
  if (!S.allowTx) { notice("TX is off: enable in MENU"); return; }
  if (S.mode != 0 && kAudioRate != 8000) { notice("TUNE needs kAudioRate = 8000"); return; }
  gTuneAudio = (S.mode != 0);
  gTuneHz = (S.mode == 2) ? rttyMarkHz() : 1500.0f;
  gTuneWant = true;
  notice(S.mode == 0 ? "TUNE: carrier, tap again to stop" : S.mode == 1 ? "TUNE: 1500 Hz, tap again to stop" : "TUNE: mark tone, tap again to stop");
  drawButtons();
}

static void doStop() {
  uint8_t f[16];
  gTuneWant = false;
  txDropUnsent();
  if (gCwActive || S.mode == 0) civSend(f, civ::cwStop(f, civAddr()));
  gCwBusyUntil = 0;
  if (gTxState == TX_ON || gTxReq) gTxAbort = true;
  notice("stopped");
}

// TYPE keyboard closed (DONE or X; kbdFinish has already dropped the unsent text for X).
static void typeDone(bool ok) {
  (void)ok;
  gFullRedraw = true;
}

static void mainTap(int x, int y) {
  if (y >= kBtnY) {
    int i = (x - 4) / kMainBtnPitch;
    if (i < 0 || i >= kMainBtnCount) return;
    switch (i) {
      case 0: toggleMode(); break;
      case 1: doType(); break;
      case 2: doTune(); break;
      case 3: doStop(); break;
      case 4: gScr = SCR_SET; gFullRedraw = true; break;
    }
    return;
  }
  if (y >= kWfTop && y < kWfTop + kWfRows) {   // tap waterfall = set tone / carrier
    int hz = (int)(x * kWfMaxHz / 320.0f);
    if (S.mode == 0) { S.pitch = hz; gCw.setFreq((float)hz); S.autoPitch = false; }
    else if (S.mode == 1) { S.pskHz = hz; gPsk.setFreq((float)hz); }
    else { S.rttyHz = hz; gRtty.setFreq((float)hz); }
    gWfDirty = true;
  }
}

// ============================================================================
// Audio -> DSP (UI task)
// ============================================================================
static uint32_t gLastWf = 0, gLastHdr = 0, gLastStat = 0, gLastAud = 0;

static void processAudio() {
  int16_t in[256], out[256];
  int guard = 0;
  while (gRxR != gRxW && guard++ < 16) {
    size_t n = 0;
    while (gRxR != gRxW && n < 256) {
      in[n++] = gRx[gRxR];
      gRxR = (gRxR + 1) & (kRxRing - 1);
    }
    size_t m = gDec.process(in, n, out);
    if (!m) continue;
    bool txing = (gTxState == TX_ON);
    if (!txing) {
      if (S.mode == 0) gCw.process(out, m);
      else if (S.mode == 1) gPsk.process(out, m);
      else gRtty.process(out, m);
    }
    if (gSpec.push(out, m)) {
      wfAddRow(gSpec.db());
      if (S.mode == 0 && S.autoPitch) {
        // Coarse acquisition only: the decoder's own tone tracker handles drift and small errors.
        // Re-tune only if a clear peak sits well away from the current tone for ~0.7 s, and never while
        // the decoder is locked on a signal.
        float mag;
        float hz = gSpec.peakHz(350, 1100, &mag);
        gLastPeakHz = hz;
        gLastPeakDb = mag;
        if (!gCw.signalValid() && mag > gWfLo + 12.0f && fabsf(hz - gCw.freq()) > 12.0f) {
          if (++gPitchVotes >= 10) { gCw.setFreq(hz); S.pitch = (int)hz; gPitchVotes = 0; }
        } else {
          gPitchVotes = 0;
        }
      }
    }
  }
  char c;
  if (S.mode == 0) { while (gCw.getChar(c)) rxPut(c); }
  else if (S.mode == 1) { while (gPsk.getChar(c)) rxPut(c); }
  else { while (gRtty.getChar(c)) rxPut(c); }
}

// ============================================================================
// Arduino entry points
// ============================================================================

// ============================================================================
// Bluetooth keyboard: screen and key handling
// ============================================================================
static uint32_t gLastBt = 0;
static bool gBtCrashed = false;   // the previous Bluetooth start-up never finished (crash): it is switched off
static char gBtSig[200] = "";

static void btLines(char* l1, size_t c1, char* l2, size_t c2) {
  l2[0] = 0;
  if (!blekbd::available()) { snprintf(l1, c1, "Library not installed"); snprintf(l2, c2, "Install NimBLE-Arduino (Library Manager), rebuild"); return; }
  // blekbd::enabled() changes the moment Bluetooth is turned on/off, while state() follows a few tens of ms later
  // from the worker task, so on/off is taken from enabled() and state() is only used while Bluetooth is on.
  if (!blekbd::enabled()) {
    snprintf(l1, c1, "Bluetooth is OFF");
    snprintf(l2, c2, "%s", gBtCrashed ? "Bluetooth crashed at start-up: see serial log" : "Tap TURN ON to use a keyboard");
    return;
  }
  switch (blekbd::state()) {
    case blekbd::kOff:        // only for a moment after turning on: show what the idle state will show
    case blekbd::kIdle:       snprintf(l1, c1, blekbd::bondCount() > 0 ? "Paired keyboard saved" : "No keyboard paired");
                      snprintf(l2, c2, "%s", blekbd::message()[0] ? blekbd::message()
                                                     : blekbd::bondCount() > 0 ? "Press a key on the paired keyboard" : "Tap PAIR to add a keyboard"); break;
    case blekbd::kScanning:   snprintf(l1, c1, blekbd::pairSearching() ? "Searching..." : "Waiting for keyboard");
                      snprintf(l2, c2, "%s", blekbd::pairSearching() ? "Radio link paused. Put keyboard in pairing mode" : "Press a key on the paired keyboard"); break;
    case blekbd::kConnecting: snprintf(l1, c1, "Connecting..."); break;
    case blekbd::kPairing:    snprintf(l1, c1, "Type this on the keyboard:"); snprintf(l2, c2, "then press Enter"); break;
    case blekbd::kReady:      snprintf(l1, c1, "Connected: %.14s", blekbd::name()); snprintf(l2, c2, "Type on the keyboard"); break;
  }
}

static void btSig(char* out, size_t cap) {
  snprintf(out, cap, "%d|%d|%d|%lu|%s|%s|%d|%d", (int)blekbd::enabled() * 10 + (int)blekbd::state(), (int)blekbd::pairSearching(), (int)blekbd::passkeyShown(),
           (unsigned long)blekbd::passkey(), blekbd::message(), blekbd::name(), blekbd::bondCount(), (int)blekbd::ukLayout());
}

static void drawBt() {
  auto& d = M5.Display;
  char l1[48], l2[64];
  // Take the change-detection signature BEFORE reading the state for the text: the Bluetooth task keeps changing
  // state while this (slow) redraw runs, and a signature taken afterwards would hide that change, leaving an
  // old text such as "Starting Bluetooth..." on screen for good.
  btSig(gBtSig, sizeof gBtSig);
  btLines(l1, sizeof l1, l2, sizeof l2);
  d.fillScreen(C_BG);
  d.setTextSize(2);
  d.setTextColor(C_ACC, C_BG);
  d.drawString("BLUETOOTH KEYBOARD", 4, 4);
  d.setTextColor(blekbd::enabled() && blekbd::state() == blekbd::kReady ? C_OK : C_FG, C_BG);
  d.drawString(l1, 4, 36);
  d.setTextSize(1);
  d.setTextColor(C_DIM, C_BG);
  d.drawString(l2, 4, 62);
  if (blekbd::enabled() && blekbd::state() == blekbd::kPairing && blekbd::passkeyShown()) {
    char pk[12];
    snprintf(pk, sizeof pk, "%06lu", (unsigned long)blekbd::passkey());
    d.setTextSize(5);
    d.setTextColor(C_WARN, C_BG);
    d.drawString(pk, (320 - d.textWidth(pk)) / 2, 88);
  }
  d.setTextSize(1);
  d.setTextColor(C_DIM, C_BG);
  char info[64];
  snprintf(info, sizeof info, "Layout: %s   Paired keyboards: %d", blekbd::ukLayout() ? "UK" : "US", blekbd::bondCount());
  d.drawString(info, 4, 150);
  d.drawString("Typing sends live   Esc=STOP/clear   Tab=CW/PSK", 4, 164);
  d.drawString("BLE keyboards only (no Classic / USB dongle)", 4, 178);
  // buttons
  // The first button is labelled with what tapping it DOES (TURN ON / TURN OFF); the state is the text above,
  // and the button is green while Bluetooth is on.
  bool en = blekbd::enabled();
  bool busy = blekbd::pairSearching() || blekbd::state() == blekbd::kPairing || blekbd::state() == blekbd::kConnecting;
  const char* lab[5] = {en ? "TURN OFF" : "TURN ON", busy ? "CANCEL" : "PAIR", blekbd::ukLayout() ? "UK" : "US", "FORGET", "BACK"};
  for (int i = 0; i < 5; i++) {
    int x = 4 + i * 63;
    uint32_t bg = (i == 0 && en) ? d.color565(20, 90, 40) : C_BTN;
    d.fillRoundRect(x, kBtnY, kBtnW, kBtnH, 6, bg);
    d.setTextColor(C_FG, bg);
    d.setTextSize(i == 0 ? 1.0f : 1.5f);   // "TURN OFF" needs the smaller size to fit
    d.drawString(lab[i], x + (kBtnW - d.textWidth(lab[i])) / 2, kBtnY + (i == 0 ? 17 : 15));
  }
}

// Start/stop Bluetooth with a crash guard: the flag stays set if the start-up never returns (reset/abort),
// so the next boot leaves Bluetooth off instead of bootlooping.
static void btSetEnabled(bool on) {
  if (on) {
    WiFi.setSleep(true);               // required before the Bluetooth controller starts (and stays on until reboot)
    gPrefs.putBool("btcrash", true);
    blekbd::enable(true);
    gPrefs.putBool("btcrash", false);
    gBtCrashed = false;
    if (!blekbd::enabled()) { S.btOn = false; saveSettings(); }   // init failed cleanly
  } else {
    blekbd::enable(false);
  }
}

static void btTap(int x, int y) {
  if (y < kBtnY) return;
  int i = (x - 4) / kBtnPitch;
  switch (i) {
    case 0: S.btOn = !blekbd::enabled(); btSetEnabled(S.btOn); saveSettings(); break;
    case 1:
      if (!blekbd::enabled()) { notice("turn Bluetooth on first"); break; }
      if (blekbd::pairSearching() || blekbd::state() == blekbd::kPairing || blekbd::state() == blekbd::kConnecting) blekbd::cancel();
      else if (gTxState == TX_ON || gTxReq || gCwActive) notice("stop TX before pairing");
      else blekbd::pair();
      break;
    case 2: S.btUk = !S.btUk; blekbd::setUkLayout(S.btUk); saveSettings(); break;
    case 3: blekbd::forget(); break;
    case 4: gScr = SCR_SET; gFullRedraw = true; return;
    default: return;
  }
  gFullRedraw = true;
}

static const int kNudgeHz = 30;   // left/right arrow step (Hz)

// Left/right arrow on the Bluetooth keyboard: move the CW pitch / PSK31 carrier by kNudgeHz.
static void nudgeFreq(int d) {
  d *= kNudgeHz;
  int lo = 200, hi = (int)kWfMaxHz - 50;
  char b[32];
  if (S.mode == 0) {
    S.pitch = constrain(S.pitch + d, lo, hi);
    gCw.setFreq((float)S.pitch);
    S.autoPitch = false;                    // a manual tuning choice switches auto-pitch off, as a waterfall tap does
    snprintf(b, sizeof b, "CW tone %d Hz", S.pitch);
  } else if (S.mode == 1) {
    S.pskHz = constrain(S.pskHz + d, lo, hi);
    gPsk.setFreq((float)S.pskHz);
    snprintf(b, sizeof b, "PSK carrier %d Hz", S.pskHz);
  } else {
    S.rttyHz = constrain(S.rttyHz + d, lo + S.rttyShift / 2, hi - S.rttyShift / 2);
    gRtty.setFreq((float)S.rttyHz);
    snprintf(b, sizeof b, "RTTY centre %d Hz", S.rttyHz);
  }
  notice(b);
}

// A key from the Bluetooth keyboard. Printable ASCII, '\r' enter, '\b' backspace, 27 esc, '\t' tab,
// 0x80 / 0x81 left / right arrow, 0x85 / 0x86 up / down arrow, 0x82 Ctrl+Tab (TYPE), 0x83 Ctrl+T (TUNE), 0x84 Ctrl+M (menu).
static void handleHwKey(uint8_t k) {
  if (gScr == SCR_MAIN) {
    if (k == 0x80 || k == 0x81) { nudgeFreq(k == 0x81 ? 1 : -1); return; }
    if (k == 0x85 || k == 0x86) {                                     // up / down arrow: CW send speed +-1 wpm
      if (S.mode == 0) {
        setCwSpeed(S.wpm + (k == 0x85 ? 1 : -1));
        char b[32];
        snprintf(b, sizeof b, "CW speed %d wpm", S.wpm);
        notice(b);
      }
      return;
    }
    if (k == 0x82) { doType(); return; }                              // Ctrl+Tab
    if (k == 0x83) { doTune(); return; }                              // Ctrl+T
    if (k == 0x84) { gScr = SCR_SET; gFullRedraw = true; return; }    // Ctrl+M
    if (k == '\r') { txType('\n'); return; }                          // PSK31: line break, CW: word space
    if (k == '\t') { toggleMode(); return; }
    if (k == 27) {
      if (txBusy()) doStop();                                         // transmitting: Esc = STOP
      else txDropUnsent();                                            // idle: clears the line
      drawTxLine();
      return;
    }
    if (k == '\b') { txBackspace(); return; }                          // only text that has not been sent yet
    if (k < 32 || k > 126) return;
    txType((char)k);
  } else if (gScr == SCR_KBD) {              // typing into an on-screen keyboard field (settings, TYPE)
    size_t len = strlen(gK.buf);
    if (gK.live) {                           // TYPE screen: live transmit line
      if (k == '\r') { txType('\n'); drawKbdText(); return; }
      if (k == 27) { kbdFinish(true); return; }
      if (k == '\b') { txBackspace(); drawKbdText(); return; }
      if (k < 32 || k > 126) return;
      txType(gK.upper ? (char)toupper(k) : (char)k);
      drawKbdText();
      return;
    }
    if (k == '\r') { kbdFinish(true); return; }
    if (k == 27) { kbdFinish(false); return; }
    if (k == '\b') { if (len) gK.buf[len - 1] = 0; drawKeyboard(); return; }
    if (k < 32 || k > 126) return;
    if (len + 1 < gK.cap) { gK.buf[len] = gK.upper ? (char)toupper(k) : (char)k; gK.buf[len + 1] = 0; }
    drawKeyboard();
  } else if (gScr == SCR_SET || gScr == SCR_BT) {
    if (k == 0x84 && gScr == SCR_SET) { gScr = SCR_MAIN; saveSettings(); gFullRedraw = true; return; }   // Ctrl+M again closes the menu
    if (gScr == SCR_SET) {                                              // Bluetooth keyboard navigation of the settings
      const int items = kSetRows + 3;
      if (k == 0x85 || k == 0x86) {                                     // up / down: previous / next item (wraps)
        gSetKeys = true;
        gSetSel = (gSetSel + (k == 0x86 ? 1 : items - 1)) % items;
        drawSettings();
        return;
      }
      if (k == 0x80 || k == 0x81) {                                     // left / right: move between the buttons
        if (gSetSel >= kSetRows) { gSetKeys = true; gSetSel = kSetRows + constrain(gSetSel - kSetRows + (k == 0x81 ? 1 : -1), 0, 2); drawSettings(); }
        return;
      }
      if (k == '\r') {                                                  // Enter: change the selected value / press the button
        gSetKeys = true;
        if (gSetSel < kSetRows) settingsRow(gSetSel); else settingsButton(gSetSel - kSetRows);
        return;
      }
    }
    if (k == 27) { gScr = (gScr == SCR_BT) ? SCR_SET : SCR_MAIN; if (gScr == SCR_MAIN) saveSettings(); gFullRedraw = true; }
  }
}

// The big buffers are allocated here, in setup(), not during static initialisation: at that point PSRAM is
// not yet available to the heap, so they used to land in the (small) internal RAM and nearly filled it.
static void allocBigBuffers() {
  gLinkP = new (bigAlloc(sizeof(icom::Link))) icom::Link();
  gRx = (int16_t*)bigAlloc(kRxRing * sizeof(int16_t));
  gSpecP = new (bigAlloc(sizeof(dsp::Spectrum))) dsp::Spectrum();
  gWf = (uint8_t (*)[320])bigAlloc(kWfRows * 320);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);
  Serial.printf("\n[boot] psram %s\n", psramFound() ? "found" : "NOT FOUND (enable Tools > PSRAM)");
  memLog("boot");
  allocBigBuffers();
  memLog("buffers");
  M5.Display.setRotation(1);
  initColors();
  M5.Display.fillScreen(C_BG);
  M5.Display.setTextColor(C_FG, C_BG);
  M5.Display.setTextSize(2);
  M5.Display.drawString("IC-705 CW/PSK31", 40, 100);

  loadSettings();
  memLog("settings");
  gCivQ = xQueueCreate(24, sizeof(CivCmd));
  gMux = xSemaphoreCreateMutex();
  gCw.setFreq((float)S.pitch);
  gPsk.setFreq((float)S.pskHz);
  gPsk.setSquelch(0.80f);
  rttyApply();
  gRtty.setSquelch(0.50f);
  gCw.setSquelchRatio(S.cwGate / 10.0f);
  rxClear();

  // Wi-Fi power save adds UDP latency, so it is off - EXCEPT when the Bluetooth keyboard is enabled: the ESP32
  // Wi-Fi/Bluetooth coexistence code aborts if Bluetooth is started while Wi-Fi power save is off.
  // With no Wi-Fi configured yet the Wi-Fi radio is switched off completely, so a Bluetooth keyboard can be paired
  // first on its own; CONNECT in the settings starts Wi-Fi later.
  if (S.ssid[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(S.btOn);
    WiFi.setAutoReconnect(true);
    WiFi.begin(S.ssid, S.wpass);
  } else {
    WiFi.mode(WIFI_OFF);
    WiFi.setSleep(true);                       // remembered for when Wi-Fi is started later
    gScr = SCR_SET;
  }
  memLog("wifi");

  xTaskCreatePinnedToCore(netTask, "net", 10240, nullptr, 2, nullptr, 0);
  blekbd::setUkLayout(S.btUk);
  memLog("tasks");
  if (S.btOn) {
    if (gPrefs.getBool("btcrash", false)) {   // last start-up never finished: do not bootloop
      S.btOn = false; saveSettings(); gBtCrashed = true;
      gPrefs.putBool("btcrash", false);
      Serial.println("[ble] previous Bluetooth start-up crashed: Bluetooth keyboard switched off");
    } else {
      btSetEnabled(true);                      // starts the Bluetooth stack and reconnects to a paired keyboard
    }
  }
  memLog("ready");
  gFullRedraw = true;
}

void loop() {
  M5.update();
  uint32_t now = millis();

  // virtual buttons below the screen
  if (M5.BtnA.wasPressed() && gScr == SCR_MAIN) { toggleMode(); }
  if (M5.BtnB.wasPressed() && gScr == SCR_MAIN) { rxClear(); }
  if (M5.BtnC.wasPressed() && gScr == SCR_MAIN) { gScr = SCR_SET; gFullRedraw = true; }

  auto t = M5.Touch.getDetail();
  if (t.wasPressed()) {
    switch (gScr) {
      case SCR_MAIN: mainTap(t.x, t.y); break;
      case SCR_KBD: kbdTap(t.x, t.y); break;
      case SCR_SET: settingsTap(t.x, t.y); break;
      case SCR_BT: btTap(t.x, t.y); break;
    }
  }

  uint8_t hk;
  while (blekbd::getKey(hk)) handleHwKey(hk);

  processAudio();

  if (gFullRedraw) {
    switch (gScr) {
      case SCR_MAIN: drawMain(); break;
      case SCR_KBD: drawKeyboard(); gFullRedraw = false; break;
      case SCR_SET: drawSettings(); gFullRedraw = false; break;
      case SCR_BT: drawBt(); gFullRedraw = false; break;
    }
  } else if (gScr == SCR_BT) {
    if (now - gLastBt > 300) {
      gLastBt = now;
      char sig[200];
      btSig(sig, sizeof sig);
      if (strcmp(sig, gBtSig) != 0) drawBt();
    }
  } else if (gScr == SCR_KBD && gK.live) {       // the net task marks characters as sent: refresh the text field
    static uint32_t lastRev = 0;
    static bool lastNote = false;
    bool nn = (int32_t)(gNoticeUntil - now) > 0;
    if (gTxRev != lastRev || nn != lastNote) { lastRev = gTxRev; lastNote = nn; drawKbdText(); }
  } else if (gScr == SCR_MAIN) {
    {
      static uint32_t lastRev = 0;
      if (gTxRev != lastRev) { lastRev = gTxRev; drawTxLine(); }
    }
    if (kTrace && S.mode == 0 && now - gLastStat > 1000) {
      gLastStat = now;
      Serial.printf("[cw] f=%.1f bw=%.0f wpm=%.1f noise=%.1f peak=%.1f keydown/noise=%.2f %s\n", gCw.freq(),
                    gCw.bandwidthHz(), gCw.wpm(), gCw.noiseLevel(), gCw.peakLevel(), gCw.signalToNoise(),
                    gCw.signalValid() ? "OPEN" : "closed");
    }
    if (kTrace && now - gLastAud > 2000) {      // audio-stream health, every 2 s, in every mode
      gLastAud = now;
      Serial.printf("[audio] pkts=%u lost=%u recovered=%u reqs=%u bypass=%u late=%u maxgap=%ums age=%ums ringdrop=%u bt=%s wifi=%ddBm\n",
                    (unsigned)gLink.audioPackets(), (unsigned)gLink.audioLost(), (unsigned)gLink.audioRecovered(),
                    (unsigned)gLink.audioRetransmitRequests(), (unsigned)gLink.audioBypassed(), (unsigned)gLink.audioLate(), (unsigned)gLink.takeAudioMaxGapMs(), (unsigned)gLink.audioAgeMs(), (unsigned)gRxDrop,
                    blekbd::state() == blekbd::kReady ? "ready" : "off/idle", (int)WiFi.RSSI());
      if (S.mode == 2) {
        Serial.printf("[rtty] f=%.1f off=%+.1f Q=%.2f %s\n", gRtty.freq(), gRtty.tuningOffset(), gRtty.quality(), gRtty.squelchOpen() ? "LOCK" : "sq");
      } else if (S.mode == 1) {
        char bits[65];
        gPsk.recentBits(bits, 64);            // last 64 bit decisions (about 2 s): lets a wrong decode be analysed offline
        Serial.printf("[psk] f=%.1f off=%+.1f Q=%.2f %s clk=%d bits=%s\n", gPsk.freq(), gPsk.tuningOffset(), gPsk.quality(),
                      gPsk.squelchOpen() ? "LOCK" : "sq", gPsk.clockPhase(), bits);
      }
    }
    static bool lastTune = false;
    if ((gTuning || gTuneWant) != lastTune) { lastTune = gTuning || gTuneWant; drawButtons(); }
    if (now - gLastHdr > 200) { gLastHdr = now; drawHeader(); }
    if (gRxDirty && now - gLastWf > 50) drawRx();
    if (gWfDirty && now - gLastWf > 100) { gLastWf = now; drawWaterfall(); }
    if (gNoticeUntil && now > gNoticeUntil) { gNoticeUntil = 0; }
  }
  delay(2);
}
