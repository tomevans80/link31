// civ.h - Icom CI-V frame helpers for the IC-705 (portable, header-only).
// Frame: FE FE <to> <from> <cmd> [<sub>] [data...] FD
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

namespace civ {

static const uint8_t kRadio = 0xA4;  // IC-705 default CI-V address
static const uint8_t kCtrl  = 0xE0;  // controller address used by RS-BA1 / wfview / kappanhang

struct Rig {
  uint32_t freq = 0;      // Hz
  int mode = -1;          // CI-V mode code
  int filter = 0;
  bool dataMode = false;
  bool tx = false;
  int sRaw = -1;          // 0..255 (0=S0, 120=S9, 241=S9+60dB)
  int swrRaw = -1;        // 0..255 SWR meter while transmitting (0=1.0, 48=1.5, 80=2.0, 120=3.0, 220=4.0)
  uint32_t lastFreqMs = 0, lastSMs = 0, lastSwrMs = 0;
};

enum Event { kNone = 0, kAck, kNak, kFreq, kMode, kSMeter, kTx, kDataMode, kSwr };

// --- BCD helpers -----------------------------------------------------------------
static inline void freqToBcd(uint32_t f, uint8_t out[5]) {
  for (int i = 0; i < 5; i++) {
    uint8_t lo = f % 10; f /= 10;
    uint8_t hi = f % 10; f /= 10;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
}
static inline uint32_t bcdToFreq(const uint8_t* p, size_t n) {
  uint32_t f = 0, mul = 1;
  for (size_t i = 0; i < n && i < 5; i++) {
    f += (p[i] & 0x0F) * mul; mul *= 10;
    f += (p[i] >> 4) * mul; mul *= 10;
  }
  return f;
}
static inline int bcd2(const uint8_t* p) {  // two bytes -> 0..9999
  return (p[0] >> 4) * 1000 + (p[0] & 15) * 100 + (p[1] >> 4) * 10 + (p[1] & 15);
}
static inline void toBcd2(int v, uint8_t out[2]) {
  if (v < 0) v = 0;
  if (v > 9999) v = 9999;
  out[0] = (uint8_t)(((v / 1000) << 4) | ((v / 100) % 10));
  out[1] = (uint8_t)((((v / 10) % 10) << 4) | (v % 10));
}

// --- frame building ----------------------------------------------------------------
static inline size_t frame(uint8_t* out, const uint8_t* body, size_t n, uint8_t radio = kRadio) {
  out[0] = 0xFE; out[1] = 0xFE; out[2] = radio; out[3] = kCtrl;
  memcpy(out + 4, body, n);
  out[4 + n] = 0xFD;
  return n + 5;
}
static inline size_t readFreq(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x03}; return frame(o, b, 1, r); }
static inline size_t readMode(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x04}; return frame(o, b, 1, r); }
static inline size_t readS(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x15, 0x02}; return frame(o, b, 2, r); }
static inline size_t readSwr(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x15, 0x12}; return frame(o, b, 2, r); }
static inline size_t readTx(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x1C, 0x00}; return frame(o, b, 2, r); }
static inline size_t readDataMode(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x1A, 0x06}; return frame(o, b, 2, r); }
static inline size_t setFreq(uint8_t* o, uint32_t hz, uint8_t r = kRadio) {
  uint8_t b[6]; b[0] = 0x05; freqToBcd(hz, b + 1); return frame(o, b, 6, r);
}
static inline size_t setMode(uint8_t* o, uint8_t mode, uint8_t filt, uint8_t r = kRadio) {
  const uint8_t b[] = {0x06, mode, filt}; return frame(o, b, 3, r);
}
static inline size_t setDataMode(uint8_t* o, bool on, uint8_t filt, uint8_t r = kRadio) {
  const uint8_t b[] = {0x1A, 0x06, (uint8_t)(on ? 1 : 0), filt}; return frame(o, b, 4, r);
}
static inline size_t ptt(uint8_t* o, bool on, uint8_t r = kRadio) {
  const uint8_t b[] = {0x1C, 0x00, (uint8_t)(on ? 1 : 0)}; return frame(o, b, 3, r);
}
// CW keyer speed: 0000..0255 maps to 6..48 WPM.
static inline size_t keySpeed(uint8_t* o, int wpm, uint8_t r = kRadio) {
  if (wpm < 6) wpm = 6;
  if (wpm > 48) wpm = 48;
  int v = ((wpm - 6) * 255 + 21) / 42;
  uint8_t b[4] = {0x14, 0x0C, 0, 0};
  toBcd2(v, b + 2);
  return frame(o, b, 4, r);
}
// Send CW text with the radio's own keyer (max 30 characters per frame).
static inline size_t cwText(uint8_t* o, const char* s, size_t n, uint8_t r = kRadio) {
  uint8_t b[32]; b[0] = 0x17;
  if (n > 30) n = 30;
  memcpy(b + 1, s, n);
  return frame(o, b, n + 1, r);
}
static inline size_t cwStop(uint8_t* o, uint8_t r = kRadio) { const uint8_t b[] = {0x17, 0xFF}; return frame(o, b, 2, r); }

// Characters the IC-705 keyer accepts in a CW message (upper-case only).
static inline char cwSanitize(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
  if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) return c;
  switch (c) {
    case ' ': case '/': case '?': case '.': case ',': case ':': case '\'':
    case '(': case ')': case '=': case '+': case '-': case '@':
      return c;
    case '\n': case '\r': case '\t': return ' ';
  }
  return 0;  // drop
}

// --- parsing -------------------------------------------------------------------------
// Returns the event type and updates rig. Ignores frames not addressed to us.
static inline Event parse(const uint8_t* f, size_t n, Rig& rig, uint32_t nowMs, uint8_t radio = kRadio) {
  if (n < 6 || f[0] != 0xFE || f[1] != 0xFE || f[n - 1] != 0xFD) return kNone;
  if (f[3] != radio) return kNone;                      // must come from the radio
  if (f[2] != kCtrl && f[2] != 0x00) return kNone;      // to us or broadcast
  uint8_t cmd = f[4];
  const uint8_t* d = f + 5;
  size_t dl = n - 6;
  if (cmd == 0xFB) return kAck;
  if (cmd == 0xFA) return kNak;
  if ((cmd == 0x03 || cmd == 0x00) && dl >= 5) { rig.freq = bcdToFreq(d, 5); rig.lastFreqMs = nowMs; return kFreq; }
  if ((cmd == 0x04 || cmd == 0x01) && dl >= 1) { rig.mode = d[0]; if (dl >= 2) rig.filter = d[1]; return kMode; }
  if (cmd == 0x15 && dl >= 3 && d[0] == 0x02) { rig.sRaw = bcd2(d + 1); rig.lastSMs = nowMs; return kSMeter; }
  if (cmd == 0x15 && dl >= 3 && d[0] == 0x12) { rig.swrRaw = bcd2(d + 1); rig.lastSwrMs = nowMs; return kSwr; }
  if (cmd == 0x1C && dl >= 2 && d[0] == 0x00) { rig.tx = d[1] != 0; return kTx; }
  if (cmd == 0x1A && dl >= 2 && d[0] == 0x06) { rig.dataMode = d[1] != 0; return kDataMode; }
  return kNone;
}

static inline const char* modeName(int m) {
  switch (m) {
    case 0x00: return "LSB"; case 0x01: return "USB"; case 0x02: return "AM"; case 0x03: return "CW";
    case 0x04: return "RTTY"; case 0x05: return "FM"; case 0x06: return "WFM"; case 0x07: return "CW-R";
    case 0x08: return "RTTY-R"; case 0x17: return "DV";
  }
  return "?";
}

// SWR from the meter reading, piecewise linear through the IC-705 calibration points
// (0 = 1.0, 48 = 1.5, 80 = 2.0, 120 = 3.0, 220 = 4.0). Above 220 returns 0 meaning "over 4".
static inline float swrFromRaw(int raw) {
  if (raw < 0) return -1.0f;
  if (raw <= 48) return 1.0f + 0.5f * raw / 48.0f;
  if (raw <= 80) return 1.5f + 0.5f * (raw - 48) / 32.0f;
  if (raw <= 120) return 2.0f + 1.0f * (raw - 80) / 40.0f;
  if (raw <= 220) return 3.0f + 1.0f * (raw - 120) / 100.0f;
  return 0.0f;
}
static inline void swrText(int raw, char* out, size_t cap) {
  float v = swrFromRaw(raw);
  if (v < 0) snprintf(out, cap, "--");
  else if (v == 0.0f) snprintf(out, cap, ">4.0");
  else snprintf(out, cap, "%.1f", (double)v);
}

// S-meter text: 0..120 -> S0..S9, 121..241 -> S9+0..60 dB
static inline void sText(int raw, char* out, size_t cap) {
  if (raw < 0) { snprintf(out, cap, "--"); return; }
  if (raw <= 120) snprintf(out, cap, "S%d", (raw * 9 + 60) / 120);
  else snprintf(out, cap, "S9+%d", ((raw - 120) * 60 + 60) / 121);
}

}  // namespace civ
