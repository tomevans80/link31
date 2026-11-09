// dsp.h - portable DSP for CW, PSK31 and RTTY (no Arduino/ESP dependencies, so it can be
// unit-tested on a PC). All processing runs at kRate = 8000 Hz, 16-bit mono.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#if defined(CW_DEBUG) || defined(PSK_DEBUG) || defined(RTTY_DEBUG)
#include <stdio.h>
#endif

namespace dsp {

static const int   kRate  = 8000;
static const float kTwoPi = 6.28318530717958647692f;

#include "varicode_table.h"

// ---------------------------------------------------------------------------
// Small char FIFO used by the decoders to hand text to the UI.
// ---------------------------------------------------------------------------
class CharFifo {
 public:
  void put(char c) {
    size_t n = (w_ + 1) % kSize;
    if (n == r_) return;  // full: drop
    buf_[w_] = c;
    w_ = n;
  }
  bool get(char& c) {
    if (r_ == w_) return false;
    c = buf_[r_];
    r_ = (r_ + 1) % kSize;
    return true;
  }
  void clear() { r_ = w_ = 0; }

 private:
  static const size_t kSize = 256;
  char buf_[kSize];
  size_t r_ = 0, w_ = 0;
};

// ---------------------------------------------------------------------------
// Varicode
// ---------------------------------------------------------------------------
class VaricodeDecoder {
 public:
  VaricodeDecoder() {
    for (int i = 0; i < 128; i++) {
      uint32_t v = 0;
      for (const char* p = kVaricode[i]; *p; p++) v = (v << 1) | (*p == '1');
      vals_[i] = v;
    }
  }
  void reset() { sh_ = 0; n_ = 0; }
  // Feed one bit. Returns an ASCII code (0..127) when a character completes, else -1.
  int push(int bit) {
    sh_ = (sh_ << 1) | (bit ? 1u : 0u);
    n_++;
    if ((sh_ & 3u) == 0) {  // "00" separator: a character just ended
      uint32_t code = sh_ >> 2;
      sh_ = 0;
      n_ = 0;
      if (code == 0) return -1;  // idle
      for (int i = 0; i < 128; i++)
        if (vals_[i] == code) return i;
    } else if (n_ > 14) {  // no separator for too long: garbage, resync
      sh_ = 0;
      n_ = 0;
    }
    return -1;
  }

 private:
  uint32_t vals_[128];
  uint32_t sh_ = 0;
  int n_ = 0;
};

// ---------------------------------------------------------------------------
// Morse
// ---------------------------------------------------------------------------
struct MorseEntry { char ch; const char* pat; };
static const MorseEntry kMorse[] = {
  {'A', ".-"},   {'B', "-..."}, {'C', "-.-."}, {'D', "-.."},  {'E', "."},    {'F', "..-."},
  {'G', "--."},  {'H', "...."}, {'I', ".."},   {'J', ".---"}, {'K', "-.-"},  {'L', ".-.."},
  {'M', "--"},   {'N', "-."},   {'O', "---"},  {'P', ".--."}, {'Q', "--.-"}, {'R', ".-."},
  {'S', "..."},  {'T', "-"},    {'U', "..-"},  {'V', "...-"}, {'W', ".--"},  {'X', "-..-"},
  {'Y', "-.--"}, {'Z', "--.."},
  {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"},
  {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."},
  {'.', ".-.-.-"}, {',', "--..--"}, {'?', "..--.."}, {'\'', ".----."}, {'!', "-.-.--"},
  {'/', "-..-."},  {'(', "-.--."},  {')', "-.--.-"}, {'&', ".-..."},   {':', "---..."},
  {';', "-.-.-."}, {'=', "-...-"},  {'+', ".-.-."},  {'-', "-....-"},  {'_', "..--.-"},
  {'"', ".-..-."}, {'$', "...-..-"}, {'@', ".--.-."},
};
static const int kMorseCount = sizeof(kMorse) / sizeof(kMorse[0]);

static inline const char* morsePattern(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
  for (int i = 0; i < kMorseCount; i++)
    if (kMorse[i].ch == c) return kMorse[i].pat;
  return nullptr;
}

// ---------------------------------------------------------------------------
// CW decoder (v2).
//   I/Q mix at the tone -> wide anti-alias poles -> decimate to 400 Hz ->
//   speed-adaptive narrow complex low-pass (bandwidth follows the estimated dot length) ->
//   envelope -> noise-floor / peak trackers -> hysteresis threshold ->
//   run-length clean-up (dropouts inside marks and blips inside spaces shorter than a
//   fraction of a dot are absorbed) -> self-calibrating dot/dash clusters ->
//   soft character decision (an invalid pattern is repaired by flipping the least certain
//   element) -> text.  A frequency-locked loop follows tone drift / tuning error.
// ---------------------------------------------------------------------------
class CwDecoder {
 public:
  CwDecoder() { reset(); setFreq(600.0f); }

  // Nominal tone. (The decoder may track +-kAfcRange around it.)
  void setFreq(float hz) { base_ = hz; freq_ = hz; dph_ = kTwoPi * hz / kRate; }
  float freq() const { return freq_; }
  void setMinLevel(float v) { minLevel_ = v; }
  // Output squelch: characters are only printed while the mean envelope during key-down periods
  // exceeds the noise floor by this factor (about 2.8 = a few dB of SNR inside the decoder's own
  // narrow bandwidth). Lower = more sensitive but more junk from noise.
  void setSquelchRatio(float r) { gate_ = r; }
  void setAfc(bool on) { afc_ = on; }
  void setBandwidthFactor(float k) { bwK_ = k; }

  void reset() {
    ph_ = 0;
    for (int i = 0; i < 3; i++) pi_[i] = pq_[i] = 0;
    s1i_ = s1q_ = s2i_ = s2q_ = 0;
    lastI_ = lastQ_ = 0; haveLast_ = false;
    sub_ = 0;
    nm_ = 0; hi_ = 0; warm_ = 60; env_ = 0; valid_ = false;
    sigMean_ = 0; markSum_ = 0; markN_ = 0; pass_ = false;
    markTicks_ = 0; raw_ = false; curMark_ = false; curLen_ = 0; pendLen_ = 0;
    dot_ = 24; dash_ = 72; hn_ = 0; learned_ = false;
    patLen_ = 0; dotHint_ = 0; spacePending_ = false;
    tickNo_ = 0;
    setPost();
    out_.clear();
  }

  void process(const int16_t* x, size_t n) {
    for (size_t i = 0; i < n; i++) {
      float s = (float)x[i];
      ph_ += dph_;
      if (ph_ > kTwoPi) ph_ -= kTwoPi;
      float I = s * cosf(ph_), Q = s * sinf(ph_);
      for (int k = 0; k < 3; k++) {          // 3 cascaded one-pole low-passes (~100 Hz each)
        pi_[k] += kPre * (I - pi_[k]);  I = pi_[k];
        pq_[k] += kPre * (Q - pq_[k]);  Q = pq_[k];
      }
      if (++sub_ >= 20) {                    // 2.5 ms ticks
        sub_ = 0;
        tick(I, Q);
      }
    }
  }

  bool getChar(char& c) { return out_.get(c); }
  bool keyDown() const { return raw_; }
  float envelope() const { return env_; }
  float noiseLevel() const { return nm_; }
  float peakLevel() const { return hi_; }
  float wpm() const { return 1200.0f / (dot_ * 2.5f); }
  bool signalValid() const { return pass_; }
  float bandwidthHz() const { return bw_; }
  float signalToNoise() const { return nm_ > 0 ? sigMean_ : 0; }   // mean key-down envelope / noise floor

 private:
  static constexpr float kAfcRange = 15.0f;
  // Peak tracker follows the key-down plateau after kHiHold ticks, so an AGC onset overshoot
  // (strong first element settling to a lower level) cannot hold the key-up threshold too high.
  static constexpr float kHiFollow = 0.02f;
  static const int kHiHold = 30;             // ticks (75 ms) of key-down before following starts
  static constexpr float kMinDot = 8.0f;     // ticks (20 ms): never report/decode faster than 60 wpm
  static const uint32_t kMinMark = 5;        // ticks (12.5 ms): shorter key-down blips are noise, not Morse
  static const int kHist = 12;

  void setPost() {
    float dotSec = dot_ * 0.0025f;
    bw_ = bwK_ / dotSec;
    if (bw_ < 12.0f) bw_ = 12.0f;
    if (bw_ > 90.0f) bw_ = 90.0f;
    if (!learned_ && bw_ < 32.0f) bw_ = 32.0f;   // speed not known yet: stay wide so fast CW still gets through
    aPost_ = 1.0f - expf(-kTwoPi * bw_ * 0.0025f);
  }

  void tick(float zi, float zq) {
    // adaptive narrow complex low-pass (two poles)
    s1i_ += aPost_ * (zi - s1i_);  s1q_ += aPost_ * (zq - s1q_);
    s2i_ += aPost_ * (s1i_ - s2i_);  s2q_ += aPost_ * (s1q_ - s2q_);
    float env = sqrtf(s2i_ * s2i_ + s2q_ * s2q_);
    env_ = env;

    // --- frequency-locked loop: phase advance of the baseband while the key is down ---
    if (haveLast_ && afc_ && raw_ && valid_) {
      float re = s2i_ * lastI_ + s2q_ * lastQ_;
      float im = s2q_ * lastI_ - s2i_ * lastQ_;
      if (re * re + im * im > 1e-12f) {
        // z = x*e^{+j*phi_nco}: it rotates at (f_nco - f_signal), hence the minus sign below
        float df = atan2f(im, re) * (400.0f / kTwoPi);   // Hz
        if (df > 40.0f) df = 40.0f;
        if (df < -40.0f) df = -40.0f;
        freq_ -= 0.01f * df;
        if (freq_ > base_ + kAfcRange) freq_ = base_ + kAfcRange;
        if (freq_ < base_ - kAfcRange) freq_ = base_ - kAfcRange;
        dph_ = kTwoPi * freq_ / kRate;
      }
    }
    lastI_ = s2i_; lastQ_ = s2q_; haveLast_ = true;

    // --- noise floor / peak trackers ---
    if (warm_ > 0) {
      warm_--;
      nm_ += 0.1f * (env - nm_);
      hi_ = nm_;                        // no peak is trusted until the noise floor is known
    } else {
      if (!raw_) {
        if (env < 2.2f * nm_ || env < 0.5f * nm_) nm_ += 0.0025f * (env - nm_);   // noise-like sample
        else nm_ += 0.0003f * (env - nm_);                                          // slow leak so it can never get stuck low
      }
      if (env > hi_) hi_ += 0.5f * (env - hi_);
      else if (raw_ && markTicks_ > kHiHold) hi_ += (env - hi_) * kHiFollow;
      else hi_ += (nm_ - hi_) * 0.0015f;
      if (hi_ < nm_) hi_ = nm_;
    }
    valid_ = (warm_ <= 0) && (hi_ > 2.0f * nm_) && (hi_ > minLevel_);
    float span = hi_ - nm_;
    bool r = raw_;
    if (!valid_) r = false;
    else if (!raw_ && env > nm_ + 0.5f * span) r = true;
    else if (raw_ && env < nm_ + 0.38f * span) r = false;
    markTicks_ = r ? markTicks_ + 1 : 0;
    raw_ = r;

    // --- run-length clean-up: a change must persist for minRun ticks to count ---
    int minRun = (int)(0.3f * dot_);
    if (minRun < 2) minRun = 2;
    if (minRun > 10) minRun = 10;
    envHist_[tickNo_ % kEnvHist] = env;
    tickNo_++;
    if (raw_ == curMark_) {
      curLen_ += 1 + pendLen_;
      pendLen_ = 0;
    } else if (++pendLen_ >= (uint32_t)minRun) {
      runEnd(curMark_, curLen_, tickNo_ - 1 - pendLen_);
      curMark_ = raw_;
      curLen_ = pendLen_;
      pendLen_ = 0;
      markSum_ = 0; markN_ = 0;
    }
    if (curMark_) { markSum_ += env; markN_++; }
    if (!curMark_) liveSpace();
  }

  // Length of the first mark after a pause, re-measured from the stored envelope: the half-height
  // points of this mark's own plateau. The running key-up threshold has decayed during the pause,
  // so a noise blip just before the first element can get merged into it and stretch it
  // (a dot then reads as a dash). lastIdx = tick number of the mark's last tick.
  uint32_t trimMark(uint32_t len, uint32_t lastIdx) const {
    if (len < 2 || len > (uint32_t)kEnvHist - 8 || tickNo_ < len + 8) return len;
    uint32_t first = lastIdx + 1 - len;
    float mx = 0;
    for (uint32_t i = 0; i < len; i++) { float e = envHist_[(first + i) % kEnvHist]; if (e > mx) mx = e; }
    float sum = 0; int cnt = 0;                       // plateau = mean of the samples above 60% of the maximum
    for (uint32_t i = 0; i < len; i++) { float e = envHist_[(first + i) % kEnvHist]; if (e > 0.6f * mx) { sum += e; cnt++; } }
    float plateau = cnt ? sum / cnt : mx;
    float thr = nm_ + 0.5f * (plateau - nm_);
    int a = -1, b = -1;
    for (uint32_t i = 0; i < len; i++) {
      if (envHist_[(first + i) % kEnvHist] > thr) { if (a < 0) a = (int)i; b = (int)i; }
    }
    if (a < 0) return len;
    return (uint32_t)(b - a + 1);
  }

  void runEnd(bool mark, uint32_t len, uint32_t lastIdx) {
    if (!mark) {
      // A gap clearly shorter than every element so far can only be the 1-dot gap inside a character,
      // so it tells the dot length even when the elements alone are ambiguous (M and T versus E and I).
      if (!learned_ && pass_ && patLen_ > 0 && len >= (uint32_t)kMinDot) {
        float mn = 1e9f;
        for (int i = 0; i < patLen_; i++) if (eld_[i] < mn) mn = eld_[i];
        if ((float)len < 0.7f * mn && (dotHint_ <= 0 || (float)len < dotHint_)) dotHint_ = (float)len;
      }
      return;
    }
    bool wasClosed = !pass_;
    if (markN_ > 0 && nm_ > 0) {   // how far above the noise floor were the key-down periods?
      float m = (markSum_ / markN_) / nm_;
      // gate closed: believe the first real mark sooner (0.85 instead of 0.6), so weak first elements are not dropped
      float w = wasClosed ? 0.85f : (m > sigMean_ ? 0.6f : 0.25f);
      sigMean_ += w * (m - sigMean_);
    }
    pass_ = (sigMean_ >= gate_);
    if (len < kMinMark) return;            // too short for a 60 wpm dot: keep it out of the speed estimate
    if (!pass_) {
      patLen_ = 0;                          // gate closed: noise blips must not disturb the timing model
      return;
    }
    if (wasClosed) len = trimMark(len, lastIdx);   // first mark after a pause: re-measure it
    updateClusters(len);
    if (patLen_ < 9) eld_[patLen_++] = (float)len;
  }

  void liveSpace() {
    if (patLen_ > 0 && curLen_ >= 2.0f * dot_) { if (emit()) spacePending_ = true; }
    if (spacePending_ && curLen_ >= 5.0f * dot_) { out_.put(' '); spacePending_ = false; }
    // a long pause means the transmission ended: close the output gate so noise cannot print junk
    if (pass_ && (float)curLen_ >= 15.0f * dot_) { pass_ = false; sigMean_ = 1.0f; }
    if (curLen_ == 800) { hn_ = 0; learned_ = false; dotHint_ = 0; dot_ = 24; dash_ = 72; setPost(); }   // 2 s of silence: a new over, maybe a different speed
  }

  void updateClusters(uint32_t d) {
    hist_[hn_ % kHist] = (float)d;
    hn_++;
    if (hn_ >= 4) learned_ = true;
    int cnt = hn_ < kHist ? hn_ : kHist;
    float lo = 1e9f, hi = 0, sum = 0;
    for (int i = 0; i < cnt; i++) {
      float v = hist_[i];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
      sum += v;
    }
    if (hi >= 2.0f * lo) {
      float c1 = lo, c2 = hi;
      for (int it = 0; it < 4; it++) {
        float thr = 0.5f * (c1 + c2), s1 = 0, s2 = 0;
        int n1 = 0, n2 = 0;
        for (int i = 0; i < cnt; i++) {
          if (hist_[i] < thr) { s1 += hist_[i]; n1++; } else { s2 += hist_[i]; n2++; }
        }
        if (n1) c1 = s1 / n1;
        if (n2) c2 = s2 / n2;
      }
      dot_ = c1;
      dash_ = c2;
    } else {
      float mean = sum / cnt;
      if (mean > 2.0f * dot_) { dash_ = mean; dot_ = mean / 3.0f; }
      else { dot_ = mean; dash_ = 3.0f * mean; }
    }
    if (!learned_ && dotHint_ > 0) { dot_ = dotHint_; dash_ = 3.0f * dotHint_; }
    if (dot_ < kMinDot) dot_ = kMinDot;   // 60 wpm ceiling
    if (dash_ < 2.0f * dot_) dash_ = 2.0f * dot_;
    if (dash_ > 4.5f * dot_) dash_ = 4.5f * dot_;
    setPost();
  }

  static char lookup(const char* pat) {
    for (int i = 0; i < kMorseCount; i++)
      if (strcmp(kMorse[i].pat, pat) == 0) return kMorse[i].ch;
    return 0;
  }

  bool emit() {   // returns true if a character was printed
    float thr = 0.5f * (dot_ + dash_);
    for (int i = 0; i < patLen_; i++) pat_[i] = (eld_[i] < thr) ? '.' : '-';
    pat_[patLen_] = 0;
    char ch = lookup(pat_);
    if (!ch && patLen_ > 0) {
      // soft repair: flip the least certain element(s) until the pattern is a valid character
      float mg[10];
      for (int i = 0; i < patLen_; i++) mg[i] = fabsf(logf(eld_[i] / thr));
      float best = 1e9f;
      for (int i = 0; i < patLen_; i++) {
        pat_[i] = (pat_[i] == '.') ? '-' : '.';
        char c = lookup(pat_);
        if (c && mg[i] < best) { best = mg[i]; ch = c; }
        pat_[i] = (pat_[i] == '.') ? '-' : '.';
      }
      if (!ch) {
        for (int i = 0; i < patLen_; i++) {
          for (int j = i + 1; j < patLen_; j++) {
            pat_[i] = (pat_[i] == '.') ? '-' : '.';
            pat_[j] = (pat_[j] == '.') ? '-' : '.';
            char c = lookup(pat_);
            if (c && mg[i] + mg[j] < best) { best = mg[i] + mg[j]; ch = c; }
            pat_[i] = (pat_[i] == '.') ? '-' : '.';
            pat_[j] = (pat_[j] == '.') ? '-' : '.';
          }
        }
      }
    }
    bool show = pass_;
    if (show) out_.put(ch ? ch : '#');
    patLen_ = 0;
    pat_[0] = 0;
    return show;
  }

  float base_ = 600, freq_ = 600, dph_ = 0, ph_ = 0;
  float pi_[3], pq_[3];
  static constexpr float kPre = 0.0755f;   // 1 - exp(-2*pi*100/8000)
  float s1i_, s1q_, s2i_, s2q_, aPost_ = 0.05f, bw_ = 30, bwK_ = 1.0f;
  float lastI_, lastQ_;
  bool haveLast_;
  bool learned_ = false;
  int sub_;
  float nm_, hi_, env_ = 0, minLevel_ = 4.0f, gate_ = 2.8f;
  float sigMean_ = 0, markSum_ = 0;
  int markN_ = 0;
  bool pass_ = false;
  int warm_;
  int markTicks_ = 0;
  bool valid_ = false, afc_ = true, raw_, curMark_;
  uint32_t curLen_, pendLen_;
  float dot_, dash_;
  float hist_[kHist];
  int hn_;
  float eld_[10];
  char pat_[12];
  int patLen_;
  float dotHint_ = 0;                      // dot length implied by a 1-dot gap inside a character (0 = none)
  static const int kEnvHist = 512;         // envelope history (1.28 s) for re-measuring the first mark
  float envHist_[kEnvHist];
  uint32_t tickNo_ = 0;
  bool spacePending_;
  CharFifo out_;
};

// ---------------------------------------------------------------------------
// PSK31 (BPSK31) decoder.
//   8 kHz audio -> NCO mix -> 32-sample boxcar / 16 -> 500 Hz complex stream ->
//   31-tap low-pass -> 16 samples/symbol -> energy-dip symbol clock ->
//   windowed symbol integration -> differential detect -> Varicode.
// ---------------------------------------------------------------------------
class Psk31Decoder {
 public:
  Psk31Decoder() {
    const int T = kTaps;
    float fcn = 45.0f / 500.0f, sum = 0;
    for (int i = 0; i < T; i++) {
      float m = (float)(i - (T - 1) / 2);
      float s = (m == 0) ? 2.0f * fcn : sinf(kTwoPi * fcn * m) / (kTwoPi / 2.0f * m);
      float w = 0.54f - 0.46f * cosf(kTwoPi * i / (T - 1));
      h_[i] = s * w;
      sum += h_[i];
    }
    for (int i = 0; i < T; i++) h_[i] /= sum;
    for (int j = 0; j < 16; j++) {
      float s = sinf(3.14159265f * (j + 0.5f) / 16.0f);
      w_[j] = s * s;
    }
    reset();
    setFreq(1000.0f);
  }

  void setFreq(float hz) { f0_ = hz; fc_ = hz; softReset(); }
  float freq() const { return fc_; }
  float tuningOffset() const { return fc_ - f0_; }     // how far the AFC has moved from the tuned frequency
  int clockPhase() const { return dip_; }
  // The last n (<= 127) bit decisions as '0'/'1' characters, oldest first, for the serial diagnostics.
  void recentBits(char* out, int n) const {
    if (n > 127) n = 127;
    for (int i = 0; i < n; i++) out[i] = bitLog_[(bitHead_ - n + i) & 127];
    out[n] = 0;
  }
  void setAfc(bool on) { afc_ = on; }
  void setSquelch(float q) { sq_ = q; }   // 0 disables
  float quality() const { return qual_; }
  bool squelchOpen() const { return open_; }
  bool getChar(char& c) { return out_.get(c); }

  void reset() {
    ph_ = 0; sI_ = sQ_ = pI_ = pQ_ = 0; dc_ = 0;
    for (int i = 0; i < kTaps; i++) bi_[i] = bq_[i] = 0;
    bn_ = 0; clk_ = 0; dip_ = 0;
    for (int i = 0; i < 16; i++) { e_[i] = 0; ri_[i] = rq_[i] = 0; }
    softReset();
    out_.clear();
  }

  void process(const int16_t* x, size_t n) {
    float dph = kTwoPi * fc_ / kRate;
    for (size_t i = 0; i < n; i++) {
      float s = (float)x[i];
      ph_ += dph;
      if (ph_ > kTwoPi) ph_ -= kTwoPi;
      sI_ += s * cosf(ph_);
      sQ_ -= s * sinf(ph_);
      if (++dc_ == 16) {
        dc_ = 0;
        baseband((sI_ + pI_) * (1.0f / 32.0f), (sQ_ + pQ_) * (1.0f / 32.0f));
        pI_ = sI_; pQ_ = sQ_; sI_ = sQ_ = 0;
        dph = kTwoPi * fc_ / kRate;  // AFC may have moved the carrier
      }
    }
  }

 private:
  static const int kTaps = 31;

  void softReset() {
    have_ = false; pSi_ = pSq_ = 0; qual_ = 0; avgMag_ = 0; open_ = false;
    zpr_ = zpi_ = fAccR_ = fAccI_ = fPow_ = 0; fN_ = 0;
    vd_.reset(); lastCR_ = false;
  }

  void baseband(float zi, float zq) {
    bi_[bn_] = zi; bq_[bn_] = zq;
    float fi = 0, fq = 0;
    int idx = bn_;
    for (int k = 0; k < kTaps; k++) {
      fi += h_[k] * bi_[idx];
      fq += h_[k] * bq_[idx];
      if (--idx < 0) idx = kTaps - 1;
    }
    bn_ = (bn_ + 1) % kTaps;

    // Data-independent frequency-error estimate (pull-in range about +-45 Hz): squaring the baseband removes the
    // BPSK phase reversals, so the phase step between successive squared samples is twice the per-sample
    // rotation caused by the tuning error. The decision-directed AFC below only works within about +-8 Hz,
    // beyond that the bits are decided wrongly and the decode is consistently garbage.
    if (afc_) {
      float zr = fi * fi - fq * fq, zi = 2.0f * fi * fq;
      fAccR_ += zr * zpr_ + zi * zpi_;
      fAccI_ += zi * zpr_ - zr * zpi_;
      fPow_ += sqrtf((zr * zr + zi * zi) * (zpr_ * zpr_ + zpi_ * zpi_));
      zpr_ = zr; zpi_ = zi;
      if (++fN_ >= 128) {                                   // every 256 ms
        float mag = sqrtf(fAccR_ * fAccR_ + fAccI_ * fAccI_);
        if (fPow_ > 0 && mag > 0.3f * fPow_) {              // coherent: a carrier is present
          float err = atan2f(fAccI_, fAccR_) * (500.0f / (4.0f * 3.14159265f));   // Hz
          if (err > 10.0f) err = 10.0f;
          if (err < -10.0f) err = -10.0f;
          fc_ += 0.6f * err;
          if (fc_ > f0_ + kAfcRange) fc_ = f0_ + kAfcRange;
          if (fc_ < f0_ - kAfcRange) fc_ = f0_ - kAfcRange;
        }
        fN_ = 0; fAccR_ = fAccI_ = fPow_ = 0;
      }
    }
    ri_[clk_] = fi; rq_[clk_] = fq;
    float en = fi * fi + fq * fq;
    e_[clk_] += (en - e_[clk_]) * (1.0f / 16.0f);

    // symbol clock: the energy minimum marks the middle of a phase transition
    if (clk_ == dip_) decide();
    clk_ = (clk_ + 1) & 15;
  }

  // Symbol-clock tracker, run once per symbol: the symbol boundary sits at the
  // minimum of the (1-2-1 smoothed) per-phase energy profile. A 10% hysteresis stops
  // it hopping between two adjacent phases when the true minimum lies between them.
  void trackClock() {
    float es[16];
    for (int k = 0; k < 16; k++) es[k] = e_[(k + 15) & 15] + 2.0f * e_[k] + e_[(k + 1) & 15];
    int mn = 0;
    for (int k = 1; k < 16; k++) if (es[k] < es[mn]) mn = k;
    if (mn != dip_ && es[mn] < 0.9f * es[dip_]) {
#ifdef PSK_DEBUG
      printf("<dip %d->%d>", dip_, mn);
#endif
      dip_ = mn;
    }
  }

  void decide() {
    trackClock();
    float Si = 0, Sq = 0;
    for (int j = 0; j < 16; j++) {
      int idx = (clk_ + 1 + j) & 15;
      Si += w_[j] * ri_[idx];
      Sq += w_[j] * rq_[idx];
    }
    if (have_) {
      float pr = Si * pSi_ + Sq * pSq_;   // Re(S * conj(prev))
      float pm = Sq * pSi_ - Si * pSq_;   // Im(S * conj(prev))
      float mag = sqrtf(pr * pr + pm * pm);
      if (mag > 1e-12f) {
        int bit = pr >= 0 ? 1 : 0;
        bitLog_[bitHead_++ & 127] = bit ? '1' : '0';
#ifdef PSK_DEBUG
        printf("%d", bit);
#endif
        if (avgMag_ > 0 && mag < 0.15f * avgMag_) qual_ *= 0.6f;  // carrier vanished
        qual_ += (fabsf(pr) / mag - qual_) * (1.0f / 12.0f);
        avgMag_ += (mag - avgMag_) * (1.0f / 32.0f);
        if (afc_) {
          float ar = pr, ai = pm;
          if (ar < 0) { ar = -ar; ai = -ai; }
          float err = atan2f(ai, ar);                       // rad per symbol
          float wgt = avgMag_ > 0 ? mag / avgMag_ : 0;
          if (wgt > 1.0f) wgt = 1.0f;
          fc_ += 0.05f * wgt * err * (31.25f / kTwoPi);
          if (fc_ > f0_ + kAfcRange) fc_ = f0_ + kAfcRange;
          if (fc_ < f0_ - kAfcRange) fc_ = f0_ - kAfcRange;
        }
        bool was = open_;
        if (sq_ <= 0) open_ = true;
        else open_ = open_ ? (qual_ > sq_ - 0.05f) : (qual_ > sq_);
        if (open_ && !was) vd_.reset();
        if (open_) {
          int ch = vd_.push(bit);
          if (ch >= 0) putAscii(ch);
        } else {
          vd_.reset();
        }
      }
    }
    pSi_ = Si; pSq_ = Sq; have_ = true;
  }

  void putAscii(int ch) {
    if (ch == 13) { out_.put('\n'); lastCR_ = true; return; }
    if (ch == 10) { if (!lastCR_) out_.put('\n'); lastCR_ = false; return; }
    lastCR_ = false;
    if (ch == 8) { out_.put('\b'); return; }
    if (ch >= 32 && ch < 127) out_.put((char)ch);
  }

  static constexpr float kAfcRange = 40.0f;

  float h_[kTaps], w_[16];
  float f0_ = 1000, fc_ = 1000, ph_ = 0;
  float sI_, sQ_, pI_, pQ_;
  int dc_;
  float bi_[kTaps], bq_[kTaps];
  int bn_;
  int clk_, dip_;
  float e_[16], ri_[16], rq_[16];
  bool have_;
  float pSi_, pSq_, qual_, avgMag_;
  char bitLog_[128] = {0};
  uint32_t bitHead_ = 0;
  float zpr_ = 0, zpi_ = 0, fAccR_ = 0, fAccI_ = 0, fPow_ = 0;
  int fN_ = 0;
  bool afc_ = true, open_ = false, lastCR_ = false;
  float sq_ = 0.80f;
  VaricodeDecoder vd_;
  CharFifo out_;
};

// ---------------------------------------------------------------------------
// PSK31 modulator: Varicode text -> BPSK31 audio (8 kHz, 16-bit), raised-cosine
// phase transitions. Sequence: preamble (idle reversals), text, postamble
// (steady carrier), done.
// ---------------------------------------------------------------------------
class Psk31Modulator {
 public:
  Psk31Modulator() { for (int k = 0; k < kSym; k++) shape_[k] = 0.5f + 0.5f * cosf(3.14159265f * k / kSym); }

  void begin(float hz, float level) {
    f_ = hz; level_ = level; ph_ = 0; prev_ = cur_ = 1.0f; k_ = kSym - 1;
    st_ = kPre; preLeft_ = 48; postLeft_ = 16; code_ = nullptr; zeros_ = 0;
    qr_ = qw_ = 0; finishReq_ = false; rise_ = 0; tail_ = 0;
  }
  void setLevel(float l) { level_ = l; }
  bool queueChar(char c) {
    int ch = (unsigned char)c;
    if (c == '\n') { return queueChar('\r') && queueRaw(10); }
    if (ch > 127) return true;  // not representable: skip
    return queueRaw((uint8_t)ch);
  }
  bool queueText(const char* s) { for (; *s; s++) if (!queueChar(*s)) return false; return true; }
  void finish() { finishReq_ = true; }
  bool done() const { return st_ == kDone; }
  bool queueEmpty() const { return qr_ == qw_ && !code_; }
  bool accepting() const { return st_ == kRun; }   // preamble over, not yet finishing: queued characters go out next

  // Fill n samples. After done() (plus a short fade-out) the output is silence.
  size_t generate(int16_t* out, size_t n) {
    const float dph = kTwoPi * f_ / kRate;
    for (size_t i = 0; i < n; i++) {
      if (st_ == kDone && tail_ <= 0) { out[i] = 0; continue; }
      if (st_ != kDone && ++k_ >= kSym) {
        k_ = 0;
        int bit = nextBit();
        prev_ = cur_;
        if (!bit) cur_ = -cur_;
        if (st_ == kDone) tail_ = 64;
      }
      float b = prev_ * shape_[k_] + cur_ * (1.0f - shape_[k_]);
      float g = 1.0f;
      if (st_ == kDone) { g = tail_ / 64.0f; tail_--; }
      if (rise_ < 64) g *= (float)(rise_++) / 64.0f;
      ph_ += dph;
      if (ph_ > kTwoPi) ph_ -= kTwoPi;
      out[i] = (int16_t)(32767.0f * level_ * g * b * sinf(ph_));
    }
    return n;
  }

 private:
  static const int kSym = 256;  // 8000 / 31.25
  enum State { kPre, kRun, kPost, kDone };

  bool queueRaw(uint8_t ch) {
    size_t nx = (qw_ + 1) % kQ;
    if (nx == qr_) return false;
    q_[qw_] = ch; qw_ = nx;
    return true;
  }

  int nextBit() {
    for (;;) {
      if (st_ == kPre) {
        if (preLeft_-- > 0) return 0;
        st_ = kRun;
      }
      if (st_ == kRun) {
        if (code_) {
          int bit = code_[ci_++] == '1';
          if (!code_[ci_]) { code_ = nullptr; zeros_ = 2; }
          return bit;
        }
        if (zeros_ > 0) { zeros_--; return 0; }
        if (qr_ != qw_) {
          code_ = kVaricode[q_[qr_]];
          qr_ = (qr_ + 1) % kQ;
          ci_ = 0;
          continue;
        }
        if (finishReq_) { st_ = kPost; continue; }
        return 0;  // idle
      }
      if (st_ == kPost) {
        if (postLeft_-- > 0) return 1;
        st_ = kDone;
        return 1;
      }
      return 1;
    }
  }

  static const size_t kQ = 512;
  float shape_[kSym];
  float f_ = 1000, level_ = 0.3f, ph_ = 0, prev_ = 1, cur_ = 1;
  int rise_ = 64, tail_ = 0;
  int k_ = kSym - 1;
  State st_ = kDone;
  int preLeft_ = 0, postLeft_ = 0, zeros_ = 0, ci_ = 0;
  const char* code_ = nullptr;
  uint8_t q_[kQ];
  size_t qr_ = 0, qw_ = 0;
  bool finishReq_ = false;
};

// ---------------------------------------------------------------------------
// RTTY (5-bit ITA2 / US-TTY, async: 1 start, 5 data LSB first, 1.5 stop). AFSK on the audio: mark and space tones
// centred on 'centre', default 45.45 baud / 170 Hz shift. reverse = false puts mark on the LOWER tone, true on the HIGHER tone
// (the sketch defaults to true: standard RTTY polarity when the radio is in USB).
// ---------------------------------------------------------------------------
static const char kItaLtrs[32] = {0, 'E', '\n', 'A', ' ', 'S', 'I', 'U', '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K',
                                  'T', 'Z', 'L', 'W', 'H', 'Y', 'P', 'Q', 'O', 'B', 'G', 0, 'M', 'X', 'V', 0};
static const char kItaFigs[32] = {0, '3', '\n', '-', ' ', '\'', '8', '7', '\r', '$', '4', 7, ',', '!', ':', '(',
                                  '5', '"', ')', '2', '#', '6', '0', '1', '9', '?', '&', 0, '.', '/', ';', 0};
static const uint8_t kItaLtrsCode = 0x1F, kItaFigsCode = 0x1B;

// Which ITA2 code sends c? shift: 0 = same in both shifts (space, CR, LF), 1 = letters, 2 = figures. False if not sendable.
inline bool rttyEncode(char c, uint8_t* code, int* shift) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
  if (c == 0 || c == 7) return false;
  for (int i = 1; i < 32; i++) {
    if (kItaLtrs[i] == c) { *code = (uint8_t)i; *shift = (c == ' ' || c == '\r' || c == '\n') ? 0 : 1; return true; }
  }
  for (int i = 1; i < 32; i++) {
    if (kItaFigs[i] == c) { *code = (uint8_t)i; *shift = 2; return true; }
  }
  return false;
}
inline bool rttyCanSend(char c) { uint8_t k; int s; return rttyEncode(c, &k, &s); }

class RttyDecoder {
 public:
  RttyDecoder() { configure(); }
  void setFreq(float centreHz) { fc0_ = centreHz; fc_ = centreHz; retune(); }
  float freq() const { return fc_; }                       // centre including AFC
  float tuningOffset() const { return fc_ - fc0_; }
  void setShift(float hz) { shift_ = hz; retune(); }
  void setBaud(float b) { baud_ = b; configure(); }
  void setReverse(bool r) { rev_ = r; retune(); }
  void setAfc(bool a) { afc_ = a; if (!a) { fc_ = fc0_; retune(); } }
  void setUsos(bool u) { usos_ = u; }                       // unshift on space (receive)
  void setSquelch(float q) { sqOpen_ = q; sqClose_ = q - 0.10f; }
  bool squelchOpen() const { return open_; }
  float quality() const { return q_; }                      // smoothed |discriminant|, 0 (noise) .. 1 (clean)
  bool getChar(char& c) { return out_.get(c); }
  void reset() {
    memset(bi_, 0, sizeof bi_); memset(bq_, 0, sizeof bq_);
    sumI_[0] = sumI_[1] = sumQ_[0] = sumQ_[1] = 0; pos_ = 0; st_ = kWaitMark; markRun_ = 0; dprev_ = 0;
    q_ = 0; open_ = false; figs_ = false; crPending_ = false; out_.clear(); zr_[0] = zr_[1] = zi_[0] = zi_[1] = 0;
    accR_ = accI_ = 0; afcN_ = 0; afcHold_ = 0; pk_[0] = pk_[1] = 20.0f;
  }

  void process(const int16_t* x, size_t n) {
    for (size_t i = 0; i < n; i++) step((float)x[i]);
  }

 private:
  static const int kMaxL = 200;                             // longest boxcar: 8000/40 baud
  enum St { kWaitMark, kArmed, kChar };

  void configure() {
    Lf_ = kRate / baud_;
    L_ = (int)(Lf_ + 0.5f);
    if (L_ > kMaxL) L_ = kMaxL;
    if (L_ < 20) L_ = 20;
    retune();
    reset();
  }
  void retune() {
    float fm = rev_ ? fc_ + shift_ * 0.5f : fc_ - shift_ * 0.5f;
    float fs = rev_ ? fc_ - shift_ * 0.5f : fc_ + shift_ * 0.5f;
    float f[2] = {fm, fs};
    for (int t = 0; t < 2; t++) {
      if (!nco_init_) { cr_[t] = 1; ci_[t] = 0; }
      cdr_[t] = cosf(kTwoPi * f[t] / kRate);
      cdi_[t] = sinf(kTwoPi * f[t] / kRate);
    }
    nco_init_ = true;
    afcHold_ = L_;                                          // boxcar holds old-frequency samples for one bit: no AFC meanwhile
  }

  void step(float s) {
    float e[2], zr[2], zi[2];
    for (int t = 0; t < 2; t++) {
      // mix down by conj(NCO)
      float mi = s * cr_[t], mq = -s * ci_[t];
      float nr = cr_[t] * cdr_[t] - ci_[t] * cdi_[t];
      float ni = cr_[t] * cdi_[t] + ci_[t] * cdr_[t];
      cr_[t] = nr; ci_[t] = ni;
      sumI_[t] += mi - bi_[t][pos_]; bi_[t][pos_] = mi;
      sumQ_[t] += mq - bq_[t][pos_]; bq_[t][pos_] = mq;
      zr[t] = sumI_[t] / L_; zi[t] = sumQ_[t] / L_;
      e[t] = sqrtf(zr[t] * zr[t] + zi[t] * zi[t]);
    }
    if (++pos_ >= L_) pos_ = 0;
    if ((++tick_ & 511) == 0) {                             // keep NCOs on the unit circle, sums free of drift
      for (int t = 0; t < 2; t++) { float m = 1.0f / sqrtf(cr_[t] * cr_[t] + ci_[t] * ci_[t]); cr_[t] *= m; ci_[t] *= m; }
      if ((tick_ & 0x1FFF) == 0) for (int t = 0; t < 2; t++) {
        float a = 0, b = 0;
        for (int k = 0; k < L_; k++) { a += bi_[t][k]; b += bq_[t][k]; }
        sumI_[t] = a; sumQ_[t] = b;
      }
    }

    // Normalise each tone by its own recent peak, so a radio/audio path that favours one tone (several dB is normal) does
    // not shift the decision point or the bit timing. Neither peak may fall below a quarter of the other one.
    {
      const float dec = 1.0f - 1.0f / 16000.0f;               // 2 s decay
      pk_[0] = e[0] > pk_[0] * dec ? e[0] : pk_[0] * dec;
      pk_[1] = e[1] > pk_[1] * dec ? e[1] : pk_[1] * dec;
      if (pk_[0] < 20.0f) pk_[0] = 20.0f;
      if (pk_[1] < 20.0f) pk_[1] = 20.0f;
      if (pk_[1] < 0.25f * pk_[0]) pk_[1] = 0.25f * pk_[0];
      if (pk_[0] < 0.25f * pk_[1]) pk_[0] = 0.25f * pk_[1];
    }
    float n0 = e[0] / pk_[0], n1 = e[1] / pk_[1];
    float d = (n0 - n1) / (n0 + n1 + 0.02f);                 // +1 = mark, -1 = space
    q_ += (fabsf(d) - q_) * (1.0f / 1200.0f);
    if (open_ ? q_ < sqClose_ : q_ > sqOpen_) open_ = !open_;

    // AFC: when one tone clearly dominates, its baseband phasor rotates at (tone - NCO) Hz
    if (afc_) {
      if (afcHold_ > 0) afcHold_--;
      else for (int t = 0; t < 2; t++) {
        if (e[t] > 3.0f * e[1 - t] && e[t] > 60.0f) {
          accR_ += zr[t] * zr_[t] + zi[t] * zi_[t];
          accI_ += zi[t] * zr_[t] - zr[t] * zi_[t];
        }
      }
      if (++afcN_ >= 1600) {                                // every 200 ms
        afcN_ = 0;
        float mag = sqrtf(accR_ * accR_ + accI_ * accI_);
        if (mag > 1e4f && open_) {
          float df = atan2f(accI_, accR_) * kRate / kTwoPi;   // Hz the tones sit above the NCOs
          // the phasor of each tone advances once per sample (both tones summed above), so this is per-sample rotation
          if (df > 40) df = 40;
          if (df < -40) df = -40;
          float nf = fc_ + 0.6f * df;
          if (nf > fc0_ + 60) nf = fc0_ + 60;
          if (nf < fc0_ - 60) nf = fc0_ - 60;
          if (fabsf(nf - fc_) > 0.05f) { fc_ = nf; retune(); }
        }
        accR_ = accI_ = 0;
      }
    }
    for (int t = 0; t < 2; t++) { zr_[t] = zr[t]; zi_[t] = zi[t]; }

    // character framing
    switch (st_) {
      case kWaitMark:
        if (d > 0.1f) { if (++markRun_ >= L_ / 8) st_ = kArmed; } else markRun_ = 0;
        break;
      case kArmed:
        if (d <= 0 && dprev_ > 0 && open_) {              // mark -> space crossing (squelch open): the start bit's edge is L/2 earlier
          st_ = kChar; bit_ = 0; code_ = 0; tNext_ = 0.5f * Lf_;
#ifdef RTTY_DEBUG
          printf("start at tick %u\n", (unsigned)tick_);
#endif
        }
        break;
      case kChar:
        if ((tNext_ -= 1.0f) <= 0) {
          int bit = d > 0 ? 1 : 0;
          if (bit_ == 0) {
            if (bit || fabsf(d) < 0.25f) { st_ = kWaitMark; markRun_ = 0; }     // not a (clear) start bit: noise
            else tNext_ += Lf_;
          } else if (bit_ <= 5) {
            code_ |= (uint8_t)(bit << (bit_ - 1));
            tNext_ += (bit_ == 5) ? 0.9f * Lf_ : Lf_;
          } else {                                          // stop bit (sampled at 0.9 of the way through the detector window, so it also works with 1 stop bit)
            if (bit && open_ && fabsf(d) > 0.25f) emit(code_);
            st_ = kWaitMark; markRun_ = 0;
          }
          bit_++;
        }
        break;
    }
    dprev_ = d;
  }

  void emit(uint8_t code) {
#ifdef RTTY_DEBUG
    printf("emit code %02x figs=%d open=%d\n", code, (int)figs_, (int)open_);
#endif
    if (code == kItaLtrsCode) { figs_ = false; return; }
    if (code == kItaFigsCode) { figs_ = true; return; }
    char c = figs_ ? kItaFigs[code & 31] : kItaLtrs[code & 31];
    if (usos_ && code == 4) figs_ = false;
    if (c == 0 || c == 7) return;
    if (c == '\r') { out_.put('\n'); crPending_ = true; return; }
    if (c == '\n') { if (!crPending_) out_.put('\n'); crPending_ = false; return; }
    crPending_ = false;
    out_.put(c);
  }

  float fc0_ = 2210, fc_ = 2210, shift_ = 170, baud_ = 45.45f, Lf_ = 176;
  int L_ = 176;
  bool rev_ = false, afc_ = true, usos_ = true, nco_init_ = false;
  float sqOpen_ = 0.50f, sqClose_ = 0.40f;
  float cr_[2], ci_[2], cdr_[2], cdi_[2];
  float bi_[2][kMaxL], bq_[2][kMaxL];
  float sumI_[2], sumQ_[2], zr_[2], zi_[2];
  int pos_ = 0;
  uint32_t tick_ = 0;
  float q_ = 0, dprev_ = 0, accR_ = 0, accI_ = 0, tNext_ = 0, pk_[2] = {20.0f, 20.0f};
  int afcN_ = 0, afcHold_ = 0, markRun_ = 0, bit_ = 0;
  uint8_t code_ = 0;
  St st_ = kWaitMark;
  bool open_ = false, figs_ = false, crPending_ = false;
  CharFifo out_;
};

class RttyModulator {
 public:
  void begin(float centreHz, float level, float baud, float shiftHz, bool reverse) {
    float lo = centreHz - shiftHz * 0.5f, hi = centreHz + shiftHz * 0.5f;
    fMark_ = reverse ? hi : lo; fSpace_ = reverse ? lo : hi;
    level_ = level; Lf_ = kRate / baud; ph_ = 0; fcur_ = fMark_; cur_ = fMark_;
    st_ = kPre; preLeft_ = 14; postLeft_ = 3; bitLeft_ = 0; sending_ = false; finishReq_ = false;
    qr_ = qw_ = 0; rise_ = 0; tail_ = 0; shift_ = 0;
    queueCode(kItaLtrsCode); shift_ = 1;                    // receivers start in letters
  }
  bool queueChar(char c) {
    if (c == '\n') return queueChar('\r') && queueCode(2);
    uint8_t code; int sh;
    if (!rttyEncode(c, &code, &sh)) return true;            // not representable: skip
    if (kQ - used() < 4) return false;
    if (sh == 1 && shift_ != 1) { queueCode(kItaLtrsCode); shift_ = 1; }
    if (sh == 2 && shift_ != 2) { queueCode(kItaFigsCode); shift_ = 2; }
    queueCode(code);
    if (c == ' ' && shift_ == 2) shift_ = 0;                // a receiver may unshift on space: shift again before the next character
    return true;
  }
  void finish() { finishReq_ = true; }
  bool done() const { return st_ == kDone; }
  bool queueEmpty() const { return qr_ == qw_ && !sending_; }
  bool accepting() const { return st_ == kRun; }

  size_t generate(int16_t* out, size_t n) {
    for (size_t i = 0; i < n; i++) {
      if (st_ == kDone && tail_ <= 0) { out[i] = 0; continue; }
      if (st_ != kDone) { bitLeft_ -= 1.0f; while (bitLeft_ <= 0 && st_ != kDone) nextElement(); }
      fcur_ += (cur_ - fcur_) * 0.15f;                      // soft frequency steps: keeps the spectrum clean
      ph_ += kTwoPi * fcur_ / kRate;
      if (ph_ > kTwoPi) ph_ -= kTwoPi;
      float g = 1.0f;
      if (st_ == kDone) { g = tail_ / 64.0f; tail_--; }
      if (rise_ < 64) g *= (float)(rise_++) / 64.0f;
      out[i] = (int16_t)(32767.0f * level_ * g * sinf(ph_));
    }
    return n;
  }

 private:
  enum State { kPre, kRun, kPost, kDone };
  static const size_t kQ = 16;
  size_t used() const { return (qw_ + kQ - qr_) % kQ; }
  bool queueCode(uint8_t code) {
    size_t nx = (qw_ + 1) % kQ;
    if (nx == qr_) return false;
    q_[qw_] = code; qw_ = nx;
    return true;
  }
  void nextElement() {
    for (;;) {
      if (st_ == kPre) {
        if (preLeft_-- > 0) { cur_ = fMark_; bitLeft_ += Lf_; return; }
        st_ = kRun;
      }
      if (st_ == kRun) {
        if (!sending_ && qr_ != qw_) { code_ = q_[qr_]; qr_ = (qr_ + 1) % kQ; sending_ = true; ei_ = 0; }
        if (sending_) {
          if (ei_ == 0) { cur_ = fSpace_; bitLeft_ += Lf_; }                       // start bit
          else if (ei_ <= 5) { cur_ = ((code_ >> (ei_ - 1)) & 1) ? fMark_ : fSpace_; bitLeft_ += Lf_; }
          else { cur_ = fMark_; bitLeft_ += 1.5f * Lf_; sending_ = false; }         // 1.5 stop bits
          ei_++;
          return;
        }
        if (finishReq_) { st_ = kPost; continue; }
        cur_ = fMark_; bitLeft_ += 0.25f * Lf_;                                    // idle: mark, short steps so new text starts promptly
        return;
      }
      if (st_ == kPost) {
        if (postLeft_-- > 0) { cur_ = fMark_; bitLeft_ += Lf_; return; }
        st_ = kDone; tail_ = 64; return;
      }
      return;
    }
  }

  float fMark_ = 2125, fSpace_ = 2295, level_ = 0.3f, Lf_ = 176, ph_ = 0, fcur_ = 2125, cur_ = 2125, bitLeft_ = 0;
  int rise_ = 64, tail_ = 0, preLeft_ = 0, postLeft_ = 0, ei_ = 0, shift_ = 0;
  State st_ = kDone;
  bool sending_ = false, finishReq_ = false;
  uint8_t code_ = 0, q_[kQ];
  size_t qr_ = 0, qw_ = 0;
};

// ---------------------------------------------------------------------------
// 512-point FFT spectrum (15.6 Hz/bin at 8 kHz) for the waterfall.
// ---------------------------------------------------------------------------
class Spectrum {
 public:
  static const int N = 512;
  static const int kBins = N / 2;
  Spectrum() {
    for (int i = 0; i < N; i++) win_[i] = 0.5f - 0.5f * cosf(kTwoPi * i / (N - 1));
    for (int i = 0; i < N / 2; i++) { cs_[i] = cosf(kTwoPi * i / N); sn_[i] = -sinf(kTwoPi * i / N); }
    int bits = 9;
    for (int i = 0; i < N; i++) {
      int r = 0;
      for (int b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
      rev_[i] = (uint16_t)r;
    }
    for (int i = 0; i < kBins; i++) db_[i] = -120.0f;
  }

  // Returns true each time a new spectrum (db()) is ready.
  bool push(const int16_t* x, size_t n) {
    bool ready = false;
    for (size_t i = 0; i < n; i++) {
      buf_[fill_++] = (float)x[i];
      if (fill_ == N) { compute(); fill_ = 0; ready = true; }
    }
    return ready;
  }
  const float* db() const { return db_; }
  static float binHz() { return (float)kRate / N; }

  // Strongest bin between lo and hi Hz, parabolic-interpolated. Returns Hz.
  float peakHz(float lo, float hi, float* magDb = nullptr) const {
    int a = (int)(lo / binHz()), b = (int)(hi / binHz());
    if (a < 1) a = 1;
    if (b > kBins - 2) b = kBins - 2;
    int best = a;
    for (int i = a; i <= b; i++) if (db_[i] > db_[best]) best = i;
    float y0 = db_[best - 1], y1 = db_[best], y2 = db_[best + 1];
    float den = y0 - 2 * y1 + y2;
    float d = (den != 0) ? 0.5f * (y0 - y2) / den : 0;
    if (magDb) *magDb = y1;
    return (best + d) * binHz();
  }

 private:
  void compute() {
    for (int i = 0; i < N; i++) { re_[rev_[i]] = buf_[i] * win_[i]; im_[rev_[i]] = 0; }
    for (int len = 2; len <= N; len <<= 1) {
      int half = len >> 1, step = N / len;
      for (int i = 0; i < N; i += len) {
        for (int j = 0; j < half; j++) {
          float wr = cs_[j * step], wi = sn_[j * step];
          int a = i + j, b = a + half;
          float tr = re_[b] * wr - im_[b] * wi;
          float ti = re_[b] * wi + im_[b] * wr;
          re_[b] = re_[a] - tr; im_[b] = im_[a] - ti;
          re_[a] += tr;         im_[a] += ti;
        }
      }
    }
    for (int i = 0; i < kBins; i++) {
      float m = sqrtf(re_[i] * re_[i] + im_[i] * im_[i]) / (N / 4.0f) / 32768.0f;
      float d = 20.0f * log10f(m + 1e-9f);
      db_[i] += (d - db_[i]) * 0.6f;
    }
  }

  float buf_[N], re_[N], im_[N], win_[N], cs_[N / 2], sn_[N / 2], db_[kBins];
  uint16_t rev_[N];
  int fill_ = 0;
};

// ---------------------------------------------------------------------------
// Sample-rate converter from the radio's audio rate (8/16/24/48 kHz) to 8 kHz.
// Box-car average over the integer ratio; adequate because the radio's own
// audio filtering already limits the content to ~3 kHz.
// ---------------------------------------------------------------------------
class Decimator {
 public:
  explicit Decimator(int ratio = 1) : r_(ratio < 1 ? 1 : ratio) {}
  void setRatio(int r) { r_ = r < 1 ? 1 : r; acc_ = 0; n_ = 0; }
  // Returns number of output samples written (<= nIn).
  size_t process(const int16_t* in, size_t nIn, int16_t* out) {
    size_t o = 0;
    for (size_t i = 0; i < nIn; i++) {
      acc_ += in[i];
      if (++n_ == r_) { out[o++] = (int16_t)(acc_ / r_); acc_ = 0; n_ = 0; }
    }
    return o;
  }

 private:
  int r_, n_ = 0;
  int32_t acc_ = 0;
};

}  // namespace dsp
