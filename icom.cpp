// icom.cpp - see icom.h. Packet layouts follow the kappanhang reference implementation.
#include "icom.h"
#include <string.h>
#include <stdio.h>

namespace icom {

#include "passcode_table.h"

void encodePasscode(const char* s, uint8_t out[16]) {
  memset(out, 0, 16);
  for (int i = 0; s[i] && i < 16; i++) {
    int p = (unsigned char)s[i] + i;
    if (p > 126) p = 32 + p % 127;
    out[i] = (p >= 32 && p <= 126) ? kPasscodeTable[p - 32] : 0;
  }
}

static inline void put32be(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}
static inline uint32_t get32be(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static inline uint16_t get16le(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// ---------------------------------------------------------------------------
void Link::Ring::init(uint8_t* m, size_t s, int cnt) {
  mem = m; slot = s; n = cnt; head = 0;
  for (int i = 0; i < 48; i++) used[i] = false;
}
void Link::Ring::add(uint16_t sq, const uint8_t* d, size_t l) {
  if (!mem || l > slot) return;
  int i = head;
  head = (head + 1) % n;
  seq[i] = sq; len[i] = (uint16_t)l; used[i] = true;
  memcpy(mem + (size_t)i * slot, d, l);
}
const uint8_t* Link::Ring::get(uint16_t sq, size_t* l) const {
  for (int i = 0; i < n; i++)
    if (used[i] && seq[i] == sq) { *l = len[i]; return mem + (size_t)i * slot; }
  return nullptr;
}

// ---------------------------------------------------------------------------
Link::Link() {}

void Link::begin(Io* io, const Config& cfg) {
  io_ = io;
  cfg_ = cfg;
  state_ = kIdle;
}

const char* Link::stateName() const {
  switch (state_) {
    case kIdle: return "idle";
    case kCtrlHello: return "contacting radio";
    case kLogin: return "logging in";
    case kAuth: return "authenticating";
    case kStreamsReq: return "requesting streams";
    case kStreamsHello: return "opening streams";
    case kRunning: return "connected";
    case kFailed: return "failed";
  }
  return "?";
}

void Link::setState(State st, uint32_t now) {
  state_ = st;
  stateSince_ = now;
  char m[48];
  snprintf(m, sizeof m, "state -> %s", stateName());
  io_->log(m);
}

// polite = tell the radio we are leaving (stops it keeping a stale session that can block the next login)
void Link::closeAll(bool polite) {
  if (polite) {
    uint32_t now = io_->millis();
    if (stm_[kCiv].open && stm_[kCiv].hs == 3) sendCivOpenClose(true, now);
    for (int s = 0; s < kStreams; s++)
      if (stm_[s].open && stm_[s].remoteSid) sendDisconnectPkt(s);
  }
  for (int s = 0; s < kStreams; s++) {
    if (stm_[s].open) io_->close(s);
    stm_[s] = Stm();
  }
}

void Link::fail(const char* why) {
  strncpy(err_, why, sizeof(err_) - 1);
  err_[sizeof(err_) - 1] = 0;
  io_->log(err_);
  closeAll(true);
  state_ = kFailed;
  failedAt_ = io_->millis();
}

void Link::openStream(int s, uint32_t now) {
  Stm& m = stm_[s];
  m = Stm();
  if (!io_->open(s, kRemotePort[s])) { fail("cannot open UDP socket"); return; }
  m.open = true;
  m.localSid = io_->localSid(s);
  m.lastRx = now;
  m.hs = 1;
  m.hsNext = now + 500;
  if (s == kCtrl) m.ring.init(memCtrl_, kSlotCtrl, kRing);
  else if (s == kCiv) m.ring.init(memCiv_, kSlotCiv, kRing);
  else m.ring.init(memAudio_, kSlotAudio, kAudioRing);
  sendPkt3(s);
}

void Link::connect() {
  uint32_t now = io_->millis();
  closeAll(true);
  err_[0] = 0;
  devName_[0] = 0;
  gotA8_ = authOk_ = false;
  audioSendSeq_ = 1;
  audioLastMs_ = 0;
  rxInit_ = false;
  lastN_ = 0;
  loginResends_ = 0;
  setState(kCtrlHello, now);
  openStream(kCtrl, now);
}

void Link::disconnect() {
  if (state_ >= kAuth && state_ <= kRunning && stm_[kCtrl].open) {
    uint32_t now = io_->millis();
    if (stm_[kCiv].open && stm_[kCiv].hs == 3) sendCivOpenClose(true, now);
    sendAuth(0x01, now);  // deauth
    for (int s = 0; s < kStreams; s++)
      if (stm_[s].open && stm_[s].remoteSid) sendDisconnectPkt(s);
  }
  closeAll();
  state_ = kIdle;
}

// ---------------------------------------------------------------------------
// packet builders
// ---------------------------------------------------------------------------
void Link::fillHeader(int s, uint8_t* p, size_t len, uint16_t type, uint16_t seq) {
  p[0] = (uint8_t)len; p[1] = (uint8_t)(len >> 8); p[2] = 0; p[3] = 0;
  p[4] = (uint8_t)type; p[5] = (uint8_t)(type >> 8);
  p[6] = (uint8_t)seq; p[7] = (uint8_t)(seq >> 8);
  put32be(p + 8, stm_[s].localSid);
  put32be(p + 12, stm_[s].remoteSid);
}

void Link::sendPkt3(int s) {
  uint8_t p[16];
  fillHeader(s, p, 16, 3, 0);
  sendRaw(s, p, 16);
  sendRaw(s, p, 16);
}
void Link::sendPkt6(int s) {
  uint8_t p[16];
  fillHeader(s, p, 16, 6, 1);
  sendRaw(s, p, 16);
  sendRaw(s, p, 16);
}
void Link::sendDisconnectPkt(int s) {
  uint8_t p[16];
  fillHeader(s, p, 16, 5, 0);
  sendRaw(s, p, 16);
  sendRaw(s, p, 16);
}

void Link::sendTracked(int s, uint8_t* p, size_t n, uint32_t now) {
  Stm& m = stm_[s];
  p[6] = (uint8_t)m.sendSeq;
  p[7] = (uint8_t)(m.sendSeq >> 8);
  m.ring.add(m.sendSeq, p, n);
  sendRaw(s, p, n);
  m.sendSeq++;
  bool idle = (n == 16 && p[4] == 0 && p[5] == 0);
  if (!idle) m.lastTracked = now;
}

void Link::sendIdle(int s, bool tracked, uint16_t seq, uint32_t now) {
  uint8_t p[16];
  fillHeader(s, p, 16, 0, seq);
  if (tracked) sendTracked(s, p, 16, now);
  else sendRaw(s, p, 16);
}

void Link::sendPing(int s, const uint8_t* replyId, uint16_t seq) {
  Stm& m = stm_[s];
  uint8_t p[21];
  fillHeader(s, p, 21, 7, seq);
  uint8_t id[4];
  if (replyId) {
    p[16] = 1;
    memcpy(id, replyId, 4);
  } else {
    p[16] = 0;
    id[0] = (uint8_t)io_->random32();
    id[1] = (uint8_t)m.pingInner;
    id[2] = (uint8_t)(m.pingInner >> 8);
    id[3] = 0x06;
    m.pingInner++;
  }
  memcpy(p + 17, id, 4);
  sendRaw(s, p, 21);
}

void Link::sendLogin(uint32_t now) {
  Stm& m = stm_[kCtrl];
  uint8_t p[128];
  memset(p, 0, sizeof(p));
  fillHeader(kCtrl, p, 128, 0, 0);
  p[19] = 0x70;
  p[20] = 0x01;
  p[23] = (uint8_t)m.inner;
  p[24] = (uint8_t)(m.inner >> 8);
  uint32_t r = io_->random32();
  p[26] = (uint8_t)r;
  p[27] = (uint8_t)(r >> 8);
  encodePasscode(cfg_.user, p + 64);
  encodePasscode(cfg_.pass, p + 80);
  memcpy(p + 96, "icom-pc", 7);
  sendTracked(kCtrl, p, 128, now);
  m.inner++;
}

void Link::sendAuth(uint8_t magic, uint32_t now) {
  Stm& m = stm_[kCtrl];
  uint8_t p[64];
  memset(p, 0, sizeof(p));
  fillHeader(kCtrl, p, 64, 0, 0);
  p[19] = 0x30;
  p[20] = 0x01;
  p[21] = magic;
  p[23] = (uint8_t)m.inner;
  p[24] = (uint8_t)(m.inner >> 8);
  memcpy(p + 26, authId_, 6);
  sendTracked(kCtrl, p, 64, now);
  m.inner++;
}

void Link::sendStreamsRequest(uint32_t now) {
  Stm& m = stm_[kCtrl];
  uint8_t p[144];
  memset(p, 0, sizeof(p));
  fillHeader(kCtrl, p, 144, 0, 0);
  p[19] = 0x80;
  p[20] = 0x01;
  p[21] = 0x03;
  p[23] = (uint8_t)m.inner;
  p[24] = (uint8_t)(m.inner >> 8);
  memcpy(p + 26, authId_, 6);
  memcpy(p + 32, a8Reply_, 16);
  memcpy(p + 64, "IC-705", 6);
  encodePasscode(cfg_.user, p + 96);
  p[112] = 0x01;  // rx enabled
  p[113] = 0x01;  // tx enabled
  p[114] = 0x04;  // rx codec: LPCM 16-bit mono
  p[115] = 0x04;  // tx codec: LPCM 16-bit mono
  put32be(p + 116, cfg_.audioRate);
  put32be(p + 120, cfg_.audioRate);
  put32be(p + 124, kRemotePort[kCiv]);
  put32be(p + 128, kRemotePort[kAudio]);
  put32be(p + 132, cfg_.txBufMs);
  p[136] = 0x01;
  sendTracked(kCtrl, p, 144, now);
  m.inner++;
}

void Link::sendCivOpenClose(bool close, uint32_t now) {
  Stm& m = stm_[kCiv];
  uint8_t p[22];
  fillHeader(kCiv, p, 22, 0, 0);
  p[16] = 0xc0;
  p[17] = 0x01;
  p[18] = 0x00;
  p[19] = (uint8_t)(m.inner >> 8);
  p[20] = (uint8_t)m.inner;
  p[21] = close ? 0x00 : 0x05;
  sendTracked(kCiv, p, 22, now);
  m.inner++;
}

bool Link::sendCiv(const uint8_t* frame, size_t n) {
  if (state_ != kRunning || n == 0 || n > 40) return false;
  Stm& m = stm_[kCiv];
  uint8_t p[21 + 40];
  fillHeader(kCiv, p, 21 + n, 0, 0);
  p[16] = 0xc1;
  p[17] = (uint8_t)n;
  p[18] = 0x00;
  p[19] = (uint8_t)(m.inner >> 8);
  p[20] = (uint8_t)m.inner;
  memcpy(p + 21, frame, n);
  sendTracked(kCiv, p, 21 + n, io_->millis());
  m.inner++;
  return true;
}

bool Link::sendAudio(const int16_t* pcm, size_t ns) {
  if (state_ != kRunning || ns == 0 || ns * 2 > 336) return false;
  size_t n = ns * 2;
  uint8_t p[24 + 336];
  fillHeader(kAudio, p, 24 + n, 0, 0);
  uint16_t aseq = (uint16_t)(audioSendSeq_ - 1);
  p[16] = 0x80;
  p[17] = 0x00;
  p[18] = (uint8_t)(aseq >> 8);
  p[19] = (uint8_t)aseq;
  p[20] = 0;
  p[21] = 0;
  p[22] = (uint8_t)(n >> 8);
  p[23] = (uint8_t)n;
  for (size_t i = 0; i < ns; i++) {
    p[24 + 2 * i] = (uint8_t)(pcm[i] & 0xff);
    p[25 + 2 * i] = (uint8_t)((uint16_t)pcm[i] >> 8);
  }
  sendTracked(kAudio, p, 24 + n, io_->millis());
  audioSendSeq_++;
  return true;
}

// ---------------------------------------------------------------------------
// receive side
// ---------------------------------------------------------------------------
void Link::handlePing(int s, const uint8_t* r) {
  Stm& m = stm_[s];
  if (r[16] == 0) {            // request from the radio: answer it
    if (m.pingOn) sendPing(s, r + 17, get16le(r + 6));
  }                            // else: reply to our own ping, nothing to do
}

void Link::handleRetransmit(int s, const uint8_t* r, size_t n) {
  Stm& m = stm_[s];
  uint32_t now = io_->millis();
  auto resend = [&](uint16_t sq) {
    size_t l = 0;
    const uint8_t* d = m.ring.get(sq, &l);
    if (d) {
      sendRaw(s, d, l);
      sendRaw(s, d, l);
    } else {
      sendIdle(s, false, sq, now);
      sendIdle(s, false, sq, now);
    }
  };
  if (r[0] == 0x10) {
    resend(get16le(r + 6));
  } else {
    for (size_t i = 16; i + 4 <= n; i += 4) {
      uint16_t a = get16le(r + i), b = get16le(r + i + 2);
      int guard = 0;
      for (;;) {
        resend(a);
        if (a == b || ++guard > 20) break;
        a++;
      }
    }
  }
}

void Link::handle(int s, const uint8_t* r, size_t n, uint32_t now) {
  Stm& m = stm_[s];
  m.lastRx = now;

  // handshake replies (16 bytes)
  if (n == 16) {
    if (m.hs == 1 && r[0] == 0x10 && r[4] == 0x04 && r[5] == 0 && r[6] == 0 && r[7] == 0) {
      m.remoteSid = get32be(r + 8);
      m.hs = 2;
      m.hsNext = now + 500;
      sendPkt6(s);
      return;
    }
    if (m.hs == 2 && r[0] == 0x10 && r[4] == 0x06 && r[5] == 0 && r[6] == 0x01 && r[7] == 0) {
      m.hs = 3;
      return;
    }
  }
  // ping (21 bytes, type 7)
  if (n == 21 && r[1] == 0 && r[2] == 0 && r[3] == 0 && r[4] == 0x07 && r[5] == 0) {
    handlePing(s, r);
    return;
  }
  // retransmit requests
  if (n >= 16 && r[1] == 0 && r[2] == 0 && r[3] == 0 && r[4] == 0x01 && r[5] == 0 &&
      (r[0] == 0x10 || r[0] == 0x18)) {
    handleRetransmit(s, r, n);
    return;
  }
  if (s == kCtrl) handleCtrl(r, n, now);
  else if (s == kCiv) handleCiv(r, n);
  else handleAudio(r, n);
}

void Link::handleCtrl(const uint8_t* r, size_t n, uint32_t now) {
  Stm& m = stm_[kCtrl];
  if (state_ == kLogin && n == 96 && r[0] == 0x60 && r[4] == 0 && r[5] == 0 && r[6] == 0x01 && r[7] == 0) {
    if (r[48] == 0xff && r[49] == 0xff && r[50] == 0xff && r[51] == 0xfe) {
      fail("invalid username/password");
      return;
    }
    memcpy(authId_, r + 26, 6);
    m.pingOn = true;
    m.pingSeq = 2;
    m.nextPing = now + 3000;
    sendAuth(0x02, now);
    m.idleOn = true;
    m.nextIdle = now + 1000;
    sendAuth(0x05, now);
    nextReauth_ = now + 60000;
    setState(kAuth, now);
    return;
  }
  if (n == 168 && r[0] == 0xa8 && r[1] == 0 && r[2] == 0 && r[3] == 0 && r[4] == 0 && r[5] == 0) {
    memcpy(a8Reply_, r + 66, 16);
    gotA8_ = true;
  } else if (n == 64 && r[0] == 0x40 && r[1] == 0 && r[2] == 0 && r[3] == 0 && r[4] == 0 && r[5] == 0) {
    if (r[21] == 0x05) authOk_ = true;
  } else if (n == 80 && r[0] == 0x50 && r[1] == 0 && r[2] == 0 && r[3] == 0 && r[4] == 0 && r[5] == 0) {
    if (r[48] == 0xff && r[49] == 0xff && r[50] == 0xff) {
      fail(state_ == kRunning ? "auth failed" : "auth failed, try rebooting the radio");
      return;
    }
    if (r[48] == 0 && r[49] == 0 && r[50] == 0 && r[64] == 0x01) {
      fail("radio disconnected us");
      return;
    }
  } else if (state_ == kStreamsReq && n == 144 && r[0] == 0x90 && r[1] == 0 && r[2] == 0 && r[3] == 0 &&
             r[4] == 0 && r[5] == 0 && r[96] == 1) {
    // streams granted: ids can change relative to the login phase
    m.remoteSid = get32be(r + 8);
    m.localSid = get32be(r + 12);
    memcpy(authId_, r + 26, 6);
    size_t i = 0;
    for (; i < sizeof(devName_) - 1 && 64 + i < n && r[64 + i]; i++) devName_[i] = (char)r[64 + i];
    devName_[i] = 0;
    startStreams(now);
    return;
  }
  if (state_ == kAuth && gotA8_ && authOk_) {
    sendStreamsRequest(now);
    setState(kStreamsReq, now);
  }
}

void Link::handleCiv(const uint8_t* r, size_t n) {
  Stm& m = stm_[kCiv];
  if (n < 22 || r[16] != 0xc1 || (size_t)(r[0] - 0x15) != r[17]) return;
  uint16_t sq = get16le(r + 6);
  if (m.haveRxSeq) {
    int16_t d = (int16_t)(sq - m.lastRxSeq);
    if (d <= 0) return;  // duplicate / old
  }
  m.lastRxSeq = sq;
  m.haveRxSeq = true;
  size_t l = r[17];
  if (21 + l > n) l = n - 21;
  civPkts_++;
  if (civCb_) civCb_(ctx_, r + 21, l);
}

void Link::handleAudio(const uint8_t* r, size_t n) {
  Stm& m = stm_[kAudio];
  if (n <= 24 || r[4] != 0 || r[5] != 0) return;
  uint32_t declared = (uint32_t)r[0] | ((uint32_t)r[1] << 8);
  if (declared != n) return;
  size_t bytes = n - 24;
  bytes &= ~(size_t)1;
  if (bytes == 0 || bytes > 1400) return;
  uint16_t sq = get16le(r + 6);
  int16_t pcm[700];
  size_t ns = bytes / 2;
  for (size_t i = 0; i < ns; i++) pcm[i] = (int16_t)(r[24 + 2 * i] | (r[25 + 2 * i] << 8));
  uint32_t now = io_->millis();
  if (audioLastMs_ && now - audioLastMs_ > audioMaxGap_) audioMaxGap_ = now - audioLastMs_;
  audioLastMs_ = now;
  (void)m;

  if (ns > (size_t)kRxSlotSamples) {                   // 24/48 kHz frames: too big for the reorder buffer, pass straight through
    audioPkts_++;
    audioBypass_++;
    if (audioCb_) audioCb_(ctx_, pcm, ns);
    return;
  }
  if (!rxInit_) { resetAudioRx(); rxInit_ = true; rxNext_ = sq; }
  int16_t d = (int16_t)(sq - rxNext_);
  if (d < 0) { audioLate_++; return; }                 // already delivered or given up on
  if (d > 150) { resetAudioRx(); rxInit_ = true; rxNext_ = sq; d = 0; }   // long stall: start over
  while (d >= kRxSlots) { audioAdvance(); d--; }       // buffer full: give up on the oldest missing frame
  int i = sq & (kRxSlots - 1);
  if (rxUsed_[i] && rxSeq_[i] == sq) { audioLate_++; return; }   // duplicate
  memcpy(rxBuf_[i], pcm, ns * sizeof(int16_t));
  rxLen_[i] = (uint16_t)ns;
  rxSeq_[i] = sq;
  rxUsed_[i] = true;
  if (rxMissAt_[i]) { audioRecovered_++; rxMissAt_[i] = 0; }
  audioPkts_++;
  // frames between the next expected one and this one are missing: ask the radio to send them again
  for (uint16_t q = rxNext_; q != sq; q++) {
    int j = q & (kRxSlots - 1);
    if (!rxUsed_[j] && !rxMissAt_[j]) {
      rxMissAt_[j] = now ? now : 1;
      rxReqAt_[j] = now;
      rxReqN_[j] = 1;
      requestAudioRetransmit(q);
    }
  }
  audioDrain();
}

void Link::resetAudioRx() {
  for (int k = 0; k < kRxSlots; k++) { rxUsed_[k] = false; rxMissAt_[k] = 0; rxReqAt_[k] = 0; rxReqN_[k] = 0; }
  lastN_ = 0;
}

void Link::audioDeliver(const int16_t* p, size_t n) {
  if (n <= (size_t)kRxSlotSamples) { memcpy(lastPcm_, p, n * sizeof(int16_t)); lastN_ = n; }
  if (audioCb_) audioCb_(ctx_, p, n);
}

// Hand the next expected frame to the decoders: the real one if it is here, otherwise a repeat of the
// previous frame (keeps the time axis intact; a lost 20 ms frame would otherwise upset the PSK31 symbol clock).
void Link::audioAdvance() {
  int i = rxNext_ & (kRxSlots - 1);
  if (rxUsed_[i] && rxSeq_[i] == rxNext_) {
    rxUsed_[i] = false;
    audioDeliver(rxBuf_[i], rxLen_[i]);
  } else {
    audioLost_++;
    if (lastN_) { audioConcealed_++; audioDeliver(lastPcm_, lastN_); }
  }
  rxMissAt_[i] = 0;
  rxNext_++;
}

void Link::audioDrain() {
  for (;;) {
    int i = rxNext_ & (kRxSlots - 1);
    if (!(rxUsed_[i] && rxSeq_[i] == rxNext_)) break;
    audioAdvance();
  }
}

// Retransmit request: a 16-byte control packet, type 1, whose sequence field is the missing sequence number.
// (The same single-sequence form the radio sends us; handled in handleRetransmit.)
void Link::requestAudioRetransmit(uint16_t seq) {
  if (!stm_[kAudio].open || !cfg_.audioRetransmit) return;
  uint8_t p[16];
  fillHeader(kAudio, p, 16, 1, seq);
  sendRaw(kAudio, p, 16);
  audioReq_++;
}

void Link::audioTick(uint32_t now) {
  if (!rxInit_) return;
  for (;;) {                                           // give up on a missing frame after kHoldMs
    int i = rxNext_ & (kRxSlots - 1);
    if (rxUsed_[i] && rxSeq_[i] == rxNext_) { audioAdvance(); continue; }
    // signed differences: 'now' is taken at the start of poll(), frames processed since then carry a later time stamp
    if (rxMissAt_[i] && (int32_t)(now - rxMissAt_[i]) > (int32_t)kHoldMs) { audioAdvance(); continue; }
    break;
  }
  for (int k = 0; k < kRxSlots; k++) {                 // ask again for frames still missing
    uint16_t q = (uint16_t)(rxNext_ + k);
    int j = q & (kRxSlots - 1);
    if (rxMissAt_[j] && !rxUsed_[j] && rxReqN_[j] < 3 && (int32_t)(now - rxReqAt_[j]) >= (int32_t)kRetryMs) {
      rxReqAt_[j] = now;
      rxReqN_[j]++;
      requestAudioRetransmit(q);
    }
  }
}

// ---------------------------------------------------------------------------
// state machine
// ---------------------------------------------------------------------------
void Link::startStreams(uint32_t now) {
  openStream(kCiv, now);
  if (state_ == kFailed) return;
  openStream(kAudio, now);
  if (state_ == kFailed) return;
  setState(kStreamsHello, now);
}

void Link::tickHandshake(int s, uint32_t now) {
  Stm& m = stm_[s];
  if (!m.open || m.hs == 0 || m.hs == 3) return;
  if ((int32_t)(now - m.hsNext) < 0) return;
  m.hsNext = now + 500;
  if (m.hs == 1) sendPkt3(s);
  else if (m.hs == 2) sendPkt6(s);
}

void Link::tickPeriodic(int s, uint32_t now) {
  Stm& m = stm_[s];
  if (!m.open) return;
  if (m.pingOn && (int32_t)(now - m.nextPing) >= 0) {
    sendPing(s, nullptr, m.pingSeq++);
    m.nextPing = now + 3000;
  }
  if (m.idleOn && (int32_t)(now - m.nextIdle) >= 0) {
    sendIdle(s, true, 0, now);
    m.nextIdle = now + ((now - m.lastTracked) >= 1000 ? 1000 : 100);
  }
}

void Link::poll() {
  if (!io_) return;
  uint32_t now = io_->millis();

  for (int s = 0; s < kStreams; s++) {
    if (!stm_[s].open) continue;
    int n;
    int guard = 0;
    while (stm_[s].open && (n = io_->recv(s, rx_, kRx)) > 0 && guard++ < 64) {
      handle(s, rx_, (size_t)n, now);
      if (state_ == kFailed || state_ == kIdle) return;
    }
  }

  switch (state_) {
    case kCtrlHello: {
      tickHandshake(kCtrl, now);
      Stm& m = stm_[kCtrl];
      if (m.hs == 3) {
        m.sendSeq = 1;
        sendLogin(now);
        loginResends_ = 0;
        setState(kLogin, now);
      } else if (now - stateSince_ > 6000) {
        fail("radio not answering (check IP / network control)");
      }
      break;
    }
    case kLogin: {
      tickPeriodic(kCtrl, now);
      // UDP can lose packets and the radio can be slow to answer: resend the (tracked) login
      // packet a couple of times, exactly as a retransmission would.
      uint32_t el = now - stateSince_;
      if (loginResends_ < 3 && el > 1500u * (uint32_t)(loginResends_ + 1)) {
        size_t l = 0;
        const uint8_t* d = stm_[kCtrl].ring.get(1, &l);
        if (d) sendRaw(kCtrl, d, l);
        loginResends_++;
        io_->log("no login reply yet, resending login");
      }
      if (el > 8000) fail("login timeout (radio never answered the login)");
      break;
    }
    case kAuth:
    case kStreamsReq:
      tickPeriodic(kCtrl, now);
      if (now - stateSince_ > 6000) fail("stream request timeout");
      break;
    case kStreamsHello: {
      tickPeriodic(kCtrl, now);
      tickHandshake(kCiv, now);
      tickHandshake(kAudio, now);
      if (stm_[kCiv].hs == 3 && stm_[kAudio].hs == 3) {
        Stm& c = stm_[kCiv];
        c.pingOn = true; c.pingSeq = 1; c.nextPing = now + 3000;
        c.sendSeq = 1;
        c.idleOn = true; c.nextIdle = now + 1000;
        sendCivOpenClose(false, now);
        Stm& a = stm_[kAudio];
        a.pingOn = true; a.pingSeq = 1; a.nextPing = now + 3000;
        a.sendSeq = 0;
        audioSendSeq_ = 1;
        setState(kRunning, now);
      } else if (now - stateSince_ > 6000) {
        fail("CI-V/audio stream handshake timeout");
      }
      break;
    }
    case kRunning:
      audioTick(now);
      for (int s = 0; s < kStreams; s++) tickPeriodic(s, now);
      if ((int32_t)(now - nextReauth_) >= 0) {
        sendAuth(0x05, now);
        nextReauth_ = now + 60000;
      }
      if (now - stm_[kCtrl].lastRx > 8000) fail("control link timeout");
      break;
    default:
      break;
  }
}

}  // namespace icom
