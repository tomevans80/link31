// icom.h - client for Icom's UDP network remote protocol (IC-705 / IC-7300 / IC-9700 ...).
//
// Three UDP streams to the radio: control (50001), CI-V (50002) and audio (50003).
// Written from the reverse-engineered kappanhang (HA2NON/ES1AKOS) and wfview
// implementations; NOT officially documented by Icom.
//
// Portable: all platform access goes through icom::Io so the same code runs on the
// ESP32 (WiFiUDP) and on a PC against a mock radio (see test/test_icom.cpp).
#pragma once
#include <stdint.h>
#include <stddef.h>

namespace icom {

enum { kCtrl = 0, kCiv = 1, kAudio = 2, kStreams = 3 };
static const uint16_t kRemotePort[kStreams] = {50001, 50002, 50003};

struct Io {
  virtual ~Io() {}
  virtual uint32_t millis() = 0;
  virtual bool open(int stream, uint16_t remotePort) = 0;   // create + bind socket
  virtual void close(int stream) = 0;
  virtual bool send(int stream, const uint8_t* d, size_t n) = 0;
  virtual int recv(int stream, uint8_t* buf, size_t cap) = 0;  // >0 = bytes, 0 = none
  virtual uint32_t localSid(int stream) = 0;  // (local IPv4 low 16 bits << 16) | local port
  virtual uint32_t random32() = 0;
  virtual void log(const char* msg) { (void)msg; }
};

struct Config {
  const char* user = "";
  const char* pass = "";
  uint32_t audioRate = 8000;   // 8000 / 16000 / 24000 / 48000 (16-bit mono LPCM)
  uint16_t txBufMs = 300;
  bool audioRetransmit = true;   // ask the radio to resend lost audio frames (set false if the radio misbehaves)
};

typedef void (*CivCb)(void* ctx, const uint8_t* frame, size_t n);
typedef void (*AudioCb)(void* ctx, const int16_t* pcm, size_t nSamples);

// Obfuscated credential encoding used in the login packet.
void encodePasscode(const char* s, uint8_t out[16]);

class Link {
 public:
  enum State {
    kIdle, kCtrlHello, kLogin, kAuth, kStreamsReq, kStreamsHello, kRunning, kFailed
  };

  Link();
  void begin(Io* io, const Config& cfg);
  void setCallbacks(void* ctx, CivCb civ, AudioCb audio) { ctx_ = ctx; civCb_ = civ; audioCb_ = audio; }

  void connect();      // start (or restart) the connection sequence
  void disconnect();   // polite shutdown, then idle
  void poll();         // call every few ms

  State state() const { return state_; }
  const char* stateName() const;
  const char* error() const { return err_; }
  bool running() const { return state_ == kRunning; }
  const char* deviceName() const { return devName_; }
  uint32_t failedAtMs() const { return failedAt_; }

  // CI-V frame (FE FE .. FD) to the radio. Returns false if not connected/too long.
  bool sendCiv(const uint8_t* frame, size_t n);
  // 16-bit mono PCM at cfg.audioRate, at most 160 samples per call (one 20 ms frame at 8 kHz).
  bool sendAudio(const int16_t* pcm, size_t nSamples);

  // statistics
  uint32_t audioPackets() const { return audioPkts_; }
  uint32_t audioLost() const { return audioLost_; }        // frames that never arrived, even after retransmit requests (concealed)
  uint32_t audioLate() const { return audioLate_; }        // duplicates / frames arriving after they were given up on
  uint32_t audioConcealed() const { return audioConcealed_; }   // lost frames replaced by a repeat of the previous one
  uint32_t audioRecovered() const { return audioRecovered_; }   // frames that arrived out of order after a gap (e.g. retransmitted)
  uint32_t audioRetransmitRequests() const { return audioReq_; }
  // ms since a packet of any kind last arrived on a stream (kCtrl / kCiv / kAudio; pings count) and since the last audio frame
  uint32_t streamAgeMs(int s) const { return io_ ? io_->millis() - stm_[s].lastRx : 0; }
  uint32_t audioAgeMs() const { return (io_ && audioLastMs_) ? io_->millis() - audioLastMs_ : 0; }
  uint32_t audioBypassed() const { return audioBypass_; }   // frames too big for the reorder buffer (not 8 kHz): no resend recovery for these
  // longest time between two audio frames since the last call (ms): shows Wi-Fi power-save bursts / stalls
  uint32_t takeAudioMaxGapMs() { uint32_t g = audioMaxGap_; audioMaxGap_ = 0; return g; }
  uint32_t civPackets() const { return civPkts_; }

 private:
  static const size_t kRx = 1500;
  static const int kRing = 24;
  static const size_t kSlotCtrl = 160, kSlotCiv = 64, kSlotAudio = 360;
  static const int kAudioRing = 24;   // sent-audio frames kept for retransmit requests (24 x 20 ms; was 40, trimmed to save internal RAM)

  struct Ring {
    uint8_t* mem = nullptr;
    size_t slot = 0;
    int n = 0;
    uint16_t seq[48];
    uint16_t len[48];
    bool used[48];
    int head = 0;
    void init(uint8_t* m, size_t s, int cnt);
    void add(uint16_t sq, const uint8_t* d, size_t l);
    const uint8_t* get(uint16_t sq, size_t* l) const;
  };

  struct Stm {
    bool open = false;
    uint32_t localSid = 0, remoteSid = 0;
    uint16_t sendSeq = 0;
    uint16_t inner = 0;          // CI-V / auth inner sequence
    uint8_t hs = 0;              // 0 none, 1 waiting pkt4, 2 waiting pkt6, 3 done
    uint32_t hsNext = 0;
    bool pingOn = false;
    uint16_t pingSeq = 0, pingInner = 0x8304;
    uint32_t nextPing = 0;
    bool idleOn = false;
    uint32_t nextIdle = 0, lastTracked = 0;
    uint32_t lastRx = 0;
    uint16_t lastRxSeq = 0;
    bool haveRxSeq = false;
    Ring ring;
  };

  void closeAll(bool polite = false);
  void setState(State st, uint32_t now);
  void fail(const char* why);
  void openStream(int s, uint32_t now);
  void handle(int s, const uint8_t* r, size_t n, uint32_t now);
  void handleCtrl(const uint8_t* r, size_t n, uint32_t now);
  void handleCiv(const uint8_t* r, size_t n);
  void handleAudio(const uint8_t* r, size_t n);
  void handlePing(int s, const uint8_t* r);
  void handleRetransmit(int s, const uint8_t* r, size_t n);
  void tickHandshake(int s, uint32_t now);
  void tickPeriodic(int s, uint32_t now);

  void fillHeader(int s, uint8_t* p, size_t len, uint16_t type, uint16_t seq);
  void sendRaw(int s, const uint8_t* p, size_t n) { io_->send(s, p, n); }
  void sendTracked(int s, uint8_t* p, size_t n, uint32_t now);
  void sendPkt3(int s);
  void sendPkt6(int s);
  void sendIdle(int s, bool tracked, uint16_t seq, uint32_t now);
  void sendDisconnectPkt(int s);
  void sendPing(int s, const uint8_t* replyId, uint16_t seq);
  void sendLogin(uint32_t now);
  void sendAuth(uint8_t magic, uint32_t now);
  void sendStreamsRequest(uint32_t now);
  void sendCivOpenClose(bool close, uint32_t now);
  void startStreams(uint32_t now);

  Io* io_ = nullptr;
  Config cfg_;
  void* ctx_ = nullptr;
  CivCb civCb_ = nullptr;
  AudioCb audioCb_ = nullptr;

  State state_ = kIdle;
  uint32_t stateSince_ = 0;
  char err_[64] = "";
  char devName_[24] = "";
  uint32_t failedAt_ = 0;

  Stm stm_[kStreams];
  uint8_t authId_[6] = {0};
  uint8_t a8Reply_[16] = {0};
  bool gotA8_ = false, authOk_ = false;
  uint32_t nextReauth_ = 0;
  uint32_t loginResendAt_ = 0;
  int loginResends_ = 0;
  uint16_t audioSendSeq_ = 1;
  uint32_t audioPkts_ = 0, civPkts_ = 0;
  uint32_t audioLost_ = 0, audioLate_ = 0, audioConcealed_ = 0, audioMaxGap_ = 0, audioLastMs_ = 0;
  uint32_t audioRecovered_ = 0, audioReq_ = 0, audioBypass_ = 0;
  static const int kRxSlotSamples = 160;   // one 20 ms frame at 8 kHz; other sample rates bypass the reorder buffer
  int16_t lastPcm_[kRxSlotSamples];
  size_t lastN_ = 0;

  // Receive reorder buffer for the audio stream. A missing frame is requested again from the radio
  // (retransmit request, type 1) and the later frames wait up to kHoldMs for it, so the decoders get the
  // real samples in the right order. If it never arrives it is concealed (previous frame repeated) so the
  // time axis stays intact. The hold must cover Wi-Fi power-save delivery (up to ~100 ms).
  static const int kRxSlots = 16;   // 16 x 20 ms = 320 ms of look-ahead
  static const uint32_t kHoldMs = 160, kRetryMs = 45;
  int16_t rxBuf_[kRxSlots][kRxSlotSamples];
  uint16_t rxLen_[kRxSlots], rxSeq_[kRxSlots];
  bool rxUsed_[kRxSlots];
  uint32_t rxMissAt_[kRxSlots], rxReqAt_[kRxSlots];
  uint8_t rxReqN_[kRxSlots];
  uint16_t rxNext_ = 0;
  bool rxInit_ = false;
  void resetAudioRx();
  void audioDeliver(const int16_t* p, size_t n);
  void audioAdvance();
  void audioDrain();
  void audioTick(uint32_t now);
  void requestAudioRetransmit(uint16_t seq);

  uint8_t rx_[kRx];
  uint8_t memCtrl_[kRing * kSlotCtrl];
  uint8_t memCiv_[kRing * kSlotCiv];
  uint8_t memAudio_[kAudioRing * kSlotAudio];
};

}  // namespace icom
