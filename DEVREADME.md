# IC-705 CW / PSK31 terminal for the M5Stack Core2

Firmware for the M5Stack **Core2** that talks to an **ICOM IC-705** over Wi-Fi using Icom's
network remote protocol (the same one RS-BA1, wfview and kappanhang use).

* **CW receive:** narrow, speed-adaptive filtering with tone tracking and noise-floor tracking; self-calibrating
  speed (about 5 to 40 wpm). See "CW decoding in noise" below.
* **CW transmit:** type on the touchscreen; text is sent with the radio's own keyer (CI-V `17`).
* **PSK31 receive:** BPSK31 decoder with AFC, symbol-clock recovery, squelch, waterfall (tap to tune).
* **PSK31 transmit:** Varicode/BPSK31 audio sent to the radio over the network audio stream, PTT via CI-V.
  *Disabled until you arm it in the menu.*
* Frequency / mode / S-meter / TX status display, touch on-screen keyboard.
* A Bluetooth keyboard is **not** included yet (planned as a later add-on).

> **Status, honestly:** the DSP and the network protocol client are tested on a PC (see
> "What has been tested"), but this firmware has **not been compiled for the ESP32 or run on a real
> radio**: the environment I built it in cannot reach the ESP32 toolchain servers. Expect a first
> build to need small fixes, and expect to debug the first connection using the serial log.

## Build and flash

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or `pip install platformio`).
2. Open this folder, connect the Core2 by USB-C.
3. `pio run -t upload`, then `pio device monitor` for the log (115200 baud).

Library: `m5stack/M5Unified` (pulled automatically). If a newer M5Unified changes an API, the places
to look are the display calls in `main.cpp` (`M5.Display`, `M5Canvas`, `M5.Touch.getDetail()`).

## Radio setup (IC-705)

Menu names differ between firmware versions; look for the equivalents:

* Enable Wi-Fi: either the radio's own **access point** (the Core2 then joins it), or put the radio
  on your home 2.4 GHz network.
* Enable **Network Control / remote** and create a **Network User** (ID + password). Those are the
  "Radio user / Radio password" in this firmware (not the Wi-Fi password).
* CI-V address default is `A4` (menu setting in the app if yours differs).
* **CW TX:** radio in CW mode with **break-in (semi or full) on**, otherwise the keyer only makes sidetone.
* **PSK31 TX:** mode USB-D (the app sets USB + DATA when you switch to PSK31), and the **DATA MOD input
  must be set to the LAN/WLAN (network audio) source**. Use a **dummy load and low power** for first tests;
  adjust "PSK31 TX level" so ALC stays at zero.

## First run

1. The Core2 boots into **SETTINGS** if no Wi-Fi is stored. Tap a row to edit it with the on-screen keyboard:
   Wi-Fi SSID/password, Radio user/password. Leave **Radio IP** empty when the radio is in access-point
   mode (the Wi-Fi gateway is then the radio); enter the IP if the radio is on your home network.
2. Tap **CONNECT**. The header shows Wi-Fi/link status (`linked` when the radio session is up).
3. Mode switch: virtual button **A** (below the screen) toggles CW / PSK31 and sets the radio mode.
   **B** clears the receive window, **C** opens the menu.

## Using it

* **Bluetooth keyboard:** left/right arrows nudge the CW pitch / PSK31 carrier by 30 Hz (`kNudgeHz`; CW auto-pitch is switched off). In CW mode up/down arrows change the send speed by 1 wpm (5 to 48; updates the CW TX speed setting and the radio's keyer).
  Shortcuts: **Ctrl+Tab** = TYPE, **Ctrl+T** = TUNE, **Ctrl+M** = menu.
* **Waterfall:** tap to set the CW pitch (auto-pitch is switched off) or the PSK31 carrier.
* **Live sending:** whatever you type is transmitted as you type it, left to right, with no SEND step. The text line shows
  **orange = already sent (locked, cannot be deleted)** and **green = not sent yet (Backspace still works)**. CW: each character
  goes to the radio's keyer just before the keyer runs dry, so only a character or two are ahead of the keying; PSK31: the first
  character starts the over (PTT, preamble) and the rest follow as typed. Enter = line break (PSK31) or word space (CW).
  Needs a connected radio; PSK31 also needs Allow PSK31 TX. A new line starts by itself once the previous one has been sent.
* **TYPE** opens the touch keyboard, which types into the same live line (**DONE** goes back and leaves everything running,
  **X** goes back and drops the text not yet sent). **TUNE** starts/stops
  a steady tune signal (PSK mode: 1500 Hz audio tone, needs Allow PSK31 TX; CW mode: carrier by PTT; auto-off after 20 s). **STOP** aborts
  (sends CI-V "stop CW", drops the unsent text, drops PTT immediately in PSK31). **Esc** on the Bluetooth keyboard = STOP while
  transmitting, otherwise clears the line.
* PSK31: after the last character the carrier idles 3 s (`kPskHangMs`), then the over ends and PTT is released.
* **SWR:** while transmitting (CW, PSK31 or TUNE) the header shows the radio's SWR reading in place of the S-meter
  (peak-held; bar full scale = 4.0).
* Settings: CW TX speed (also sent to the radio's keyer), PSK level, **Allow PSK31 TX**, CW auto-pitch,
  CW squelch.

## CW decoding in noise

How it works: the audio is mixed to baseband at the tone, filtered with a bandwidth that follows the measured
sending speed (about 1 / dot-length, so ~17 Hz at 18 wpm), and a frequency-locked loop follows tone drift and
small tuning errors (+/-15 Hz). The key-up noise floor and key-down level are tracked separately, brief dropouts
inside an element and brief blips inside a gap are absorbed, and a character that is not valid Morse is repaired by
flipping its least certain element. Characters are only printed while the key-down level is clearly above the noise
floor, so pure noise prints nothing.

Header line in CW mode: `CW <tone>Hz <wpm>wpm BW<filter Hz> x<key-down/noise> AUTO SIG`.
`x` is how far above the noise floor the key-down envelope sits (needs to exceed the squelch, 2.8 by default).

Tuning knobs:
* **CW squelch (x0.1)** in the menu: default 28 (= 2.8). Lower (e.g. 22) to decode weaker signals at the cost of
  occasional junk characters from noise; higher (e.g. 35) to be stricter.
* **Tone:** leave auto-pitch on, or tap the waterfall on the signal. Within about +/-15 Hz the decoder follows by itself.
* **On the radio:** a narrow CW filter (e.g. 250-500 Hz) helps a lot. The radio's **noise blanker** is much better against
  static crashes than anything done after the audio; use it if you have QRN. Use noise reduction (NR) sparingly,
  since it can smear CW keying.

Serial diagnostics: in CW mode the firmware prints one line per second (when `kTrace` is on), e.g.
`[cw] f=601.2 bw=17 wpm=18.3 noise=12.4 peak=88.0 keydown/noise=6.20 OPEN`. If decoding is poor, copy a few
seconds of these lines (plus a note of what you could hear); they show the filter, speed estimate and levels.

Benchmark (PC, synthetic, 12 random QSO messages per point, white noise, SNR quoted in 500 Hz, human timing jitter):

| Character error rate, 18 wpm | 20 dB | 14 dB | 10 dB | 7 dB | 4 dB | 2 dB | 0 dB |
|---|---|---|---|---|---|---|---|
| first release | 0% | 0% | 0% | 40% | 93% | 99% | 100% |
| this decoder | 0% | 0% | 0% | 0% | 0.8% | 1.6% | 10% |
| with radio-like AGC pumping, 50% QSB fading, tone drift, 5 Hz mistuning | 0.5% | 0.5% | 0% | 0.5% | 14% | 35% | 62% |

Known weak spots: heavy static crashes (about 3 per second, several times the signal level) still cause errors; an
equally strong neighbouring station within about 50 Hz can capture the tone tracker; deep fades cost characters
(a person would lose them too); and the first character after a pause or a speed change can still occasionally be wrong at very weak signal levels
(about 0 dB in 500 Hz). The first character after a pause is now re-measured and decoded much more reliably (see HANDOFF.md).
These numbers come from synthetic signals; real band conditions will differ.

## If something does not work

| Symptom | Likely cause |
|---|---|
| `radio not answering` | Wrong IP, radio not in network-control mode, Core2 not on the radio's network |
| `invalid username/password` | The **network user** credentials (not Wi-Fi) |
| `auth failed, try rebooting the radio` | Radio kept a stale session; power-cycle it |
| Linked but no audio / no decode | Try `kAudioRate = 16000` (or 48000) in `main.cpp`; check the serial log |
| CW decodes garbage | Tap the waterfall on the tone, or enable auto-pitch; adjust radio AF level |
| PSK31 `Q` stays low | Tune within ~15 Hz of the signal; the radio's filter must include the carrier |

PSK31 transmit requires `kAudioRate = 8000` (the generated audio is 8 kHz).

## Debugging the radio link (serial trace)

`kTrace` in `main.cpp` (on by default) prints the radio protocol to the serial port at 115200 baud:
`[TX ctrl 16] ...` / `[RX ctrl 16] ...` hex dumps and `state -> ...` lines. Credential bytes are masked (`xx`).
Open the Serial Monitor **before** the Core2 connects (press its reset button, or MENU > CONNECT) and
copy the output. Things the trace tells you:

* `RX ctrl 16 ... 04 00` then `06 00 01 00`: the radio answered the hello handshake.
* `TX ctrl 128`: the login was sent; `RX ctrl 96`: the radio answered it. No `RX ctrl 96` = the radio ignored it.
* Only one client can hold the radio's network session. Close wfview (and any other remote software) before
  testing, and power-cycle the radio if an earlier attempt left a stale session.

## Source layout

| File | What it is |
|---|---|
| `src/dsp.h` | Portable DSP: CW decoder, BPSK31 decoder + modulator, Varicode, FFT, decimator |
| `src/icom.h/.cpp` | Icom network protocol client (control/CI-V/audio streams, login, keep-alive, retransmit) |
| `src/civ.h` | CI-V frame builders/parsers |
| `src/main.cpp` | Core2 app: Wi-Fi, net task, UI, keyboard, settings |
| `hosttest/` | PC tests: DSP round trips and an end-to-end test against a mock radio |

## What has been tested (on a PC, not on hardware)

Run `hosttest/run_icom_test.sh` and build/run `hosttest/test_dsp.cpp` (instructions in the files).

* **PSK31:** modulator-to-decoder round trips with noise (down to 12 dB SNR in 3 kHz), +4/-6 Hz tuning error,
  and +200 ppm sample-clock error all decode exactly. At -500 ppm the first character is lost. Noise-only input
  produces no output with the squelch on.
* **CW:** 5/12/20/30/40 wpm decode exactly; works at 10 dB SNR (3 kHz); 12 Hz tuning error; speed change between
  overs; silent on noise (0 characters in 120 s of pure noise). `hosttest/bench_cw.cpp` sweeps SNR with fading, AGC
  pumping, drift, static crashes and neighbouring stations (see "CW decoding in noise").
* **Varicode** table taken from fldigi's source and checked (128 unique codes).
* **Protocol client:** against a mock radio written from the kappanhang reference's packet layouts:
  handshake, login (including wrong-password rejection), authentication, stream request, CI-V frequency/mode/
  S-meter queries, keep-alive pings and retransmit handling, receiving a 8 kHz audio stream, sending TX audio,
  clean disconnect. The mock was written by me from the same source, so it proves the code is internally
  consistent and matches the reference layouts, **not** that a real IC-705 behaves identically.
* `main.cpp` passes a syntax/type check against stub headers; it has not been built against the real libraries.

## Known limits and things I'm unsure of

* Icom's protocol is undocumented officially. Sample rates other than 8 kHz, and the audio codec byte
  (`0x04` = 16-bit mono LPCM), follow wfview/kappanhang; unverified on your firmware.
* CI-V details taken from memory of the IC-705 CI-V manual and worth confirming: `17 FF` as "stop CW message",
  `14 0C` keyer speed range (0 to 255 = 6 to 48 wpm), `1A 06` data-mode command.
* Retransmission requests *from* the radio are answered; this client does not request retransmits of lost
  radio packets (a lost audio frame is just a glitch).
* CW decoding assumes normal timing (not Farnsworth) and a steady audio tone; PSK31 AFC pulls up to +/-30 Hz (a squared-carrier frequency estimator, then a decision-directed fine AFC).
* The squelch thresholds (PSK31 quality 0.80, CW key-down/noise 2.8) are tuned on synthetic signals and
  will need adjusting on real band noise; both are discussed above.

## Safety

PSK31 TX is off by default. When you enable it the radio transmits whatever audio level and power you have set.
Test into a dummy load first, and remember you are responsible for your licence conditions.

## Credits

Protocol layouts follow the open-source **kappanhang** (nonoo) and **wfview** projects; the PSK31 Varicode table
is the G3PLX standard, verified against **fldigi**.
