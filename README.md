# Link31
M5 Stack Core 2 firmware to link an IC-705 to a keyboard, so PSK31, RTTY, and morse can be sent, received, and decoded on the go.

This allows you to connect to the IC-705 over wifi, see decoded morse and decoded text on screen, and type text to send on a bluetooth BLE keyboard.

It can also be used as a decoder for morse if you don't want to send.

Features:-
* Wifi connectivity with the IC-705
* BLE keyboard support
* Send PSK31, RTTY, morse by keyboard in real time as you type
* Decode and display PSK31, RTTY, morse text on screen
* SWR meter

Download flashable binary from Releases ->
Or install with M5 Burner

To build from source:

* Download cpp, h and ino files to a folder with a flat structure
* Open ino in Arduino IDE
* Install NimBLE-Arduino 2.5.2 library
* Install ESP32 boards and choose M5Stack-Core2
* Build and upload

Setup once flashed:

* Open the menu
* Pair your BLE keyboard, if you are using one (this makes the rest of the setup easier and avoids touchscreen typing)
* Enter Wifi SSID and  password
* Enter the radio IP (found in the IC-705 menu WLAN  -> Connection settings)
* Enter the radio login credentials (IC-705 menu WLAN  -> Remote Settings)
* Connection tends to take a few attempts, as with other IC-705 apps. You might need to cycle wifi on/off.
* Transmit needs to be armed in the M5's menu, if you wish to use it. Enable "Allow TX"
  
Credits to:
Happankang https://github.com/nonoo/kappanhang/tree/main
WfView https://wfview.org/

for IC-705 protocol work and information

Fldigi https://sourceforge.net/projects/fldigi/

for PSK31 technical information

Claude

design and coding
