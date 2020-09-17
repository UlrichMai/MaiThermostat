# MaiThermostat
New firmware for room thermostat BHT-002-GALW to control it via HomeKit.
I added a humidity sensor and published the value in homekit.

(draft)

## How to build?
- build firmware (platformio.ini)
  - -D BME280  // if you installed the additional sensor
  - -D PIN_RELAY_MONITOR=13  // if you attached the relais signal to gpio13
  - fill Private.h with wifi and ota credentials 
  - test firmware on a spare ESP-8266 device, test the homekit pairing and the ota update. If ota is not working, you must flash it via serial, solder wire to the chip. It did not happen to me yet, and I hope it will not. 
- buy BHT-002-GALW
- use tuya-convert to flash new firmware
  - not working for the last 4 thermostats bought 
  - need to solder wires for flashing
- make hardware modifications
  - wire GPIO13 to get the relais heating state
  - install the BME280 sensor and connect it to GPIO4+5 and GND+VCC
- connect temporary power for testing before installation to the wall
- pair with HomeKit

## debugging
Enable debugging output if needed with define DBG_TCP_ENABLED.
telnet to port 8888 to get debug output.
DO NOT enable DBG_SERIAL_ENABLED, because the serial port is used to communicate with the MCU.
