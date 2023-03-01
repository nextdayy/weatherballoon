# weatherballoon
Pico weather balloon datalogger project. Powered by the Raspberry Pi Pico.

## Info
todo

## Hardware and Libraries
This project is designed to work with the following components (links are examples):
 - [Raspberry Pi Pico (can be downclocked)](https://www.adafruit.com/product/4864)
 - [RFM9x LoRa radio chip](https://www.adafruit.com/product/3072)
 - [MCP9808 Temperature Sensor](https://www.adafruit.com/product/1782)
 - [BMP3XX Temperature and Humidity Sensor](https://www.adafruit.com/product/4816)
 - [Fat32 SD Card + Reader](https://www.adafruit.com/product/4682)
 - GPS/NMEA GPS chip (using UART)
 - RGB LED (**common cathode**)
 
You will need the following libraries (all are in the Arduino library manager as well):
 - [Arduino-Pico](https://github.com/earlephilhower/arduino-pico) (board manager)
 - [Adafruit BMP3XX](https://github.com/adafruit/Adafruit_BMP3XX)
 - [Adafruit MCP9808](https://github.com/adafruit/Adafruit_MCP9808_Library)
 - [Arduino-LoRa](https://github.com/sandeepmistry/arduino-LoRa)
 - [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)

The hardware used is fully customizable using [these headers](/weatherballoon.ino#L38-L43).

The pinouts can be found [here](/pinout.txt) and can be customized using [these headers](/weatherballoon.ino#L10-L34). Make sure that they use the corresponding interface and are valid pins for that interface, otherwise the Pico will just hang.
