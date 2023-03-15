/** 
 * Weather Balloon controller v1.0 by nextdaydelivery
 * Weather balloon datalogger project. Powered by the Raspberry Pi Pico.
 * https://github.com/nxtdaydelivery/weatherballoon
 * 
 * Licensed under the MIT License - Copyright (c) 2023 nextdaydelivery
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "TinyGPSPlus.h"
#include "Adafruit_BMP3XX.h"
#include "Adafruit_MCP9808.h"
#include "SoftwareSerial.h"
#include "SdFat.h"
#include "sdios.h"
#include "LoRa.h"
#include "ArduCAM.h"
#include "hardware/watchdog.h"

// PINS //
#define IO_MISO D12   //
#define IO_MOSI D11   // aka SPI1
#define IO_SCK D10    // 
#define IO_SD SDCARD_SS_PIN // see %localappdata%\Arduino15\packages\rp2040\hardware\rp2040\3.0.0\libraries\ESP8266SdFat\src\SdFatConfig.h and add these pins to there at the top, like:
// #define SDCARD_SPI      SPI1
// #define SDCARD_MISO_PIN D12
// #define SDCARD_MOSI_PIN D11
// #define SDCARD_SCK_PIN  D10
// #define SDCARD_SS_PIN   D9
#define IO_RADIO D13

#define RADIO_DIO0 D14
#define RADIO_RST D15

#define CAM_CS D17
#define CAM_MISO D16 //
#define CAM_MOSI D19 // aka SPI0
#define CAM_SCK D18  //
#define CAM_SDA D20  //
#define CAM_SCL D21  // aka I2C0

#define I2C_SDA D6  //
#define I2C_SCL D7  // aka I2C1

#define GPS_TX D0   //
#define GPS_RX D1   // aka UART0

#define LED_R D28
#define LED_G D27
#define LED_B D26

// STATES //
#define DEBUG true   // serial output
#define USE_LED true
#define USE_GPS true
#define USE_I2C true
#define USE_CAMERA true
#define USE_RADIO true
#define USE_SD true

// weather or not to send thumbnail (low-res) images over the radio. See CYCLES_PER_THUMBNAIL.
#define SEND_THUMBNAILS true

// weather or not to use flight mode for the GPS, a feature provided for many uBLOX GPS systems.
// it optimizes the GPS for low temperature and high altitude usage.
#define GPS_FLIGHTMODE true
// weather the system should wait for the GPS to have a data before starting.
#define WAIT_FOR_GPS true
// weather the system should wait for USB serial in order to begin running when DEBUG is true.
#define WAIT_FOR_DEBUG false

// set this value to a positive integer to enable the watchdog. The value is the ms between polls that has to be reached before resetting. Max: 8300
// note that images can take a VERY long time on lower clock speeds (5 seconds), along with radio signals on high spreading factors.
// so make sure to include plenty of time around that to avoid accidental resets.
// also note that if you flash using arduino pico default settings, it will report a reboot as being watchdog when you flash it.
#define WATCHDOG 8000
// set this value to enable if the system can reboot itself on a fatal error or not.
#define REBOOT_ON_FATAL true
// weather or not the system should be able to reboot itself if the last reboot was caused by watchdog (to prevent bootlooping and potentially damaging hardware)
#define ALLOW_WATCHDOG_BOOTLOOP false

// enable the CSV header on the data log files.
#define ENABLE_CSV_HEADER false
// enable boot record file, which tracks how many times the system has been rebooted and by watchdog.
#define ENABLE_BOOTRECORD true
// amount of times to try writing, opening, sending on radio (IO operations) etc. before crashing.
#define MAX_IO_TRIES 5

// supported resolutions for OV5642 JPEG: OV5642_320x240, OV5642_640x480, OV5642_1024x768, OV5642_1280x960, OV5642_1600x1200, OV5642_2048x1536, OV5642_2592x1944
// note that larger sensor images doesn't necessarily mean better pictures. The default of 1280x960 is chosen becuase it seems to be a good balance between file size, speed and picture quality.
// camera picture resolution that is saved to the SD.
#define PIC_RESOLUTION OV5642_1280x960
// camera thumbnail resolution that is sent over the radio. Recommended to be small, especially when using a high spreading factor.
#define THUMB_RESOLUTION OV5642_320x240

// spreading factor for the radio. This affects the long-range performance of the radio, at the cost of speed and power.
// see https://www.thethingsnetwork.org/docs/lorawan/spreading-factors/
// range 6-12 (with 12 being highest range but slowest and highest power consumption). Default 7.
#define RADIO_SPREAD_FACTOR 12
// amount of times to send each batch of radio log data.
// note that using this with a high spreading factor isn't necessarily a good idea (it will use a LOT of time + power)
#define RADIO_REPEATS 1


// cycle millihertz (how many milliseconds between samples) for the system. Basically its clock. Every time this is reached, a sensor sample is taken, and the various cycle counters for the below are incremented.
// very low values are not a good idea, and will waste power.
#define CYCLE_MILLIHERTZ 2000
// how many samples should be written to one file before swapping to next one. Default: 35. This means 4kb/file.
#define CYCLES_PER_FILE 35
// how many sensor cycles to skip before taking a picture (e.g. 15xSENSOR_SAMPLE_WAIT of 2000 = every 30s)
#define CYCLES_PER_PIC 15
// how many sensor cycles to skip before sending radio (same as above)
#define CYCLES_PER_RX 15
// how many sensor cycles to skip before sending a thumbnail on the radio (same as above)
#define CYCLES_PER_THUMBNAIL 150
// how many sensor cycles to do before turning off the radio. (same as above)
// use this if you anticipate the payload to go out of range before the flight ends. 0 to disable.
#define CYCLES_RADIO 0

// how many cycles to wait for color updates (does not use the above clock system, much more random for efficiency)
// This value controls the breathe speed of the LED basically. Set this value to whatever you think looks best for you.
// lower number = faster breathing, higher = slower
#define COLOR_UPD_SKIP 800




// COLORS // (max of 200 becuase it looks nicer)
static const uint8_t WHITE[3] = {200, 200, 200};
static const uint8_t GREEN[3] = {0, 200, 0};      // IO messages
static const uint8_t PURPLE[3] = {160, 0, 160};   // GPS messages
static const uint8_t YELLOW[3] = {160, 160, 0};   // camera messages
static const uint8_t RED[3] = {200, 0, 0};
static const uint8_t OFF[3] = {0, 0, 0};


// temp
Adafruit_MCP9808 temp = Adafruit_MCP9808();
// pressure
Adafruit_BMP3XX pres = Adafruit_BMP3XX();

// gps object
TinyGPSPlus gps;

// sd object
SdFat32 sd;
// currently open file
File32 file;

// camera object
ArduCAM camera(OV5642, CAM_CS);

// color data
uint8_t rgb[3]; 
uint8_t trgb[3];
uint8_t frgb[3];
bool br = false;
uint8_t brt = 0; 
uint8_t brtcap = 0;

// cycling data
uint8_t camcycles = 0; 
uint8_t radiocycles = 0; 
uint8_t sdcycles = 0; 
uint8_t thumbcycles = 0; 
uint32_t radiotimes = 0;

// image data
uint32_t img_length = 0; 
uint32_t sent_bytes = 0;
uint8_t sent_pckts = 0;
uint8_t pckts_to_send = 0;

// flags
bool fatalerr = false;
// WatchDog Rebooted flag
bool wdr = false;
// weather the system is ready (controlled by WAIT_FOR_GPS and gps_ready())
bool ready = false;

// data
float pressure = 0;
double altitude = 0;
float temperature = 0;
float temp2 = 0;
uint16_t state = 0;



void setup() {
  pinMode(LED_BUILTIN, OUTPUT_2MA);
  digitalWrite(LED_BUILTIN, HIGH);

  if(DEBUG) {
    Serial.begin(115200);
    // wait for serial init...
    if(WAIT_FOR_DEBUG) while (!Serial) ;
    Serial.print("Init begin... Running at ");
    Serial.print(rp2040.f_cpu() / 1000000);
    Serial.print("MHz. ");
  }
  wdr = watchdog_caused_reboot();

  if(WATCHDOG > 0) {
    logln("Watchdog enabled!");
    rp2040.wdt_begin(WATCHDOG);
  } else logln("");

  // init LED
  if(USE_LED) {
    pinMode(LED_R, OUTPUT_2MA);
    pinMode(LED_G, OUTPUT_2MA);
    pinMode(LED_B, OUTPUT_2MA);
  }
  

  // init CS pins
  pinMode(IO_SD, OUTPUT);
  pinMode(IO_RADIO, OUTPUT);
  

  // init I2C + SPI + Serial1 for GPS (UART0)
  //analogReadResolution(12);
  
  if(USE_RADIO || USE_SD) {
    SPI1.setRX(IO_MISO);
    SPI1.setTX(IO_MOSI);
    SPI1.setSCK(IO_SCK);
  }

  if(USE_GPS) {
    // init gps
    Serial1.setTX(GPS_TX);
    Serial1.setRX(GPS_RX);
    Serial1.begin(9600);
    if(GPS_FLIGHTMODE) {
      delay(100); // let the GPS start
      uint8_t flightMode[] = {
        0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
        0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC 
      };
      sendUBX(flightMode, sizeof(flightMode) / sizeof(uint8_t));
      if(!getUBX_ACK(flightMode)) {
        logln("GPS Flight mode configuration error!");
        fatalerr = true;
        breathe(GREEN, RED, 0);
        return;
      } else log(" -> ");
    }
    logln("GPS OK");
  }
  wdt();

  // init SD
  if(USE_SD) {
    digitalWrite(IO_RADIO, HIGH);
    digitalWrite(IO_SD, LOW);
    if(!sd.begin(SdSpiConfig(IO_SD, SHARED_SPI, SD_SCK_MHZ(10)))) {
      logln("Failed to init SD Card SPI!");
      if(DEBUG) sd.initErrorPrint(&Serial);
      fatalerr = true;
      breathe(GREEN, RED, 0);
      return;
    } else {
      // simple check for SD r/w capabilities. deletes the file, create one, write some text, close it. Open it again, read it back, then close and check if they are the same.
      if(!file.open("bootrecord.txt", O_WRONLY | O_CREAT | O_AT_END)) {
        logln("Failed to create open boot record file!");
        fatalerr = true;
      } else {
        file.print("Boot OK");
        if(wdr) {
          file.println(" W: restarted by watchdog!");
          logln("Rebooted by watchdog!");
          breathe(PURPLE, RED, 5);
        } else file.println();
        if(!file.close()) {
          logln("boot record file didn't close!");
          fatalerr = true;
        }
        if(!file.open("bootrecord.txt", O_RDONLY)) {
          logln("Failed to open boot record file!");
          fatalerr = true;
        }
        uint32_t fs = file.fileSize();
        uint16_t i = 0;
        uint16_t w = 0;
        char buffer[fs];
        if(file.readBytes(buffer, fs) != fs) {
          logln("File readback was not OK!");
          fatalerr = true;
        }
        for(char c : buffer) {
          if(c == '\n') {
            ++i;
          } else if(c == 'W') {
            ++w;
          }
        }
        log("Found "); logi(i); log(" reboots, of which "); logi(w); logln(" were caused by watchdog");
        if(!file.close()) {
          logln("boot record file didn't close!");
          fatalerr = true;
        }
        if(!ENABLE_BOOTRECORD) {
          sd.remove("bootrecord.txt");
        }
        if(fatalerr) {
          breathe(GREEN, RED, 0);
          logln("SD failed check!");
          return;
        }
      }
      logln("SD OK");
    }
    digitalWrite(IO_RADIO, HIGH);
  }
  wdt();

  if(USE_RADIO) {
    digitalWrite(IO_SD, HIGH);
    digitalWrite(IO_RADIO, LOW);
    LoRa.setPins(IO_RADIO, RADIO_RST, RADIO_DIO0);
    LoRa.setSPI(SPI1);
    if(!LoRa.begin(868E6)) {
      logln("Radio failed to start!");
      fatalerr = true;
    } else {
      logln("Radio OK");
    }
    LoRa.setTxPower(20);
    LoRa.setSpreadingFactor(RADIO_SPREAD_FACTOR);
    if(SEND_THUMBNAILS) LoRa.onTxDone(thumbnail_send);
    LoRa.sleep();
    digitalWrite(IO_RADIO, HIGH);
    digitalWrite(IO_SD, LOW);
  }
  wdt();

  // init camera
  if(USE_CAMERA || SEND_THUMBNAILS) {
    SPI.setTX(CAM_MOSI);
    SPI.setRX(CAM_MISO);
    SPI.setSCK(CAM_SCK);
    Wire.setSCL(CAM_SCL);
    Wire.setSDA(CAM_SDA);
    Wire.begin();
    SPI.begin();
    uint8_t vid,pid;
    uint8_t state;
    camera_wake();
    camera.write_reg(0x07, 0x80);
    delay(100);
    camera.write_reg(0x07, 0x00);
    delay(100);
    // Check if the ArduCAM SPI bus is OK
    camera.write_reg(ARDUCHIP_TEST1, 0x55);
    state = camera.read_reg(ARDUCHIP_TEST1);
    if(state != 0x55) {
        logln("Camera SPI interface error!");
        fatalerr = true;
        breathe(YELLOW, RED, 0);
        return;
    }  
    // Change MCU mode
    camera.write_reg(ARDUCHIP_MODE, 0x00); 
    camera.wrSensorReg16_8(0xff, 0x01);
    camera.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    camera.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    delay(100);
    if((vid != 0x56) || (pid != 0x42)) {
      logln("Can't find camera module?!");
      fatalerr = true;
      breathe(YELLOW, RED, 0);
      return;
    }
    delay(50);
    camera.set_format(JPEG);
    camera.InitCAM();
    camera.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    camera.OV5642_set_JPEG_size(PIC_RESOLUTION);
    camera.clear_fifo_flag();   
    camera.write_reg(ARDUCHIP_FRAMES, 0x00);
    delay(100);
    camera_sleep();
    logln("Camera OK");
  }
  wdt();

  // init i2c devices
  if(USE_I2C) {
    Wire1.setSDA(I2C_SDA);
    Wire1.setSCL(I2C_SCL);
    Wire1.begin();
    if(!temp.begin(0x18, &Wire1)) {
      logln("Failed to initialize temperature sensor!");
      fatalerr = true;
      breathe(YELLOW, RED, 0);
      return;
    } else {
      // setup temperature resolution
      logln("Temperature Sensor OK");
      temp.setResolution(3);
    }
    if(!pres.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1)) {
      logln("Failed to initialize pressure sensor!");
      fatalerr = true;
      breathe(YELLOW, RED, 0);
      return;
    } else {
      // setup pressure resolution
      logln("Pressure Sensor OK");
    }
  }
  wdt();

  state = 1;
  logln("initialized successfully");
  if(WAIT_FOR_GPS) breathe(OFF, PURPLE, 0);
  else ready = true;
  digitalWrite(LED_BUILTIN, LOW);
  wdt();
}


void loop() {
  unsigned long now = millis();
  if(fatalerr) {
    state = 500;
    if(!REBOOT_ON_FATAL || (wdr && !ALLOW_WATCHDOG_BOOTLOOP)) {
      logln("Fatal error! Please restart.");
      await(5000);
    } else {
      logln("Fatal error! Rebooting in 5 seconds...");
      await(5000);
      rp2040.reboot();
    }
    return;
  }
  wdt();
  if(!ready && gps_ready()) {
    logln("GPS ready, ready!");
    ready = true;
    toColor(OFF);
  }
  if(!ready) {
    log("Waiting for GPS... (c=");
    logi(gps.charsProcessed()); log("; s=");
    logi(gps.satellites.value()); logln(")");
    if(gps.charsProcessed() != 0) state = 101;
    await(2000);
    return;
  } else state = 200;

  digitalWrite(LED_BUILTIN, HIGH);
  // TODO?
  //logln(((analogRead(A3) * 3.3) / 65535) * 3);
  pressure = pres.readPressure();
  temperature = temp.readTempC();
  temp2 = pres.readTemperature();
  if(gps.altitude.isValid()) {
    altitude = gps.altitude.meters();
  } else altitude = 0;
  if(USE_CAMERA) {
    ++camcycles;
    if(camcycles > CYCLES_PER_PIC) {
      take_picture();
      camcycles = 0;
    }
    wdt();
  }

  if(USE_SD) {
    ++sdcycles;
    if(sdcycles > CYCLES_PER_FILE) {
      inc_file_safe();
      sdcycles = 0;
    }
    write_data();
    wdt();
  }

  if(USE_RADIO) {
    if(CYCLES_RADIO == 0 || !radiotimes > CYCLES_RADIO) {
      ++radiocycles;
      if(radiocycles > CYCLES_PER_RX) {
        send_radio_data();
        radiocycles = 0;
        wdt();
      }
      if(SEND_THUMBNAILS) {
        ++thumbcycles;
        if(thumbcycles > CYCLES_PER_THUMBNAIL) {
          thumbnail_begin();
          thumbcycles = 0;
          wdt();
        }
      }
      ++radiotimes;
    }
  }

  if(DEBUG) {
    logln("State\tTime\t\tAlt.\tSpeed\t\tLat\t\tLong\t\tPressure\tTemperature\tTemperature 2\tSatellites");
    char cz[128];
    // tasty
    snprintf(cz, 128, "%i\t%02i:%02i:%02i\t%.4fm\t%.4fm/s\t%.6f\t%.6f\t%.4fhPa\t%.4fc\t%.4fc\t%i", state, gps.time.hour(), gps.time.minute(), gps.time.second(), altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, temp2, gps.satellites.value());
    logln(cz);
  }


  // wait the remaining time. Use this to prevent a possible negative number in ulong if it takes longer than CYCLE_MILLIHERTZ.
  wdt();
  digitalWrite(LED_BUILTIN, LOW);
  long dur = millis() - now;
  if(dur > CYCLE_MILLIHERTZ) {
    if(DEBUG) {
      char c[48];
      snprintf(c, 48, "Running %010i millis behind!", dur);
      logln(c);
    }
    // just to make sure that everything is OK
    await(250);
    wdt();
    return;
  }
  else await(CYCLE_MILLIHERTZ - dur);
}

// delay which still polls the GPS and updates the LED.
// if the GPS and LED is disabled, this will use pico low power sleep mode.
void await(unsigned long ms) {
  wdt();
  if(!USE_GPS && !USE_LED) {
    sleep_ms(ms);
    return;
  }
  unsigned long now = millis();
  uint32_t cycl = 0;
  do 
  {
    wdt();
    if(USE_LED) {
      ++cycl;
      if(cycl > COLOR_UPD_SKIP) {
        update_colors();
        cycl = 0;
      }
    }
    if(!fatalerr && USE_GPS) feed_gps();
  } while (millis() - now < ms);
}





// SD //

// increment the file to a new one, closing the old one. Will try MAX_IO_TRIES before dying.
bool inc_file() {
  uint8_t tries = 0;
  while(tries <= MAX_IO_TRIES) {
    log("Incrementing file to new one. Attempt ");
    logi(tries);
    logln(".");

    if(file.isOpen()) {
      // fix for busy retries
      await(50);
      if(file.isBusy()) {
        logln("File busy? waiting 50ms");
        await(50);
        ++tries;
        continue;
      } 
      if(!file.close()) {
        logln("Failed to close old file, waiting 50ms");
        await(50);
        ++tries;
        continue;
      }
      await(50);
    }
    char cz[32];
    snprintf(cz, 32, "log_%02i-%02i-%02i.csv", gps.time.hour(), gps.time.minute(), gps.time.second());
    if(sd.exists(cz)) { 
      logln("File already exists? Using time since startup");
      snprintf(cz, 32, "log_%010i.csv", millis());
    }
    if(!file.open(cz, O_WRONLY | O_CREAT)) {
      logln("Failed to open file, waiting 50ms");
      await(50);
      ++tries;
      continue;
    }

    // add csv header
    if(ENABLE_CSV_HEADER) file.println("state, time, alt, speed, latitude, longitude, pressure, temperature, temp2");
    log("File "); log(cz); logln(" opened. OK");
    return true;
  }
  logln("SD IO error! ran out of tries!");
  return false;
}

inline bool inc_file_safe() {
  if(!inc_file()) {
    logln("FATAL: File increment fail!");
    fatalerr = true;
    breathe(GREEN, RED, 0);
    return false;
  } 
  return true;
}

// write the data to the currently open file.
void write_data() {
  if(!file.isOpen()) {
    logln("IllegalStateException: File wasn't open when write was attempted?");
    if(!inc_file_safe()) {
      logln("Trying one last time!");
      fatalerr = false;
      stop_breathe();
      inc_file_safe();
    }
  }
  // plenty of space to avoid overflows
  char cz[128];
  snprintf(cz, 128, "%i, %02i:%02i:%02i, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f", state, gps.time.hour(), gps.time.minute(), gps.time.second(), altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, temp2);
  file.println(cz);
}







// RADIO //

// send data on the radio RADIO_REPEATS times.
void send_radio_data() {
  digitalWrite(IO_SD, HIGH);
  digitalWrite(IO_RADIO, LOW);
  logln("Sending data on radio...");
  uint8_t i = 0;
  while(i < RADIO_REPEATS) {
    LoRa.beginPacket();
    char cz[80];
    snprintf(cz, 80, "%03i;%.1f;%.8f;%.8f;%.8f;%.6f;%.8f", state, altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temp2);
    LoRa.print(cz);
    wdt();
    LoRa.endPacket();
    wdt();
    i++;
  }
  logln("Packet(s) sent");
  LoRa.sleep();
  digitalWrite(IO_RADIO, HIGH);
  digitalWrite(IO_SD, LOW);
}

inline bool begin_packet_safe() {
  uint8_t i = 0;
  while(i < MAX_IO_TRIES) {
    if(LoRa.beginPacket()) return true;
    logln("Radio busy? Trying again in 100ms");
    await(100);
    ++i;
  }
  logln("Error: radio didn't free!");
  return false;
}




// CAMERA // 

// close the log file, open a picture file and write out the data, then re-open the log.
bool take_picture() {
  logln("Taking picture");
  camera_wake();
  camera.OV5642_set_JPEG_size(PIC_RESOLUTION);
  delay(50);
  camera.flush_fifo();
  camera.clear_fifo_flag();
  //Start capture
  camera.start_capture();
  wdt();
  // it is intentional that wdt is not called in this loop, so that if the camera hangs, the program can still restart.
  while (!camera.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) ;
  
  logln("Camera finished taking image, now saving");
  wdt();

  char oldName[32];
  file.getName(oldName, 32);  
  // I have these in different functions for three reasons:
  // So I can have them return false if they fail without having to nest loads of indents
  // so that if it fails I don't have to crash the system
  // so i can reuse the read_fifo_burst function
  if(!close_log_and_open_img()) {
    logln("Error while opening picture file, going to force increment next log!");
    // set max for next time so it will increment the file
    breathe(GREEN, RED, 5);
    sdcycles = 255;
    return false;
  }
  wdt();

  if(!read_img_length() || !read_img()) {
    breathe(YELLOW, RED, 5);
    logln("Camera picture reading failed!");
    return false;
  }
  wdt();
  //Clear the capture done flag
  camera.clear_fifo_flag();
  delay(50);
  camera_sleep();

  if(!close_img_and_reopen_log(oldName)) {
    logln("Error while closing picture file, going to force increment next log!");
    // set max for next time so it will increment the file
    breathe(GREEN, RED, 5);
    sdcycles = 255;
    return false;
  }
  wdt();
  logln("Image saved successfully");
  return true;
}

// close the log file and open a new image file.
bool close_log_and_open_img() {
  uint8_t tries = 0;
  while(tries < MAX_IO_TRIES) {
    wdt();
    if(!file.close()) {
      logln("Failed to close log file, waiting 50ms");
      await(50);
      ++tries;
      continue;
    }
    await(50);
    char cz[32];
    snprintf(cz, 32, "pic_%02i-%02i-%02i.jpg", gps.time.hour(), gps.time.minute(), gps.time.second());
    if(sd.exists(cz)) { 
      logln("picture file already exists? Using time since startup");
      snprintf(cz, 32, "pic_%010i.jpg", millis());
    }
    if(!file.open(cz, O_WRONLY | O_CREAT)) {
      logln("Failed to open picture file, waiting 50ms");
      await(50);
      ++tries;
      continue;
    } else {
      return true;
    }
  }
  return false;
}

bool close_img_and_reopen_log(char *oldName) {
  uint8_t tries = 0;
  while(tries < MAX_IO_TRIES) {
    wdt();
    if(!file.close()) {
      logln("Failed to close picture file!");
      await(50);
      ++tries;
      continue;
    }
    await(50);
    if(!file.open(oldName, O_WRONLY | O_AT_END)) {
      logln("Failed to open log file, waiting 50ms");
      await(50);
      ++tries;
      continue;
    } else {
      return true;
    }
  }
  return false;
}

bool read_img_length() {
  img_length = camera.read_fifo_length();
  if (img_length >= MAX_FIFO_SIZE) { // 8mb
    img_length = 0;
    logln("Picture oversize!");
    return false;
  }
  if (img_length == 0) {
    logln("Picture size is 0!");
    return false;
  }
  return true;
}

// read the data from the camera, and write to the output to the file.
// modified from the arducam example.
bool read_img() {
  bool is_header = false;
  uint8_t temp = 0, temp_last = 0;
  camera.CS_LOW();
  camera.set_fifo_burst(); // Set fifo burst mode
  wdt();
  while (--img_length) {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    if (is_header == true) {
      file.write(temp);
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF)) {
      is_header = true;
      file.write(temp_last);
      file.write(temp);
    }
    if ((temp == 0xD9) && (temp_last == 0xFF)) break; // end of the image
    delayMicroseconds(5);
  }
  camera.CS_HIGH();
  wdt();
  return true;
}

bool thumbnail_begin() {
  if(pckts_to_send != 0) {
    logln("Warning: attempted to send a thumbnail while the radio hadn't finished sending the last one!");
    return false;
  }
  logln("Sending thumbnail image");
  camera_wake();
  camera.OV5642_set_JPEG_size(THUMB_RESOLUTION);
  delay(50);
  camera.flush_fifo();
  camera.clear_fifo_flag();
  //Start capture
  camera.start_capture();
  wdt();
  // it is intentional that wdt is not called in this loop, so that if the camera hangs, the program can still restart.
  while (!camera.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) ;
  
  logln("Camera finished taking thumbnail, now sending.");
  //Clear the capture done flag
  camera.clear_fifo_flag();
  if(!read_img_length()) {
    breathe(YELLOW, RED, 5);
    logln("Camera thumbnail length reading failed!");
    return false;
  }
  wdt();
  sent_bytes = 0;
  sent_pckts = 0;
  pckts_to_send = ceil(img_length / 254.0);
  // send the amount of packets the reciever should expect.
  digitalWrite(IO_SD, HIGH);
  digitalWrite(IO_RADIO, LOW);
  LoRa.beginPacket();
  LoRa.print("L"); LoRa.print(pckts_to_send); LoRa.print("");
  LoRa.endPacket();
  // no need to change pins as they will be reset by thumbnail_send
  wdt();
  await(100);
  thumbnail_send();
  return true;
}

void thumbnail_send() {
  if(sent_pckts == pckts_to_send) {
    logln("Finished sending image!");
    sent_pckts = 0;
    pckts_to_send = 0;
    sent_bytes = 0;
    LoRa.sleep();
    return;
  }
  digitalWrite(IO_SD, HIGH);
  digitalWrite(IO_RADIO, LOW);
  bool is_header = false;
  uint8_t temp = 0, temp_last = 0;
  uint8_t i = 0;
  uint8_t m = 0;
  camera.CS_LOW();
  camera_wake();
  // fallback if radio is busy
  wdt();
  if(!begin_packet_safe()) {
    sent_pckts = 0;
    pckts_to_send = 0;
    sent_bytes = 0;
    LoRa.sleep();
    return;
  }
  // crude fix for two byte writes
  m = min(254, img_length - sent_bytes);
  log("Sending thumbnail packet ("); logi(sent_pckts); log("/"); logi(pckts_to_send); log("); length "); logi(m); logln(" bytes");
  while (i < m) {
    temp_last = temp;
    temp = SPI.transfer(0x00);
    if (is_header == true) {
      LoRa.write(temp);
    } else if ((temp == 0xD8) & (temp_last == 0xFF)) {
      is_header = true;
      LoRa.write(temp_last);
      LoRa.write(temp);
      ++i;
    }
    if ((temp == 0xD9) && (temp_last == 0xFF)) break; // end of the image
    delayMicroseconds(5);
    ++i;
    ++sent_bytes;
  }
  ++sent_pckts; 
  LoRa.endPacket(true);
  camera.CS_HIGH();
  camera_sleep();
  digitalWrite(IO_SD, LOW);
  digitalWrite(IO_RADIO, HIGH);
  wdt();
}

inline void camera_sleep() {
  camera.set_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK); 
}

inline void camera_wake() {
  camera.clear_bit(ARDUCHIP_GPIO, GPIO_PWDN_MASK); 
}









// GPS 

// feed the GPS object.
inline void feed_gps() {
  while (Serial1.available()) { 
    gps.encode(Serial1.read());
  }
}

/** thanks to https://ukhas.org.uk/doku.php?id=guides:ublox6 for this! */

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
  }
  Serial1.println();
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  wdt();
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  while (true) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order, Ok!
      wdt();
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      wdt();
      return false;
    }
 
    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        if(DEBUG) Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
    }
  }
}





// validation
inline bool temp_valid() {
  return !isinf(temp.readTempC());
}

inline bool time_valid() {
  return gps.time.isValid() && !(gps.time.hour() == 0 && gps.time.minute() == 0 && gps.time.second() == 0);
}

inline bool gps_ready() {
  return time_valid && !(gps.location.lat() == 0 && gps.location.lng() == 0 && gps.altitude.meters() == 0);
}








// GENERAL //

inline void logln(const char c[]) {
  if(DEBUG) Serial.println(c);
}

inline void logln(String s) {
  if(DEBUG) Serial.println(s);
}

inline void logln(double num) {
  if(DEBUG) Serial.println(num);
}

inline void logln(int num) {
  if(DEBUG) Serial.println(num);
}

inline void log(const char c[]) {
  if(DEBUG) Serial.print(c);
}

inline void log(String s) {
  if(DEBUG) Serial.print(s);
}

inline void logi(int num) {
  if(DEBUG) Serial.print(num);
}

inline void log(char c) {
  if(DEBUG) Serial.print(c);
}

// poll the watchdog
inline void wdt() {
  if(WATCHDOG > 0) rp2040.wdt_reset();
}






// COLORS //

// update the RGB LED
void update_colors() {
  if(!br && clrcmp(OFF, rgb)) return;
  if(trgb[0] > rgb[0]) {
    ++rgb[0];
    analogWrite(LED_R, rgb[0]);
  }
  else if(trgb[0] < rgb[0]) {
    --rgb[0];
    analogWrite(LED_R, rgb[0]);
  }

  if(trgb[1] > rgb[1]) {
    ++rgb[1];
    analogWrite(LED_G, rgb[1]);
  }
  else if(trgb[1] < rgb[1]) {
    --rgb[1];
    analogWrite(LED_G, rgb[1]);
  }

  if(trgb[2] > rgb[2]) {
    ++rgb[2];
    analogWrite(LED_B, rgb[2]);
  }
  else if(trgb[2] < rgb[2]) {
    --rgb[2];
    analogWrite(LED_B, rgb[2]);
  }
  if(br) {
    if(clrcmp(rgb, trgb)) {
      const uint8_t temp[3] = {trgb[0], trgb[1], trgb[2]};
      trgb[0] = frgb[0]; trgb[1] = frgb[1]; trgb[2] = frgb[2];
      frgb[0] = temp[0]; frgb[1] = temp[1]; frgb[2] = temp[2];
      if(brtcap != 0) {
        ++brt;
        if(brt == brtcap) {
          brt = 0;
          brtcap = 0;
          br = false;
          toColor(OFF);
        }
      }
    }
  }
}

void toColor(const uint8_t to[3]) {
  trgb[0] = to[0];
  trgb[1] = to[1];
  trgb[2] = to[2];
  br = false;
  brt = 0;
  brtcap = 0;
}

void color(const uint8_t to[3]) {
  rgb[0] = to[0];
  rgb[1] = to[1];
  rgb[2] = to[2];
  br = false;
  brt = 0;
  brtcap = 0;
}

void breathe(const uint8_t from[3], const uint8_t to[3], uint8_t times) {
  if(!clrcmp(from, rgb)) {
    color(from);
  }
  toColor(to);
  frgb[0] = from[0];
  frgb[1] = from[1];
  frgb[2] = from[2];
  br = true;
  brt = 0;
  brtcap = times;
}

inline bool clrcmp(const uint8_t a[3], const uint8_t b[3]) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

inline void stop_breathe() {
  br = false;
  brt = 0;
  brtcap = 0;
  color(OFF);
}

