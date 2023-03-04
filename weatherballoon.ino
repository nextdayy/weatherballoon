#include "TinyGPSPlus.h"
#include "Adafruit_BMP3XX.h"
#include "Adafruit_MCP9808.h"
#include "SoftwareSerial.h"
#include "SdFat.h"
#include "sdios.h"
#include "LoRa.h"
#include "ArduCAM.h"

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
#define DEBUG true
#define USE_LED true
#define USE_GPS true
#define USE_I2C true
#define USE_CAMERA true
#define USE_RADIO true
#define USE_SD true
// weather the system should wait for the GPS to have a data before starting.
#define WAIT_FOR_GPS true
// weather the system should wait for USB in order to begin running when DEBUG is true.
#define WAIT_FOR_DEBUG false
// set this value to a positive integer to enable the watchdog. The value is the ms between polls that has to be reached before resetting. Max: 8300
// note that images can take a VERY long time on lower clock speeds (5 seconds), so make sure to include plenty of time around that to avoid accidental resets.
#define WATCHDOG 8000


// cycle hertz for the system. Basically its clock. Every time this is reached, a sensor sample is taken, and the various cycle counters for the below are incremented.
#define CYCLE_MILLIHERTZ 2000
// how many samples should be written to one file before swapping to next one.
#define CYCLES_PER_FILE 35
// how many sensor cycles to skip before taking a picture (e.g. 3xSENSOR_SAMPLE_WAIT of 5000 = every 15s)
#define CYCLES_PER_PIC 30
// how many sensor cycles to skip before sending radio (same as above)
#define CYCLES_PER_RX 5

// how many cycles to wait for color updates (does not use the above clock system, much more random for efficiency)
#define COLOR_UPD_SKIP 800
// amount of times to try writing before dying.
#define MAX_SD_TRIES 5
// amount of times to send each batch of radio data.
#define RADIO_REPEATS 1

// COLORS // (max of 200 becuase it looks nicer)
static const uint8_t WHITE[3] = {200, 200, 200};
static const uint8_t GREEN[3] = {0, 200, 0};      // IO messages
static const uint8_t PURPLE[3] = {160, 0, 160};   // GPS messages
static const uint8_t YELLOW[3] = {200, 200, 0};   // camera messages
static const uint8_t RED[3] = {200, 0, 0};
static const uint8_t OFF[3] = {0, 0, 0};


// temp
Adafruit_MCP9808 temp = Adafruit_MCP9808();
// pressure
Adafruit_BMP3XX pres = Adafruit_BMP3XX();

// gps object
TinyGPSPlus gps;
// GPS Software Serial
SoftwareSerial gpsSS(GPS_RX, GPS_TX);

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

uint8_t camcycles = 0;
uint8_t radiocycles = 0;
uint8_t sdcycles = 0;
bool fatalerr = false;
bool ready = false;

// data
float pressure = 0;
double altitude = 0;
float temperature = 0;
float internal_temp = 0;
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
  //pinMode(CAM_CS, OUTPUT);
  

  // init I2C + SPI
  analogReadResolution(12);
  SPI.setTX(CAM_MOSI);
  SPI.setRX(CAM_MISO);
  SPI.setSCK(CAM_SCK);
  SPI.begin();
  SPI1.setRX(IO_MISO);
  SPI1.setTX(IO_MOSI);
  SPI1.setSCK(IO_SCK);
  Wire.setSCL(CAM_SCL);
  Wire.setSDA(CAM_SDA);
  Wire.begin();
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();

  logln("Communication buses OK");

  if(USE_GPS) {
    // init gps
    gpsSS.begin(9600);
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
      breathe(GREEN, RED);
      return;
    } else {
      // simple check for SD r/w capabilities. deletes the file, create one, write some text, close it. Open it again, read it back, then close and check if they are the same.
      sd.remove("check.txt");
      if(!file.open("check.txt", O_WRONLY | O_CREAT)) {
        logln("Failed to create check file!");
        fatalerr = true;
      } else {
        file.print("200 OK");
        if(!file.close()) {
          logln("Check file didn't close!");
          fatalerr = true;
        }
        if(!file.open("check.txt")) {
          logln("Failed to open check file!");
          fatalerr = true;
        }
        if(file.readString() != "200 OK") {
          logln("File check was not OK!");
          fatalerr = true;
        }
        if(!file.close()) {
          logln("Check file didn't close!");
          fatalerr = true;
        }
        sd.remove("check.txt");
        if(fatalerr) {
          breathe(GREEN, RED);
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
      logln("LoRa failed to start!");
      fatalerr = true;
    } else {
      logln("Radio OK");
    }
    LoRa.sleep();
    digitalWrite(IO_RADIO, HIGH);
    digitalWrite(IO_SD, LOW);
  }
  wdt();

  // init camera
  if(USE_CAMERA) {
    uint8_t vid,pid;
    uint8_t state;
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
        breathe(YELLOW, RED);
        return;
    }  
    // Change MCU mode
    camera.write_reg(ARDUCHIP_MODE, 0x00); 
    camera.wrSensorReg16_8(0xff, 0x01);
    camera.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    camera.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if((vid != 0x56) || (pid != 0x42)) {
      logln("Can't find camera module?!");
      fatalerr = true;
      breathe(YELLOW, RED);
    }
    else logln("Camera OK");
    delay(100);
    camera.set_format(JPEG);
    camera.InitCAM();
    camera.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
    camera.OV5642_set_JPEG_size(OV5642_1280x960);
    camera.clear_fifo_flag();   
    camera.write_reg(ARDUCHIP_FRAMES, 0x00);
    delay(100);
    cameraSleep();
  }
  wdt();

  // init i2c devices
  if(USE_I2C) {
    if(!temp.begin(0x18, &Wire1)) {
      logln("Failed to initialize temperature sensor!");
      fatalerr = true;
      breathe(YELLOW, RED);
      return;
    } else {
      // setup temperature resolution
      logln("Temperature Sensor OK");
      temp.setResolution(3);
    }
    if(!pres.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &Wire1)) {
      logln("Failed to initialize pressure sensor!");
      fatalerr = true;
      breathe(YELLOW, RED);
      return;
    } else {
      // setup pressure resolution
      logln("Pressure Sensor OK");
    }
  }
  wdt();

  state = 1;
  logln("initialized successfully");
  if(WAIT_FOR_GPS) breathe(OFF, PURPLE);
  else ready = true;
  digitalWrite(LED_BUILTIN, LOW);
  wdt();
}


void loop() {
  unsigned long now = millis();
  if(fatalerr) {
    state = 500;
    logln("Fatal error! Please restart.");
    await(5000);
    return;
  }
  wdt();
  if(!ready && gpsReady()) {
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
  // TODO
  logln(((analogRead(A3) * 3.3) / 65535) * 3);
  pressure = pres.readPressure();
  temperature = temp.readTempC();
  internal_temp = pres.readTemperature();
  if(gps.altitude.isValid()) {
    altitude = gps.altitude.meters();
  } else altitude = 0;
  if(USE_CAMERA) {
    ++camcycles;
    if(camcycles > CYCLES_PER_PIC) {
      take_picture();
      camcycles = 0;
    }
  }
  wdt();

  if(USE_SD) {
    ++sdcycles;
    if(sdcycles == CYCLES_PER_FILE) {
      incFileSafe();
      sdcycles = 0;
    }
    writeData();
  }
  wdt();

  if(USE_RADIO) {
    ++radiocycles;
    if(radiocycles > CYCLES_PER_RX) {
      sendRadio();
      radiocycles = 0;
    }
  }
  wdt();

  if(DEBUG) {
    logln("State\tTime\t\tAlt.\tSpeed\t\tLat\t\tLong\t\tPressure\tTemperature\tInternal Temp\tSatellites");
    char cz[128];
    // tasty
    snprintf(cz, 128, "%i\t%02i:%02i:%02i\t%.4fm\t%.4fm/s\t%.6f\t%.6f\t%.4fhPa\t%.4fc\t%.4fc\t%i", state, gps.time.hour(), gps.time.minute(), gps.time.second(), altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, internal_temp, gps.satellites.value());
    logln(cz);
  }


  // wait the remaining time. Use this to prevent a possible negative number in ulong if it takes longer than CYCLE_MILLIHERTZ.
  digitalWrite(LED_BUILTIN, LOW);
  long dur = millis() - now;
  if(dur > CYCLE_MILLIHERTZ) {
    char c[48];
    snprintf(c, 48, "Running %010i millis behind!", dur);
    logln(c);
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
        updateColors();
        cycl = 0;
      }
    }
    if(!fatalerr && USE_GPS) feedGPS();
  } while (millis() - now < ms);
}





// SD //

// increment the file to a new one, closing the old one. Will try MAX_SD_TRIES before dying.
bool incFile() {
  uint8_t tries = 0;
  while(tries <= MAX_SD_TRIES) {
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
    file.println("state, time, alt, speed, latitude, longitude, pressure, temperature, internaltemperature");
    log("File "); log(cz); logln(" opened. OK");
    return true;
  }
  logln("SD IO error! ran out of tries!");
  return false;
}

inline bool incFileSafe() {
  if(!incFile()) {
    logln("FATAL: File increment fail!");
    fatalerr = true;
    breathe(GREEN, RED);
    return false;
  } 
  return true;
}

// write the data to the currently open file.
void writeData() {
  if(!file.isOpen()) {
    logln("IllegalStateException: File wasn't open when write was attempted?");
    if(!incFileSafe()) {
      logln("Trying one last time!");
      fatalerr = false;
      cancelBreathe();
      incFileSafe();
    }
  }
  // plenty of space to avoid overflows
  char cz[128];
  snprintf(cz, 128, "%i, %02i:%02i:%02i, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f, %.8f", state, gps.time.hour(), gps.time.minute(), gps.time.second(), altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, internal_temp);
  file.println(cz);
}







// RADIO //

// send data on the radio RADIO_REPEATS times. Packets start with a ? and end with a !
void sendRadio() {
  digitalWrite(IO_SD, HIGH);
  digitalWrite(IO_RADIO, LOW);
  logln("Sending data on radio...");
  uint8_t i = 0;
  while(i<RADIO_REPEATS) {
    LoRa.beginPacket();
    char cz[128];
    snprintf(cz, 128, "?%05i;%.8f;%.8f;%.8f;%.8f;%.8f;%.8f;%.8f!", state, altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, internal_temp);
    LoRa.print(cz);
    LoRa.endPacket();
    i++;
  }
  logln("Packet(s) sent");
  LoRa.sleep();
  digitalWrite(IO_RADIO, HIGH);
  digitalWrite(IO_SD, LOW);
}




// CAMERA // 

// close the log file, open a picture file and write out the data, then re-open the log.
bool take_picture() {
  cameraWake();
  delay(50);
  camera.flush_fifo();
  camera.clear_fifo_flag();
  //Start capture
  camera.start_capture();
  wdt();
  while (!camera.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) ;
  
  logln("Camera took picture successfully");
  wdt();
  if(!read_fifo_burst(camera)) {
    logln("Camera picture failed!");
    fatalerr = true;
    breathe(YELLOW, RED);
    return false;
  }
  //Clear the capture done flag
  camera.clear_fifo_flag();
  delay(50);
  cameraSleep();
  wdt();
  return true;
}

// read the data and actually do the file opening/closing/writing.
bool read_fifo_burst(ArduCAM myCAM) {
  uint8_t tries = 0;
  bool is_header = false;
  char oldName[32];
  file.getName(oldName, 32);
  while(tries < MAX_SD_TRIES) {
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
    }
    wdt();

    uint8_t temp = 0, temp_last = 0;
    uint32_t length = 0; 
    length = myCAM.read_fifo_length();
    if (length >= MAX_FIFO_SIZE) { // 8mb
      logln("Picture oversize!");
      return false;
    }
    if (length == 0 ) //0 kb
    {
      logln("Picture size is 0!");
      return false;
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();//Set fifo burst mode
    wdt();
    while ( length-- )
    {
      temp_last = temp;
      temp =  SPI.transfer(0x00);
      if (is_header == true)
      {
        file.write(temp);
      }
      else if ((temp == 0xD8) & (temp_last == 0xFF))
      {
        is_header = true;
        file.write(temp_last);
        file.write(temp);
      }
      if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
      break;
      delayMicroseconds(5);
    }
    myCAM.CS_HIGH();
    is_header = false;

    wdt();
    await(50);
    if(!file.close()) {
      logln("Failed to close picture file!");
      await(50);
      ++tries;
      continue;
    }
    await(50);
    if(!file.open(oldName, O_WRONLY)) {
      logln("Failed to open log file, waiting 50ms");
      await(50);
      ++tries;
      continue;
    }
    logln("Image successfully taken");
    wdt();
    return true;
  }
  return true;
}

inline void cameraSleep() {
  camera.set_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK); 
}

inline void cameraWake() {
  camera.clear_bit(ARDUCHIP_GPIO,GPIO_PWDN_MASK); 
}









// GPS 

// feed the GPS object.
inline void feedGPS() {
   while (gpsSS.available()) { gps.encode(gpsSS.read()); }
}





// validation
inline bool tempValid() {
  return !isinf(temp.readTempC());
}

inline bool pressureValid() {
  return true;
}

inline bool timeValid() {
  return gps.time.isValid() && !(gps.time.hour() == 0 && gps.time.minute() == 0 && gps.time.second() == 0);
}

inline bool gpsReady() {
  return timeValid && !(gps.location.lat() == 0 && gps.location.lng() == 0 && gps.altitude.meters() == 0);
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

// poll the watchdog
inline void wdt() {
  if(WATCHDOG > 0) rp2040.wdt_reset();
}






// COLORS //

// update the RGB LED
void updateColors() {
  if(trgb[0] > rgb[0]) {
    rgb[0]++;
    analogWrite(LED_R, rgb[0]);
  }
  else if(trgb[0] < rgb[0]) {
    --rgb[0];
    analogWrite(LED_R, rgb[0]);
  }

  if(trgb[1] > rgb[1]) {
    rgb[1]++;
    analogWrite(LED_G, rgb[1]);
  }
  else if(trgb[1] < rgb[1]) {
    --rgb[1];
    analogWrite(LED_G, rgb[1]);
  }

  if(trgb[2] > rgb[2]) {
    rgb[2]++;
    analogWrite(LED_B, rgb[2]);
  }
  else if(trgb[2] < rgb[2]) {
    --rgb[2];
    analogWrite(LED_B, rgb[2]);
  }
  if(br) {
    if(colorcmp(rgb, trgb)) {
      const uint8_t temp[3] = {trgb[0], trgb[1], trgb[2]};
      trgb[0] = frgb[0];
      trgb[1] = frgb[1];
      trgb[2] = frgb[2];
      frgb[0] = temp[0];
      frgb[1] = temp[1];
      frgb[2] = temp[2];
    }
  }
}

void toColor(const uint8_t to[3]) {
  trgb[0] = to[0];
  trgb[1] = to[1];
  trgb[2] = to[2];
  br = false;
}

void color(const uint8_t to[3]) {
  rgb[0] = to[0];
  rgb[1] = to[1];
  rgb[2] = to[2];
  br = false;
}

void breathe(const uint8_t from[3], const uint8_t to[3]) {
  if(!colorcmp(from, rgb)) {
    color(from);
  }
  toColor(to);
  frgb[0] = from[0];
  frgb[1] = from[1];
  frgb[2] = from[2];
  br = true;
}

inline bool colorcmp(const uint8_t a[3], const uint8_t b[3]) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

inline void cancelBreathe() {
  br = false;
  color(OFF);
}

