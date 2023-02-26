#include "TinyGPSPlus.h"
#include "Adafruit_BMP3XX.h"
#include "Adafruit_MCP9808.h"
#include "SoftwareSerial.h"
#include "SdFat.h"
#include "sdios.h"

// PINS //
#define IO_MISO D12   //
#define IO_MOSI D11   // aka SPI1
#define IO_SCK D10    // 
#define IO_SD SDCARD_SS_PIN // see %localappdata%\Arduino15\packages\rp2040\hardware\rp2040\2.7.3\libraries\ESP8266SdFat\src\SdFatConfig.h
#define IO_RADIO D13

#define CAM_CS D17
#define CAM_MISO D16 //
#define CAM_MOSI D19 // aka SPI0
#define CAM_SCK D18  //
#define CAM_SDA D20  //
#define CAM_SCL D21  // aka I2C0

#define I2C_SDA D2  //
#define I2C_SCL D3  // aka I2C1

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
#define USE_CAMERA false
#define USE_RADIO false
#define USE_SD true
// weather the system should wait for the GPS to have a time before starting.
#define WAIT_FOR_GPS true


#define SD_CONFIG SdSpiConfig(IO_SD, SHARED_SPI, SD_SCK_MHZ(16))

// cycle hertz for the system. Basically its clock. Every time this is reached, a sensor sample is taken, and the various cycle counters for the below are incremented.
#define CYCLE_MILLIHERTZ 2000
// how many samples should be written to one file before swapping to next one.
#define CYCLES_PER_FILE 35
// how many sensor cycles to skip before taking a picture (e.g. 3xSENSOR_SAMPLE_WAIT of 5000 = every 15s)
#define CYCLES_PER_PIC 10
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
static const uint8_t YELLOW[3] = {200, 200, 0};   // I2C messages
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
  pinMode(LED_BUILTIN, OUTPUT);
  // turn off onboard LED until init done.
  digitalWrite(LED_BUILTIN, LOW);

  if(DEBUG) {
    Serial.begin(115200);
    // wait for serial init...
    while (!Serial) ;
    Serial.println("Init begin...");
  }

  // init LED
  if(USE_LED) {
    pinMode(LED_R, OUTPUT_2MA);
    pinMode(LED_G, OUTPUT_2MA);
    pinMode(LED_B, OUTPUT_2MA);
  }
  

  // init CS pins
  pinMode(IO_SD, OUTPUT);
  pinMode(IO_RADIO, OUTPUT);
  pinMode(CAM_CS, OUTPUT);


  // init I2C + SPI
  SPI1.setRX(IO_MISO);
  SPI1.setTX(IO_MOSI);
  SPI1.setSCK(IO_SCK);
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin();

  logln("Communication buses OK");

  if(USE_GPS) {
    // init gps
    gpsSS.begin(9600);
    logln("GPS OK");
  }

  // init SD
  if(USE_SD) {
    if(!sd.begin(SD_CONFIG)) {
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
  }

  /* init camera
  if(USE_CAMERA) {
    uint8_t vid,pid;
    uint8_t state;
    camera.CS_HIGH();
    camera.ioInit(CAM_SDA, CAM_SCL, CAM_MISO, CAM_MOSI, CAM_SCK);
    delay(500);
    // Check if the ArduCAM SPI bus is OK
    camera.write_reg(ARDUCHIP_TEST1, 0x55);
    state = camera.read_reg(ARDUCHIP_TEST1);
    if(state != 0x55) {
        logln("Camera SPI interface error!");
        return;
    }  
    // Change MCU mode
    camera.write_reg(ARDUCHIP_MODE, 0x00); 
    camera.wrSensorReg16_8(0xff, 0x01);
    camera.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
    camera.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
    if((vid != 0x56) || (pid != 0x42)) logln("Can't find camera module!");
    else logln("Camera OK");
    camera.InitCAM();
    camera.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);		//VSYNC is active HIGH   	  
    delay(500);
    take_picture();
  }*/

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

  state = 1;
  logln("initialized successfully");
  digitalWrite(LED_BUILTIN, HIGH);
  if(WAIT_FOR_GPS) breathe(OFF, PURPLE);
  else ready = true;
}


void loop() {
  unsigned long now = millis();
  if(fatalerr) {
    logln("Fatal error! Please restart.");
    await(5000);
    return;
  }
  if(!ready && timeValid()) {
    logln("Got time, ready!");
    if(USE_SD) incFileSafe();
    state = 200;
    ready = true;
    toColor(OFF);
  }
  if(!ready) {
    log("Waiting for GPS time... (c=");
    logi(gps.charsProcessed()); log("; s=");
    logi(gps.satellites.value()); logln(")");
    await(2000);
    return;
  }

  pressure = pres.readPressure();
  temperature = temp.readTempC();
  internal_temp = pres.readTemperature();
  if(gps.altitude.isValid()) {
    altitude = gps.altitude.meters();
  } else altitude = 0;
  if(gps.charsProcessed() != 0) state = 201;
  if(USE_CAMERA) {
    ++camcycles;
    if(camcycles > CYCLES_PER_PIC) {
      camcycles = 0;
      //take_picture();
    }
  }

  if(USE_SD) {
    ++sdcycles;
    if(sdcycles == CYCLES_PER_FILE) {
      incFileSafe();
      sdcycles = 0;
    }
    writeData();
  }

  if(USE_RADIO) {
    ++radiocycles;
    if(radiocycles > CYCLES_PER_RX) {
      sendRadio();
    }
  }

  if(DEBUG) {
    logln("State\tTime\t\tAlt.\tSpeed\t\tLat\t\tLong\t\tPressure\tTemperature\tInternal Temp\tSatellites");
    char cz[128];
    // tasty
    snprintf(cz, 128, "%i\t%02i:%02i:%02i\t%.4fm\t%.4fm/s\t%.6f\t%.6f\t%.4fhPa\t%.4fc\t%.4fc\t%i", state, gps.time.hour(), gps.time.minute(), gps.time.second(), altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, internal_temp, gps.satellites.value());
    logln(cz);
  }


  // wait the remaining time. Use this to prevent a possible negative number in ulong if it takes longer than CYCLE_MILLIHERTZ.
  long dur = millis() - now;
  if(dur < 0) {
    char c[32];
    snprintf(c, 32, "Running %i millis behind!", -dur);
    logln(c);
    return;
  }
  else await(CYCLE_MILLIHERTZ - dur);
}

// delay which still polls the GPS and updates the LED.
// if the GPS and LED is disabled, this will use pico low power sleep mode.
void await(unsigned long ms) {
  if(!USE_GPS && !USE_LED) {
    sleep_ms(ms);
    return;
  }
  unsigned long now = millis();
  uint32_t cycl = 0;
  do 
  {
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
  logln("Sending packet on radio...");
  uint8_t i = 0;
  while(i<=RADIO_REPEATS) {
    //lora.beginPacket();
    char cz[72];
    snprintf(cz, 72, "?%05X;%.8f;%.8f;%.8f;%.8f;%.8f;%.8f;%.8f!", state, altitude, gps.speed.mps(), gps.location.lat(), gps.location.lng(), pressure, temperature, internal_temp);
    //lora.print(cz);
    //lora.endPacket();
  }
  logln("Packet(s) sent");
}




// CAMERA // 
/*
void take_picture() {
  // Flush the FIFO
  camera.flush_fifo();    
  // Clear the capture done flag
  camera.clear_fifo_flag();
  camera.set_format(JPEG);
  camera.OV5642_set_JPEG_size(OV5642_320x240);
  // Start capture
  logln("Start capture\n");  
  camera.start_capture();
  while (!(camera.read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)){}
  logln("CAM Capture Done\n");
  read_fifo_burst();
  camera.clear_fifo_flag();
  camera.flush_fifo();
}

uint8_t read_fifo_burst()
{
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  bool is_header = false;
  static int i = 0;
  static int k = 0;
  char str[8];
  //File outFile;
  byte buf[256];
  length = camera.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //8M
  {
    logln("Over size.");
    return 0;
  }
  if (length == 0 ) //0 kb
  {
    log(F("Size is 0."));
    return 0;
  }
  camera.set_fifo_burst();//Set fifo burst mode
  i = 0;
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      camera.CS_HIGH();
      Serial.write(buf, i);
      //Close the file
      //outFile.close();
      log(F("OK"));
      is_header = false;
      camera.set_fifo_burst();
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        camera.CS_HIGH();
        Serial.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        camera.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      camera.CS_HIGH();
      String fileName = Stime + ".jpg";

      //outFile = SD.open(fileName, O_WRITE | O_CREAT | O_TRUNC);
      Serial.flush();

      //if (!outFile)
      //{
      //  log(F("File open failed"));
      //}
      camera.set_fifo_burst();
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
  camera.CS_HIGH();
  return 1;

}
*/




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








// GENERAL //

inline void logln(const char c[]) {
  if(DEBUG) Serial.println(c);
}

inline void logln(String s) {
  if(DEBUG) Serial.println(s);
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

