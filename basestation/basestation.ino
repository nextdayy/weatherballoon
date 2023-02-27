#include <SPI.h>
#include <LoRa.h>

String lastPacket;
uint16_t state = 0;
float temperature = 0;
float internal_temp = 0;
float pressure = 0;
double altitude = 0;
float mps = 0;
float lat = 0;
float lng = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver Callback");

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Uncomment the next line to disable the default AGC and set LNA gain, values between 1 - 6 are supported
  // LoRa.setGain(6);
  
  // register the receive callback
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive();
}

void loop() {
  // do nothing
}

void onReceive(int packetSize) {
  // received a packet
  Serial.print("Packet recieved: ");

  char packet[packetSize];
  // read packet
  for (int i = 0; i < packetSize; i++) {
    packet[i] = LoRa.read();
  }
  if(packet[0] != '?' && packet[packetSize - 1] != '!') {
    Serial.print("invalid; '");
    Serial.print(packet);
    Serial.println("', ignoring");
    return;
  }

  if(strcmp(packet, lastPacket.c_str()) == 0) {
    Serial.println("identical, ignoring");
  } else {
    Serial.println("OK!");
    lastPacket = packet;

    String p = lastPacket.substring(1, sizeof(packet) - 2);
    char* token;
    uint8_t i = 0;
    token = strtok(p.c_str(), ";");
    while(token != NULL) {
      switch(i) {
        case 0:
          state = atoi(token);
          break;
        case 1:
          altitude = atof(token);
          break;
        case 2:
          mps = atof(token);
          break;
        case 3:
          lat = atof(token);
          break;
        case 4:
          lng = atof(token);
          break;
        case 5:
          pressure = atof(token);
          break;
        case 6:
          temperature = atof(token);
          break;
        case 7:
          internal_temp = atof(token);
          break;
      }
      ++i;
      token = strtok(NULL, ";");
    }

    logdata();
  }
}

void logdata() {
  Serial.println("State\tAlt.\tSpeed\t\tLat\t\tLong\t\tPressure\tTemperature\tInternal Temp");
  Serial.print(state); Serial.print("\t");
  Serial.print(altitude); Serial.print("m\t");
  Serial.print(mps); Serial.print("m/s\t");
  Serial.print(lat); Serial.print("\t");
  Serial.print(lng); Serial.print("\t");
  Serial.print(pressure); Serial.print("hPa\t");
  Serial.print(temperature); Serial.print("c\t");
  Serial.print(internal_temp); Serial.println("c");
  // sprintf on arduino doesnt support floats?... ok
  //char cz[128];
  // tasty
  //snprintf(cz, 128, "%i\t%.4fm\t%.4fm/s\t%.6f\t%.6f\t%.4fhPa\t%.4fc\t%.4fc", state, altitude, mps, lat, lng, pressure, temperature, internal_temp);
  //Serial.println(cz);
}
