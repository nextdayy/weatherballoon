#include <SPI.h>
#include <LoRa.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  Serial.print("initbegin");

  if (!LoRa.begin(868E6)) {
    Serial.print("lorafail;");
    while (1);
  }

  // Uncomment the next line to disable the default AGC and set LNA gain, values between 1 - 6 are supported
  // LoRa.setGain(6);

  // put the radio into receive mode
  LoRa.receive();
  Serial.print("initok;");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("pckt'");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.print("';");
  }
}

