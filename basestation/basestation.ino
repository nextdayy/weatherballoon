#include <SPI.h>
#include <LoRa.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) ;

  Serial.print("init");

  if (!LoRa.begin(868E6)) {
    Serial.println("fail;");
    while (1);
  }

  // Uncomment the next line to disable the default AGC and set LNA gain, values between 1 - 6 are supported
  // LoRa.setGain(6);
  LoRa.setSpreadingFactor(12);

  // put the radio into receive mode
  LoRa.receive();
  Serial.println("ok;");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print(";");
    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println(";");
  }
}

