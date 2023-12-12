#include <SoftwareSerial.h>

// Define the XBee module's RX and TX pins
const int xbee_Rx = 2;  // Connect XBee's TX to Arduino's pin 2
const int xbee_Tx = 3;  // Connect XBee's RX to Arduino's pin 3

SoftwareSerial Xbee(xbee_Rx, xbee_Tx);

void setup() {
  Serial.begin(9600);
  Xbee.begin(9600);
  Serial.println("Receiver Init");
}

void loop() {
  if (Xbee.available()) {
    char data = Xbee.read();
    Serial.print("XBee: ");
    Serial.println(data);

    if (data == '1') {
      Serial.print("11111");
    }
    if (data == '2') {
      Serial.print("11111");
    }
  }
}
