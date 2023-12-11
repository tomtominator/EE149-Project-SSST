#include <SoftwareSerial.h>

// Define the XBee module's RX and TX pins
const int xbee_Rx = 0;  // Connect XBee's TX to Arduino's D2
const int xbee_Tx = 1;  // Connect XBee's RX to Arduino's D3

SoftwareSerial Xbee(xbee_Rx, xbee_Tx);

void setup() {
  Serial.begin(9600);
  Xbee.begin(9600);
  Serial.print("Init");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available()) { 
    Serial.print("hahahaha");
    Xbee.write(Serial.read());
  }
  if (Xbee.available()) { 
    Serial.print("hohohohoho");
    Serial.write(Xbee.read());
  }
}