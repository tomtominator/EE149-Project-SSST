#include <SoftwareSerial.h>

// Define the XBee module's RX and TX pins
const int xbee_Rx = 0;  // Connect XBee's TX to Arduino's pin 0
const int xbee_Tx = 1;  // Connect XBee's RX to Arduino's pin 1

SoftwareSerial Xbee(xbee_Rx, xbee_Tx);

void setup() {
  Serial.begin(9600);
  Xbee.begin(9600);
  Serial.print("Init");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Xbee.available()) {
    char data = Xbee.read();
    Serial.print("Received from XBee: ");
    Serial.println(data);
  }
  
  if (Serial.available()) { 
    char data = Serial.read();
    Serial.print("Received from Serial Monitor: ");
    Serial.println(data);
  }
}