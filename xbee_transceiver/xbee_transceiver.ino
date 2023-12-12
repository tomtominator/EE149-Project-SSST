#include <SoftwareSerial.h>

// Define the XBee module's RX and TX pins
const int xbee_Rx = 2;  // Connect XBee's TX to Arduino's pin 0
const int xbee_Tx = 3;  // Connect XBee's RX to Arduino's pin 1

SoftwareSerial Xbee(xbee_Rx, xbee_Tx);

void setup() {
  Serial.begin(115200);
  Xbee.begin(115200);
  Serial.println("Transmitter Init");
}

void loop() {
  // Send a message to the XBee receiver
  sendMessage("Hello from XBee Transmitter");
  if (Serial.parseInt())

  // Wait for a moment before sending the next message
  delay(2000);
}

void sendMessage(const char* message) {
  Serial.print("Sending message: ");
  Serial.println(message);
  
  Xbee.print(message);
  Xbee.print('\n'); // Add a newline character to indicate the end of the message
}
