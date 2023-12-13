#include <SoftwareSerial.h>

// Define the XBee module's RX and TX pins
const int xbee_Rx = 2;  // Connect XBee's TX to Arduino's pin 0
const int xbee_Tx = 3;  // Connect XBee's RX to Arduino's pin 1

SoftwareSerial Xbee(xbee_Rx, xbee_Tx);
String throttle;

void setup() {
  Serial.begin(115200);
  Xbee.begin(115200);
  Serial.println("Transmitter Init");
}

void loop() {
  // Send a message to the XBee receiver
  // sendMessage("Hello from XBee Transmitter");
  if (Serial.available()) {
    throttle = Serial.readString();
  }
  sendMessage(throttle);

  readMessage();

  // Wait for a moment before sending the next message
  delay(2000);
}

void readMessage() {
  String receivedString = "";

  while (Xbee.available() > 0) {
    char data = Xbee.read();
    receivedString += data;
  }

  Serial.print("Reading message: ");
  Serial.println(receivedString);
}

void sendMessage(const String message) {
  Serial.print("Sending message: ");
  Serial.println(message);
  Xbee.write(46);
  Xbee.write(message.charAt(0));
  Xbee.write(message.charAt(1));
  Xbee.write(message.charAt(2));
  Xbee.write(message.charAt(3));
  Xbee.write(46);

  // Serial.write(46);
  // Serial.write(49);
  // Serial.write(48);
  // Serial.write(48);
  // Serial.write(48);
  // Serial.write(46);
  
  // Xbee.print(message);
  // Xbee.print('\n'); // Add a newline character to indicate the end of the message
}
