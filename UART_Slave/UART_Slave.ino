// Code for NodeMCU ESP32S

#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2


int incoming  = 'a';
int LED = 2;

void setup()
{
  SerialPort.begin(9600, SERIAL_8N1, 16, 17); 
  pinMode(LED, OUTPUT);
}
void loop()
{
  if (SerialPort.available() > 0)
  {
    incoming = SerialPort.read();
    if (incoming == 'a') {
      digitalWrite(LED, LOW);
    }
    if (incoming == 'b') {
      digitalWrite(LED, HIGH);
    }
  }
}

// void setup() {
//   pinMode(33, OUTPUT);
// }

// void loop() {
//   digitalWrite(33, LOW);
// }
