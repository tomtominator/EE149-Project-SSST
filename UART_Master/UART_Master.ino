#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2


// For testing
int incomingByte = 0; // for incoming serial data


void setup()  
{
  //Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, 16, 17); 
} 
void loop()  
{ 
  // Transmit
  SerialPort.print('a');
  delay(1000);
  SerialPort.print('b');
  delay(1000);
}
