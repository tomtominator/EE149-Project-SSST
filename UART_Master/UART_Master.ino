// ESP32 CAM - UART Master

void setup()  
{
  Serial.begin(9600, SERIAL_8N1, 3, 1); 
} 
void loop()  
{ 
  // Transmit
  Serial.print('a');
  delay(50);
  Serial.print('b');
  delay(50);
}