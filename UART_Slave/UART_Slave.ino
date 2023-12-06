// Ardunio Uno - UART Slave

int incoming  = 'a';
int LED = 13;

void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
}
void loop()
{
  if (Serial.available() > 0)
  {
    incoming = Serial.read();
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