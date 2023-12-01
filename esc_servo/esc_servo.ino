#include <Servo.h>

byte servo1 = 3; // signal pin for the ESC.
byte servo2 = 5;
byte servo3 = 6;
byte servo4 = 9;
Servo esc1, esc2, esc3, esc4;

void setup() {
  esc1.attach(servo1);
  esc2.attach(servo2);
  esc3.attach(servo3);
  esc4.attach(servo4);
  delay(3000); // delay to allow the ESC to recognize the stopped signal.
}

void loop() {
  bool ran = false;
  if(!ran){
    esc1.writeMicroseconds(1050);
    esc2.writeMicroseconds(1050);
    esc3.writeMicroseconds(1050);
    esc4.writeMicroseconds(1050);
    delay(2000);  // Run for 5 seconds
  
    // Stop the motor
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    delay(1000); // Send signal to ESC.
    ran = true;
  }
}
