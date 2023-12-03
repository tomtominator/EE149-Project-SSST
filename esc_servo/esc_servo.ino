#include <Servo.h>

byte servo1 = 3;
byte servo2 = 5;
byte servo3 = 6;
byte servo4 = 9;
Servo esc1, esc2, esc3, esc4;

int STOPSPEED = 1000;

void setup() {
  esc1.attach(servo1);
  esc2.attach(servo2);
  esc3.attach(servo3);
  esc4.attach(servo4);
  delay(10000); // delay to allow the ESC to recognize the stopped signal.
}

void loop() {
  int motor_speeds[5] = {1010, 1000, 1000, 1000, 1000};

  for(int i =0; i<1; i++){
    int speed = motor_speeds[i];

    esc1.writeMicroseconds(speed);
    esc2.writeMicroseconds(speed);
    esc3.writeMicroseconds(speed);
    esc4.writeMicroseconds(speed);
    delay(1000);  // Run for some seconds
  }

  // Stop them just in case
  esc1.writeMicroseconds(STOPSPEED);
  esc2.writeMicroseconds(STOPSPEED);
  esc3.writeMicroseconds(STOPSPEED);
  esc4.writeMicroseconds(STOPSPEED);
  delay(20000); // Send signal to ESC.

  // Stop program execution
  while(true);

  

}
