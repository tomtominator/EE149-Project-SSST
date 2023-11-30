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
  delay(1000); // delay to allow the ESC to recognize the stopped signal.
}

void loop() {
  esc1.writeMicroseconds(1050);
  esc2.writeMicroseconds(1050);
  esc3.writeMicroseconds(1050);
  esc4.writeMicroseconds(1050);
  delay(5000);  // Run at full speed for 5 seconds

  // Stop the motor
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(10000); // Send signal to ESC.
}

//
//#include <Servo.h>
//
//Servo ESC1, ESC2, ESC3;    
////Servo ESC2;    
//
//
//void setup() {
//  // Attach the ESC on pin 9
//  ESC1.attach(3,1000,2000);
//  ESC2.attach(5,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
//  ESC3.attach(6,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
//}
//
//void loop() {
////  int escVal = map(50, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
////  int escVal1 = 1; // needs to be between 0 and 180
//  ESC1.write(1);    // Send the signal to the ESC
////  int escVal2 = 1; // needs to be between 0 and 180
//  ESC2.write(1);    // Send the signal to the ESC
////  int escVal3 = 1; // needs to be between 0 and 180
//  ESC3.write(1);    // Send the signal to the ESC
//
//}

