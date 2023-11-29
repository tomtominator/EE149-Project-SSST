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

#include <Servo.h>

byte servoPin = 3; // signal pin for the ESC.
byte potentiometerPin = A0; // analog input pin for the potentiometer.
Servo servo;

void setup() {
servo.attach(servoPin);
//servo.writeMicroseconds(1500); // send "stop" signal to ESC. Also necessary to arm the ESC.

delay(7000); // delay to allow the ESC to recognize the stopped signal.
}

void loop() {

//int potVal = analogRead(potentiometerPin); // read input from potentiometer.

int pwmVal = map(1,0, 1023, 1100, 1900); // maps potentiometer values to PWM value.

servo.write(360); // Send signal to ESC.
}
