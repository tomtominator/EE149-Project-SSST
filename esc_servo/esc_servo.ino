#include <Servo.h>
#include "FastIMU.h"
#include <Wire.h>
MPU9250 IMU; 
byte servo3 = 3;
byte servo5 = 5;
byte servo6 = 6;
byte servo9 = 9;
Servo esc3, esc5, esc6, esc9;
AccelData accelData;

int LED3 = 12;
int LED5 = 12;
int LED6 = 12;
int LED9 = 12;


int STOP = 1000;
int STEADY = 1015;
int FASTER = 1020;
int SLOWER = 1010;


int TARGETX = -0.01;
int TARGETY = 0.04;
int TARGETZ = -1.00;

unsigned long START_TIME;
unsigned long RUNTIME = 10000;

//Directions named off of which propellors need increased speed to stabilize drone
enum DIRECTION{
  D3,
  D5,
  D6,
  D9,
  D39,
  D59,
  D36,
  D56
  };

void setup() {
  pinMode(LED3, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED9, OUTPUT);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED5, HIGH);
  digitalWrite(LED6, HIGH);
  digitalWrite(LED9, HIGH);

  while(true);

  start_imu();
  esc3.attach(servo3);
  esc5.attach(servo5);
  esc6.attach(servo6);
  esc9.attach(servo9);
  delay(10000); // delay to allow the ESC to recognize the stopped signal.
  START_TIME = millis();
}

void stop_drone(){
  esc3.writeMicroseconds(STOP);
  esc5.writeMicroseconds(STOP);
  esc6.writeMicroseconds(STOP);
  esc9.writeMicroseconds(STOP);
  delay(20000); // Send signal to ESC.

  // Stop program execution
  while(true);
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  int accelX = accelData.accelX;
  int accelY = accelData.accelY;
  bool one_proppelor_on = abs(abs(accelX) - abs(accelY)) >= 0.08;
  DIRECTION dir = NULL;
  if(accelX <=0 && accelY<=0){
    if (one_proppelor_on){
      dir = D6;
    }else{
      dir = D56;
    }
  }
  else if (accelX >=0 && accelY>=0){
    if (one_proppelor_on){
      dir = D9;
    }else{
      dir = D39;
    }
  }
  else if (accelX >=0 && accelY<=0){
    if (one_proppelor_on){
      dir = D3;
    }else{
      dir = D36;
    }
  }
  else if (accelX <=0 && accelY>=0){
    if (one_proppelor_on){
      dir = D5;
    }else{
      dir = D59;
    }
  }

  switch (dir){
    case D3: 
      esc3.writeMicroseconds(FASTER);
      digitalWrite(LED3, HIGH);
      break;
    case D5: 
      esc5.writeMicroseconds(FASTER);
      digitalWrite(LED5, HIGH);
      break;
    case D6: 
      esc6.writeMicroseconds(FASTER);
      digitalWrite(LED6, HIGH);
      break;
    case D9: 
      esc9.writeMicroseconds(FASTER);
      digitalWrite(LED9, HIGH);
      break;
    case D39: 
      esc3.writeMicroseconds(FASTER);
      esc9.writeMicroseconds(FASTER);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED9, HIGH);
      break;
    case D59: 
      esc5.writeMicroseconds(FASTER);
      esc9.writeMicroseconds(FASTER);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED9, HIGH);
      break;
    case D36: 
      esc3.writeMicroseconds(FASTER);
      esc6.writeMicroseconds(FASTER);
      digitalWrite(LED3, HIGH);
      digitalWrite(LED6, HIGH);
      break;
    case D56: 
      esc5.writeMicroseconds(FASTER);
      esc6.writeMicroseconds(FASTER);
      digitalWrite(LED5, HIGH);
      digitalWrite(LED6, HIGH);
      break;
  }
  delay(1000);
  esc3.writeMicroseconds(STEADY);
  esc5.writeMicroseconds(STEADY);
  esc6.writeMicroseconds(STEADY);
  esc9.writeMicroseconds(STEADY);
  digitalWrite(LED3, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  digitalWrite(LED9, LOW);
  
  delay(1000);
  if (millis () - START_TIME >= RUNTIME){
    stop_drone();
  }

}
