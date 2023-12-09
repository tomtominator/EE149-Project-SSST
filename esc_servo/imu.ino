#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU9250 IMU;               //Change to the name of any supported IMU! 


calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

void start_imu(){
  Serial.begin(115200);
  Serial.println("STARTING");
  
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  delay(1000);
  if (err != 0) {
    Serial.println("IMU ERROR");
    // digitalWrite(LED3, HIGH);
    // digitalWrite(LED5, HIGH);
    // digitalWrite(LED6, HIGH);
    // digitalWrite(LED9, HIGH);
    while (true);
  }

  if (IMU.hasMagnetometer()) {
    // delay(1000);
    // Serial.println("Move IMU in figure 8 pattern until done.");
    delay(1000);
    IMU.calibrateMag(&calib);
    // Serial.println("Magnetic calibration done!");
  }
  else {
    Serial.println("No magnometer");
    delay(1000);
  }
  delay(1000);
  IMU.calibrateAccelGyro(&calib);
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);

}

void setup(){
  start_imu();
}

void loop(){
  imu_loop();
}

void imu_loop() {
  Serial.print("accelX, accelY, accelZ, gyroX, gyroY, gyroZ \n");
  while(true){
    IMU.update();
    IMU.getAccel(&accelData);
    Serial.print(accelData.accelX);
    Serial.print("\t");
    Serial.print(accelData.accelY);
    Serial.print("\t");
    Serial.print(accelData.accelZ);
    Serial.print("\t");
    IMU.getGyro(&gyroData);
    Serial.print(gyroData.gyroX);
    Serial.print("\t");
    Serial.print(gyroData.gyroY);
    Serial.print("\t");
    Serial.print(gyroData.gyroZ);
    Serial.print("\n");
    // if (IMU.hasMagnetometer()) {
    //   IMU.getMag(&magData);
    //   Serial.print("\t");
    //   Serial.print(magData.magX);
    //   Serial.print("\t");
    //   Serial.print(magData.magY);
    //   Serial.print("\t");
    //   Serial.print(magData.magZ);
    // }
    // if (IMU.hasTemperature()) {
    //   Serial.print("\t");
    //   Serial.println(IMU.getTemp());
    // }
    delay(1000);
  }
}
