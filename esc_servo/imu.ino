#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
// MPU9250 IMU;               //Change to the name of any supported IMU! 

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

calData calib = { 0 };  //Calibration data
// AccelData accelData;    //Sensor data
GyroData gyroData;
// MagData magData;


void start_imu(){
  
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  delay(1000);
  if (err != 0) {
    digitalWrite(LED3, HIGH);
    digitalWrite(LED5, HIGH);
    digitalWrite(LED6, HIGH);
    digitalWrite(LED9, HIGH);
    while (true);
  }

  if (IMU.hasMagnetometer()) {
    delay(1000);
    // Serial.println("Move IMU in figure 8 pattern until done.");
    delay(1000);
    IMU.calibrateMag(&calib);
    // Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }
  delay(2000);
  IMU.calibrateAccelGyro(&calib);
  delay(2000);
  IMU.init(calib, IMU_ADDRESS);

}

void setup_imu() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  int err = IMU.init(calib, IMU_ADDRESS);
  delay(1000);
  if (err != 0) {
    while (true);
  }
#ifdef PERFORM_CALIBRATION
  // Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }

  delay(2000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
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