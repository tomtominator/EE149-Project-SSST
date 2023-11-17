#include "mpu9250.h"

bfs::Mpu9250 imu;

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  //I2C_ADDR_PRIM (0x68) if AD0 pin grounded
  //I2C_ADDR_SEC (0x69) if AD0 pin high
  
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while(1) {}
  }

  // added the config below jic, should be default regardless
  if (!imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_16G);) {
    Serial.println("Error configuring accelrange");
    while(1) {}
  }
  if (!imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_2000DPS) {   
    Serial.println("Error configuring gyrorange");
    while(1) {}
  }

} 

}

void loop() {
  /* Check if data read */
  if (imu.Read()) {
    Serial.print(imu.new_imu_data());
    Serial.print("\t");
    Serial.print(imu.new_mag_data());
    Serial.print("\t");
    Serial.print(imu.accel_x_mps2());
    Serial.print("\t");
    Serial.print(imu.accel_y_mps2());
    Serial.print("\t");
    Serial.print(imu.accel_z_mps2());
    Serial.print("\t");
    Serial.print(imu.gyro_x_radps());
    Serial.print("\t");
    Serial.print(imu.gyro_y_radps());
    Serial.print("\t");
    Serial.print(imu.gyro_z_radps());
    Serial.print("\t");
    Serial.print(imu.mag_x_ut());
    Serial.print("\t");
    Serial.print(imu.mag_y_ut());
    Serial.print("\t");
    Serial.print(imu.mag_z_ut());
    Serial.print("\t");
    Serial.print(imu.die_temp_c());
    Serial.print("\n");
  }

}
