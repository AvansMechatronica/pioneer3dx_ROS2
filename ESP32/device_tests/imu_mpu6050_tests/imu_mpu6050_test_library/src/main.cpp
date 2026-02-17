/*
  MPU6050 DMP6 for ROS

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes auto-calibration and offsets generator tasks. Different 
  output formats available.

  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki
*/

#include "imu_mpu6050.h"
#include "pins.h"


imu_mpu6050 *imu;

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor to open
  
  imu = new imu_mpu6050(I2C_SCL_PIN, I2C_SDA_PIN, IMU_INT_PIN);
}

void loop() {

    delay(1000);

}