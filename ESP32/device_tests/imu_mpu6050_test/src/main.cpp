#include <Arduino.h>
#include "pins.h"
#include "imu_mpu6050.h"

// Global IMU sensor object pointer
imu_mpu6050* imu;


// ------------------- Setup -------------------
void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  delay(500);
  Serial.println("IMU MPU6050 - Setup gestart");

  // Create IMU object with specified I2C and interrupt pins
  imu = new imu_mpu6050(I2C_SCL_PIN, I2C_SDA_PIN, IMU_INT_PIN);

  Serial.printf("IMU initialized\n");
  
  // Test the connection to the MPU6050 sensor
  delay(1000);
  if(imu->testConnection()) {
    Serial.printf("MPU6050 connection successful\n");
  } else {
    // Connection failed - halt execution
    Serial.printf("MPU6050 connection failed\n");
    while(1);
  } 
}


// ------------------- Loop -------------------
void loop() {
  // Update IMU readings
  imu->update();
  
  // Print orientation angles (in degrees)
  Serial.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", imu->getRoll(), imu->getPitch(), imu->getYaw());
  
  // Print gravity vector components
  Serial.printf("Gravity: X: %.2f, Y: %.2f, Z: %.2f\n", imu->getGravity().x, imu->getGravity().y, imu->getGravity().z); 
  
  // Wait 500ms before next reading
  delay(500); 
}
