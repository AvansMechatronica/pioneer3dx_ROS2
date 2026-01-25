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
  delay(2000);
  if(imu->testConnection()) {
    Serial.printf("MPU6050 connection successful\n");
  } else {
    // Connection failed - halt execution
    Serial.printf("MPU6050 connection failed\n");
    while(1);
  } 
  delay(5000);
}


// ------------------- Loop -------------------
int i = 0;
void loop() {
  // Update IMU readings
  imu->update();

  // Convert radians to degrees
  float roll_deg = imu->getRoll() * 180.0 / PI;
  float pitch_deg = imu->getPitch() * 180.0 / PI;
  float yaw_deg = imu->getYaw() * 180.0 / PI;

  // Print orientation angles (in degrees)
  Serial.printf("%i, Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", i, roll_deg, pitch_deg, yaw_deg);

  // Print gravity vector components
  Serial.printf("Gravity: X: %.2f, Y: %.2f, Z: %.2f\n", imu->getGravity().x, imu->getGravity().y, imu->getGravity().z);
  i++;
  // Wait 500ms before next reading
  delay(500);
}
