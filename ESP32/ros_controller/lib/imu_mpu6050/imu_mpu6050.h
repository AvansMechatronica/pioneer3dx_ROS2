// Author: Gerard Harkema
// Date: November 2025
// Description: IMU_MPU6050 interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

// =============================================================
// IMU_MPU6050.h — Fully Commented Version
// =============================================================
#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

// I2C communication library
#include <Wire.h>
// I2C device communication abstraction
#include "I2Cdev.h"
// MPU6050 Digital Motion Processor (DMP) library
#include "MPU6050_6Axis_MotionApps20.h"



#include "Arduino.h"

// Conditional compilation: exclude ROS dependencies during testing
#ifndef TESTING
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // For string handling in ROS messages
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#endif

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

// Main IMU class for MPU6050 sensor integration
class imu_mpu6050 {
  public:

    #ifdef TESTING
    // Constructor for testing mode (without ROS node)
    imu_mpu6050(int scl_pin, int sda_pin, int int_pin);
    #else
    // Constructor for ROS mode (with node reference)
    imu_mpu6050(const rcl_node_t *node, int scl_pin, int sda_pin, int int_pin);
    #endif
    
    // Destructor
    ~imu_mpu6050();
    
    // Initialize I2C communication and MPU6050
    bool initialize(int scl_pin, int sda_pin, int speed = 400000);
    
    // Test if MPU6050 is connected and responding
    bool testConnection();
    
    // Update sensor readings and calculate orientation
    void update();
    
    // Get roll angle in degrees
    float getRoll();
    
    // Get pitch angle in degrees
    float getPitch();
    
    // Get yaw angle in degrees
    float getYaw();

    VectorFloat getGravity();

#ifndef TESTING
    // Publishes IMU message to ROS topic
    rcl_ret_t publish();
#endif

  private:
    // MPU6050 sensor object
    MPU6050 *mpu;
    
    // Flag indicating DMP is ready to use
    bool dmpReady = false;
    
    // Stores interrupt status byte from MPU
    uint8_t mpuIntStatus;
    
    // Device status after initialization (0 = success)
    uint8_t devStatus;
    
    // Expected DMP packet size (default 42 bytes)
    uint16_t packetSize;
    
    // Count of bytes currently in FIFO buffer
    uint16_t fifoCount;
    
    // FIFO storage buffer
    uint8_t fifoBuffer[64];
    
    // Quaternion container [w, x, y, z]
    Quaternion q;
    
    // Gravity vector container [x, y, z]
    VectorFloat gravity;
    
    // Yaw, Pitch, Roll container [yaw, pitch, roll]
    float ypr[3];

    VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
    VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
    VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
    float euler[3];         // [psi, theta, phi]    Euler angle container


    // FreeRTOS task handle for IMU processing
    TaskHandle_t imuTaskHandle;
    
    // Static task function for FreeRTOS
    static void imuTaskFunction(void* parameter);

#ifndef TESTING

    // micro-ROS publisher for IMU data
    rcl_publisher_t imu_pub;
    
    // ROS Imu message buffer
    sensor_msgs__msg__Imu imu_msg;
#endif

};

#endif // IMU_MPU6050_H