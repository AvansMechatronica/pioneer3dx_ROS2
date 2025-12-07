#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Arduino.h"

#ifndef TESTING
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // For string handling in ROS messages
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#endif


class IMU_MPU6050 {
  public:
    IMU_MPU6050(const rcl_node_t *node = nullptr);
    ~IMU_MPU6050();
    void initialize();
    bool testConnection();
    void update();
    float getRoll();
    float getPitch();
    float getYaw();
#ifndef TESTING
    //static void scanTaskFunction(void* parameter);

    // Publishes LaserScan message
    rcl_ret_t publish();
#endif
  private:
    MPU6050 mpu;
    bool dmpReady;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

#ifndef TESTING
    rcl_publisher_t imu_pub;    // micro-ROS publisher
    sensor_msgs__msg__Imu imu_msg; // ROS Imu message buffer
#endif

};

#endif