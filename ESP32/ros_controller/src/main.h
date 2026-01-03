#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <odometry.h>
#include <jointstate.h>
#if defined(INCLUDE_LIDAR)
#include <lds08_lidar.h>
#include <buzzer.h>
//#include <rplidar.h>
#endif
#if defined(INCLUDE_IMU)
#include <imu_mpu6050.h>
#endif

#include <p3dx_interfaces/msg/status.h>


#include "wifi_network_config.h"

#include "motor_controller.h"
#include "pins.h"

// Display libraries
#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/Tiny3x3a2pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
//#include <SPI.h>
#include "tft_printf.h"

#define NODE_NAME "p3dx_controller"

// Voltage divider resistor values
#define R2 560000.0f // Resistor R2 value in ohms, according schematics
#define R3 100000.0f // Resistor R3 value in ohms, according schematics


#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

// Macro to check ROS return codes and trigger error handler on failure
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      DEBUG_PRINT("Fatal Error: %s (code %d) at line %d\n", rcl_error_string(temp_rc), temp_rc, __LINE__); \
      microros_error_handler(temp_rc, __LINE__); } \
  }

// Soft check version 
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      DEBUG_PRINT("uROS Warning: %s (code %d) at line %d\n", rcl_error_string(temp_rc), temp_rc, __LINE__); \
      microros_warning_handler(temp_rc, __LINE__); } \
  }

/**
 * Convert RCL return code to human-readable string
 */
const char* rcl_error_string(rcl_ret_t ret);

/**
 * Error handler that stops the lidar, displays error, and restarts ESP32
 */
void microros_error_handler(rcl_ret_t temp_rc, int line);

/**
 * Error warning that stops the lidar, displays error, and restarts ESP32
 */
void microros_warning_handler(rcl_ret_t temp_rc, int line);

/**
 * Error handler - displays error and restarts ESP32
 */
void error_handler(int line);

/**
 * Callback for cmd_vel topic - receives velocity commands
 */
void cmd_vel_subscription_callback(const void* msgin);

/**
 * Reset error state by checking both bumpers are pressed
 */
void reset_p3dx();

/**
 * Callback for reset topic - clears error state
 */
void reset_subscription_callback(const void* msgin);

/**
 * Read and publish the system state
 */
void statusPublisher();

#if defined(MULTIPLE_PUBLISH_EXECUTORS)
/**
 * Timer callback for odometry publisher
 */
void odomPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time);

#if defined(INCLUDE_IMU)
/**
 * Timer callback for IMU publisher
 */
void imuPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time);
#endif

/**
 * Timer callback for status publisher
 */
void statusPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time);

#if defined(INCLUDE_LIDAR)
/**
 * Timer callback for lidar publisher
 */
void lidarPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time);
#endif

/**
 * Timer callback for joint state publisher
 */
void jointstatePublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time);
#else
void Publisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time);
#endif



/**
 * Motor control task function
 * Performs PID control, odometry calculation, and motor command execution
 */
void motorcontrolTaskFunction(void* parameter);

/**
 * Interrupt handler for left wheel encoder
 */
void updateEncoderL();

/**
 * Interrupt handler for right wheel encoder
 */
void updateEncoderR();



#if defined(HANDLE_BUMPERS)
/**
 * Interrupt handler for bumper collision
 */
void bumber_hit();
#endif

/**
 * Reset button interrupt handler
 */
void reset_button_hit();

/**
 * Motors button interrupt handler
 */
void motors_button_hit();

/**
 * Initialize SPI display
 */
void init_display();

/**
 * Convert snake_case to camelCase
 */
char* convertToCamelCase(const char *input);

/**
 * Polling task function for buttons and bumpers
 */
void pollingTaskFunction(void* parameter);

#if 0
/**
 * Get synchronized time with ROS2 agent
 */
struct timespec getTime();

/**
 * Synchronize ESP32 time with ROS2 agent
 */
void syncTime();

#endif

#endif // MAIN_H
