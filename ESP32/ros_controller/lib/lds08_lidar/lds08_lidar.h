// Author: Gerard Harkema
// Date: November 2025
// Description: LDS08_LIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

// =============================================================
// LDS08_LIDAR.h — Fully Commented Version
// =============================================================
#ifndef LDS08_LIDAR_H
#define LDS08_LIDAR_H

#include "Arduino.h"

#ifndef TESTING
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // For string handling in ROS messages
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/laser_scan.h>
#endif

// Mathematical constant
constexpr double pi = 3.14159265358979323846;

// =============================================================
// LIDAR scan parameters
// =============================================================
#define LDS08_NUMBER_OF_SAMPLES_PER_SCAN 360 // Number of samples collected per scan
#define LDS08_LIDAR_MIN_RANGE_M  0.16    // Minimum valid reading in meters
#define LDS08_LIDAR_MAX_RANGE_M  12.0    // Maximum valid reading in meters

#define DEFAULT_LIDAR_MOTOR_PWM 60 // Default motor PWM value

static const uint8_t CrcTable[256] =
{
  0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
  0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
  0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
  0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
  0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
  0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
  0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
  0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
  0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
  0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
  0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
  0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
  0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
  0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
  0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
  0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
  0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
  0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
  0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
  0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
  0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
  0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

enum
{
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

// Individual scan point
typedef struct {
    float angle;       // graden [0-360)
    float distance;    // mm
    uint8_t quality;   // reflectie kwaliteit
} lds08_lidarMeasurementElement;


typedef struct {
    lds08_lidarMeasurementElement point[POINT_PER_PACK];
    int speed;
    float start_angle;
    float end_angle;
} lds08_lidarMeasurement;

typedef struct  __attribute__((packed))
{
  uint16_t distance;
  uint8_t confidence;
} LidarPointStructDef;

typedef struct  __attribute__((packed))
{
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;


#define ANGLE_TO_RADIAN(angle) ((angle) * 3141.59 / 180000)
#define RADIAN_TO_ANGLE(angle) ((angle) * 180000 / 3141.59)

// =============================================================
// LDS08_LIDAR class definition
// =============================================================
class lds08_lidar
{
private:
    HardwareSerial *LIDARSerial; // Pointer to UART interface
    uint8_t motor_pin;            // Motor PWM pin
    TaskHandle_t scanTaskHandle;  // FreeRTOS task handle for scanning
    bool scan_enable = false;          // Flag to control scanning task
    bool express_mode = false;        // Flag for express scan mode
    unsigned long scan_start_time;
    unsigned long scan_time = 0;
    float *distance;
    // Control motor speed (PWM percent)
    void setupMotorPWM(int percent);

#ifndef TESTING
    rcl_publisher_t laser_pub;    // micro-ROS publisher
    sensor_msgs__msg__LaserScan scan_msg; // ROS LaserScan message buffer
#endif

public:
#ifndef TESTING
    lds08_lidar(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin,
            uint8_t lidar_rx_pin, uint8_t motor_pin);
#else
    lds08_lidar(uint8_t uart_channel, uint8_t lidar_tx_pin,
            uint8_t lidar_rx_pin, uint8_t motor_pin);
#endif


    // Sends RESET command
    bool reset();


    // Starts scanning (normal or express)
    void startScan(uint32_t lidar_speed, uint32_t timeout_ms = 1000);


    // Reads 1 scan point
    bool getScanValue(lds08_lidarMeasurement* value, uint32_t timeout_ms = 1000);


    // Stops scan
    void stopScan();
#ifndef TESTING
    static void scanTaskFunction(void* parameter);

    // Publishes LaserScan message
    rcl_ret_t publish();
#endif

    ~lds08_lidar();
};

#endif // LDS08_LIDAR_H
