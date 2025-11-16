// Author: Gerard Harkema
// Date: November 2025
// Description: RPLIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

// =============================================================
// RPLIDAR.h — Fully Commented Version
// =============================================================
#ifndef RPLIDAR_H
#define RPLIDAR_H

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
#define RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN 720 // Number of samples collected per scan
#define RPLIDAR_MIN_RANGE_M  0.16    // Minimum valid reading in meters
#define RPLIDAR_MAX_RANGE_M  12.0    // Maximum valid reading in meters

// =============================================================
// Command bytes used by the RPLIDAR protocol
// =============================================================
#define RPLIDAR_CMD_SYNC_BYTE       0xA5
#define RPLIDAR_CMD_SYNC_BYTE2      0x5A

// Basic commands
#define RPLIDAR_CMD_GET_INFO        0x50
#define RPLIDAR_CMD_GET_HEALTH      0x52
#define RPLIDAR_CMD_GET_SAMPLERATE  0x59

// Scan control commands
#define RPLIDAR_CMD_SCAN            0x20
#define RPLIDAR_CMD_FORCE_SCAN      0x21
#define RPLIDAR_CMD_STOP            0x25

// Express/HQ scan (used by A2/A3 models)
#define RPLIDAR_CMD_EXPRESS_SCAN    0x82
#define RPLIDAR_CMD_HQ_SCAN         0x83

// Motor PWM control
#define RPLIDAR_CMD_SET_MOTOR_PWM   0xF0
#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG 0xFF

// Configuration commands
#define RPLIDAR_CMD_GET_LIDAR_CONF  0x84
#define RPLIDAR_CMD_SET_LIDAR_CONF  0x85

// Typical debug/internal
#define RPLIDAR_CMD_SCAN_EXPRESS    0x82
#define RPLIDAR_CMD_GET_COUNTER     0xC0
#define RPLIDAR_CMD_RESET           0x40

#define RPLIDAR_PAYLOAD_FLAG_HAS_PAYLOAD 0x80

// =============================================================
// Expected response descriptors
// =============================================================
static const uint8_t RPLIDAR_INFO_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x14, 0x00, 0x00, 0x00, 0x04
};

static const uint8_t RPLIDAR_HEALTH_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x03, 0x00, 0x00, 0x00, 0x06
};

static const uint8_t RPLIDAR_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t RPLIDAR_FORCE_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t RPLIDAR_EXPRESS_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x54, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t RPLIDAR_HQ_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x60, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t RPLIDAR_SAMPLERATE_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x04, 0x00, 0x00, 0x00, 0x15
};

#if 0
static const uint8_t RPLIDAR_GET_LIDAR_CONF_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x00, 0x00, 0x00, 0x00, 0x20
};
#endif
static const uint8_t RPLIDAR_START_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x40, 0x81
};



// =============================================================
// Data structures for responses
// =============================================================
typedef struct {
    uint8_t status;      // 0=Good, 1=Warning, 2=Error
    uint16_t error_code; // Detailed error information
} RplidarHealth;

// General information about the LIDAR
typedef struct {
    uint8_t model;            // Model ID
    uint8_t firmware_major;   // Firmware major version
    uint8_t firmware_minor;   // Firmware minor version
    uint8_t hardware;         // Hardware revision
    uint8_t serial[16];       // 128-bit unique serial number
} RplidarInfo;

// Measured sample rates
struct RplidarSampleRate {
    uint16_t standardScan_us;
    uint16_t expressScan_us;
};

// Individual scan point
typedef struct {
    float angle;       // graden [0-360)
    float distance;    // mm
    uint8_t quality;   // reflectie kwaliteit
    bool startFlag;    // S-bit: begin van nieuwe 360° scan
} RplidarMeasurement;

#define MAX_LIDAR_CONF_PAYLOAD 32

// Configuration response structure
typedef struct {
    uint8_t type;
    uint8_t payload[MAX_LIDAR_CONF_PAYLOAD];
    uint8_t length;
} RplidarConf;

// =============================================================
// RPLIDAR class definition
// =============================================================
class rplidar
{
private:
    HardwareSerial *LIDARSerial; // Pointer to UART interface
    uint8_t motor_pin;            // Motor PWM pin
    TaskHandle_t scanTaskHandle;  // FreeRTOS task handle for scanning
    bool scan_enable = false;          // Flag to control scanning task

#ifndef TESTING
    rcl_publisher_t laser_pub;    // micro-ROS publisher
    sensor_msgs__msg__LaserScan scan_msg; // ROS LaserScan message buffer
#endif

public:
#ifndef TESTING
    rplidar(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin,
            uint8_t lidar_rx_pin, uint8_t motor_pin);
#else
    rplidar(uint8_t uart_channel, uint8_t lidar_tx_pin,
            uint8_t lidar_rx_pin, uint8_t motor_pin);
#endif

    // Control motor speed (PWM percent)
    void setupMotorPWM(int percent);

    // Sends RESET command
    bool reset();

    // Queries device info
    bool getDeviceInfo(RplidarInfo *info, uint32_t timeout_ms = 1000);

    // Queries device health
    bool getDeviceHealth(RplidarHealth *health, uint32_t timeout_ms = 1000);

    // Starts scanning (normal or express)
    void startScan(bool express, uint32_t timeout_ms = 1000);

    // Reads the scan rate
    bool getSampleRate(RplidarSampleRate* rate, uint32_t timeout_ms = 1000);

    // Reads 1 scan point
    bool getScanValue(RplidarMeasurement* value, uint32_t timeout_ms = 1000);

    // Retrieves configuration block
    bool getLidarConf(uint8_t type, const uint8_t* requestPayload,
                      uint8_t requestLength, RplidarConf* conf,
                      uint32_t timeout_ms = 100);

    // Stops scan
    void stopScan();
#ifndef TESTING
    static void scanTaskFunction(void* parameter);

    // Publishes LaserScan message
    rcl_ret_t publish();
#endif

    ~rplidar();
};

#endif // RPLIDAR_H
