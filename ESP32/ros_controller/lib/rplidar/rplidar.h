#ifndef RPLIDAR_H
#define RPLIDAR_H

#include "Arduino.h"

#ifndef TESTING
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/laser_scan.h>
#endif


// --- Algemene protocol bytes ---
#define RPLIDAR_CMD_SYNC_BYTE                0xA5
#define RPLIDAR_CMD_SYNC_BYTE2               0x5A    // gebruikt voor payload commands

// --- Basis informatie commando's ---
#define RPLIDAR_CMD_GET_INFO                 0x50
#define RPLIDAR_CMD_GET_HEALTH               0x52
#define RPLIDAR_CMD_GET_SAMPLERATE           0x59

// --- Scan control ---
#define RPLIDAR_CMD_SCAN                     0x20
#define RPLIDAR_CMD_FORCE_SCAN               0x21
#define RPLIDAR_CMD_STOP                     0x25

// --- Express scan modes ---
#define RPLIDAR_CMD_EXPRESS_SCAN             0x82    // A2/A3
#define RPLIDAR_CMD_HQ_SCAN                  0x83    // HQ-mode

// --- Motor control (via driver IC) ---
#define RPLIDAR_CMD_SET_MOTOR_PWM            0xF0
#define RPLIDAR_CMD_GET_ACC_BOARD_FLAG       0xFF

// --- Configuration interface (spec >= 1.29) ---
#define RPLIDAR_CMD_GET_LIDAR_CONF           0x84
#define RPLIDAR_CMD_SET_LIDAR_CONF           0x85   // (niet voor alle modellen)

// --- Dual / Ultra sampling modes ---
#define RPLIDAR_CMD_SCAN_EXPRESS              0x82  // zelfde als EXPRESS_SCAN
#define RPLIDAR_CMD_GET_COUNTER               0xC0  // interne debug
#define RPLIDAR_CMD_RESET                     0x40

// --- Payload specificaties ---
#define RPLIDAR_PAYLOAD_FLAG_HAS_PAYLOAD     0x80


// ========================================================
//  RPLIDAR RESPONSE DESCRIPTORS (7 bytes each)
// ========================================================

// --- GET_INFO (20 bytes payload, single response) ---
static const uint8_t RPLIDAR_INFO_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x14, 0x00, 0x00, 0x00, 0x04
};

// --- GET_HEALTH (3 bytes payload, single response) ---
static const uint8_t RPLIDAR_HEALTH_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x03, 0x00, 0x00, 0x00, 0x06
};

// --- SCAN (5 bytes per packet, continuous) ---
static const uint8_t RPLIDAR_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x00, 0x00
};

// --- FORCE_SCAN (same as SCAN) ---
static const uint8_t RPLIDAR_FORCE_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x05, 0x00, 0x00, 0x00, 0x00
};

// --- EXPRESS_SCAN (84 bytes per packet, continuous) ---
static const uint8_t RPLIDAR_EXPRESS_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x54, 0x00, 0x00, 0x00, 0x00
};

// --- HQ_SCAN (96 bytes per packet, continuous) ---
static const uint8_t RPLIDAR_HQ_SCAN_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x60, 0x00, 0x00, 0x00, 0x00
};

// --- GET_SAMPLERATE (4 bytes payload, single response) ---
static const uint8_t RPLIDAR_SAMPLERATE_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x04, 0x00, 0x00, 0x00, 0x04
};

// --- GET_LIDAR_CONF (variable length, single with payload) ---
static const uint8_t RPLIDAR_GET_LIDAR_CONF_DESCRIPTOR[7] = {
    RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_SYNC_BYTE2, 0x00, 0x00, 0x00, 0x00, 0x20
};




// Health struct
typedef struct {
    uint8_t status;      // 0 = Good, 1 = Warning, 2 = Error
    uint16_t error_code; // LSB first
} RplidarHealth;

// Struct die alle device info bevat
typedef struct {
    uint8_t model;
    uint8_t firmware_major;
    uint8_t firmware_minor;
    uint8_t hardware;
    uint8_t serial[16];   // 128-bit serial
} RplidarInfo;

struct RplidarSampleRate {
    uint16_t standardScan_us;  // tijd per meting in microseconden
    uint16_t expressScan_us;   // tijd per meting in express scan mode
};

typedef struct{
    float distance; // in cm
    float angle;    // in degrees
    uint8_t quality;
} RplidarValue;

class rplidar
{
private:
    /* data */
    HardwareSerial *LIDARSerial; // UART
    uint8_t motor_pin;
#ifndef TESTING
    rcl_publisher_t laser_pub;
    sensor_msgs__msg__LaserScan scan_msg;
#endif

public:
#ifndef TESTING
    rplidar(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin);
#else
    rplidar(uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin);
#endif
    void setupMotorPWM(int percent);
    bool getDeviceInfo(RplidarInfo *info);
    bool getDeviceHealth(RplidarHealth *health);
    void startScan(bool express);
    bool getSampleRate(RplidarSampleRate* rate);
    bool getScanValue(RplidarValue* value, uint32_t timeout_ms);
    void stopScan();
    void publish();
    ~rplidar();
};


#endif