// Author: Gerard Harkema
// Date: November 2025
// Description: ydlidar_t_mini_plus interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

// =============================================================
// ydlidar_t_mini_plus.h — Fully Commented Version
// =============================================================
#ifndef ydlidar_t_mini_plus_H
#define ydlidar_t_mini_plus_H

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
#define ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN 720//360 // Number of samples collected per scan
#define ydlidar_t_mini_plus_MIN_RANGE_M  0.10    // Minimum valid reading in meters
#define ydlidar_t_mini_plus_MAX_RANGE_M  12.0    // Maximum valid reading in meters

#define DEFAULT_LIDAR_MOTOR_PWM 0 // Default motor PWM value, not used for YDLIDAR T-mini Plus


enum
{
    // Maximum number of measurement nodes (Si) in a single protocol packet.
  ydlidar_t_mini_plus_POINT_PER_PACK = 80,
    // Packet header value seen on the wire as bytes [0xAA, 0x55] (little-endian 0x55AA).
  ydlidar_t_mini_plus_PACKET_HEADER = 0x55AA,
    // One Si node is 3 bytes: intensity, distance/flag low byte, distance high byte.
  ydlidar_t_mini_plus_SAMPLE_BYTES = 3,
    // LSB check bit for encoded FSA/LSA angle words (must be set).
  ydlidar_t_mini_plus_ANGLE_CHECKBIT = 0x0001,
};

// Individual scan point
typedef struct {
    float angle;       // graden [0-360)
    float distance;    // mm
    uint8_t quality;   // reflectie kwaliteit
    uint8_t interference_flag; // protocol interference marker
} ydlidar_t_mini_plusMeasurementElement;


typedef struct {
    ydlidar_t_mini_plusMeasurementElement point[ydlidar_t_mini_plus_POINT_PER_PACK];
    // Number of valid points parsed from the current packet (<= POINT_PER_PACK).
    uint8_t count;
    // Convenience value: scan_frequency_hz * 10, derived from CT metadata.
    int speed;
    // Current scan frequency estimate decoded from start packet CT bits.
    float scan_frequency_hz;
    // Decoded start angle of packet in degrees [0, 360).
    float start_angle;
    // Decoded end angle of packet in degrees [0, 360).
    float end_angle;
    // True when CT bit0 indicates start of a new revolution.
    bool start_packet;
} ydlidar_t_mini_plusMeasurement;


#define ANGLE_TO_RADIAN(angle) ((angle) * 3141.59 / 180000)
#define RADIAN_TO_ANGLE(angle) ((angle) * 180000 / 3141.59)

// =============================================================
// ydlidar_t_mini_plus class definition
// =============================================================
class ydlidar_t_mini_plus
{
private:
    // UART channel used for communication with the YDLIDAR module.
    HardwareSerial *LIDARSerial;
    // Background task that continuously reads packets and populates scan buffers.
    TaskHandle_t scanTaskHandle;
    // Enables/disables packet acquisition in scanTaskFunction.
    bool scan_enable = false;
    // Reserved for protocol mode differences; currently standard scan flow is used.
    bool express_mode = false;
    // Timestamp (millis) when the current scan cycle started.
    unsigned long scan_start_time;
    // Last measured/estimated scan period in milliseconds.
    unsigned long scan_time = 0;
    // Cached scan frequency decoded from CT metadata in start packets.
    float scan_frequency_hz = 0.0f;
    // Optional distance buffer placeholder (kept for compatibility).
    float *distance;
    // Control motor speed (PWM percent), currently not actively used for T-mini Plus.
    void setupMotorPWM(int percent);
    // Indicates parser synchronization to valid packet stream.
    bool syncronized = false;


#ifndef TESTING
    rcl_publisher_t laser_pub;    // micro-ROS publisher
    sensor_msgs__msg__LaserScan scan_msg; // ROS LaserScan message buffer
#endif

public:
#ifndef TESTING
    ydlidar_t_mini_plus(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin,
            uint8_t lidar_rx_pin, uint8_t motor_pin = 0);
#else
    ydlidar_t_mini_plus(uint8_t uart_channel, uint8_t lidar_tx_pin,
            uint8_t lidar_rx_pin, uint8_t motor_pin = 0);
#endif


    // Sends the soft restart command [A5 40].
    bool reset();


    // Starts scanning by sending command [A5 60] and waiting for scan descriptor.
    void startScan(uint32_t lidar_speed, uint32_t timeout_ms = 1000);


    // Parses one full packet and returns decoded points/angles/flags.
    bool getScanValue(ydlidar_t_mini_plusMeasurement* value, uint32_t timeout_ms = 1000);


    // Stops scanning with command [A5 65] and clears pending UART bytes.
    void stopScan();

    // Returns true when packet synchronization is valid.
    bool isSyncronized();
#ifndef TESTING
    static void scanTaskFunction(void* parameter);

    // Publishes LaserScan message
    rcl_ret_t publish();
#endif

    ~ydlidar_t_mini_plus();
};

#endif // ydlidar_t_mini_plus_H
