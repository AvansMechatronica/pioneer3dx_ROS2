// =============================================================
// ydlidar_t_mini_plus.cpp 
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: RPLIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "ydlidar_t_mini_plus.h"

#ifndef TESTING
#include <rmw_microros/time_sync.h>
#endif

//#define DEBUG
#ifdef DEBUG_LIDAR
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("LIDAR DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

namespace {

/*
 * YDLIDAR T-mini Plus protocol constants
 * --------------------------------------
 * Command frame format: [0xA5, <cmd>]
 * - 0x60: start scan stream
 * - 0x65: stop scan
 * - 0x40: soft restart
 * - 0x0B: increase scan frequency by +1 Hz
 *
 * Scan descriptor response (7 bytes) starts with [0xA5, 0x5A] and must have
 * type code 0x81 for continuous scan data.
 */

constexpr uint8_t ydlidar_t_mini_plus_CMD_SYNC = 0xA5;
constexpr uint8_t ydlidar_t_mini_plus_CMD_SCAN = 0x60;
constexpr uint8_t ydlidar_t_mini_plus_CMD_STOP = 0x65;
constexpr uint8_t ydlidar_t_mini_plus_CMD_RESTART = 0x40;
constexpr uint8_t ydlidar_t_mini_plus_CMD_SCAN_FREQ_UP_1HZ = 0x0B;
constexpr uint8_t ydlidar_t_mini_plus_ANS_SYNC1 = 0xA5;
constexpr uint8_t ydlidar_t_mini_plus_ANS_SYNC2 = 0x5A;
constexpr uint8_t ydlidar_t_mini_plus_ANS_SCAN_TYPE = 0x81;
constexpr size_t ydlidar_t_mini_plus_SCAN_DESCRIPTOR_SIZE = 7;
constexpr uint8_t ydlidar_t_mini_plus_PACKET_HEADER_LOW = 0xAA;
constexpr uint8_t ydlidar_t_mini_plus_PACKET_HEADER_HIGH = 0x55;

struct ydlidar_t_mini_plusPacketHeader {
    // CT: bit0=start-of-rotation flag, bits7:1 contain metadata (frequency in first packet).
    uint8_t ct;
    // LSN: number of S(i) samples in this packet.
    uint8_t lsn;
    // FSA/LSA encoded as angle*64 with check bit in bit0.
    uint16_t start_angle;
    uint16_t end_angle;
    // XOR checksum over packet fields and sample payload according to manual.
    uint16_t checksum;
};

void clearSerialInput(HardwareSerial* serial) {
    while (serial->available() > 0) {
        serial->read();
    }
}

bool readByteWithTimeout(HardwareSerial* serial, uint8_t& value, uint32_t timeout_ms) {
    const uint32_t start_time = millis();
    while ((millis() - start_time) <= timeout_ms) {
        if (serial->available() > 0) {
            value = static_cast<uint8_t>(serial->read());
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

bool readBytesWithTimeout(HardwareSerial* serial, uint8_t* buffer, size_t length, uint32_t timeout_ms) {
    const uint32_t start_time = millis();
    size_t offset = 0;

    while (offset < length) {
        if (serial->available() > 0) {
            buffer[offset++] = static_cast<uint8_t>(serial->read());
            continue;
        }

        if ((millis() - start_time) > timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return true;
}

bool readScanDescriptor(HardwareSerial* serial, uint32_t timeout_ms) {
    // Descriptor format for scan command reply:
    // [A5 5A len_l len_m mode/type_l mode/type_h type]
    // We only validate synchronization bytes and packet type for scan stream.
    uint8_t descriptor[ydlidar_t_mini_plus_SCAN_DESCRIPTOR_SIZE];
    const uint32_t start_time = millis();

    while ((millis() - start_time) <= timeout_ms) {
        if (!readByteWithTimeout(serial, descriptor[0], timeout_ms)) {
            DEBUG_PRINT("Timeout while waiting for descriptor byte 0\n");
            return false;
        }

        //DEBUG_PRINT("Descriptor byte[0] = 0x%02X\n", descriptor[0]);

        if (descriptor[0] != ydlidar_t_mini_plus_ANS_SYNC1) {
            continue;
        }

        if (!readByteWithTimeout(serial, descriptor[1], timeout_ms)) {
            DEBUG_PRINT("Timeout while waiting for descriptor byte 1\n");
            return false;
        }

        //DEBUG_PRINT("Descriptor byte[1] = 0x%02X\n", descriptor[1]);

        if (descriptor[1] != ydlidar_t_mini_plus_ANS_SYNC2) {
            continue;
        }

        if (!readBytesWithTimeout(serial, &descriptor[2], sizeof(descriptor) - 2, timeout_ms)) {
            DEBUG_PRINT("Timeout while reading descriptor payload\n");
            return false;
        }

        //DEBUG_PRINT(
        //    "Descriptor payload = [%02X %02X %02X %02X %02X], type = 0x%02X\n",
        //    descriptor[2], descriptor[3], descriptor[4], descriptor[5], descriptor[6], descriptor[6]
        //);

        return descriptor[6] == ydlidar_t_mini_plus_ANS_SCAN_TYPE;
    }

    //DEBUG_PRINT("Descriptor search timed out after %lu ms\n", static_cast<unsigned long>(timeout_ms));
    return false;
}

float normalizeAngleDegrees(float angle_deg) {
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float clockwiseAngleDiff(float start_angle_deg, float end_angle_deg) {
    float diff = end_angle_deg - start_angle_deg;
    if (diff < 0.0f) {
        diff += 360.0f;
    }
    return diff;
}

uint16_t computePacketChecksum(const ydlidar_t_mini_plusPacketHeader& header, const uint8_t* sample_bytes) {
    // Checksum follows the vendor XOR rule using 16-bit words:
    // PH ^ (LSN|CT) ^ FSA ^ LSA ^ each sample as (00|S1) and (S3|S2).
    uint16_t checksum = ydlidar_t_mini_plus_PACKET_HEADER;
    checksum ^= header.start_angle;

    for (uint8_t index = 0; index < header.lsn; index++) {
        const size_t offset = static_cast<size_t>(index) * ydlidar_t_mini_plus_SAMPLE_BYTES;
        checksum ^= sample_bytes[offset];
        checksum ^= static_cast<uint16_t>(sample_bytes[offset + 2] << 8 | sample_bytes[offset + 1]);
    }

    checksum ^= static_cast<uint16_t>(header.lsn << 8 | header.ct);
    checksum ^= header.end_angle;
    return checksum;
}

uint16_t extractDistanceMm(const uint8_t* sample_bytes) {
    // S node layout (3 bytes):
    // byte0 = intensity, byte1 = [distance low bits + flags], byte2 = distance high bits.
    // Distance in mm is encoded as ((byte2 << 8) | byte1) >> 2.
    return static_cast<uint16_t>((static_cast<uint16_t>(sample_bytes[2]) << 8) | sample_bytes[1]) >> 2;
}

uint8_t extractInterferenceFlag(const uint8_t* sample_bytes) {
    return sample_bytes[1] & 0x03;
}

} // namespace

#ifndef TESTING
// Constructor used when running under micro-ROS
ydlidar_t_mini_plus::ydlidar_t_mini_plus(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#else
// Constructor for standalone testing (no ROS)
ydlidar_t_mini_plus::ydlidar_t_mini_plus(uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#endif
{

    delay(5000);

    // Create UART interface for LIDAR
    LIDARSerial =  new HardwareSerial(uart_channel);
    DEBUG_PRINT("Initialiseer LIDAR op UART kanaal %d, TX pin %d, RX pin %d\n", uart_channel, lidar_tx_pin, lidar_rx_pin);
    LIDARSerial->begin(230400, SERIAL_8N1, lidar_tx_pin, lidar_rx_pin);

    //reset(); // Reset LIDAR
    stopScan(); // Ensure scanning is stopped
    delay(100); // Wait for device to stabilize


#ifndef TESTING
    // Initialize ROS publisher for LaserScan
    rclc_publisher_init_default(
        &laser_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"
    );

    // Setup message fields
    rosidl_runtime_c__String__init(&scan_msg.header.frame_id);
    scan_msg.header.frame_id = micro_ros_string_utilities_set(scan_msg.header.frame_id, "laser");
    scan_msg.angle_min = -pi;
    scan_msg.angle_max =  pi;
    scan_msg.angle_increment = (2 * pi) / ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
    scan_msg.range_min = ydlidar_t_mini_plus_MIN_RANGE_M;
    scan_msg.range_max = ydlidar_t_mini_plus_MAX_RANGE_M;
    scan_msg.scan_time = 1.0 / 10.0; // Assuming 10 Hz scan rate
    scan_msg.time_increment = scan_msg.scan_time / ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
    // Allocate range and intensity arrays
    scan_msg.ranges.data = (float*) malloc(ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    if (scan_msg.ranges.data == NULL) {
        DEBUG_PRINT("Fout bij toewijzen geheugen voor ranges array\n");
        // Handle memory allocation error appropriately
        scan_msg.ranges.size = 0;
        scan_msg.ranges.capacity = 0;
    }
    else { 
            scan_msg.ranges.size = ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
            scan_msg.ranges.capacity = ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
        // Initialize scan arrays
        for(int i = 0; i < ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
            scan_msg.ranges.data[i] = INFINITY;
        }
    }

    scan_msg.intensities.data = (float*) malloc(ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    if (scan_msg.intensities.data == NULL) {
        DEBUG_PRINT("Fout bij toewijzen geheugen voor intensities array\n");
        // Handle memory allocation error appropriately
        scan_msg.intensities.size = 0;
        scan_msg.intensities.capacity = 0;
    } else { 
        scan_msg.intensities.size = ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
        scan_msg.intensities.capacity = ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
        // Initialize scan arrays
        for(int i = 0; i < ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
            scan_msg.intensities.data[i] = 0.0;
        }
    }


    // Create FreeRTOS task for scanning
    const uint16_t stackSize = 8192; // Stack size in bytes
    const UBaseType_t priority = 1;   // Task priority
    BaseType_t result = xTaskCreate(
        scanTaskFunction,       // Task function
        "LIDAR_Scan",          // Task name
        stackSize,             // Stack size
        this,                  // Parameter (this pointer)
        priority,              // Priority
        &scanTaskHandle        // Task handle
    );
    if (result != pdPASS) {
        DEBUG_PRINT("Fout bij aanmaken scan taak\n");
        // Handle task creation error appropriately
    }   
#endif

}

#ifndef TESTING
void ydlidar_t_mini_plus::scanTaskFunction(void* parameter) {
    ydlidar_t_mini_plus* lidar = static_cast<ydlidar_t_mini_plus*>(parameter);
    sensor_msgs__msg__LaserScan& scan_msg = lidar->scan_msg;   
     
    ydlidar_t_mini_plusMeasurement measurement;
    lidar->scan_start_time = millis();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (lidar->scan_enable == false) {
            continue;
        }
        if (lidar->getScanValue(&measurement, 100)) {
            if (scan_msg.ranges.data == NULL || scan_msg.intensities.data == NULL) {
                DEBUG_PRINT("Geheugen voor scan arrays niet toegewezen, verwerking overslaan\n");
                // Memory not allocated, skip processing
                continue;
            }
            // Process measurement (e.g., store in buffer, publish, etc.)
            // Convert angle from degrees to radians

            if (measurement.start_packet) {
                for (int i = 0; i < ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
                    lidar->scan_msg.ranges.data[i] = INFINITY;
                    lidar->scan_msg.intensities.data[i] = 0.0f;
                }

                if (measurement.scan_frequency_hz > 0.0f) {
                    lidar->scan_msg.scan_time = 1.0f / measurement.scan_frequency_hz;
                    lidar->scan_msg.time_increment = lidar->scan_msg.scan_time / ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN;
                }
            }

            for(int i = 0; i < measurement.count; i++) {
                
#if defined(LIDAR_ANGLE_OFFSET)
                measurement.point[i].angle += LIDAR_ANGLE_OFFSET;
#endif

                float angle_rad = measurement.point[i].angle * (pi / 180.0f);
                
                // Normalize angle to range [-π, π]
                while (angle_rad > pi) angle_rad -= 2.0f * pi;
                while (angle_rad < -pi) angle_rad += 2.0f * pi;
                
                // Convert angle from [-π, π] to array index [0, ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN-1]
                // Formula: map [-π, π] to [0, N-1]
                int index = (int)(((angle_rad + pi) / (2.0f * pi)) * ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN);
                
                // Clamp index to valid range
                if (index < 0) index = 0;
                if (index >= ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN) 
                    index = ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN - 1;
#if defined(LIDAR_INVERT_SCAN)
                index = ydlidar_t_mini_plus_NUMBER_OF_SAMPLES_PER_SCAN - 1 - index;
#endif

                const float distance_m = measurement.point[i].distance / 1000.0f;
                const bool invalid_point = measurement.point[i].distance <= 0.0f ||
                                           distance_m < ydlidar_t_mini_plus_MIN_RANGE_M ||
                                           distance_m > ydlidar_t_mini_plus_MAX_RANGE_M ||
                                           measurement.point[i].interference_flag == 2 ||
                                           measurement.point[i].interference_flag == 3;

                if (invalid_point) {
                    lidar->scan_msg.ranges.data[index] = INFINITY;
                    lidar->scan_msg.intensities.data[index] = 0.0f;
                    continue;
                }

                // Update scan data
                lidar->scan_msg.ranges.data[index] = distance_m;
                lidar->scan_msg.intensities.data[index] = (float)measurement.point[i].quality;
            }
        }
    }
}
#endif

// Set motor speed via PWM [%]
void ydlidar_t_mini_plus::setupMotorPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

//  analogWrite(this->motor_pin , map(percent, 0, 100, 0, 255));
  DEBUG_PRINT("Motor PWM ingesteld op %d%%\n", percent);
}


// Start scanning in standard or express mode
void ydlidar_t_mini_plus::startScan(uint32_t lidar_speed, uint32_t timeout_ms) {
    const uint8_t command[] = {ydlidar_t_mini_plus_CMD_SYNC, ydlidar_t_mini_plus_CMD_SCAN};
    const uint8_t restart_command[] = {ydlidar_t_mini_plus_CMD_SYNC, ydlidar_t_mini_plus_CMD_RESTART};
    const uint8_t freq_up_1hz_command[] = {ydlidar_t_mini_plus_CMD_SYNC, ydlidar_t_mini_plus_CMD_SCAN_FREQ_UP_1HZ};

    // Reset local parser state before issuing new scan commands.
    scan_enable = false;
    syncronized = false;
    scan_frequency_hz = 0.0f;

    clearSerialInput(LIDARSerial);


    // Send a soft restart so the sensor starts from a known state.
    LIDARSerial->write(restart_command, sizeof(restart_command));
    LIDARSerial->flush();
    delay(200);

    DEBUG_PRINT("Start scan command: [%02X %02X], timeout=%lu ms, uart_available=%d\n",
                command[0], command[1], static_cast<unsigned long>(timeout_ms), LIDARSerial->available());
//    setupMotorPWM(lidar_speed);
    delay(300);

    // Optional frequency tuning stage.
    // Each command [A5 0B] requests +1 Hz according to the protocol.
    for (int index = 0; index < 0; ++index) {
        LIDARSerial->write(freq_up_1hz_command, sizeof(freq_up_1hz_command));
        LIDARSerial->flush();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    DEBUG_PRINT("Scan frequency increase command sent (+3Hz)\n");

    LIDARSerial->write(command, sizeof(command));
    LIDARSerial->flush();
    DEBUG_PRINT("Start scan command sent\n");

    if (!readScanDescriptor(LIDARSerial, timeout_ms)) {
        DEBUG_PRINT("Geen geldige scan descriptor ontvangen\n");
        setupMotorPWM(0);
        return;
    }
    DEBUG_PRINT("Geldige scan descriptor ontvangen, start scan\n");

    scan_enable = true;
    delay(50);

}

// Stop scanning
void ydlidar_t_mini_plus::stopScan() {
    const uint8_t command[] = {ydlidar_t_mini_plus_CMD_SYNC, ydlidar_t_mini_plus_CMD_STOP};

    scan_enable = false; // Disable scanning in task
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for task to settle
    LIDARSerial->write(command, sizeof(command));
    LIDARSerial->flush();
    clearSerialInput(LIDARSerial);
//    setupMotorPWM(0); // Stop motor
    scan_frequency_hz = 0.0f;
    syncronized = false;
    delay(200); // Allow time to stop


    DEBUG_PRINT("Scan gestopt\n");
}

bool ydlidar_t_mini_plus::reset() {
    const uint8_t command[] = {ydlidar_t_mini_plus_CMD_SYNC, ydlidar_t_mini_plus_CMD_RESTART};

    scan_enable = false;
    syncronized = false;
    scan_frequency_hz = 0.0f;
    LIDARSerial->write(command, sizeof(command));
    LIDARSerial->flush();
    delay(200);
    clearSerialInput(LIDARSerial);
    return true;
}

#ifndef TESTING
// Publish LaserScan message (placeholder implementation)
rcl_ret_t ydlidar_t_mini_plus::publish() {

    rcl_ret_t return_code = RCL_RET_OK;
    int64_t now_ms = rmw_uros_epoch_millis();
    if (now_ms <= 0) {
        now_ms = static_cast<int64_t>(millis());
    }
    scan_msg.header.stamp.sec = static_cast<int32_t>(now_ms / 1000);
    scan_msg.header.stamp.nanosec = static_cast<uint32_t>((now_ms % 1000) * 1000000ULL);

    return_code = rcl_publish(&laser_pub, &scan_msg, NULL);

    if (return_code != RCL_RET_OK) {
        DEBUG_PRINT("Fout bij publiceren LaserScan bericht: %d\n", return_code);
    }   
    return return_code;
}
#endif

// Read one 5-byte scan measurement packet
bool ydlidar_t_mini_plus::getScanValue(ydlidar_t_mini_plusMeasurement* value, uint32_t timeout_ms) {
    // Packet parse pipeline:
    // 1) Find PH bytes [0xAA, 0x55]
    // 2) Read fixed header fields CT/LSN/FSA/LSA/CS
    // 3) Read LSN sample nodes (3 bytes each)
    // 4) Validate checksum
    // 5) Decode angles/distances/quality into measurement points
    uint32_t startTime = millis();
    ydlidar_t_mini_plusPacketHeader header;
    uint8_t first_byte = 0;
    uint8_t second_byte = 0;

    value->count = 0;
    value->speed = 0;
    value->scan_frequency_hz = scan_frequency_hz;
    value->start_packet = false;

    while ((millis() - startTime) <= timeout_ms) {
        if (!readByteWithTimeout(LIDARSerial, first_byte, timeout_ms)) {
            DEBUG_PRINT("Timeout: geen scan data ontvangen\n");
            syncronized = false;
            return false;
        }

        if (first_byte != ydlidar_t_mini_plus_PACKET_HEADER_LOW) {
            DEBUG_PRINT("Skip byte while searching packet header: 0x%02X\n", first_byte);
            continue;
        }

        if (!readByteWithTimeout(LIDARSerial, second_byte, timeout_ms)) {
            DEBUG_PRINT("Timeout: onvolledige packet header\n");
            syncronized = false;
            return false;
        }

        DEBUG_PRINT("Packet header candidate: [0x%02X 0x%02X]\n", first_byte, second_byte);

        if (second_byte == ydlidar_t_mini_plus_PACKET_HEADER_HIGH) {
            break;
        }
    }

    if (second_byte != ydlidar_t_mini_plus_PACKET_HEADER_HIGH) {
        DEBUG_PRINT("Geen geldige packet header gevonden\n");
        syncronized = false;
        return false;
    }

    uint8_t header_bytes[8];
    if (!readBytesWithTimeout(LIDARSerial, header_bytes, sizeof(header_bytes), timeout_ms)) {
        DEBUG_PRINT("Timeout bij lezen packet header\n");
        syncronized = false;
        return false;
    }

    header.ct = header_bytes[0];
    header.lsn = header_bytes[1];
    header.start_angle = static_cast<uint16_t>(header_bytes[2] | (header_bytes[3] << 8));
    header.end_angle = static_cast<uint16_t>(header_bytes[4] | (header_bytes[5] << 8));
    header.checksum = static_cast<uint16_t>(header_bytes[6] | (header_bytes[7] << 8));

    //DEBUG_PRINT("Packet header parsed: ct=0x%02X lsn=%u start=0x%04X end=0x%04X checksum=0x%04X\n",
    //            header.ct, header.lsn, header.start_angle, header.end_angle, header.checksum);

    if (header.lsn == 0 || header.lsn > ydlidar_t_mini_plus_POINT_PER_PACK) {
        DEBUG_PRINT("Ongeldig aantal samples in packet: %u\n", header.lsn);
        syncronized = false;
        return false;
    }

    if ((header.start_angle & ydlidar_t_mini_plus_ANGLE_CHECKBIT) == 0 ||
        (header.end_angle & ydlidar_t_mini_plus_ANGLE_CHECKBIT) == 0) {
        // The manual specifies bit0 of encoded angle words must always be 1.
        DEBUG_PRINT("Hoek checkbit ontbreekt\n");
        syncronized = false;
        return false;
    }

    uint8_t sample_bytes[ydlidar_t_mini_plus_POINT_PER_PACK * ydlidar_t_mini_plus_SAMPLE_BYTES];
    const size_t sample_length = static_cast<size_t>(header.lsn) * ydlidar_t_mini_plus_SAMPLE_BYTES;
    if (!readBytesWithTimeout(LIDARSerial, sample_bytes, sample_length, timeout_ms)) {
        DEBUG_PRINT("Timeout bij lezen packet samples\n");
        syncronized = false;
        return false;
    }

    const uint16_t checksum = computePacketChecksum(header, sample_bytes);
    if (checksum != header.checksum) {
        DEBUG_PRINT("Checksum fout: berekend 0x%04X, ontvangen 0x%04X\n", checksum, header.checksum);
        syncronized = false;
        return false;
    }

    DEBUG_PRINT("Packet checksum OK: 0x%04X\n", checksum);

    value->count = header.lsn;
    value->start_packet = (header.ct & 0x01) != 0;
    value->start_angle = static_cast<float>(header.start_angle >> 1) / 64.0f;
    value->end_angle = static_cast<float>(header.end_angle >> 1) / 64.0f;
    value->start_angle = normalizeAngleDegrees(value->start_angle);
    value->end_angle = normalizeAngleDegrees(value->end_angle);

    if (value->start_packet) {
        // In the first packet of a revolution, CT[7:1] encodes scan frequency*10.
        const float packet_scan_frequency = static_cast<float>((header.ct & 0xFE) >> 1) / 10.0f;
        if (packet_scan_frequency > 0.0f) {
            scan_frequency_hz = packet_scan_frequency;
        }
        DEBUG_PRINT("Start packet detected, scan_frequency_hz=%.2f\n", scan_frequency_hz);
    }

    value->scan_frequency_hz = scan_frequency_hz;
    value->speed = static_cast<int>(scan_frequency_hz * 10.0f);

    const float angle_diff = clockwiseAngleDiff(value->start_angle, value->end_angle);
    DEBUG_PRINT("Angles: start=%.2f end=%.2f diff=%.2f count=%u\n",
                value->start_angle, value->end_angle, angle_diff, value->count);

    for (uint8_t i = 0; i < header.lsn; i++) {
        const uint8_t* sample = &sample_bytes[static_cast<size_t>(i) * ydlidar_t_mini_plus_SAMPLE_BYTES];
        float angle = value->start_angle;

        // Interpolate each sample angle between FSA and LSA across LSN samples.
        if (header.lsn > 1) {
            angle += angle_diff * static_cast<float>(i) / static_cast<float>(header.lsn - 1);
        }

        value->point[i].angle = normalizeAngleDegrees(angle);
        value->point[i].distance = static_cast<float>(extractDistanceMm(sample));
        value->point[i].quality = sample[0];
        value->point[i].interference_flag = extractInterferenceFlag(sample);
    }

    syncronized = true;
    return true;
}


bool ydlidar_t_mini_plus::isSyncronized() {
    return syncronized;
}

ydlidar_t_mini_plus::~ydlidar_t_mini_plus() {}
