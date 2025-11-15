#include "rplidar.h"


#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("LIDAR DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif


#ifndef TESTING
rplidar::rplidar(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#else
rplidar::rplidar(uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#endif
{
    this->motor_pin = motor_pin;
#if 0

    LIDARSerial->begin(115200, SERIAL_8N1, lidar_tx_pin, lidar_rx_pin);
#else
    LIDARSerial =  new HardwareSerial(uart_channel); // UART
    LIDARSerial->begin(115200, SERIAL_8N1, lidar_tx_pin, lidar_rx_pin);

#endif
    int8_t motorSpeedPercent = 100;
    pinMode(motor_pin, OUTPUT);
    //setupMotorPWM(motorSpeedPercent);

#ifndef TESTING
        // Create publisher
    rclc_publisher_init_default(
        &laser_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"
    );

    // Prepare the LaserScan message (fixed-size arrays)
    const int num_points = 314;  // ~180° scan with 0.01 rad increments

    scan_msg.ranges.data = (float*) malloc(num_points * sizeof(float));
    scan_msg.ranges.size = num_points;
    scan_msg.ranges.capacity = num_points;

    scan_msg.intensities.data = NULL;
    scan_msg.intensities.size = 0;
    scan_msg.intensities.capacity = 0;
#endif
}


void rplidar::setupMotorPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  // PWM via analogWrite (0-255)
  analogWrite(this->motor_pin , map(percent, 0, 100, 0, 255));
  DEBUG_PRINT("Motor PWM ingesteld op %d%%\n", percent);
}



bool rplidar::getDeviceInfo(RplidarInfo *info)
{
    uint8_t cmd[2] = {
        RPLIDAR_CMD_SYNC_BYTE,
        RPLIDAR_CMD_GET_INFO
    };

    // 1) Stuur request
    if (LIDARSerial->write(cmd, 2) != 2){
        DEBUG_PRINT("Fout bij het verzenden van het commando\n");
        return false;
    }

    delay(500); // Wacht even op response
    // 2) Lees descriptor
    uint8_t descriptor[7];
    if (LIDARSerial->available() < 7) {
      DEBUG_PRINT("Niet genoeg data voor descriptor\n");
      return false;
    }
    LIDARSerial->readBytes(descriptor, 7);

#if 0
    // Controleer of descriptor klopt
    if (memcmp(descriptor, RPLIDAR_INFO_DESCRIPTOR, 7) != 0){
        DEBUG_PRINT("Descriptor komt niet overeen\n");
        return false;
    }
#endif
    // 3) Lees 20 bytes van de payload
    uint8_t data[20];

    if (LIDARSerial->available() < 20) {
      DEBUG_PRINT("Niet genoeg data voor payload\n");
      return false;
    }
    LIDARSerial->readBytes(data, 20);


    // 4) Struct vullen
    info->model          = data[0];
    info->firmware_minor = data[1];
    info->firmware_major = data[2];
    info->hardware       = data[3];

    // serial number (LSB first)
    memcpy(info->serial, &data[4], 16);

    return true;
}

bool rplidar::getDeviceHealth(RplidarHealth *health)
{
    uint8_t cmd[2] = {
        RPLIDAR_CMD_SYNC_BYTE,
        RPLIDAR_CMD_GET_HEALTH
    };

    // 1) Stuur request
    if (LIDARSerial->write(cmd, 2) != 2){
        DEBUG_PRINT("Fout bij het verzenden van het commando\n");
        return false;
    }
    delay(200); // Wacht even op response

    // 2) Lees descriptor
    uint8_t descriptor[7];
    if (LIDARSerial->available() < 7) {
      DEBUG_PRINT("Niet genoeg data voor descriptor\n");
      return false;
    }
    LIDARSerial->readBytes(descriptor, 7);

    // Controleer descriptor
    if (memcmp(descriptor, RPLIDAR_HEALTH_DESCRIPTOR, 7) != 0){
      DEBUG_PRINT("Descriptor komt niet overeen\n");
      return false;
    }

    // 3) Lees 3-byte health data packet
    uint8_t data[3];
    if (LIDARSerial->available() < 3) {
      DEBUG_PRINT("Niet genoeg data voor payload\n");
      return false;
    }
    LIDARSerial->readBytes(data, 3);

    // 4) Struct vullen
    health->status = data[0];
    health->error_code = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

    return true;
}

bool rplidar::getSampleRate(RplidarSampleRate* rate) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_SAMPLERATE };

    // 1) Stuur request
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon GET_SAMPLERATE niet verzenden\n");
        return false;
    }

    delay(200); // Wacht even op response

    // 2) Wacht op descriptor
    uint32_t startTime = millis();
    if(LIDARSerial->available() < 7) {
      DEBUG_PRINT("Niet genoeg data voor descriptor\n");
      return false;
    }


    uint8_t descriptor[7];
    if (LIDARSerial->readBytes(descriptor, 7) != 7) return false;

    if (memcmp(descriptor, RPLIDAR_SAMPLERATE_DESCRIPTOR, 7) != 0) {
        DEBUG_PRINT("Descriptor komt niet overeen\n");
        return false;
    }

    // 3) Lees payload (4 bytes)
    if(LIDARSerial->available() < 4) {
      DEBUG_PRINT("Niet genoeg data voor payload\n");
      return false;
    }

    uint8_t data[4];
    if (LIDARSerial->readBytes(data, 4) != 4) return false;

    // 4) Decodeer
    rate->standardScan_us = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    rate->expressScan_us  = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

    return true;
}


void rplidar::startScan(bool express) {
    uint8_t cmd[2];
    
    if (express) {
        // Express scan commando (voor A2/A3 modellen)
        cmd[0] = RPLIDAR_CMD_SYNC_BYTE;
        cmd[1] = RPLIDAR_CMD_EXPRESS_SCAN;  // 0x82
    } else {
        // Standaard scan
        cmd[0] = RPLIDAR_CMD_SYNC_BYTE;
        cmd[1] = RPLIDAR_CMD_SCAN;          // 0x20
    }

    // Verzend het commando en controleer of alles correct is verzonden
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon scan commando niet verzenden\n");
        return;
    }

    if (express) {
        DEBUG_PRINT("Express scan gestart\n");
    } else {
        DEBUG_PRINT("Standaard scan gestart\n");
    }
}


void rplidar::stopScan() {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_STOP }; // 0x25

    // Verzend het stop commando
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon stop commando niet verzenden\n");
        return;
    }

    DEBUG_PRINT("Scan gestopt\n");
}


void rplidar::publish() {
#ifndef TESTING
    // Simuleer wat data voor testen
    DEBUG_PRINT("Publiceren van scan data (testmodus)...\n");
  // Implementatie voor het publiceren van scan data
      // Fill LaserScan data
    scan_msg.header.stamp.sec = millis() / 1000;
    scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    rosidl_runtime_c__String__init(&scan_msg.header.frame_id);
    micro_ros_string_utilities_set(scan_msg.header.frame_id, "lidar_frame\n");


    scan_msg.angle_min = -1.57;
    scan_msg.angle_max =  1.57;
    scan_msg.angle_increment = 0.01;
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 5.0;

    // Example filling ranges
    for (int i = 0; i < scan_msg.ranges.size; i++) {
        scan_msg.ranges.data[i] = 1.0;  // fake constant distance
    }

    rcl_publish(&laser_pub, &scan_msg, NULL);
#endif

}

bool rplidar::getScanValue(RplidarValue* value, uint32_t timeout_ms) {
    uint32_t startTime = millis();

    // Wacht tot er minstens 5 bytes beschikbaar zijn of timeout
    while (LIDARSerial->available() < 5) {
        if (millis() - startTime > timeout_ms) {
            return false; // Timeout
        }
    }

    uint8_t data[5];
    if (LIDARSerial->readBytes(data, 5) != 5) {
        return false; // Kon niet volledig lezen
    }

    // --- Decode standaard 5-byte scan packet ---
    bool startFlag = data[0] & 0x01;
    bool invertedFlag = (data[0] >> 1) & 0x01;

    // Hoek in Q6 (1/64 graden)
    uint16_t angleQ6 = ((data[1] >> 1) | ((uint16_t)data[2] << 7));
    float angle = angleQ6 / 64.0f;

    // Afstand in Q2 (1/4 mm)
    uint16_t distanceQ2 = (data[3] | ((uint16_t)data[4] << 8));
    float distance = distanceQ2 / 4.0f;

    // Controle op plausibele waarde
    if (distance <= 0.0f || distance > 6000.0f) {
        return false;
    }

    value->angle = angle;
    value->distance = distance;
    value->quality = 0; // Quality niet gedecodeerd (optioneel implementeren)

    return true;
}

rplidar::~rplidar()
{
}



