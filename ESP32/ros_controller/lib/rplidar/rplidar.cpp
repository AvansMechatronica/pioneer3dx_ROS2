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

    LIDARSerial =  new HardwareSerial(uart_channel); // UART
    LIDARSerial->begin(115200, SERIAL_8N1, lidar_tx_pin, lidar_rx_pin);

    pinMode(motor_pin, OUTPUT);

#ifdef DEBUG
  // Device info en health uitlezen
  RplidarInfo info;
  if(getDeviceInfo(&info)){
    Serial.println("Device info succesvol uitgelezen.");
    Serial.printf("Model: %u\n", info.model);
    Serial.printf("Firmware: %u.%u\n", info.firmware_major, info.firmware_minor);
    Serial.printf("Hardware: %u\n", info.hardware);

    Serial.print("Serial: ");
    for (int i = 15; i >= 0; i--)   // reverse print
        Serial.printf("%02X", info.serial[i]);
    Serial.println();
  } else {
    Serial.println("Fout bij het uitlezen van device info.");
    }

  RplidarHealth health;

  if(getDeviceHealth(&health)) {
    Serial.println("Device health succesvol uitgelezen.");
    Serial.printf("Status: %u\n", health.status);
    Serial.printf("Error code: %u\n", health.error_code);
  } else {
    Serial.println("Fout bij het uitlezen van device health.");
  } 

  RplidarSampleRate sampleRate;
  if(getSampleRate(&sampleRate)) {
    Serial.println("Sample rate succesvol uitgelezen.");
    Serial.printf("Standard scan: %u us\n", sampleRate.standardScan_us);
    Serial.printf("Express scan: %u us\n", sampleRate.expressScan_us);
  } else {
    Serial.println("Fout bij het uitlezen van sample rate.");
  }

#endif
    
#ifndef TESTING
        // Create publisher
    rclc_publisher_init_default(
        &laser_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"
    );

    // Prepare the LaserScan message (fixed-size arrays)
    rosidl_runtime_c__String__init(&scan_msg.header.frame_id);
    scan_msg.header.frame_id = micro_ros_string_utilities_set(scan_msg.header.frame_id, "rplidar");
    scan_msg.angle_min = -pi;
    scan_msg.angle_max =  pi;
    scan_msg.angle_increment = (2 * pi) / RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
    scan_msg.range_min = RPLIDAR_MIN_RANGE_M;
    scan_msg.range_max = RPLIDAR_MAX_RANGE_M;

    scan_msg.ranges.data = (float*) malloc(RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    scan_msg.ranges.size = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
    scan_msg.ranges.capacity = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;

    scan_msg.intensities.data = (float*) malloc(RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    scan_msg.intensities.size = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
    scan_msg.intensities.capacity = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
    for(int i = 0; i < RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
        scan_msg.ranges.data[i] = 0.0;
        scan_msg.intensities.data[i] = 0.0;
    }
#endif
}


void rplidar::setupMotorPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  // PWM via analogWrite (0-255)
  analogWrite(this->motor_pin , map(percent, 0, 100, 0, 255));
  DEBUG_PRINT("Motor PWM ingesteld op %d%%\n", percent);
}

bool rplidar::reset() {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_RESET };

    // Stuur RESET request
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon RESET niet verzenden\n");
        return false;
    }
    setupMotorPWM(0);  // Motor uitzetten
    // Wacht minstens 2 ms volgens protocol
    delay(2);

    return true;
}


bool rplidar::getDeviceInfo(RplidarInfo *info, uint32_t timeout_ms) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_INFO };

    // --- 1) Stuur request ---
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon GET_INFO niet verzenden\n");
        return false;
    }

    uint32_t startTime = millis();

    // --- 2) Wacht op descriptor (7 bytes) ---
    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: descriptor niet ontvangen\n");
            return false;
        }
    }

    uint8_t descriptor[7];
    if (LIDARSerial->readBytes(descriptor, 7) != 7) {
        DEBUG_PRINT("Fout: descriptor niet volledig gelezen\n");
        return false;
    }

    if (memcmp(descriptor, RPLIDAR_INFO_DESCRIPTOR, 7) != 0) {
        DEBUG_PRINT("Descriptor komt niet overeen\n");
        for(int i = 0; i < 7; i++) {
          DEBUG_PRINT("0x%02X\n", descriptor[i]);
        }
        return false;
    }
 
    // --- 3) Wacht op payload (20 bytes) ---
    while (LIDARSerial->available() < 20) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: payload niet ontvangen\n");
            return false;
        }
    }

    uint8_t data[20];
    if (LIDARSerial->readBytes(data, 20) != 20) {
        DEBUG_PRINT("Fout: payload niet volledig gelezen\n");
        return false;
    }

    // --- 4) Struct vullen ---
    info->model          = data[0];
    info->firmware_minor = data[1];
    info->firmware_major = data[2];
    info->hardware       = data[3];

    // Serial number (LSB first)
    memcpy(info->serial, &data[4], 16);

    return true;
}



bool rplidar::getLidarConf(uint8_t type, const uint8_t* requestPayload, uint8_t requestLength, RplidarConf* conf, uint32_t timeout_ms) {
    if (requestLength > MAX_LIDAR_CONF_PAYLOAD) return false;

    // 1) Bouw request packet
    uint8_t cmd[1 + MAX_LIDAR_CONF_PAYLOAD];
    cmd[0] = type;  // type
    if (requestPayload && requestLength > 0) {
        memcpy(&cmd[1], requestPayload, requestLength);
    }

    // 2) Stuur request (A5 84 + payload)
    if (LIDARSerial->write(RPLIDAR_CMD_SYNC_BYTE) != 1){
      DEBUG_PRINT("Fout: kon LIDAR config sync byte niet verzenden\n");
      return false;
    } 
    if (LIDARSerial->write(RPLIDAR_CMD_GET_LIDAR_CONF) != 1){
      DEBUG_PRINT("Fout: kon LIDAR config command byte niet verzenden\n");
      return false;
    }
    if (requestLength > 0) {
        if (LIDARSerial->write(cmd, requestLength + 1) != requestLength + 1){
          DEBUG_PRINT("Fout: kon LIDAR config request niet volledig verzenden\n");
          return false;
        }
    }

    // 3) Wacht op descriptor
    uint32_t startTime = millis();
    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms){
          DEBUG_PRINT("Timeout: descriptor niet ontvangen\n");
          return false;
        } 
    }

    uint8_t descriptor[7];
    if (LIDARSerial->readBytes(descriptor, 7) != 7){
      DEBUG_PRINT("Fout: descriptor niet volledig gelezen\n");
      return false;
    }

    if ((descriptor[6] & 0x20) != 0x20) {
        Serial.println("Descriptor komt niet overeen of geen single response");
        return false;
    }

    // 4) Lees payload (variabele lengte)
    uint8_t payloadLength = descriptor[2]; // S-byte in descriptor = payload lengte
    if (payloadLength > MAX_LIDAR_CONF_PAYLOAD) payloadLength = MAX_LIDAR_CONF_PAYLOAD;

    uint8_t payload[MAX_LIDAR_CONF_PAYLOAD];
    while (LIDARSerial->available() < payloadLength) {
        if (millis() - startTime > timeout_ms){
          DEBUG_PRINT("Timeout: payload niet ontvangen\n");
          return false;
        }
    }

    if (LIDARSerial->readBytes(payload, payloadLength) != payloadLength) {
      DEBUG_PRINT("Fout: payload niet volledig gelezen\n");
      return false;
    }

    // 5) Vul struct
    conf->type = type;
    memcpy(conf->payload, payload, payloadLength);
    conf->length = payloadLength;

    return true;
}



bool rplidar::getDeviceHealth(RplidarHealth *health, uint32_t timeout_ms) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_HEALTH };

    // --- 1) Stuur request ---
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon GET_HEALTH niet verzenden\n");
        return false;
    }

    uint32_t startTime = millis();

    // --- 2) Wacht op descriptor (7 bytes) ---
    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: descriptor niet ontvangen\n");
            return false;
        }
    }

    uint8_t descriptor[7];
    if (LIDARSerial->readBytes(descriptor, 7) != 7) {
        DEBUG_PRINT("Fout: descriptor niet volledig gelezen\n");
        return false;
    }

    // Controleer descriptor
    if (memcmp(descriptor, RPLIDAR_HEALTH_DESCRIPTOR, 7) != 0) {
        DEBUG_PRINT("Descriptor komt niet overeen\n");
        return false;
    }

    // --- 3) Wacht op payload (3 bytes) ---
    while (LIDARSerial->available() < 3) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: payload niet ontvangen\n");
            return false;
        }
    }

    uint8_t data[3];
    if (LIDARSerial->readBytes(data, 3) != 3) {
        DEBUG_PRINT("Fout: payload niet volledig gelezen\n");
        return false;
    }

    // --- 4) Struct vullen ---
    health->status = data[0];
    health->error_code = (uint16_t)data[1] | ((uint16_t)data[2] << 8);

    return true;
}

bool rplidar::getSampleRate(RplidarSampleRate* rate, uint32_t timeout_ms) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_SAMPLERATE };

    // --- 1) Stuur request ---
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon GET_SAMPLERATE niet verzenden\n");
        return false;
    }

    uint32_t startTime = millis();

    // --- 2) Wacht op descriptor (7 bytes) ---
    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: descriptor niet ontvangen\n");
            return false;
        }
    }

    uint8_t descriptor[7];
    if (LIDARSerial->readBytes(descriptor, 7) != 7) {
        DEBUG_PRINT("Fout: descriptor niet volledig gelezen\n");
        return false;
    }

    // Controleer descriptor
    if (memcmp(descriptor, RPLIDAR_SAMPLERATE_DESCRIPTOR, 7) != 0) {
        DEBUG_PRINT("Descriptor komt niet overeen\n");
        return false;
    }

    // --- 3) Wacht op payload (4 bytes) ---
    while (LIDARSerial->available() < 4) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: payload niet ontvangen\n");
            return false;
        }
    }

    uint8_t data[4];
    if (LIDARSerial->readBytes(data, 4) != 4) {
        DEBUG_PRINT("Fout: payload niet volledig gelezen\n");
        return false;
    }

    // --- 4) Decodeer ---
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

#ifndef TESTING
rcl_ret_t rplidar::publish() {
    // Simuleer wat data voor testen
    DEBUG_PRINT("Publiceren van scan data (testmodus)...\n");
  // Implementatie voor het publiceren van scan data
      // Fill LaserScan data
    scan_msg.header.stamp.sec = millis() / 1000;
    scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;


    // Example filling ranges
    for (int i = 0; i < scan_msg.ranges.size; i++) {
        scan_msg.ranges.data[i] = 1.0;  // fake constant distance
    }

    return rcl_publish(&laser_pub, &scan_msg, NULL);

}
#endif

bool rplidar::getScanValue(RplidarValue* value, uint32_t timeout_ms) {
    uint32_t startTime = millis();

    // Wacht tot er minstens 5 bytes beschikbaar zijn of timeout
    while (LIDARSerial->available() < 5) {
        if (millis() - startTime > timeout_ms) {
            DEBUG_PRINT("Timeout: scan data niet ontvangen\n");
            return false; // Timeout
        }
    }

    uint8_t data[5];
    if (LIDARSerial->readBytes(data, 5) != 5) {
        DEBUG_PRINT("Fout: scan data niet volledig gelezen\n");
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

    if(angle > 360.0f || angle < 0.0f) {
      DEBUG_PRINT("Ongeldige hoek gemeten: %.2f graden\n", angle);
      return false;
    }


    // Controle op plausibele waarde
    if (distance <= 0.0f || distance > 6000.0f) {
        DEBUG_PRINT("Ongeldige afstand gemeten: %.2f cm\n", distance);
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



