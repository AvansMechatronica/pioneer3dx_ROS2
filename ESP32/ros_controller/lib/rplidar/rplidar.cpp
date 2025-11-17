// =============================================================
// rplidar.cpp 
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: RPLIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

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
// Constructor used when running under micro-ROS
rplidar::rplidar(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#else
// Constructor for standalone testing (no ROS)
rplidar::rplidar(uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#endif
{
    this->motor_pin = motor_pin;

    // Create UART interface for LIDAR
    LIDARSerial =  new HardwareSerial(uart_channel);
    DEBUG_PRINT("Initialiseer LIDAR op UART kanaal %d, TX pin %d, RX pin %d\n", uart_channel, lidar_tx_pin, lidar_rx_pin);
    LIDARSerial->begin(115200, SERIAL_8N1, lidar_tx_pin, lidar_rx_pin);
    delay(100);
    //reset(); // Reset LIDAR
    stopScan(); // Ensure scanning is stopped
    delay(100); // Wait for device to stabilize
    // Motor PWM pin
    pinMode(motor_pin, OUTPUT);
    setupMotorPWM(50); // Start motor at 50% speed

#ifdef DEBUG
    // Read device information for debugging
    RplidarInfo info;
    if(getDeviceInfo(&info)){
        Serial.println("Device info succesvol uitgelezen.");
        Serial.printf("Model: %u\n", info.model);
        Serial.printf("Firmware: %u.%u\n", info.firmware_major, info.firmware_minor);
        Serial.printf("Hardware: %u\n", info.hardware);

        Serial.print("Serial: ");
        for (int i = 15; i >= 0; i--)
            Serial.printf("%02X", info.serial[i]);
        Serial.println();
    }
    else {
        Serial.println("Fout bij uitlezen device info.");
    }   

    RplidarHealth health;
    if(getDeviceHealth(&health)) {
        Serial.println("Device health succesvol uitgelezen.");
        Serial.printf("Status: %u\n", health.status);
        Serial.printf("Error code: %u\n", health.error_code);
    }
    else {
        Serial.println("Fout bij uitlezen device health.");
    }   

    // Read sample rate
    RplidarSampleRate sampleRate;
    if(getSampleRate(&sampleRate)) {
        Serial.println("Sample rate succesvol uitgelezen.");
        Serial.printf("Standard scan: %u us\n", sampleRate.standardScan_us);
        Serial.printf("Express scan: %u us\n", sampleRate.expressScan_us);
    }
    else {
        Serial.println("Fout bij uitlezen sample rate.");
    }
#endif

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
    scan_msg.header.frame_id = micro_ros_string_utilities_set(scan_msg.header.frame_id, "rplidar_link");
    scan_msg.angle_min = -pi;
    scan_msg.angle_max =  pi;
    scan_msg.angle_increment = (2 * pi) / RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
    scan_msg.range_min = RPLIDAR_MIN_RANGE_M;
    scan_msg.range_max = RPLIDAR_MAX_RANGE_M;
    scan_msg.scan_time = 1.0 / 10.0; // Assuming 10 Hz scan rate
    scan_msg.time_increment = scan_msg.scan_time / RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;


    // Allocate range and intensity arrays
    scan_msg.ranges.data = (float*) malloc(RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    if (scan_msg.ranges.data == NULL) {
        DEBUG_PRINT("Fout bij toewijzen geheugen voor ranges array\n");
        // Handle memory allocation error appropriately
        scan_msg.ranges.size = 0;
        scan_msg.ranges.capacity = 0;
    }
    else { 
        scan_msg.ranges.size = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
        scan_msg.ranges.capacity = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
        // Initialize scan arrays
        for(int i = 0; i < RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
            scan_msg.ranges.data[i] = 0.0; // Default to 1 meter(dummy value)
        }
    }

    scan_msg.intensities.data = (float*) malloc(RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    if (scan_msg.intensities.data == NULL) {
        DEBUG_PRINT("Fout bij toewijzen geheugen voor intensities array\n");
        // Handle memory allocation error appropriately
        scan_msg.intensities.size = 0;
        scan_msg.intensities.capacity = 0;
    } else { 
        scan_msg.intensities.size = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
        scan_msg.intensities.capacity = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN;
        // Initialize scan arrays
        for(int i = 0; i < RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
            scan_msg.intensities.data[i] = 0.0;
        }
    }


    // Create FreeRTOS task for scanning
    const uint16_t stackSize = 4096; // Stack size in bytes
    const UBaseType_t priority = 5;   // Task priority
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
void rplidar::scanTaskFunction(void* parameter) {
    rplidar* lidar = static_cast<rplidar*>(parameter);
    sensor_msgs__msg__LaserScan& scan_msg = lidar->scan_msg;   
     
    RplidarMeasurement measurement;
    while (true) {
        if (lidar->scan_enable == false) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }
        if (lidar->getScanValue(&measurement, 100)) {
            if(scan_msg.ranges.data == NULL || scan_msg.intensities.data == NULL) {
                DEBUG_PRINT("Geheugen voor scan arrays niet toegewezen, verwerking overslaan\n");
                // Memory not allocated, skip processing
                continue;
            }
            // Process measurement (e.g., store in buffer, publish, etc.)
            #if 0
            DEBUG_PRINT("Angle: %.2f, Distance: %.2f mm, Quality: %u\n",
                        measurement.angle,
                        measurement.distance,
                        measurement.quality);
            #endif
            // Convert angle from degrees to radians
            float angle_rad = measurement.angle * (pi / 180.0f);
            
            // Normalize angle to range [-π, π]
            while (angle_rad > pi) angle_rad -= 2.0f * pi;
            while (angle_rad < -pi) angle_rad += 2.0f * pi;
            
            // Convert angle from [-π, π] to array index [0, RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN-1]
            // Formula: map [-π, π] to [0, N-1]
            int index = (int)(((angle_rad + pi) / (2.0f * pi)) * RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN);
            
            // Clamp index to valid range
            if (index < 0) index = 0;
            if (index >= RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN) 
                index = RPLIDAD_NUMBER_OF_SAMPLES_PER_SCAN - 1;
            
            // Update scan data
            lidar->scan_msg.ranges.data[index] = measurement.distance / 1000.0f; // mm to meters
            //lidar->scan_msg.intensities.data[index] = (float)measurement.quality;
        } else {
            DEBUG_PRINT("Geen scanwaarde ontvangen binnen timeout\n");
        }
        //taskYIELD();
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }
}
#endif

// Set motor speed via PWM [%]
void rplidar::setupMotorPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  analogWrite(this->motor_pin , map(percent, 0, 100, 0, 255));
  DEBUG_PRINT("Motor PWM ingesteld op %d%%\n", percent);
}

// Send reset command
bool rplidar::reset() {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_RESET };

    LIDARSerial->flush(); // Clear any old data
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon RESET niet verzenden\n");
        return false;
    }

    setupMotorPWM(0);
    delay(2); // Required minimum delay
    return true;
}

// Get basic device info (model, firmware version, serial)
bool rplidar::getDeviceInfo(RplidarInfo *info, uint32_t timeout_ms) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_INFO };

    LIDARSerial->flush(); // Clear any old data
    if (LIDARSerial->write(cmd, 2) != 2){
        DEBUG_PRINT("Fout bij verzenden device info commando\n");
        return false;   
    }

    delay(100); // Short delay to allow device to respond
    uint32_t startTime = millis();
#if 0
    // Descriptor is always 7 bytes
    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen device info descriptor ontvangen\n");
            return false;
        }
    }
#endif
    uint8_t descriptor[7];
    LIDARSerial->readBytes(descriptor, 7);

    if (memcmp(descriptor, RPLIDAR_INFO_DESCRIPTOR, 7) != 0){
        DEBUG_PRINT("Descriptor mismatch bij device info\n");
        DEBUG_PRINT("Verwacht: ");
        for(int i = 0; i < 7; i++) {
            DEBUG_PRINT("%i: %02X %02X \n", i, descriptor[i], RPLIDAR_INFO_DESCRIPTOR[i]);
        }
        
        return false;   
    }


    // Payload = 20 bytes
    while (LIDARSerial->available() < 20) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen device info data ontvangen\n");
            return false;
        }
    }

    uint8_t data[20];
    LIDARSerial->readBytes(data, 20);

    // Decode
    info->model          = data[0];
    info->firmware_minor = data[1];
    info->firmware_major = data[2];
    info->hardware       = data[3];

    memcpy(info->serial, &data[4], 16);

    return true;
}

// Generic LIDAR config request
bool rplidar::getLidarConf(uint8_t type, const uint8_t* requestPayload, uint8_t requestLength, RplidarConf* conf, uint32_t timeout_ms) {
    if (requestLength > MAX_LIDAR_CONF_PAYLOAD){
        DEBUG_PRINT("Fout: request payload lengte te groot\n");
        return false;
    }

    // Build request packet
    uint8_t cmd[1 + MAX_LIDAR_CONF_PAYLOAD];
    cmd[0] = type;
    if (requestPayload && requestLength > 0)
        memcpy(&cmd[1], requestPayload, requestLength);

    LIDARSerial->flush(); // Clear any old data
    // Send command
    if (LIDARSerial->write(RPLIDAR_CMD_SYNC_BYTE) != 1){
        DEBUG_PRINT("Fout: kon LIDAR CONF commando niet verzenden\n");
        return false;
    }
    if (LIDARSerial->write(RPLIDAR_CMD_GET_LIDAR_CONF) != 1){
        DEBUG_PRINT("Fout: kon LIDAR CONF commando niet verzenden\n");
        return false;
    }
    if (requestLength > 0)
        if (LIDARSerial->write(cmd, requestLength + 1) != requestLength + 1){
        DEBUG_PRINT("Fout: kon LIDAR CONF commando niet verzenden\n");
        return false;
        }

    uint32_t startTime = millis();

    // Descriptor
    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen device info descriptor ontvangen\n");
            return false;
        }
    }

    uint8_t descriptor[7];
    LIDARSerial->readBytes(descriptor, 7);

    // Must be single-response
    if ((descriptor[6] & 0x20) != 0x20){
        DEBUG_PRINT("Fout: geen single-response bericht ontvangen\n");
        return false;
    }

    uint8_t payloadLength = descriptor[2];
    if (payloadLength > MAX_LIDAR_CONF_PAYLOAD)
        payloadLength = MAX_LIDAR_CONF_PAYLOAD;

    uint8_t payload[MAX_LIDAR_CONF_PAYLOAD];

    while (LIDARSerial->available() < payloadLength) {
        if (millis() - startTime > timeout_ms) return false;
    }

    LIDARSerial->readBytes(payload, payloadLength);

    // Fill struct
    conf->type = type;
    memcpy(conf->payload, payload, payloadLength);
    conf->length = payloadLength;

    return true;
}

// Read device health
bool rplidar::getDeviceHealth(RplidarHealth *health, uint32_t timeout_ms) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_HEALTH };

    LIDARSerial->flush(); // Clear any old data
    if (LIDARSerial->write(cmd, 2) != 2){
        DEBUG_PRINT("Fout bij verzenden device health commando.");
        return false;
    }

    uint32_t startTime = millis();

    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen device health descriptor ontvangen.");
            return false;
        }
    }

    uint8_t descriptor[7];
    LIDARSerial->readBytes(descriptor, 7);

    if (memcmp(descriptor, RPLIDAR_HEALTH_DESCRIPTOR, 7) != 0){
        DEBUG_PRINT("Descriptor mismatch bij device health.");
        return false;
    }

    while (LIDARSerial->available() < 3) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen device health data ontvangen.");
            return false;
        }
    }

    uint8_t data[3];
    LIDARSerial->readBytes(data, 3);

    health->status = data[0];
    health->error_code = data[1] | (data[2] << 8);

    return true;
}

// Get scan sample rate
bool rplidar::getSampleRate(RplidarSampleRate* rate, uint32_t timeout_ms) {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_GET_SAMPLERATE };


    LIDARSerial->flush(); // Clear any old data
    if (LIDARSerial->write(cmd, 2) != 2){
        DEBUG_PRINT("Fout: kon SAMPLE RATE commando niet verzenden\n");
        return false;
    }

    uint32_t startTime = millis();

    while (LIDARSerial->available() < 7) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen SAMPLE RATE descriptor ontvangen\n");
            return false;
        }
    }

    uint8_t descriptor[7];
    LIDARSerial->readBytes(descriptor, 7);

    if (memcmp(descriptor, RPLIDAR_SAMPLERATE_DESCRIPTOR, 7) != 0) {
        DEBUG_PRINT("Descriptor mismatch\n");
        return false;
    }

    while (LIDARSerial->available() < 4) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen SAMPLE RATE data ontvangen\n");
            return false;
        }
    }

    uint8_t data[4];
    LIDARSerial->readBytes(data, 4);

    rate->standardScan_us = data[0] | (data[1] << 8);
    rate->expressScan_us  = data[2] | (data[3] << 8);

    return true;
}

// Start scanning in standard or express mode


void rplidar::startScan(bool express, uint32_t timeout_ms) {
    uint8_t cmd[2] = {
        RPLIDAR_CMD_SYNC_BYTE,
        express ? RPLIDAR_CMD_EXPRESS_SCAN : RPLIDAR_CMD_SCAN
    };

    // Clear any old data
    LIDARSerial->flush();

    // Verstuur scan commando
    if (LIDARSerial->write(cmd, 2) != 2) {
        DEBUG_PRINT("Fout: kon scan commando niet verzenden\n");
        return;
    }

    if (!express) {
        // Voor standaard SCAN, wacht op descriptor
        uint32_t startTime = millis();
        while (LIDARSerial->available() < 7) {
            if (millis() - startTime > timeout_ms) {
                DEBUG_PRINT("Timeout: geen SCAN descriptor ontvangen\n");
                return;
            }
        }

        uint8_t descriptor[7];
        if (LIDARSerial->readBytes(descriptor, 7) != 7) {
            DEBUG_PRINT("Fout bij lezen descriptor\n");
            return;
        }

        // Optioneel: descriptor check (kan je aanpassen)
        if(memcmp(descriptor, RPLIDAR_START_SCAN_DESCRIPTOR, 7) != 0){
            DEBUG_PRINT("Descriptor mismatch\n");
            //return;
        }
    }

    scan_enable = true; // Enable scanning in task
    if (express) {
        DEBUG_PRINT("Express scan gestart\n");
    } else {
        DEBUG_PRINT("Standaard scan gestart\n");
    }
}

// Stop scanning
void rplidar::stopScan() {
    uint8_t cmd[2] = { RPLIDAR_CMD_SYNC_BYTE, RPLIDAR_CMD_STOP };
   
    
    scan_enable = false; // Disable scanning in task
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for task to settle
    LIDARSerial->flush(); // Clear any old data
    if (LIDARSerial->write(cmd, 2) != 2)   {
        DEBUG_PRINT("Fout: kon STOP commando niet verzenden\n");
        return;
    }

    DEBUG_PRINT("Scan gestopt\n");
}

#ifndef TESTING
// Publish LaserScan message (placeholder implementation)
rcl_ret_t rplidar::publish() {

    rcl_ret_t return_code;
    scan_msg.header.stamp.sec = millis() / 1000;
    scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;


    return_code = rcl_publish(&laser_pub, &scan_msg, NULL);

    if (return_code != RCL_RET_OK) {
        DEBUG_PRINT("Fout bij publiceren LaserScan bericht: %d\n", return_code);
    }   
    return return_code;
}
#endif

// Read one 5-byte scan measurement packet
bool rplidar::getScanValue(RplidarMeasurement* value, uint32_t timeout_ms) {
    uint32_t startTime = millis();

    // Wacht tot minstens 5 bytes beschikbaar zijn
    while (LIDARSerial->available() < 5) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen scan data ontvangen\n");
            return false;
        }
        //taskYIELD();
    }

    uint8_t data[5];
    if (LIDARSerial->readBytes(data, 5) != 5) {
        DEBUG_PRINT("Fout bij lezen scan data\n");
        return false;
    }

    // --- Decodeer packet ---
    uint8_t byte0 = data[0];
    value->startFlag = byte0 & 0x1;             // S-bit
    bool invertedFlag = (byte0 >> 1) & 0x1;    // !S
    uint8_t checkBit = (byte0 >> 2) & 0x1;     // C-bit
    value->quality = byte0 >> 2;               // kwaliteit (bovenste bits)

    // hoek in Q6 (1/64 graden)
    uint16_t angleQ6 = ((data[1] >> 1) | ((uint16_t)data[2] << 7));
    value->angle = angleQ6 / 64.0f;

    // afstand in Q2 (1/4 mm)
    uint16_t distanceQ2 = data[3] | ((uint16_t)data[4] << 8);
    value->distance = distanceQ2 / 4.0f;

    return true;
}

rplidar::~rplidar() {}
