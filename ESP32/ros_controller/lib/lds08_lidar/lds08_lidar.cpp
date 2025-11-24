// =============================================================
// lds08_lidar.cpp 
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: RPLIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "lds08_lidar.h"

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
lds08_lidar::lds08_lidar(const rcl_node_t *node, uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#else
// Constructor for standalone testing (no ROS)
lds08_lidar::lds08_lidar(uint8_t uart_channel, uint8_t lidar_tx_pin, uint8_t lidar_rx_pin, uint8_t motor_pin)
#endif
{
    this->motor_pin = motor_pin;
    pinMode(motor_pin, OUTPUT);
    setupMotorPWM(0); // Start motor at 50% speed
    delay(5000);

    // Create UART interface for LIDAR
    LIDARSerial =  new HardwareSerial(uart_channel);
    DEBUG_PRINT("Initialiseer LIDAR op UART kanaal %d, TX pin %d, RX pin %d\n", uart_channel, lidar_tx_pin, lidar_rx_pin);
    LIDARSerial->begin(115200, SERIAL_8N1, lidar_tx_pin, lidar_rx_pin);

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
    scan_msg.header.frame_id = micro_ros_string_utilities_set(scan_msg.header.frame_id, "lds08_lidar");
    scan_msg.angle_min = -pi;
    scan_msg.angle_max =  pi;
    scan_msg.angle_increment = (2 * pi) / LDS08_NUMBER_OF_SAMPLES_PER_SCAN;
    scan_msg.range_min = LDS08_LIDAR_MIN_RANGE_M;
    scan_msg.range_max = LDS08_LIDAR_MAX_RANGE_M;
    scan_msg.scan_time = 1.0 / 10.0; // Assuming 10 Hz scan rate
    scan_msg.time_increment = scan_msg.scan_time / LDS08_NUMBER_OF_SAMPLES_PER_SCAN;
    // Allocate range and intensity arrays
    scan_msg.ranges.data = (float*) malloc(LDS08_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    if (scan_msg.ranges.data == NULL) {
        DEBUG_PRINT("Fout bij toewijzen geheugen voor ranges array\n");
        // Handle memory allocation error appropriately
        scan_msg.ranges.size = 0;
        scan_msg.ranges.capacity = 0;
    }
    else { 
        scan_msg.ranges.size = LDS08_NUMBER_OF_SAMPLES_PER_SCAN;
        scan_msg.ranges.capacity = LDS08_NUMBER_OF_SAMPLES_PER_SCAN;
        // Initialize scan arrays
        for(int i = 0; i < LDS08_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
            scan_msg.ranges.data[i] = 0.0; // Default to 1 meter(dummy value)
        }
    }

    scan_msg.intensities.data = (float*) malloc(LDS08_NUMBER_OF_SAMPLES_PER_SCAN * sizeof(float));
    if (scan_msg.intensities.data == NULL) {
        DEBUG_PRINT("Fout bij toewijzen geheugen voor intensities array\n");
        // Handle memory allocation error appropriately
        scan_msg.intensities.size = 0;
        scan_msg.intensities.capacity = 0;
    } else { 
        scan_msg.intensities.size = LDS08_NUMBER_OF_SAMPLES_PER_SCAN;
        scan_msg.intensities.capacity = LDS08_NUMBER_OF_SAMPLES_PER_SCAN;
        // Initialize scan arrays
        for(int i = 0; i < LDS08_NUMBER_OF_SAMPLES_PER_SCAN; i++) {
            scan_msg.intensities.data[i] = 0.0;
        }
    }


    // Create FreeRTOS task for scanning
    const uint16_t stackSize = 4096; // Stack size in bytes
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
void lds08_lidar::scanTaskFunction(void* parameter) {
    lds08_lidar* lidar = static_cast<lds08_lidar*>(parameter);
    sensor_msgs__msg__LaserScan& scan_msg = lidar->scan_msg;   
     
    lds08_lidarMeasurement measurement;
    lidar->scan_start_time = millis();

    while (true) {
        taskYIELD();
        //vTaskDelay(1/ portTICK_PERIOD_MS);
        if (lidar->scan_enable == false) {
            continue;
        }
        if (lidar->getScanValue(&measurement, 100)) {
            if(scan_msg.ranges.data == NULL || scan_msg.intensities.data == NULL || lidar->distance == NULL) {
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

            for(int i = 0; i < POINT_PER_PACK; i++) {

                float angle_rad = measurement.point[i].angle * (pi / 180.0f);
                
                // Normalize angle to range [-π, π]
                while (angle_rad > pi) angle_rad -= 2.0f * pi;
                while (angle_rad < -pi) angle_rad += 2.0f * pi;
                
                // Convert angle from [-π, π] to array index [0, LDS08_NUMBER_OF_SAMPLES_PER_SCAN-1]
                // Formula: map [-π, π] to [0, N-1]
                int index = (int)(((angle_rad + pi) / (2.0f * pi)) * LDS08_NUMBER_OF_SAMPLES_PER_SCAN);
                
                // Clamp index to valid range
                if (index < 0) index = 0;
                if (index >= LDS08_NUMBER_OF_SAMPLES_PER_SCAN) 
                    index = LDS08_NUMBER_OF_SAMPLES_PER_SCAN - 1;
                
                // Update scan data
                lidar->scan_msg.ranges.data[index] = measurement.point[i].distance / 1000.0f; // mm to meters
                lidar->scan_msg.intensities.data[index] = (float)measurement.point[i].quality;
                //angle[index] = angle_rad;
            }
            float angle_increment = ANGLE_TO_RADIAN(measurement.speed / 2300);
            lidar->scan_msg.angle_increment = angle_increment;
            // mSpeed = rotation_degree / sec
            lidar->scan_msg.time_increment = (360 / measurement.speed) / ((lidar->scan_msg.angle_max - lidar->scan_msg.angle_min)/ angle_increment);
            lidar->scan_msg.scan_time = 360 / measurement.speed;
        } else {
            DEBUG_PRINT("Geen scanwaarde ontvangen binnen timeout\n");
        }
    }
}
#endif

// Set motor speed via PWM [%]
void lds08_lidar::setupMotorPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  analogWrite(this->motor_pin , map(percent, 0, 100, 0, 255));
  DEBUG_PRINT("Motor PWM ingesteld op %d%%\n", percent);
}


// Start scanning in standard or express mode
void lds08_lidar::startScan(uint32_t lidar_speed, uint32_t timeout_ms) {


    delay(1000);

    LIDARSerial->flush(); // Clear any old data

    scan_enable = true; // Enable scanning in task
    delay(100); // Allow some time to start
    setupMotorPWM(lidar_speed);

}

// Stop scanning
void lds08_lidar::stopScan() {
   
    
    scan_enable = false; // Disable scanning in task
    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for task to settle
    LIDARSerial->flush(); // Clear any old data
    setupMotorPWM(0); // Stop motor
    delay(1000); // Allow time to stop


    DEBUG_PRINT("Scan gestopt\n");
}




#ifndef TESTING
// Publish LaserScan message (placeholder implementation)
rcl_ret_t lds08_lidar::publish() {

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
bool lds08_lidar::getScanValue(lds08_lidarMeasurement* value, uint32_t timeout_ms) {
    uint32_t startTime = millis();
    LiDARFrameTypeDef frame;

    // Wacht tot minstens 5 bytes beschikbaar zijn
    while (LIDARSerial->available() < 1) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen scan data ontvangen\n");
            return false;
        }
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }

    // Read header bytes
    if (LIDARSerial->readBytes((uint8_t*)&frame, 1) != 1) {
        DEBUG_PRINT("Fout bij lezen scan data\n");
        return false;
    }

    while (LIDARSerial->available() < 1) {
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen scan data ontvangen\n");
            return false;
        }
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }

    if(frame.header != PKG_HEADER) {
        // Invalid start byte, discard and continue
        DEBUG_PRINT("Ongeldige start byte: 0x%02X\n", frame.header);
        return false;
    }

    // Read length byte
    if (LIDARSerial->readBytes((uint8_t*)&frame + 1, 1) != 1) {
        DEBUG_PRINT("Fout bij lezen scan data\n");
        return false;
    }

    if(frame.ver_len != PKG_VER_LEN) {
        // Invalid version/length byte, discard and continue
        DEBUG_PRINT("Ongeldige lengte byte: 0x%02X, expected 0x2C\n", frame.ver_len);
        return false;
    }   

    int size_of_frame = sizeof(LiDARFrameTypeDef) - 2; // Exclude header bytes already read
    while (LIDARSerial->available() < size_of_frame){
        if (millis() - startTime > timeout_ms){
            DEBUG_PRINT("Timeout: geen scan data ontvangen\n");
            return false;
        }
        vTaskDelay(1/ portTICK_PERIOD_MS);
    }

    if(LIDARSerial->readBytes(((uint8_t*)&frame) + 2, size_of_frame) != size_of_frame) {
        DEBUG_PRINT("Fout bij lezen volledige scan frame\n");
        return false;
    }

    uint8_t crc = 0;
    for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef) - 1; i++) {
      crc = CrcTable[(crc ^ ((uint8_t*)&frame)[i]) & 0xff];
    }

    if (crc != frame.crc8){
        DEBUG_PRINT("CRC fout: berekend 0x%02X, ontvangen 0x%02X\n", crc, frame.crc8);
        return false;
    }
    // Parse measurement from frame
    // Assuming POINT_PER_PACK is 1 for simplicity
    uint32_t diff =
        ((uint32_t)frame.end_angle + 36000 - (uint32_t)frame.start_angle) % 36000;
    float step = diff / (POINT_PER_PACK - 1) / 100.0;
    float start = static_cast<double>(frame.start_angle) / 100.0;
    float end = static_cast<double>(frame.end_angle % 36000) / 100.0;
    value->speed = frame.speed;
    value->start_angle = start;
    value->end_angle = end;
    for (int i = 0; i < POINT_PER_PACK; i++) {
        value->point[i].distance = frame.point[i].distance;
        value->point[i].angle = start + i * step;
        if (value->point[i].angle >= 360.0) {
            value->point[i].angle -= 360.0;
        }
        value->point[i].quality = frame.point[i].confidence;
    }
    return true;
}

lds08_lidar::~lds08_lidar() {}
