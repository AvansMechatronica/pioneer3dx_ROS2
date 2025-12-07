// =============================================================
// imu_mpu6050.cpp 
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: MPU6050 IMU interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "imu_mpu6050.h"

// Debug macro configuration - enables serial output for debugging
#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("IMU DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

// Constructor: Initializes IMU sensor and sets up ROS publisher
// Parameters:
//   - node: ROS node handle
//   - scl_pin: I2C clock pin
//   - sda_pin: I2C data pin
//   - int_pin: Interrupt pin (currently unused)
#ifndef TESTING
imu_mpu6050::imu_mpu6050(const rcl_node_t *node, int scl_pin, int sda_pin, int int_pin) : dmpReady(false), mpu() {
#else
imu_mpu6050::imu_mpu6050(int scl_pin, int sda_pin, int int_pin) : dmpReady(false), mpu() {
#endif
    
    // Initialize I2C communication and MPU6050 sensor
    initialize(scl_pin, sda_pin);

#ifndef TESTING
    // Configure ROS publisher with default options
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    
    // Initialize publisher for IMU data on topic "imu/data"
    rcl_ret_t ret = rcl_publisher_init(
        &imu_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data",
        &publisher_options
    );
    if (ret != RCL_RET_OK) {
        // Handle error - publisher initialization failed
    }

    // Initialize and set the frame_id for the IMU message
    rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

    // Create dedicated FreeRTOS task for continuous IMU data reading
    const uint16_t stackSize = 4096; // Stack size in bytes
    const UBaseType_t priority = 1;   // Task priority
    BaseType_t result = xTaskCreate(
        imuTaskFunction,       // Task function pointer
        "IMU_Task",          // Human-readable task name
        stackSize,             // Stack size
        this,                  // Pass 'this' pointer as parameter
        priority,              // Priority level
        &imuTaskHandle        // Handle to store task reference
    );
    if (result != pdPASS) {
        DEBUG_PRINT("Fout bij aanmaken scan taak\n"); // Error creating scan task
        // Handle task creation error appropriately
    }   
#endif

}

// Destructor: Cleans up ROS resources
imu_mpu6050::~imu_mpu6050() {
#ifndef TESTING
    // Clean up publisher and release resources
    rcl_publisher_fini(&imu_pub, nullptr);  
#endif
}

#ifndef TESTING
// Static task function for FreeRTOS - continuously updates IMU data
// This runs in a separate thread on the ESP32
void imu_mpu6050::imuTaskFunction(void* parameter) {
    // Cast void pointer back to imu_mpu6050 object
    imu_mpu6050* imu = static_cast<imu_mpu6050*>(parameter);
    while (true) {
        taskYIELD(); // Yield to allow other tasks to run
        //vTaskDelay(1/ portTICK_PERIOD_MS); // Optional delay
        imu->update(); // Update IMU readings
    }
}
#endif

// Initialize I2C communication and configure MPU6050 DMP (Digital Motion Processor)
void imu_mpu6050::initialize(int scl_pin, int sda_pin) {
    // Initialize I2C with custom pins
    Wire.begin(sda_pin, scl_pin);
    
    // Initialize MPU6050 sensor
    mpu.initialize();

    // Initialize DMP (Digital Motion Processor) for sensor fusion
    devStatus = mpu.dmpInitialize();

    // Check if DMP initialization was successful
    if (devStatus == 0) {
        mpu.setDMPEnabled(true); // Enable DMP
        dmpReady = true;          // Mark DMP as ready
        packetSize = mpu.dmpGetFIFOPacketSize(); // Get FIFO packet size
    }
}

// Test if MPU6050 is connected and responding
bool imu_mpu6050::testConnection() {
    return mpu.testConnection();
}

// Publish IMU data to ROS topic
// Returns: RCL_RET_OK on success, RCL_RET_ERROR on failure
#ifndef TESTING
rcl_ret_t imu_mpu6050::publish() {
    if (!dmpReady) return RCL_RET_ERROR; // DMP not initialized

    // Get number of bytes available in FIFO buffer
    fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow
    if (fifoCount >= 1024) {
        mpu.resetFIFO(); // Reset FIFO if overflow detected
        return RCL_RET_ERROR;
    } else if (fifoCount >= packetSize) {
        // Read data from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Extract quaternion orientation from FIFO data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        // Calculate gravity vector
        mpu.dmpGetGravity(&gravity, &q);
        // Calculate Yaw, Pitch, Roll angles
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Populate ROS message with orientation (quaternion)
        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.orientation.w = q.w;

        // Populate angular velocity (using YPR - note: this is not true angular velocity)
        imu_msg.angular_velocity.x = ypr[0];
        imu_msg.angular_velocity.y = ypr[1];
        imu_msg.angular_velocity.z = ypr[2];

        // Set timestamp from system milliseconds
        imu_msg.header.stamp.sec = millis() / 1000;
        imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

        // Publish message to ROS topic
        return rcl_publish(&imu_pub, &imu_msg, nullptr);
    }
    return RCL_RET_OK;
}
#endif

// Update internal IMU data (called by FreeRTOS task)
// Does not publish - only updates internal state
void imu_mpu6050::update(){
    if (!dmpReady) return; // DMP not initialized

    // Get number of bytes available in FIFO
    fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow
    if (fifoCount >= 1024) {
        mpu.resetFIFO(); // Reset FIFO if overflow detected
    } else if (fifoCount >= packetSize) {
        // Read data from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Extract and calculate orientation data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

// Get roll angle in radians
float imu_mpu6050::getRoll(){
    return ypr[2];
}

// Get pitch angle in radians
float imu_mpu6050::getPitch(){
    return ypr[1];
}

// Get yaw angle in radians
float imu_mpu6050::getYaw(){
    return ypr[0];
}       

VectorFloat imu_mpu6050::getGravity(){
    return gravity;
}