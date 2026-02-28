// =============================================================
// imu_mpu6050.cpp 
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: MPU6050 IMU interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "imu_mpu6050.h"

#ifndef TESTING
#include <rmw_microros/time_sync.h>
#endif

// Debug macro configuration - enables serial output for debugging

#ifdef DEBUG_IMU
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
imu_mpu6050::imu_mpu6050(const rcl_node_t *node, int scl_pin, int sda_pin, int int_pin){
#else
//imu_mpu6050::imu_mpu6050(int scl_pin, int sda_pin, int int_pin) : dmpReady(false), mpu() {
imu_mpu6050::imu_mpu6050(int scl_pin, int sda_pin, int int_pin){
#endif
    DEBUG_PRINT("Initialiseer IMU op SCL pin %d, SDA pin %d, INT pin %d\n", scl_pin, sda_pin, int_pin);
    mpu = new MPU6050();

    // Initialize I2C communication and MPU6050 sensor
    bool init_ok = initialize(scl_pin, sda_pin);

#ifndef TESTING

    // Initialize publisher for IMU data on topic "imu/data"
    rcl_ret_t ret = rclc_publisher_init_default(
        &imu_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    );  

    if (ret != RCL_RET_OK) {
        DEBUG_PRINT("Fout bij initialiseren IMU publisher: %d\n", ret); // Error initializing IMU publisher
        // Handle error - publisher initialization failed
    }

    if (!sensor_msgs__msg__Imu__init(&imu_msg)) {
        DEBUG_PRINT("Fout bij initialiseren IMU message\n");
    }

    // Initialize and set the frame_id for the IMU message
    rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

    // Covariances (row-major). Non-zero values indicate known/estimated uncertainty.
    // Start with all zeros, then set diagonal variances.
    for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
        imu_msg.angular_velocity_covariance[i] = 0.0;
        imu_msg.linear_acceleration_covariance[i] = 0.0;
    }
    imu_msg.orientation_covariance[0] = 0.02;
    imu_msg.orientation_covariance[4] = 0.02;
    imu_msg.orientation_covariance[8] = 0.02;

    imu_msg.angular_velocity_covariance[0] = 0.03;
    imu_msg.angular_velocity_covariance[4] = 0.03;
    imu_msg.angular_velocity_covariance[8] = 0.03;

    imu_msg.linear_acceleration_covariance[0] = 0.10;
    imu_msg.linear_acceleration_covariance[4] = 0.10;
    imu_msg.linear_acceleration_covariance[8] = 0.10;


#endif
    // Create dedicated FreeRTOS task for continuous IMU data reading
    const uint16_t stackSize = 4096 * 2; // Stack size in bytes
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
}

// Destructor: Cleans up ROS resources
imu_mpu6050::~imu_mpu6050() {
#ifndef TESTING
    // Clean up publisher and release resources
    sensor_msgs__msg__Imu__fini(&imu_msg);
    rcl_publisher_fini(&imu_pub, nullptr);  
#endif
}

// Static task function for FreeRTOS - continuously updates IMU data
// This runs in a separate thread on the ESP32
void imu_mpu6050::imuTaskFunction(void* parameter) {
    // Cast void pointer back to imu_mpu6050 object
    imu_mpu6050* imu = static_cast<imu_mpu6050*>(parameter);
    while (true) {
#ifdef TESTING
        vTaskDelay(500/ portTICK_PERIOD_MS); 
#else
        vTaskDelay(100 / portTICK_PERIOD_MS); // Prevent I2C hammering
#endif

        imu->update(); // Update IMU readings
    }
}

// Initialize I2C communication and configure MPU6050 DMP (Digital Motion Processor)
bool imu_mpu6050::initialize(int scl_pin, int sda_pin, int speed) {
    // Initialize I2C with custom pins and speed
    Wire.begin(sda_pin, scl_pin, speed);
    
    // Initialize MPU6050 sensor
    mpu->initialize();

    if(mpu->testConnection() == false){
        DEBUG_PRINT("MPU6050 connection failed");
        return false;
    }
    else {
        DEBUG_PRINT("MPU6050 connection successful");
    }

    // Initialize DMP (Digital Motion Processor) for sensor fusion
    devStatus = mpu->dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu->setXGyroOffset(0);
    mpu->setYGyroOffset(0);
    mpu->setZGyroOffset(0);
    mpu->setXAccelOffset(0);
    mpu->setYAccelOffset(0);
    mpu->setZAccelOffset(0);


    /* Making sure it worked (returns 0 if so) */ 
    if (devStatus == 0) {
        mpu->CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
        mpu->CalibrateGyro(6);

        DEBUG_PRINT("These are the Active offsets: ");
        mpu->PrintActiveOffsets();
        DEBUG_PRINT("Enabling DMP...");   //Turning ON DMP

        mpu->setDMPEnabled(true); 
        mpuIntStatus = mpu->getIntStatus();
        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */

        DEBUG_PRINT("DMP ready! Waiting for first interrupt...");
        dmpReady = true;
        packetSize = mpu->dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    } else {
        DEBUG_PRINT("DMP Initialization failed");
        return false;
    }
    return true;
}

// Test if MPU6050 is connected and responding
bool imu_mpu6050::testConnection() {
    return false;//mpu->testConnection();
}

// Publish IMU data to ROS topic
// Returns: RCL_RET_OK on success, RCL_RET_ERROR on failure
#ifndef TESTING
rcl_ret_t imu_mpu6050::publish() {

    // Read latest DMP packet before publishing
    //update();


    // Populate ROS message with linear acceleration in sensor frame (m/s^2)
    imu_msg.linear_acceleration.x = aa.x * mpu->get_acce_resolution() * EARTH_GRAVITY_MS2;
    imu_msg.linear_acceleration.y = aa.y * mpu->get_acce_resolution() * EARTH_GRAVITY_MS2;
    imu_msg.linear_acceleration.z = aa.z * mpu->get_acce_resolution() * EARTH_GRAVITY_MS2;

    // Populate ROS message with orientation (quaternion)
    imu_msg.orientation.x = q.x;
    imu_msg.orientation.y = q.y;
    imu_msg.orientation.z = q.z;
    imu_msg.orientation.w = q.w;

    // Populate angular velocity in rad/s (sensor frame)
    imu_msg.angular_velocity.x = gg.x * mpu->get_gyro_resolution() * DEG_TO_RAD;
    imu_msg.angular_velocity.y = gg.y * mpu->get_gyro_resolution() * DEG_TO_RAD;
    imu_msg.angular_velocity.z = gg.z * mpu->get_gyro_resolution() * DEG_TO_RAD;

    // Set timestamp from synchronized micro-ROS epoch time
    int64_t now_ms = rmw_uros_epoch_millis();
    if (now_ms <= 0) {
        now_ms = static_cast<int64_t>(millis());
    }
    imu_msg.header.stamp.sec = static_cast<int32_t>(now_ms / 1000);
    imu_msg.header.stamp.nanosec = static_cast<uint32_t>((now_ms % 1000) * 1000000ULL);

    // Publish message to ROS topic
    return rcl_publish(&imu_pub, &imu_msg, nullptr);

}
#endif

// Update internal IMU data (called by FreeRTOS task)
// Does not publish - only updates internal state
void imu_mpu6050::update(){

    if (!dmpReady){
        DEBUG_PRINT("DMP not ready, cannot update IMU data\n");
        return;
    }   
    /* Read a packet from FIFO */
    if (mpu->dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        /*Display quaternion values in easy matrix form: w x y z */
        mpu->dmpGetQuaternion(&q, fifoBuffer);
#if defined(DEBUG_IMU_AQUISITION)
        DEBUG_PRINT("quat: %f, %f, %f, %f\n", q.w, q.x, q.y, q.z);
#endif
        mpu->dmpGetGravity(&gravity, &q);

        /* Display initial world-frame acceleration, adjusted to remove gravity
        and rotated based on known orientation from Quaternion */
        mpu->dmpGetAccel(&aa, fifoBuffer);
        mpu->dmpConvertToWorldFrame(&aaWorld, &aa, &q);
#if defined(DEBUG_IMU_AQUISITION)
        DEBUG_PRINT("aworld: %f, %f, %f\n", aaWorld.x * mpu->get_acce_resolution() * EARTH_GRAVITY_MS2, aaWorld.y * mpu->get_acce_resolution() * EARTH_GRAVITY_MS2, aaWorld.z * mpu->get_acce_resolution() * EARTH_GRAVITY_MS2);
#endif
        /* Display gyro data in world frame (rotated based on known orientation from Quaternion) */
        mpu->dmpGetGyro(&gg, fifoBuffer);
        mpu->dmpConvertToWorldFrame(&ggWorld, &gg, &q);
#if defined(DEBUG_IMU_AQUISITION)
        DEBUG_PRINT("ggWorld: %f, %f, %f\n", ggWorld.x * mpu->get_gyro_resolution() * DEG_TO_RAD, ggWorld.y * mpu->get_gyro_resolution() * DEG_TO_RAD, ggWorld.z * mpu->get_gyro_resolution() * DEG_TO_RAD);
#endif
        /* Display Euler angles in degrees */
        mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
#if defined(DEBUG_IMU_AQUISITION)
        DEBUG_PRINT("ypr: %f, %f, %f\n\n", ypr[0] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[2] * RAD_TO_DEG);
#endif
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