#include "imu_mpu6050.h"

IMU_MPU6050::IMU_MPU6050(const rcl_node_t *node) : dmpReady(false), mpu() {
    
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    rcl_ret_t ret = rcl_publisher_init(
        &imu_pub,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data",
        &publisher_options
    );
    if (ret != RCL_RET_OK) {
        // Handle error
    }

    rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");

}

IMU_MPU6050::~IMU_MPU6050() {
#ifndef TESTING
    // Clean up publisher
    rcl_publisher_fini(&imu_pub, nullptr);  
#endif
}

void IMU_MPU6050::initialize() {
    Wire.begin();
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}
bool IMU_MPU6050::testConnection() {
    return mpu.testConnection();
}

rcl_ret_t IMU_MPU6050::publish() {
    if (!dmpReady) return RCL_RET_ERROR;

    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        return RCL_RET_ERROR;
    } else if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        imu_msg.orientation.x = q.x;
        imu_msg.orientation.y = q.y;
        imu_msg.orientation.z = q.z;
        imu_msg.orientation.w = q.w;

        imu_msg.angular_velocity.x = ypr[0];
        imu_msg.angular_velocity.y = ypr[1];
        imu_msg.angular_velocity.z = ypr[2];

        imu_msg.header.stamp.sec = millis() / 1000;
        imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

        return rcl_publish(&imu_pub, &imu_msg, nullptr);
    }
    return RCL_RET_OK;
}


void IMU_MPU6050::update(){
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= 1024) {
        mpu.resetFIFO();
    } else if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}
float IMU_MPU6050::getRoll(){
    return ypr[2];
}
float IMU_MPU6050::getPitch(){
    return ypr[1];
}
float IMU_MPU6050::getYaw(){
    return ypr[0];
}       