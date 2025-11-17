// =============================================================
// odometry.cpp 
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: RPLIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "odometry.h"

# define COVARIANCE_SIZE 36
// Constructor initializes odometry state and sets up the ROS2 publisher
Odometry::Odometry(const rcl_node_t *node):
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0)
{
    // Create the odometry publisher (micro-ROS)
    rclc_publisher_init_default(
        &odom_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered");

    // Set frame identifiers
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "p3dx_base");
    for(int i = 0; i < COVARIANCE_SIZE; i++) {
        odom_msg.pose.covariance.data[i] = 0.0; // Default to 1 meter(dummy value)
    }

    // Initialize pose to zero
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0; // Robot stays on ground

    // Orientation (quaternion)
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;

    // Twist (velocities)
    odom_msg.twist.twist.linear.x = 0.0; 
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Twist covariance
    odom_msg.twist.covariance[0] = 0.0001;
    odom_msg.twist.covariance[7] = 0.0001;
    odom_msg.twist.covariance[35] = 0.0001;


#if 0
    odom_msg.pose.covariance.data = (float*) malloc(COVARIANCE_SIZE * sizeof(float));
    if (odom_msg.pose.covariance.data == NULL) {
        // Handle memory allocation error appropriately
        odom_msg.pose.covariance.size = 0;
        odom_msg.pose.covariance.capacity = 0;
    }
    else {          
         odom_msg.pose.covariance.size = COVARIANCE_SIZE;
        odom_msg.pose.covariance.capacity = COVARIANCE_SIZE;
        // Initialize scan arrays
    }
#endif

}

// =============================================================
// Odometry update
// Computes position and orientation based on velocity inputs
// =============================================================
void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    // Change in heading (yaw) in radians
    float delta_heading = angular_vel_z * vel_dt;

    // Cache cos/sin of current heading
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);

    // Compute robot-relative motion transformed to world frame
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt;
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt;

    // Integrate position
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // Convert Euler heading to quaternion
    float q[4];
    euler_to_quat(0, 0, heading_, q);

    // Fill pose fields
    odom_msg.pose.pose.position.x = x_pos_;
    odom_msg.pose.pose.position.y = y_pos_;
    odom_msg.pose.pose.position.z = 0.0; // Robot stays on ground

    // Orientation (quaternion)
    odom_msg.pose.pose.orientation.x = (double) q[1];
    odom_msg.pose.pose.orientation.y = (double) q[2];
    odom_msg.pose.pose.orientation.z = (double) q[3];
    odom_msg.pose.pose.orientation.w = (double) q[0];

    // Pose covariance (small uncertainty)
//    if(odom_msg.pose.covariance.data != NULL) {
        odom_msg.pose.covariance[0] = 0.001;
        odom_msg.pose.covariance[7] = 0.001;
        odom_msg.pose.covariance[35] = 0.001;
//    }

    // Twist (velocities)
    odom_msg.twist.twist.linear.x = linear_vel_x;
    odom_msg.twist.twist.linear.y = linear_vel_y;
    odom_msg.twist.twist.linear.z = 0.0;

    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_vel_z;

    // Twist covariance
    odom_msg.twist.covariance[0] = 0.0001;
    odom_msg.twist.covariance[7] = 0.0001;
    odom_msg.twist.covariance[35] = 0.0001;
}

// =============================================================
// Convert Euler angles (roll, pitch, yaw) to quaternion order: [w, x, y, z]
// =============================================================
const void Odometry::euler_to_quat(float roll, float pitch, float yaw, float* q)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    // Standard quaternion conversion
    q[0] = cy * cp * cr + sy * sp * sr; // w
    q[1] = cy * cp * sr - sy * sp * cr; // x
    q[2] = sy * cp * sr + cy * sp * cr; // y
    q[3] = sy * cp * cr - cy * sp * sr; // z
}

// =============================================================
// Publish odometry message with current timestamp
// =============================================================
rcl_ret_t Odometry::publish() {
    // Timestamp in ROS2 format
    odom_msg.header.stamp.sec = millis() / 1000;
    odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    return rcl_publish(&odom_publisher, &odom_msg, NULL);
}
