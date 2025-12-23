#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // String utilities for ROS 2 messages
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

#include <nav_msgs/msg/odometry.h>

// Odometry class computes robot position and orientation
// based on wheel-derived velocity inputs and publishes a
// nav_msgs/Odometry message over micro-ROS.
class Odometry
{
    public:
        // Constructor: initializes publisher and message frame IDs
        Odometry(const rcl_node_t *node);

        // Update internal odometry state and fill odometry message
        // vel_dt        : time delta (s)
        // linear_vel_x  : forward velocity (m/s)
        // linear_vel_y  : lateral velocity (m/s) — usually 0 for differential drive
        // angular_vel_z : angular velocity around Z (rad/s)
        void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);

        // Publish odometry message over micro-ROS
        rcl_ret_t publish();

    private:
        // Convert Euler angles (roll=x, pitch=y, yaw=z) to quaternion
        // q must be an array of 4 floats {w, x, y, z}
        void euler_to_quat(float x, float y, float z, float* q);

        // micro-ROS publisher handle
        rcl_publisher_t odom_publisher;

        // Odometry message instance
        nav_msgs__msg__Odometry odom_msg;

        // Internal estimated robot position (meters)
        float x_pos_;
        float y_pos_;

        // Robot heading (yaw angle in radians)
        float heading_;
};

#endif