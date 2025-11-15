#ifndef JOINTSTATE_H
#define JOINTSTATE_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // ROS string helpers
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>

#include <sensor_msgs/msg/joint_state.h>  // ROS2 JointState message type

// ------------------------------------------------------------
// Jointstate Class
// Responsible for publishing JointState messages for robot joints.
// Handles initialization of message fields and updates wheel
// positions/velocities before publishing.
// ------------------------------------------------------------
class Jointstate
{
    public:
        // Constructor: initializes publisher and message fields
        Jointstate(const rcl_node_t *node);

        // Updates joint positions & velocities before publishing
        void update(double current_rpm_l, double current_rpm_r,
                    double pos_left, double pos_right);

        // Publishes the JointState message to ROS2
        rcl_ret_t publish();

    private:
        // (Unused) quaternion conversion helper — should be removed if not needed
        const void euler_to_quat(float x, float y, float z, float* q);

        // Internal joint tracking variables (currently unused)
        float x_pos_;
        float y_pos_;
        float heading_;

        // Publisher object for /joint_states
        rcl_publisher_t joint_state_publisher;

        // JointState message instance stored in memory
        sensor_msgs__msg__JointState joint_state_msg;
};

#endif