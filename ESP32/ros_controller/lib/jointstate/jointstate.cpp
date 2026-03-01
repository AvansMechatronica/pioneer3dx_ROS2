// =============================================================
// jointstate.cpp
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: JointState publisher implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "jointstate.h"
#include <rmw_microros/time_sync.h>

// Constructor initializes the JointState publisher and message fields
Jointstate::Jointstate(const rcl_node_t *node){
    // Create a joint state publisher
    rclc_publisher_init_default(
        &joint_state_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states");

    // Initialize JointState message structure
    sensor_msgs__msg__JointState__init(&joint_state_msg);

    // Initialize header frame ID string
    rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
    joint_state_msg.header.frame_id = micro_ros_string_utilities_set(joint_state_msg.header.frame_id, "p3dx_base");

    // Initialize name array for exactly 3 joints
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 3);

    // Assign joint names
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2], "caster_swivel_hubcap_joint");

    // Allocate arrays for position, velocity, and effort
    joint_state_msg.position.size = 3;
    joint_state_msg.position.capacity = 3;
    joint_state_msg.position.data = (double*)malloc(3 * sizeof(double));

    joint_state_msg.velocity.size = 3;
    joint_state_msg.velocity.capacity = 3;
    joint_state_msg.velocity.data = (double*)malloc(3 * sizeof(double));

    joint_state_msg.effort.size = 3;
    joint_state_msg.effort.capacity = 3;
    joint_state_msg.effort.data = (double*)malloc(3 * sizeof(double));

    // Check allocation success
    if (!joint_state_msg.position.data || !joint_state_msg.velocity.data || !joint_state_msg.effort.data) {
        // Allocation failed — clean up any successfully allocated memory.
        if (joint_state_msg.position.data) {
            free(joint_state_msg.position.data);
            joint_state_msg.position.data = NULL;
            joint_state_msg.position.size = 0;
            joint_state_msg.position.capacity = 0;
        }
        if (joint_state_msg.velocity.data) {
            free(joint_state_msg.velocity.data);
            joint_state_msg.velocity.data = NULL;
            joint_state_msg.velocity.size = 0;
            joint_state_msg.velocity.capacity = 0;
        }
        if (joint_state_msg.effort.data) {
            free(joint_state_msg.effort.data);
            joint_state_msg.effort.data = NULL;
            joint_state_msg.effort.size = 0;
            joint_state_msg.effort.capacity = 0;
        }
        return;
    }

    // Initialize all fields to 0
    for (int i = 0; i < 3; i++) {
        joint_state_msg.position.data[i] = 0.0;
        joint_state_msg.velocity.data[i] = 0.0;
        joint_state_msg.effort.data[i] = 0.0;
    }

    // Initialize timestamp
    int64_t now_ms = rmw_uros_epoch_millis();
    if (now_ms <= 0) {
        now_ms = static_cast<int64_t>(millis());
    }
    joint_state_msg.header.stamp.sec = static_cast<int32_t>(now_ms / 1000);
    joint_state_msg.header.stamp.nanosec = static_cast<uint32_t>((now_ms % 1000) * 1000000ULL);
}

// Update joint positions (rad) and velocities (rad/s)
void Jointstate::update(double left_wheel_rad_s, double right_wheel_rad_s, double pos_left, double pos_right){
    // Position values; right wheel inverted for ROS coordinate conventions
    joint_state_msg.position.data[0] = pos_left;
    joint_state_msg.position.data[1] = -pos_right;

    // Wheel angular velocities in rad/s
    joint_state_msg.velocity.data[0] = left_wheel_rad_s;
    joint_state_msg.velocity.data[1] = -right_wheel_rad_s;
}

// Publish the joint state message
rcl_ret_t Jointstate::publish(){
    // Update timestamp
    int64_t now_ms = rmw_uros_epoch_millis();
    if (now_ms <= 0) {
        now_ms = static_cast<int64_t>(millis());
    }
    joint_state_msg.header.stamp.sec = static_cast<int32_t>(now_ms / 1000);
    joint_state_msg.header.stamp.nanosec = static_cast<uint32_t>((now_ms % 1000) * 1000000ULL);

    return rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}