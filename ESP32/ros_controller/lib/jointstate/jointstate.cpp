// =============================================================
// jointstate.cpp
// =============================================================
// Author: Gerard Harkema
// Date: November 2025
// Description: RPLIDAR interface implementation for ESP32 with micro-ROS
// License: CC BY-NC-SA 4.0
// Note: Comments added for clarity and explanation.

#include "jointstate.h"

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
        // Allocation failed — ideally log or handle this safely
        return;
    }

    // Initialize all fields to 0
    for (int i = 0; i < 3; i++) {
        joint_state_msg.position.data[i] = 0.0;
        joint_state_msg.velocity.data[i] = 0.0;
        joint_state_msg.effort.data[i] = 0.0;
    }
}

// Update joint positions and velocities
void Jointstate::update(double current_rpm_l, double current_rpm_r, double pos_left, double pos_right){
    // Position values; right wheel inverted for ROS coordinate conventions
    joint_state_msg.position.data[0] = pos_left;
    joint_state_msg.position.data[1] = -pos_right;

    // Wheel RPMs (converted externally if needed)
    joint_state_msg.velocity.data[0] = current_rpm_l;
    joint_state_msg.velocity.data[1] = -current_rpm_r;
}

// Publish the joint state message
rcl_ret_t Jointstate::publish(){
    // Update timestamp
    joint_state_msg.header.stamp.sec = millis() / 1000;
    joint_state_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;

    return rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}