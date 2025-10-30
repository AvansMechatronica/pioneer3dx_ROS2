#!/bin/bash
set -e
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -y

mkdir uros_ws && cd uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo "source ~/uros_ws/install/local_setup.bash" >> $HOME/.bashrc

source install/local_setup.bash
