# Commands

## Under construction

Clone repository
```
mkdir pioneer_ws/src -p
git clone .....
```

Buid workspace
```
cd ~/pioneer_ws/
rosdep update
rosdep install --from-paths src/pioneer3dx_ROS2/p3dx --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Start simualtor
```
ros2 launch p3dx_gazebo p3dx_mazeworld.launch.py 
```


On remote computer
```
ros2 launch p3dx_navigation slam.launch.py
```

```
ros2 launch p3dx_navigation slam.launch.py sync:=false
```

```
ros2 launch p3dx_navigation localization.launch.py map:=/home/gerard/pioneer_ws/src/pioneer3dx_ROS2/p3dx_navigation/maps/maze.yaml
```


Hardware
USB:

WiFi:
Setup EPS32 lite filesystem see:
[setup filesystem](https://randomnerdtutorials.com/esp32-vs-code-platformio-littlefs/)

Bringup microros:
Wifi
```
ros2 launch p3dx_bringup wifi.launch.py
```
of
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

USB
```
ros2 launch p3dx_bringup usb.launch.py
```
of
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```


Install microros Agent
```
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

rosdep update && rosdep install --from-paths src --ignore-src -y

colcon build

source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo "source ~/uros_ws/install/local_setup.bash" >> $HOME/.bashrc

source install/local_setup.bash
```