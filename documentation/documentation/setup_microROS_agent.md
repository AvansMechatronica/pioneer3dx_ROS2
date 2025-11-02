# Install microROS Agent

Under Construction

The microROS Agent acts as a (WiFi)communication-channel between the ROS-Humble system and the embedded ESP32 devices controlling __turnouts__ or the __Marklin RailBox bridge__.


You can install using the following options:
* Using script:run __install_microros_agent.sh__ script in scripts directory of this repository.
```bash
cd ~/railtrack_ws/src/railtrack/scripts
./install_microros_agent.sh
```
* Manually: [Install microROS](https://micro.ros.org/docs/tutorials/core/first_application_linux/)
    * Follow only the instructions in the "Installing ROS 2 and the micro-ROS build system
 " and "Creating the micro-ROS agent" sections

### Add microROS to the environment
```bash
echo "source ~/microros_ws/install/setup.bash" >> ~/.bashrc
```
Note: Execute this command only once. If multiple lines exists in "~/.bashrc" remove all other lines.

### Start microROS Agent

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

The microROS Agent is the first program you always need to start. You can start it by the command above or alternative by:
* Using script: run __start_micro_ros_agent.sh__ script in scripts directory of this repository.
```bash
cd ~/railtrack_ws/src/railtrack/scripts
./start_micro_ros_agent.sh
```
* Install a Linux-service using the __install_railtrack_control_service.sh__ script in the linux_service directory of this repository (note: the micro-ros agent might not be running, abort with ctrl-C).
```bash
cd ~/railtrack_ws/src/railtrack/linux_service
./install_railtrack_control_service.sh
```

