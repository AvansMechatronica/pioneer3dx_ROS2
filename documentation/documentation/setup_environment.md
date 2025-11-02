# Setup environment

Under Construction

### Ubuntu

The system is developed under ___Ubuntu 22 linux___. You can chose between two setups:
* **Desktop/Laptop setup**
    * **Developer**: 
        * Use this if you want to configure en programm ESP32 devices
        * Acts also as server for the microROS agent and the grapical use interface
    * [Install Ubuntu Desktop](https://ubuntu.com/download/desktop)
    * You can als use a virtual environment(for example BusyBox or VM-ware)
* **Raspberry PI setup**
    * **Server**: 
        * Acts as an server for the microROS agant and the graphical user interface
        * It is __not possible__ to programm ESP32 devices
        * Hybride system with Desktop/Laptop server for programming ESP32 devices
    * [Install Ubuntu server](https://ubuntu.com/download/raspberry-pi)

Note: Be sure to update Ubuntu after installing
```bash
cd ~
sudo apt update
sudo apt upgrade
```

### ROS
The system uses ___ROS2 Humble___ as a middelware for controlling locomotives en Turnouts. You can install using the following options:
* Using script: run __install_ros2_humble_desktop.sh__ script in scripts directory of this repository.
```bash
cd ~/railtrack_ws/src/railtrack/scripts
./install_ros2_humble_desktop.sh
```
* Manually: [Install ROS-Humbe](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)


### Colcon
The system uses the Colcon build system for compiling the ROS2 Humble workspace.
You can install using the following options:
* Using script:run __install_colcon.sh__ script in scripts directory of this repository.
```bash
cd ~/railtrack_ws/src/railtrack/scripts
./install_colcon.sh
```
* Manually: [Install Colcon](https://colcon.readthedocs.io/en/released/user/installation.html)

### Add ROS2 environment
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
Note: Execute this command only once. If multiple lines exists in "~/.bashrc" remove all other lines.

### Create ROS2 railtrack workspace
Use the folowing commands to create the railtrack workspace
```bash
cd ~
mkdir railtrack_ws/src -p
cd railtrack_ws/src
git clone https://github.com/GerardHarkema/railtrack.git
```
### Build ROS2 railtrack workspace
```bash
cd ~/railtrack_ws
colcon build --symlink-install
```
Add the workspace to the ROS environment:
```bash
echo "source ~/railtrack_ws/src/install/setup.bash" >> ~/.bashrc
```
Note: Execute this command only once. If multiple lines exists in "~/.bashrc" remove all other lines

### Visual Code & PlatformIO
Visual Code acts as an system for modifying,  editing files in the railtrack system. The plugin from PlatformIO makes it possible to program ESP32 devices for controlling turnouts and the Marklin railtrack to CAN bridge.

Following instructions for [Installing Visual Studio Code on Linux](https://code.visualstudio.com/docs/setup/linux)

Following instruction for [Add platfomIO plugin to VisualCode](https://platformio.org/install/ide?install=vscode)

### Python Libraries
Use follwing script to install missing Python libraries
```bash
cd ~/railtrack_ws/src/railtrack/scripts
./install_required_packages.sh
```


Next thing to do is creating the microROS-agent. See: [Install microROS agent](setup_microROS_agent.md)


