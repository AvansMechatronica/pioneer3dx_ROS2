# P3DX Utils

Utilities for Pioneer 3DX robot in ROS2.

## Features

### Status Monitor GUI

A comprehensive graphical user interface for monitoring the Pioneer 3DX robot status in real-time.

#### Monitored Topics

The status monitor subscribes to the following ROS2 topics:

- `/odom` (nav_msgs/Odometry) - Robot position, orientation, and velocity
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands sent to the robot
- `/battery_state` (sensor_msgs/BatteryState) - Battery status information
- `/scan` (sensor_msgs/LaserScan) - Laser scanner data

#### GUI Tabs

1. **Odometry**: Displays robot position (x, y, z), orientation (quaternion), and current velocity
2. **Velocity Commands**: Shows the latest velocity commands (linear and angular)
3. **Battery**: Shows battery voltage, current, charge level, and health status
4. **Sensors**: Displays laser scanner statistics (min/max/avg ranges, number of readings)
5. **Log**: Message log for system events

## Installation

### Prerequisites

Make sure you have PyQt5 installed:

```bash
sudo apt-get update
sudo apt-get install python3-pyqt5
```

### Build the Package

From your ROS2 workspace:

```bash
cd ~/pioneer_ws
colcon build --packages-select p3dx_utils
source install/setup.bash
```

## Usage

### Running the Status Monitor

After building and sourcing the workspace, launch the status monitor GUI:

```bash
ros2 run p3dx_utils p3dx_status_monitor
```

The GUI will open and automatically start monitoring the configured topics. Make sure your robot or simulator is running and publishing to these topics.

### Customizing Topics

If your robot uses different topic names, you can modify them in the `status_monitor_gui.py` file. Look for the subscription creation in the `ROS2Monitor.__init__()` method:

```python
# Change topic names here
self.odom_sub = self.create_subscription(
    Odometry,
    '/odom',  # Change this to your topic name
    self.odom_callback,
    qos_profile
)
```

## Features Overview

- **Real-time Updates**: GUI updates at 20 Hz showing live robot data
- **Color-coded Battery Status**: Battery percentage changes color based on charge level
  - Green: > 50%
  - Yellow: 20-50%
  - Red: < 20%
- **Comprehensive Data Display**: All major robot parameters in an organized tabbed interface
- **Event Logging**: Built-in log viewer for tracking status messages
- **Thread-safe**: Uses Qt signals for safe communication between ROS2 and GUI threads

## Development

To extend the monitor with additional sensors or topics:

1. Add the message type import at the top of `status_monitor_gui.py`
2. Create a new subscription in `ROS2Monitor.__init__()`
3. Add a callback method to process the data
4. Create a Qt signal to emit the data
5. Add UI elements in a new or existing tab
6. Connect the signal to update the UI

## Troubleshooting

### GUI doesn't start

- Ensure PyQt5 is installed: `python3 -c "import PyQt5; print('OK')"`
- Check that the workspace is properly sourced

### No data appearing in GUI

- Verify that the robot/simulator is running
- Check topic names with: `ros2 topic list`
- Verify topic types: `ros2 topic info /odom`
- Echo topics to confirm data is being published: `ros2 topic echo /odom --once`

### Best Effort QoS Warning

If you see warnings about QoS compatibility, the monitor uses BEST_EFFORT reliability by default. You can change this in the `qos_profile` settings if your topics use RELIABLE.

## License

TODO: Add license information
