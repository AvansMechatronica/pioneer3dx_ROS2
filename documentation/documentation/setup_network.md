# Setup network

Under Construction

Before configuring/compiling and downloading the turnout-controller or Marklin Railbox bridge in an ESP32 device you need to configure the network parameters.
Edit the micro_ros_agent_config.json file, according your network, in the config directory of this repository.


Note: The following command tells the ip-address of the system:
```bash
ifconfig | grep 'inet ' | awk '{print $2}'
```
