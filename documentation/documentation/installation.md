# Installatie

## P3dx workspace

### Clone repository
```bash
mkdir pioneer_ws/src -p
git clone https://github.com/AvansMechatronica/pioneer3dx_ROS2
```

### Buid workspace
```bash
cd ~/pioneer_ws/
rosdep update
rosdep install --from-paths src/pioneer3dx_ROS2/p3dx --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

### Add setup.bash to .bashrc
```bash
if ! grep -Fxq "source $(pwd)/install/setup.bash" ~/.bashrc; then
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
    echo "Added source command to .bashrc"
else
    echo "Source command already exists in .bashrc"
fi  
```

## microROS agent

De p3dx-robot maakt gebruik van microROS om te communiceren met de host-computer.

### Installatie microROS agent

```bash
mkdir -p ~/microROS_agent_ws/src
cd ~/microROS_agent_ws/src

# Verkrijg de juiste ROS2 distributie
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git

cd ..
# Build de microROS agent
colcon build --symlink-install
source install/setup.bash
echo "source ~/microROS_agent_ws/install/setup.bash" >> ~/.bashrc
```


### Add setup.bash to .bashrc
```bash
if ! grep -Fxq "source $(pwd)/install/setup.bash" ~/.bashrc; then
    echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
    echo "Added source command to .bashrc"
else
    echo "Source command already exists in .bashrc"
fi  
```
