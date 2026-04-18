# Installatie

## P3dx workspace

### Clone repository

:::::{card} 

::::{tab-set}

:::{tab-item} Met GIT-repository support

* Maak een account aan bij [Github](https://github.com/) en login op dit account

* Open de [pioneer3dx_ROS2](https://github.com/AvansMechatronica/pioneer3dx_ROS2) repository

* Maak een Fork van de repository naar je eigen Github account door op het **Fork icoon**  te klikken:

![image](../images/fork.jpg)

* Volg de instructies, maar wijzig de naam van de nieuwe repository niet. Bevestig met **Create Fork**  

* Nu kun je de workspace als volgt creëren

```bash
mkdir -p ~/p3dx_ws/src
cd ~/p3dx_ws/src
git clone https://github.com/<jouw_account_naam>/pioneer3dx_ROS2.git
```

*ps. Het gebruik van github (zoals add, commit & push commando's) valt  buiten de scope van deze documentatie*

:::

:::{tab-item} Zonder GIT-repository support

* Je kunt de workspace als volgt creëren
```bash
mkdir -p ~/p3dx_ws/src
cd ~/p3dx_ws/src
git clone https://github.com/AvansMechatronica/pioneer3dx_ROS2.git
```

:::

::::

:::::





### Build workspace
```bash
cd ~/p3dx_ws/
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


### Test setup
```bash
ros2 launch p3dx_gazebo p3dx_mazeworld.launch.py 
```