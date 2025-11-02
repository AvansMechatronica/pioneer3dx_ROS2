# Commands
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


