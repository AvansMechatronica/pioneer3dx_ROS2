# Navigatie


## Bouwen navigatie package
Open `p3dx_navigation/launch/navigation.launch.py` en wijzig *MAP_NAME* naar de naam van de nieuw aangemaakte kaart. 

Bouw de p3dx_navigatie package(er zijn nieuwe bestanden toegevoegd):

```bash
cd ~/pioneer_ws/
colcon build --symlink-install --packages-select p3dx_navigation
source install/setup.bash
```

## Starten Navigatie
Voer het [Nav2](https://docs.nav2.org/tutorials/docs/navigation2_on_real_turtlebot3.html) pakket uit:

    ros2 launch p3dx_navigation navigation.launch.py

Optionele parameter voor het laden van kaarten:
- **map** - Pad naar nieuw aangemaakte kaart <kaartnaam.yaml>.


cd ~/pioneer_ws/
rosdep update
rosdep install --from-paths src/pioneer3dx_ROS2/p3dx --ignore-src -y
colcon build --symlink-install