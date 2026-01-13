# Cartografie

## Start de robot
### Simulatie
Start 1 van de virtuele werelden die beschreven zijn in [simulatie](./simulation.md)
### Real World
Start de p3dx robot zoals beschreven in [echte robot](./real_robot_setup.md)


## Start [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox):

```bash
ros2 launch p3dx_navigation slam.launch.py
```

## Beweeg de robot om te starten met mappen

Bestuur de robot [handmatig](./teleoperation.md) totdat de robot het volledige werkgebied heeft afgedekt. Als alternatief kun je de `2D Goal Pose` tool in RVIZ gebruiken om een autonoom doel in te stellen tijdens het mappen. Meer informatie [hier](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html).

## Sla de kaart op

```bash
cd p3dx/p3dx_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <map_name> --ros-args -p save_map_timeout:=10000.
```

