# Inleiding

In deze documentatie wordt beschreven hoe een Pioneer 3DX robot, een ontwerp uit 2007, kan worden omgebouwd tot een robot die geschikt is om te besturen met het ROS2 jazzy systeem.

De robot is te besturen met een z.g.n. cmd_vel topic (twist.geometry_msgs.msg). Op robot zijn de volgende sensoren aanwezig:
* Lidar, topic /scan
* Imu, topic /imu
* Odometer, topics /odom/unfiltered & /odom/filtered
* Bumpers, topics /p3dx/status

De robot kan worden geset met het /p3dx/reset (bool) topic
