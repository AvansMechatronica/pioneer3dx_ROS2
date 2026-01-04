# Inleiding

![p3dx](../images/p3dx_robot.jpeg)

Originele Pioneer P3DX robot

In deze documentatie wordt beschreven hoe een Pioneer 3DX robot (aangeduid als P3DX-robot), een ontwerp uit 2007, kan worden omgebouwd tot een robot die geschikt is om te besturen met het ROS2 jazzy systeem.
De basis is hiervoor een nieuw processorbord gebaseerd op een ESP32-S3-N16R8 processor 

De P3DX-robot heeft de volgende eigenschappen:
* Bestaand processorboard is vervangen door nieuw processorboard gebaseerd op een ESP32-S3-N16R8 processor
* 1.77" status display
* Beperkte functionaliteit van het bestaande controle paneel.
* De P3DX-robot communiceert met host-computer d.m.v. een MicroROS agent gebruik makend van 1 de volgende twee MicroROS protocollen
    * Serial
    * Wifi  
* Besturen met een z.g.n. cmd_vel topic (twist.geometry_msgs.msg)
* Op de robot zijn de volgende sensoren aanwezig:
    * Lidar, topic /scan
    * Imu, topic /imu
    * Odometer, topics /odom/unfiltered & /odom/filtered
    * Bumpers, topics /p3dx/status
* De robot kan worden gereset met het /p3dx/reset (bool) topic.
* Een grafische user interface is voor raadplegen status en resetten van de p3dx-robot 

![Pioneer 3DX mobile robot with red chassis and black top deck displaying P3DX branding, equipped with mounted LIDAR sensor on top platform and 1.77 inch status display on front panel, featuring yellow-rimmed wheels with rugged tires, positioned on light wooden floor against wooden furniture background](../images/p3dxWithLidarAndDisplay.jpg)

Gemodificeerde P3DX-robot met lidar en 1.77" status display.


![Pioneer 3DX robot showing mounted LIDAR sensor on top and 1.77 inch status display mounted on front panel, both integrated into the modified chassis for autonomous navigation capabilities](../images/lidaAndDisplay.jpg)

Detail van lidar en status display

![Green PCB circuit board labeled avans hogeschool showing Pioneer 3DX Mobile Robot Controller with ESP32-S3-WROOM microcontroller module, multiple rainbow-colored ribbon cables, blue sensor modules, capacitors, and various electronic components. Board designed by Gerard Harkema for Alix/Mechatronics Department in 2025](../images/ESP32_board.jpg)

Detail van het processorboard.

In deze documentatie worden de benodigde stappen, configuraties en voorbeeldcommando's beschreven om de robot in gebruik te nemen met ROS2 Jazzy.

Tevens wordt beschreven hoe een bestaande robot kan worden omgebouwd. Alle tekeningen en bijbehorende documentatie zijn beschikbaar in de github repository.
