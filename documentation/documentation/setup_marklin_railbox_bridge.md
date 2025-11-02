# Setup Marklin Railbox Bridge

Under Construction

With the Marklin Railbox Bridge it is possible to control(by the graphical user interface) turnouts and locomotives over the railtarcks in the following (rail)protocols:
* Marklin Motorola
* DCC
* mfx

Configuraion is done by the ___track_config.json___ file in the ___config___ directory of this repository. It is also posible to use the turnout controller to switch different kind of accessories.
_Note: The track config.json file is als used for the user interface. For the Bridge and the user interface they sould be the same._

![Image](images/MarklinRailboxBridge.jpg)
___Example of prototyping the railbox___

(New one wil follow soon)
(In the future there wil be a dedicated PCB developed)

[List of Components](https://docs.google.com/spreadsheets/d/18TnnurDtuM7WNLGjFARisbmxjI6zFRmcXOZzPikqvao/edit?usp=sharing)

[ESP32 pinouts](./esp32_pinouts.md)


[Connectiontable Railbox Bridge](https://docs.google.com/spreadsheets/d/1lZ0bYcd9MC5ZJBcrFeeLdLd2YKi_yTghrkXhHs3W9o0/edit?usp=sharing)

_Warning for the assambly/wireing_:
* _Before connecting the output of the step-down converter to the rest of the system, adjust the output voltage to 5 Volts_
* _Solder only in inner rows headers on the ESP32 device_




Connections to the Marklin Railbox can be made by:
* Soldering wires to the PCB of the Railbox, see image below
* Connection by a 10 pin plug(_I did not find a compliant plug on the internet_)

![Image](images/RailBoxConnection.jpg)
___Connections to the Digital Connector Box PCB___
[Reference](https://www.marklin-users.net/forum/posts/t36089-Computer-interface-for-Marklin-Track-Box-and-mfx-programming)


[Programming instructions](instructions_programming_esp32.md)(Note: Select project ___ros2_marklin_canbus_controller___)

