# Setup turnout controller

Under Construction

With the turnout controller it is possible to control classical solenoid turnouts as well as servo, digital or analog controlled turnouts. The control takes place wireless over the wifi-netork. Configuring can be done by simple json configuration files. All needed is a ESP32-wroom32 device and a power switching element such as a relais(_don't forget to use a freewheeling diode_), MOS-FET or power amplifier. An example is shown in: [Example of implementation](./turnout_example.md).

An example of a configuration is given in the ___turnout_multi_config_test.json___ file in the config directory of this repository. It is also posible to use the turnout controller to switch different kind of accessories.

To program the ESP32 chose the ___ros2_turnout_controller___ project in the ___ESP32___ workspace of this repositoy. When downloading or compile the project a popup apperas for selecting the configuration file.

_Note: Every configuration in your system should have a unique node name, defined in the configuration file._

[ESP32 pinouts](./esp32_pinouts.md)

[Programming instructions](instructions_programming_esp32.md)(Note: Select project ___ros2_turnout_controller___)

[Example of implementation](./turnout_example.md)

