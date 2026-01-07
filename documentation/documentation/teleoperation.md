# P3dx besturen

## Toetsenbord Teleop
Voer [teleop_twist_keyboard](https://index.ros.org/r/teleop_twist_keyboard/) uit om de robot te besturen met je toetsenbord:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Druk op:
- **i** - Om de robot vooruit te laten rijden.
- **,** - Om de robot achteruit te laten rijden.
- **j** - Om de robot CCW (tegen de klok in) te draaien.
- **l** - Om de robot CW (met de klok mee) te draaien.
- **shift + j** - Om de robot naar links te laten schuiven (voor mecanum robots).
- **shift + l** - Om de robot naar rechts te laten schuiven (voor mecanum robots).
- **u / o / m / .** - Gebruikt voor het draaien van de robot, door lineaire snelheid x en hoeksnelheid z te combineren.

## Joystick
Voer onderstaand commando uit om de robot te besturen met je joystick:

```bash
ros2 launch p3dx_bringup joy_teleop.launch.py
```

- Op de F710 Gamepad moet de bovenste schakelaar op 'X' staan en de 'MODE' LED moet uit zijn.

Druk op Knop/Beweeg Joystick:
- **RB (Eerste bovenste rechter knop)** - Houd deze knop ingedrukt terwijl je de joysticks beweegt om de besturing in te schakelen.
- **Linker Joystick Omhoog/Omlaag** - Om de robot vooruit/achteruit te laten rijden.
- **Linker Joystick Links/Rechts** - Om de robot naar links/rechts te laten schuiven.
- **Rechter Joystick Links/Rechts** - Om de robot CW/CCW te draaien.
