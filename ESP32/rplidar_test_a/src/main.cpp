#include <Arduino.h>

// Aansluitingen ESP32 ↔ RPLIDAR A1M8
// RPLIDAR TX → ESP32 RX2 (GPIO16)
// RPLIDAR RX → ESP32 TX2 (GPIO17)
// GND → GND, VCC → 5V
// Motor-pin via transistor of MOSFET naar 5V (optioneel te schakelen)

#define RPLIDAR_MOTOR 25   // optioneel: PWM of GPIO voor motor aan/uit

HardwareSerial LIDARSerial(2); // Gebruik UART2

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("RPLIDAR A1M8 uitlezen gestart...");

  // Start hardware-serial voor de LIDAR
  LIDARSerial.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(RPLIDAR_MOTOR, OUTPUT);
  digitalWrite(RPLIDAR_MOTOR, HIGH); // motor aan
  delay(1000);

  Serial.println("Verbinding maken met RPLIDAR...");

  // Start commando naar RPLIDAR sturen
  byte startScan[2] = {0xA5, 0x20};
  LIDARSerial.write(startScan, 2);
}

void loop() {
  static uint8_t buffer[5];
  static int index = 0;

  // Wacht tot er data is
  if (LIDARSerial.available()) {
    uint8_t b = LIDARSerial.read();

    // Synchronisatie zoeken: elk scanpunt start met bit7 = 1, bit6 = 0
    if (index == 0) {
      if ((b & 0x01) && ((b & 0x02) == 0)) {
        buffer[index++] = b;
      }
    } else {
      buffer[index++] = b;
      if (index == 5) {
        index = 0;

        // Decodeer de data
        bool startFlag = buffer[0] & 0x01;
        bool invertedFlag = (buffer[0] >> 1) & 0x01;
        uint16_t angleQ6 = ((buffer[1] >> 1) | ((uint16_t)buffer[2] << 7));
        float angle = angleQ6 / 64.0;
        uint16_t distanceQ2 = (buffer[3] | (buffer[4] << 8));
        float distance = distanceQ2 / 4.0;

        if (distance > 0 && distance < 6000) { // 6 meter max
          Serial.printf("Hoek: %6.2f°  |  Afstand: %6.1f mm\n", angle, distance);
        }
      }
    }
  }
}
