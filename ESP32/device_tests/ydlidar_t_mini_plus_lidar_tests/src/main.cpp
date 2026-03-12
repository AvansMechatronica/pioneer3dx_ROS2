#include <Arduino.h>
#include "pins.h"
#include "ydlidar_t_mini_plus.h"



ydlidar_t_mini_plus* lidar;
// Motor PWM snelheid (0-100%)
int motorSpeedPercent = 40;


// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("RPLIDAR A1M8 - Setup gestart");

  lidar = new ydlidar_t_mini_plus( RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN);

  Serial.printf("LIDAR initialized\n");
  // Device info en health uitlezen
  delay(1000);

  // Start scan (false = standaard scan, true = express scan)
  lidar->startScan(DEFAULT_LIDAR_MOTOR_PWM);
  Serial.println("Scan gestart");
  
}


int loop_counter = 0;
ydlidar_t_mini_plusMeasurement scanValue;
// ------------------- Loop -------------------
void loop() {
#if 1
  if(lidar->getScanValue(&scanValue, 100)) {
#if 1
    if(loop_counter++ % 1000 == 0) {
      Serial.printf("Speed: %d\n", scanValue.speed);
      Serial.printf("Start angle: %.2f deg, End angle: %.2f deg\n",
        scanValue.start_angle,
        scanValue.end_angle);
      for(int i = 0; i < POINT_PER_PACK; i++) {
        Serial.printf("Angle: %.2f deg, Distance: %.2f mm, Quality: %d\n",
          scanValue.point[i].angle,
          scanValue.point[i].distance,
          scanValue.point[i].quality);
      }

    }
#endif
  }
#endif
//  delay(10);
}

