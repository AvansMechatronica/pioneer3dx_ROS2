#include <Arduino.h>
#include "pins.h"
#include "lds08_lidar.h"



lds08_lidar* lidar;
// Motor PWM snelheid (0-100%)
int motorSpeedPercent = 40;


// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("RPLIDAR A1M8 - Setup gestart");

  lidar = new lds08_lidar( LIDAR_COM_PORT, LIDAR_TX_PIN, LIDAR_RX_PIN, LIDAR_MOTOR_PIN );


  Serial.printf("LIDAR initialized\n");
  // Device info en health uitlezen
  delay(1000);

  // Start scan (false = standaard scan, true = express scan)
  lidar->startScan(DEFAULT_LIDAR_MOTOR_PWM);
  lidar->startScan(60);
}


int loop_counter = 0;
lds08_lidarMeasurement scanValue;
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

