#include <Arduino.h>
#include "pins.h"
#include "rplidar.h"



rplidar* lidar;

// Motor PWM snelheid (0-100%)
int motorSpeedPercent = 50;


// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("RPLIDAR A1M8 - Setup gestart");


  lidar = new rplidar( RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN, RPLIDAR_MOTOR_PIN );

  lidar->setupMotorPWM(motorSpeedPercent);
  delay(1000);

  // Device info en health uitlezen
  RplidarInfo info;
  if(lidar->getDeviceInfo(&info)){
    Serial.println("Device info succesvol uitgelezen.");
    Serial.printf("Model: %u\n", info.model);
    Serial.printf("Firmware: %u.%u\n", info.firmware_major, info.firmware_minor);
    Serial.printf("Hardware: %u\n", info.hardware);

    Serial.print("Serial: ");
    for (int i = 15; i >= 0; i--)   // reverse print
        Serial.printf("%02X", info.serial[i]);
    Serial.println();
  } else {
    Serial.println("Fout bij het uitlezen van device info.");
    }

  RplidarHealth health;

  if(lidar->getDeviceHealth(&health)) {
    Serial.println("Device health succesvol uitgelezen.");
    Serial.printf("Status: %u\n", health.status);
    Serial.printf("Error code: %u\n", health.error_code);
  } else {
    Serial.println("Fout bij het uitlezen van device health.");
  } 

  RplidarSampleRate sampleRate;
  if(lidar->getSampleRate(&sampleRate)) {
    Serial.println("Sample rate succesvol uitgelezen.");
    Serial.printf("Standard scan: %u us\n", sampleRate.standardScan_us);
    Serial.printf("Express scan: %u us\n", sampleRate.expressScan_us);
  } else {
    Serial.println("Fout bij het uitlezen van sample rate.");
  }

  // Start scan (false = standaard scan, true = express scan)
  lidar->startScan(false);
}

RplidarValue scanValue;
// ------------------- Loop -------------------
void loop() {
#if 1
  if(lidar->getScanValue(&scanValue, 100)) {
    Serial.printf("Hoek: %6.2f°  |  Afstand: %6.1f mm\n", scanValue.angle, scanValue.distance);
  }
#endif
  delay(10);
}

