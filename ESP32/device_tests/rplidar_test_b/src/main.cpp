#include <Arduino.h>

// --- Pinnen ---
#define RPLIDAR_MOTOR 25
#define LIDAR_RX 16
#define LIDAR_TX 17

HardwareSerial LIDARSerial(2); // UART2

// Motor PWM snelheid (0-100%)
int motorSpeedPercent = 100;

// --- Functieprototypes ---
void getDeviceInfo();
void getDeviceHealth();
void startScan(bool express);
void stopScan();
void setupMotorPWM(int percent);

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("RPLIDAR A1M8 - Setup gestart");

  // Motor aansturen
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  setupMotorPWM(motorSpeedPercent);

  // UART2 starten voor LIDAR
  LIDARSerial.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  delay(1000);

  // Device info en health uitlezen
  getDeviceInfo();
  getDeviceHealth();

  // Start scan (false = standaard scan, true = express scan)
  startScan(false);
}

// ------------------- Loop -------------------
void loop() {
  // Lees data van LIDAR (5-byte frames voor standaard scan)
  if (LIDARSerial.available() >= 5) {
    uint8_t data[5];
    LIDARSerial.readBytes(data, 5);

    // Decodeer standaard scan frame
    bool startFlag = data[0] & 0x01;
    bool invertedFlag = (data[0] >> 1) & 0x01;
    uint16_t angleQ6 = ((data[1] >> 1) | ((uint16_t)data[2] << 7));
    float angle = angleQ6 / 64.0;
    uint16_t distanceQ2 = (data[3] | (data[4] << 8));
    float distance = distanceQ2 / 4.0;

    if (distance > 0 && distance < 6000) { // 6 meter max
      Serial.printf("Hoek: %6.2f°  |  Afstand: %6.1f mm\n", angle, distance);
    }
  }
}

// ------------------- Functies -------------------

void setupMotorPWM(int percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  // PWM via analogWrite (0-255)
  analogWrite(RPLIDAR_MOTOR, map(percent, 0, 100, 0, 255));
  Serial.printf("Motor PWM ingesteld op %d%%\n", percent);
}

void getDeviceInfo() {
  byte cmd[2] = {0xA5, 0x50};
  LIDARSerial.write(cmd, 2);
  Serial.println("Device info opgevraagd");
  delay(100);
  while (LIDARSerial.available()) {
    Serial.write(LIDARSerial.read()); // raw bytes
  }
  Serial.println();
}

void getDeviceHealth() {
  byte cmd[2] = {0xA5, 0x52};
  LIDARSerial.write(cmd, 2);
  Serial.println("Device health opgevraagd");
  delay(100);
  while (LIDARSerial.available()) {
    Serial.write(LIDARSerial.read());
  }
  Serial.println();
}

void startScan(bool express) {
  if (express) {
    byte expressScan[2] = {0xA5, 0xF0};
    LIDARSerial.write(expressScan, 2);
    Serial.println("Express scan gestart");
  } else {
    byte startScan[2] = {0xA5, 0x20};
    LIDARSerial.write(startScan, 2);
    Serial.println("Standaard scan gestart");
  }
}

void stopScan() {
  byte stopScan[2] = {0xA5, 0x25};
  LIDARSerial.write(stopScan, 2);
  Serial.println("Scan gestopt");
}
