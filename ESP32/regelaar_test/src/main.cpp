#include <Arduino.h>

// ====== Pin-definities ======
#define ENCODER_PIN_A 32
#define ENCODER_PIN_B 33
#define PWM_PIN 25
#define DIR_PIN 26

// ====== PWM-configuratie ======
const int pwmChannel = 0;
const int pwmFreq = 20000;      // 20 kHz
const int pwmResolution = 10;   // 10-bit = 0–1023

// ====== Variabelen ======
volatile long encoderPos = 0;   // Positie vanuit encoder
int setpoint = 0;               // Gewenste waarde (-100 .. 100)
float output = 0;               // PID-uitgang (-100 .. 100)

// PID-parameters
float Kp = 1.2;
float Ki = 0.4;
float Kd = 0.05;

// PID-hulpvariabelen
float integral = 0;
float prevError = 0;
unsigned long lastTime = 0;

// ====== Interrupt routine ======
void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);
  if (A == B) encoderPos++;
  else encoderPos--;
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("PID regelaar gestart.");
  Serial.println("Voer setpoint in tussen -100 en 100:");

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderA, CHANGE);

  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(PWM_PIN, pwmChannel);

  lastTime = millis();
}

// ====== PID-berekening ======
float computePID(float setpoint, float actual) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // seconden
  if (dt <= 0) return output;

  float error = setpoint - actual;
  integral += error * dt;
  float derivative = (error - prevError) / dt;

  float pidOutput = Kp * error + Ki * integral + Kd * derivative;

  // beperk tot -100 .. 100
  if (pidOutput > 100) pidOutput = 100;
  if (pidOutput < -100) pidOutput = -100;

  prevError = error;
  lastTime = now;

  return pidOutput;
}

// ====== Loop ======
void loop() {
  // --- Seriële invoer ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int value = input.toInt();
    if (value < -100) value = -100;
    if (value > 100) value = 100;
    setpoint = value;
    Serial.printf("Nieuw setpoint: %d%%\n", setpoint);
  }

  // --- Encoder uitlezen ---
  noInterrupts();
  long pos = encoderPos;
  interrupts();

  // Encoderwaarde schalen naar -100..100 (pas aan op jouw setup)
  float actualValue = pos / 10.0; // aanpasbare schaalfactor
  if (actualValue > 100) actualValue = 100;
  if (actualValue < -100) actualValue = -100;

  // --- PID ---
  output = computePID(setpoint, actualValue);

  // --- PWM-uitgang ---
  if (output >= 0) {
    digitalWrite(DIR_PIN, HIGH);
    ledcWrite(pwmChannel, map(output, 0, 100, 0, 1023));
  } else {
    digitalWrite(DIR_PIN, LOW);
    ledcWrite(pwmChannel, map(-output, 0, 100, 0, 1023));
  }

  // --- Debug print elke seconde ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.printf("Setpoint: %d | Positie: %.1f | PID-uitgang: %.1f | Richting: %s\n",
                  setpoint, actualValue, output, (digitalRead(DIR_PIN) ? "HIGH" : "LOW"));
  }

  delay(10); // korte vertraging om CPU te ontlasten
}
