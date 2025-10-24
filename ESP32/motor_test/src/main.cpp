#include <Arduino.h>
#include <Arduino.h>

#define PWM_PIN 25       // PWM output pin
#define DIR_PIN 26       // Direction pin

const int freq = 20000;  // 20 kHz PWM
const int pwmChannel = 0;
const int resolution = 10; // 10-bit = 0–1023

int dutyCycle = 0;

void setup() {
  Serial.begin(115200);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM_PIN, pwmChannel);
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.println("PWM controller gestart.");
  Serial.println("Voer een waarde in tussen -100 en 100:");
}

void loop() {
  // Check of er seriële invoer is
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int value = input.toInt();

    if (value < -100) value = -100;
    if (value > 100) value = 100;

    // Bepaal richting
    if (value >= 0) {
      digitalWrite(DIR_PIN, HIGH);
      dutyCycle = map(value, 0, 100, 0, 1023);
    } else {
      digitalWrite(DIR_PIN, LOW);
      dutyCycle = map(-value, 0, 100, 0, 1023);
    }

    ledcWrite(pwmChannel, dutyCycle);

    Serial.printf("Invoer: %d%% → PWM duty = %d / 1023, richting = %s\n",
                  value, dutyCycle, (digitalRead(DIR_PIN) ? "HIGH" : "LOW"));
  }
}
