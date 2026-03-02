#include <Arduino.h>
#include "pins.h"



#define CLOCK_WISE            HIGH
#define COUNTER_CLOCK_WISE    LOW

//#define R_CHANNEL         1

#if defined(R_CHANNEL)
#define PWM_PIN        R_PWM_PIN
#define DIR_PIN        R_DIR_PIN

//#define CLOCK_WISE            HIGH
//#define COUNTER_CLOCK_WISE    LOW
#else
#define PWM_PIN        L_PWM_PIN
#define DIR_PIN        L_DIR_PIN

#endif


const int freq = 20000;  // 20 kHz PWM
const int pwmChannel = 0;
const int resolution = 10; // 10-bit = 0–1023

int dutyCycle = 0;

void setup() {
  Serial.begin(115200);

  ledcSetup(pwmChannel, freq, resolution);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  digitalWrite(L_PWM_PIN, LOW); // Start met motor uit
  digitalWrite(L_DIR_PIN, LOW); // Start richting
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  digitalWrite(R_PWM_PIN, LOW); // Start richting
  digitalWrite(R_DIR_PIN, LOW); // Start richting

  ledcAttachPin(PWM_PIN, pwmChannel);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE); // Motor driver inschakelen
  
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
      digitalWrite(DIR_PIN, CLOCK_WISE);
      dutyCycle = map(value, 0, 100, 0, 1023);
    } else {
      digitalWrite(DIR_PIN, COUNTER_CLOCK_WISE);
      dutyCycle = map(-value, 0, 100, 0, 1023);
    }

    ledcWrite(pwmChannel, dutyCycle);

    Serial.printf("Invoer: %d%% → PWM duty = %d / 1023, richting = %s\n",
                  value, dutyCycle, (digitalRead(DIR_PIN) ? "HIGH" : "LOW"));
  }
}
