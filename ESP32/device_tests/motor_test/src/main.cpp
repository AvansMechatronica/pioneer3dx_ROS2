#include <Arduino.h>
#include "pins.h"



#define CLOCK_WISE            HIGH
#define COUNTER_CLOCK_WISE    LOW

//#define R_CHANNEL         1

#if defined(R_CHANNEL)
#undef L_PWM_PIN        
#undef L_DIR_PIN        
#undef L_ENCODER_PINA   
#undef L_ENCODER_PINB

//#undef CLOCK_WISE
//#undef COUNTER_CLOCK_WISE

#define L_PWM_PIN        R_PWM_PIN
#define L_DIR_PIN        R_DIR_PIN
#define L_ENCODER_PINA   R_ENCODER_PINA
#define L_ENCODER_PINB   R_ENCODER_PINB

//#define CLOCK_WISE            HIGH
//#define COUNTER_CLOCK_WISE    LOW

#endif


const int freq = 20000;  // 20 kHz PWM
const int pwmChannel = 0;
const int resolution = 10; // 10-bit = 0–1023

int dutyCycle = 0;

void setup() {
  Serial.begin(115200);
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(L_PWM_PIN, pwmChannel);
  pinMode(L_DIR_PIN, OUTPUT);
  
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
      digitalWrite(L_DIR_PIN, CLOCK_WISE);
      dutyCycle = map(value, 0, 100, 0, 1023);
    } else {
      digitalWrite(L_DIR_PIN, COUNTER_CLOCK_WISE);
      dutyCycle = map(-value, 0, 100, 0, 1023);
    }

    ledcWrite(pwmChannel, dutyCycle);

    Serial.printf("Invoer: %d%% → PWM duty = %d / 1023, richting = %s\n",
                  value, dutyCycle, (digitalRead(L_DIR_PIN) ? "HIGH" : "LOW"));
  }
}
