#include <Arduino.h>

// --- Rotary encoder pinnen ---
#define ENCODER_PIN_A 32
#define ENCODER_PIN_B 33

// --- Variabelen ---
volatile int encoderPos = 0;  // Huidige positie
unsigned long lastPrint = 0;

// --- ISR (Interrupt Service Routine) ---
void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);

  // Standaard quadratuur-decoder logica
  if (A == B) encoderPos++;
  else encoderPos--;
}

#if 0

void IRAM_ATTR handleEncoderA() {
  if (digitalRead(ENCODER_PIN_A) > digitalRead(ENCODER_PIN_B))
  encoderPos++;
  else
  encoderPos--;
}
#endif

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Rotary Encoder uitlezen gestart.");
  Serial.println("Draai de encoder...");

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Interrupt op pin A, zowel stijgende als dalende flank
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), handleEncoderA, CHANGE);
}

void loop() {
  // 1× per seconde printen
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    noInterrupts();  // tijdelijk stoppen om stabiel te lezen
    int pos = encoderPos;
    interrupts();

    Serial.printf("Encoder positie: %d\n", pos);
  }
}
