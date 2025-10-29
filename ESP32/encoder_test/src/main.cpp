#include <Arduino.h>

// --- Rotary encoder pinnen ---
#define L_PWM_PIN        32
#define L_DIR_PIN        33
#define L_ENCODER_PINA   25  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define L_ENCODER_PINB   26
//right wheel
#define R_PWM_PIN        23
#define R_DIR_PIN        22
#define R_ENCODER_PINA   21  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define R_ENCODER_PINB   19

#define R_CHANNEL         1

#if defined(R_CHANNEL)
#undef L_PWM_PIN        
#undef L_DIR_PIN        
#undef L_ENCODER_PINA   
#undef L_ENCODER_P

#define L_PWM_PIN        R_PWM_PIN
#define L_DIR_PIN        R_DIR_PIN
#define L_ENCODER_PINA   R_ENCODER_PINA
#define L_ENCODER_PINB   R_ENCODER_PINB

#endif



// --- Variabelen ---
volatile int encoderPos = 0;  // Huidige positie
unsigned long lastPrint = 0;

// --- ISR (Interrupt Service Routine) ---
void IRAM_ATTR handleEncoderA() {
  bool A = digitalRead(L_ENCODER_PINA);
  bool B = digitalRead(L_ENCODER_PINB);

  // Standaard quadratuur-decoder logica
  if (A == B) encoderPos++;
  else encoderPos--;
}

#if 0

void IRAM_ATTR handleEncoderA() {
  if (digitalRead(L_ENCODER_PINA) > digitalRead(L_ENCODER_PINB))
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

  pinMode(L_ENCODER_PINA, INPUT_PULLUP);
  pinMode(L_ENCODER_PINB, INPUT_PULLUP);

  // Interrupt op pin A, zowel stijgende als dalende flank
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PINA), handleEncoderA, CHANGE);
}

void loop() {
  // 1× per seconde printen
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    //noInterrupts();  // tijdelijk stoppen om stabiel te lezen
    int pos = encoderPos;
    interrupts();

    Serial.printf("Encoder positie: %d\n", pos);
  }
}
