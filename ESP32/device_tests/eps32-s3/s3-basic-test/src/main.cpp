#include <Arduino.h>



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello, World!");

}

int i = 0;
void loop() {
  Serial.printf("Count: %d\n", i++);
  delay(1000);
}

