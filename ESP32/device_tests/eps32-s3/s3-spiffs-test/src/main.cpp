#include <Arduino.h>
#include "SPIFFS.h"


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello, World!");
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  File file = SPIFFS.open("/test_file.txt");
  if(!file || file.isDirectory()){
    Serial.println("Failed to open file for reading");
    return;
  }
  String fileContent;
  int lineCount = 0;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    lineCount++;
    Serial.printf("Line %d: %s\n", lineCount, fileContent.c_str());
    break;     
  }
  Serial.printf("File Content: %s", fileContent.c_str());
  file.close();
}

int i = 0;
void loop() {
  Serial.printf("Count: %d\n", i++);
  delay(1000);
}

