#include <Arduino.h>
#include "SPIFFS.h"


void setup() {
  Serial.begin(115200);
  delay(1000);
  
  if(!SPIFFS.begin(true)){  // Changed to 'true' to format on first mount
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  // List files to see what's there
  Serial.println("Files in SPIFFS:");
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while(file){
    Serial.print("  ");
    Serial.print(file.name());
    Serial.print("  Size: ");
    Serial.println(file.size());
    file = root.openNextFile();
  }
  
  // Create/write test file
  Serial.println("\nCreating test file...");
  File writeFile = SPIFFS.open("/test_file.txt", FILE_WRITE);
  if(!writeFile){
    Serial.println("Failed to open file for writing");
    return;
  }
  writeFile.println("Line 1: Hello World");
  writeFile.println("Line 2: Testing SPIFFS");
  writeFile.println("Line 3: ESP32-S3");
  writeFile.flush();  // Force flush before closing
  writeFile.close();
  Serial.println("File written successfully");
  
  delay(100);  // Small delay to ensure filesystem operations complete
  
  // Read the file
  File readFile = SPIFFS.open("/test_file.txt", FILE_READ);  // Explicitly specify FILE_READ
  if(!readFile || readFile.isDirectory()){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  Serial.printf("File size from handle: %d bytes\n", readFile.size());
  
  String fileContent = readFile.readString();
  readFile.close();
  
  Serial.println("\nFile Content:");
  Serial.println(fileContent);
  Serial.printf("File size: %d bytes\n", fileContent.length());
}

int i = 0;
void loop() {
  Serial.printf("Count: %d\n", i++);
  delay(1000);
}