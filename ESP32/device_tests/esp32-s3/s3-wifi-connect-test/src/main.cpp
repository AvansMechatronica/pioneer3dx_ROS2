#include <Arduino.h>
#include "SPIFFS.h"
#include "wifi_network_config.h"
#include <WiFi.h>
/**
 * Convert snake_case to camelCase
 */
char* convertToCamelCase(const char *input) {
    int i, j;
    int len = strlen(input);
    char *output = (char *)malloc((len + 1) * sizeof(char));
    
    if(output == NULL) {
        Serial.printf("Error allocating memory\n");

    }

    strcpy(output, input);

    for (i = 0; i < len; i++) {
        if (output[i] == '_') {
            for (j = i; j < len; j++) {
                output[j] = output[j + 1];
            }
            output[i] = toupper(output[i]);
            len--;
        }
    }
    return output;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);


  const char *host_name = convertToCamelCase("test_node");
  Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(host_name);
  
  NETWORK_CONFIG networkConfig;
  bool wifiUp = configureNetwork(true, &networkConfig);
  if(!wifiUp){
    Serial.printf("Error configuring WiFi Restarting...\n");
    delay(5000);
    ESP.restart();
  };

  Serial.printf("Setup complete\n");  
}

int i = 0;
void loop() {
  Serial.printf("Count: %d\n", i++);
  delay(1000);
}

