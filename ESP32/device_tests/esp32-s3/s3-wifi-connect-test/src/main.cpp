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

bool wifiUp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);


  const char *host_name = convertToCamelCase("test_node");
  Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(host_name);
  
  NETWORK_CONFIG networkConfig;
  wifiUp = configureNetwork(true, &networkConfig);

  // Give async tasks time to initialize
  delay(500);
//  yield();

  Serial.printf("Setup complete\n");
  
  if(wifiUp) {
    Serial.printf("WiFi connected to: %s\n", networkConfig.ssid.c_str());
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("microROS agent: %s:%d\n", 
                  networkConfig.microros_agent_ip_address.toString().c_str(),
                  networkConfig.microros_agent_port);
  } else {
    Serial.printf("Started AP mode for configuration\n");
    Serial.printf("Connect to WiFi: Pioneer3DX\n");
    Serial.printf("Open browser to: 192.168.4.1\n");
  }
}

int i = 0;
void loop() {
  if(!wifiUp) {
    delay(100);  // In AP mode, give async tasks time to run
    return;
  }
  Serial.printf("Count: %d\n", i++);
  delay(1000);
}

