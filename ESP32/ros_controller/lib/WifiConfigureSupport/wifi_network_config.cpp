
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include "wifi_network_config.h"
#ifndef IGNOR_TFT_PRINT
#include "tft_printf.h"
#endif

#include <string>
#include <iostream>

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

// Create AsyncWebServer object on port 80#include "tft_printf.h"
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ros_agent_ipPath";
const char* PARAM_INPUT_4 = "ros_agent_port";

//Variables to save values from HTML form
String ssid;
String pass;
String ros_agent_ipPath;
String ros_agent_port;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ros_server_ipPath = "/ros_agent_ipPath.txt";
const char* ros_server_portPath = "/ros_agent_port.txt";


// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)


// Initialize LittleFS
void initLittleFS() {
  if (!LittleFS.begin(false)) {
    DEBUG_PRINT("An error has occurred while mounting LittleFS\n");
#ifndef IGNOR_TFT_PRINT
    tft_printf(ST77XX_BLUE, "No filesystem\ndetected\nInstall by\nPlatfomIO\n");
#endif
    while(true){};
  }
  DEBUG_PRINT("LittleFS mounted successfully\n");
}


// Read File from LittleFS
String readFile(fs::FS &fs, const char * path){
  DEBUG_PRINT("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    DEBUG_PRINT("- failed to open file for reading\n");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Write file to LittleFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  DEBUG_PRINT("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    DEBUG_PRINT("- failed to open file for writing\n");
    return;
  }
  if(file.print(message)){
    DEBUG_PRINT("- file written\n");
  } else {
    DEBUG_PRINT("- write failed\n");
  }
}

// Initialize WiFi
bool testWifi() {
  if(ssid=="" || pass=="" || ros_agent_ipPath=="" || ros_agent_port==""){
    DEBUG_PRINT("Undefined SSID, Password, microROS server IP address or Port\n");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  DEBUG_PRINT("Connecting to WiFi...\n");
#ifndef IGNOR_TFT_PRINT
  tft_printf(ST77XX_MAGENTA, "Connecting\nto WiFi...\n");
#endif

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      DEBUG_PRINT("Failed to connect\n");
#ifndef IGNOR_TFT_PRINT
      tft_printf(ST77XX_MAGENTA, "Failed to\nconnect to\nWiFi\n");
#endif
      return false;
    }
  }

  return true;
}


NETWORK_CONFIG network_config;

bool configureNetwork(bool forceConfigure, NETWORK_CONFIG *networkConfig) {


  initLittleFS();

  // detect webserver files

  File file = LittleFS.open("/wifimanager.html");
  if(!file){
    DEBUG_PRINT("No webpages file found\n");
#ifndef IGNOR_TFT_PRINT
    tft_printf(ST77XX_BLUE, "No webpages\nfound\nInstall by\nPlatfomIO\n");
#endif
    while(true){};
  }
 
  // Load values saved in LittleFS
  ssid = readFile(LittleFS, ssidPath);
  pass = readFile(LittleFS, passPath);
  ros_agent_ipPath = readFile(LittleFS, ros_server_ipPath);
  ros_agent_port = readFile(LittleFS, ros_server_portPath);

  DEBUG_PRINT("ssid : %s\n", ssid.c_str());
  DEBUG_PRINT("pass : %s\n", pass.c_str());
  DEBUG_PRINT("ros_agent_ipPath : %s\n", ros_agent_ipPath.c_str());
  DEBUG_PRINT("ros_agent_port : %s\n", ros_agent_port.c_str());

  if(testWifi() & !forceConfigure) {
    networkConfig->password = pass;
    networkConfig->ssid = ssid;
    networkConfig->microros_agent_ip_address.fromString(ros_agent_ipPath);
    networkConfig->microros_agent_port = std::stoi(ros_agent_port.c_str());
    return true;
  }
  else {
    // Connect to Wi-Fi network with SSID and password
    WiFi.mode(WIFI_AP);
    const char *ap_name = "Railtrack Wifi";
    DEBUG_PRINT("Setting AP (Access Point)\n");

    // NULL sets an open Access Point
    WiFi.softAP(ap_name, NULL);

    IPAddress IP = WiFi.softAPIP();
#ifndef IGNOR_TFT_PRINT
    tft_printf(ST77XX_MAGENTA, "Connect to AP:\n%s\nIP: %s\n", ap_name, IP.toString().c_str());
#endif
    DEBUG_PRINT("Connect to AP: %s, IP: %s\n", ap_name, IP.toString().c_str());
    
    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", LittleFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            DEBUG_PRINT("SSID set to: %s\n", ssid );
            // Write file to save value
            writeFile(LittleFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            DEBUG_PRINT("Password set to: %s\n", pass);
            // Write file to save value
            writeFile(LittleFS, passPath, pass.c_str());
          }
          // HTTP POST microROS server ip value
          if (p->name() == PARAM_INPUT_3) {
            ros_agent_ipPath = p->value().c_str();
            DEBUG_PRINT("micoROS server IP Address set to: %s\n", ros_agent_ipPath);
            // Write file to save value
            writeFile(LittleFS, ros_server_ipPath, ros_agent_ipPath.c_str());
          }
          // HTTP POST microROS port value
          if (p->name() == PARAM_INPUT_4) {
            ros_agent_port = p->value().c_str();
            DEBUG_PRINT("microROS Port-number set to: %s\n", ros_agent_port);
            // Write file to save value
            writeFile(LittleFS, ros_server_portPath, ros_agent_port.c_str());
          }
        }
      }
      request->send(200, "text/plain", "Done. Controller will restart");
#ifndef IGNOR_TFT_PRINT
      tft_printf(ST77XX_MAGENTA, "WiFi\nconfiguration\nstored\nRestarting...\n");
#endif
      delay(3000);
      ESP.restart();
    });
    server.begin();
  }
  return false;
}

