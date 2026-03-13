#ifdef WIFI
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Include appropriate filesystem
#include "LittleFS.h"
#define FS_SYSTEM LittleFS
#define FS_NAME "LittleFS"

#include "wifi_network_config.h"
#if defined (WIFI_INCLUDE_TFT_PRINT)
#include "tft_printf.h"
#endif

#include <string>


#ifdef NETWORK_CONFIG_DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ros_agent_ipPath";
const char* PARAM_INPUT_4 = "ros_agent_port";

// Variables to save values from HTML form
String ssid;
String pass;
String ros_agent_ipPath;
String ros_agent_port;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ros_server_ipPath = "/ros_agent_ip.txt";
const char* ros_server_portPath = "/ros_agent_port.txt";

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)

static bool hasStoredNetworkConfig() {
  return !ssid.isEmpty() && !pass.isEmpty() && !ros_agent_ipPath.isEmpty() && !ros_agent_port.isEmpty();
}

static String escapeJson(const String& input) {
  String output;
  output.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    const char ch = input[i];
    if (ch == '"' || ch == '\\') {
      output += '\\';
    }
    output += ch;
  }
  return output;
}

static String buildWifiScanJson() {
  String payload = "[";
  int network_count = WiFi.scanComplete();

  if (network_count == -2) {
    WiFi.scanNetworks(true, true);
    payload += "]";
    return payload;
  }

  if (network_count == -1) {
    payload += "]";
    return payload;
  }

  if (network_count <= 0) {
    payload += "]";
    WiFi.scanDelete();
    WiFi.scanNetworks(true, true);
    return payload;
  }

  for (int index = 0; index < network_count; ++index) {
    if (index > 0) {
      payload += ",";
    }
    payload += "\"";
    payload += escapeJson(WiFi.SSID(index));
    payload += "\"";
  }

  payload += "]";
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true);
  return payload;
}

// Initialize filesystem (LittleFS or SPIFFS)
void initFS() {
  bool result;
  
  result = LittleFS.begin(true);

  if (!result) {
    DEBUG_PRINT("An error has occurred while mounting %s\n", FS_NAME);
#if defined (WIFI_INCLUDE_TFT_PRINT)
    tft_printf(ST77XX_BLUE, "No filesystem\ndetected\nInstall by\nPlatformIO\n");
#endif
    while(true){};
  }
  DEBUG_PRINT("%s mounted successfully\n", FS_NAME);
}

// Read File from filesystem
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
  file.close();
  DEBUG_PRINT("- read from file: %s\n", fileContent.c_str());
  return fileContent;
}

// Write file to filesystem
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
  file.close();
}

// Initialize WiFi
bool testWifi() {
  if(ssid=="" || pass=="" || ros_agent_ipPath=="" || ros_agent_port==""){
    DEBUG_PRINT("Undefined SSID, Password, microROS server IP address or Port\n");
    return false;
  }

  WiFi.useStaticBuffers(true);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true, true);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_MODE_NULL);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_STA);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), pass.c_str());
  DEBUG_PRINT("Connecting to WiFi...\n");
#if defined (WIFI_INCLUDE_TFT_PRINT)
  tft_printf(ST77XX_MAGENTA, "Connecting\nto WiFi...\n");
#endif

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;
  vTaskDelay(500 / portTICK_PERIOD_MS);
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    wl_status_t status = WiFi.status();
    if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
      DEBUG_PRINT("WiFi failed with status=%d\n", status);
      WiFi.disconnect(true, true);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      return false;
    }
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      DEBUG_PRINT("Failed to connect\n");
#if defined (WIFI_INCLUDE_TFT_PRINT)
      tft_printf(ST77XX_MAGENTA, "Failed to\nconnect to\nWiFi\n");
#endif
      WiFi.disconnect(true, true);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      return false;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  DEBUG_PRINT("Connected to WiFi\n");
  DEBUG_PRINT("IP Address: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

NETWORK_CONFIG network_config;

// Handle POST request for WiFi configuration
void handleConfigPost(AsyncWebServerRequest *request) {
  int params = request->params();
  for(int i=0; i<params; i++){
    const AsyncWebParameter* p = request->getParam(i);
    if(p->isPost()){
      // HTTP POST ssid value
      if (p->name() == PARAM_INPUT_1) {
        ssid = p->value().c_str();
        DEBUG_PRINT("SSID set to: %s\n", ssid.c_str());
        // Write file to save value
        writeFile(FS_SYSTEM, ssidPath, ssid.c_str());
      }
      // HTTP POST pass value
      if (p->name() == PARAM_INPUT_2) {
        pass = p->value().c_str();
        DEBUG_PRINT("Password set to: %s\n", pass.c_str());
        // Write file to save value
        writeFile(FS_SYSTEM, passPath, pass.c_str());
      }
      // HTTP POST microROS server ip value
      if (p->name() == PARAM_INPUT_3) {
        ros_agent_ipPath = p->value().c_str();
        DEBUG_PRINT("micoROS server IP Address set to: %s\n", ros_agent_ipPath.c_str());
        // Write file to save value
        writeFile(FS_SYSTEM, ros_server_ipPath, ros_agent_ipPath.c_str());
      }
      // HTTP POST microROS port value
      if (p->name() == PARAM_INPUT_4) {
        ros_agent_port = p->value().c_str();
        DEBUG_PRINT("microROS Port-number set to: %s\n", ros_agent_port.c_str());
        // Write file to save value
        writeFile(FS_SYSTEM, ros_server_portPath, ros_agent_port.c_str());
      }
    }
  }
  request->send(200, "text/plain", "Done. Controller will restart");
#if defined (WIFI_INCLUDE_TFT_PRINT)
  tft_printf(ST77XX_MAGENTA, "WiFi\nconfiguration\nstored\nRestarting...\n");
#endif
  delay(3000);
  ESP.restart();
}

// Returns true if the network was configured and WiFi connection succeeded (station mode).
// Returns false if the device entered AP mode for configuration or if configuration is incomplete.
bool configureNetwork(bool forceConfigure, NETWORK_CONFIG *networkConfig) {

  initFS();

  // Detect webserver files
  File file = FS_SYSTEM.open("/wifimanager.html");
  if(!file){
    DEBUG_PRINT("No webpages file found\n");
#if defined (WIFI_INCLUDE_TFT_PRINT)
    tft_printf(ST77XX_BLUE, "No webpages\nfound\nInstall by\nPlatformIO\n");
#endif
    while(true){};
  }
  file.close();
 
  // Load values saved in filesystem
  ssid = readFile(FS_SYSTEM, ssidPath);
  pass = readFile(FS_SYSTEM, passPath);
  ros_agent_ipPath = readFile(FS_SYSTEM, ros_server_ipPath);
  ros_agent_port = readFile(FS_SYSTEM, ros_server_portPath);

  DEBUG_PRINT("ssid : %s\n", ssid.c_str());
  DEBUG_PRINT("pass : %s\n", pass.c_str());
  DEBUG_PRINT("ros_agent_ipPath : %s\n", ros_agent_ipPath.c_str());
  DEBUG_PRINT("ros_agent_port : %s\n", ros_agent_port.c_str());

  if(!forceConfigure && hasStoredNetworkConfig() && testWifi()) {
    networkConfig->password = pass;
    networkConfig->ssid = ssid;
    networkConfig->microros_agent_ip_address.fromString(ros_agent_ipPath);

    int port = 0;
    if (!ros_agent_port.isEmpty()) {
      bool valid = true;
      for (size_t i = 0; i < ros_agent_port.length(); ++i) {
        if (!isdigit(ros_agent_port[i])) {
          valid = false;
          break;
        }
      }
      if (valid) {
        port = ros_agent_port.toInt();
      } else {
        DEBUG_PRINT("Invalid port string: %s\n", ros_agent_port.c_str());
      }
    } else {
      DEBUG_PRINT("Port string is empty\n");
    }
    networkConfig->microros_agent_port = port;
    return true;
  }
  else {
    if (forceConfigure) {
      DEBUG_PRINT("Forced WiFi configuration requested\n");
    } else if (!hasStoredNetworkConfig()) {
      DEBUG_PRINT("No stored WiFi credentials found, entering AP mode\n");
    }

    // Connect to Wi-Fi network with SSID and password
    WiFi.useStaticBuffers(true);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false, true);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    WiFi.mode(WIFI_AP_STA);
    vTaskDelay(50 / portTICK_PERIOD_MS);
    const char *ap_name = "Pioneer3DX";
    DEBUG_PRINT("Setting AP (Access Point)\n");

        if (!WiFi.softAP(ap_name, NULL)) {
      DEBUG_PRINT("Failed to start AP\n");
    #if defined (WIFI_INCLUDE_TFT_PRINT)
      tft_printf(ST77XX_MAGENTA, "Failed to\nstart AP\nRestarting...\n");
    #endif
      delay(3000);
      ESP.restart();
        }

    IPAddress IP = WiFi.softAPIP();
    DEBUG_PRINT("AP IP address: %s\n", IP.toString().c_str());

    WiFi.scanDelete();
    WiFi.scanNetworks(true, true);

#if defined (WIFI_INCLUDE_TFT_PRINT)
    tft_printf(ST77XX_MAGENTA, "Connect to AP:\n%s\nIP: %s\n", ap_name, IP.toString().c_str());
#endif

    // Web Server Root URL - serve static files first
    server.serveStatic("/style.css", FS_SYSTEM, "/style.css");
    

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(FS_SYSTEM, "/wifimanager.html", "text/html");
    });

    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "application/json", buildWifiScanJson());
    });
    
#if 1
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      handleConfigPost(request);
    });
#endif 
    // Handle not found
    //server.onNotFound([](AsyncWebServerRequest *request){
    //  request->send(404, "text/plain", "Not found");
    //});
    
    server.begin();
    DEBUG_PRINT("Web server started\n");
    // Do not block here: keep AP web server running asynchronously.
    // Caller can remain in configuration mode while the AsyncWebServer handles requests.
    return false;
  }
}
#endif //WIFI