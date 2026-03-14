#include "main.h"

extern Buzzer *buzzer;
extern Adafruit_ST7735 *tft;
#if defined(INCLUDE_LIDAR)
#if defined(LIDAR_LDS08)
    extern lds08_lidar *lidar;
#elif defined(LIDAR_YD_T_MINI)
  extern ydlidar_t_mini_plus *lidar;
#endif
#endif

/**
 * Convert RCL return code to human-readable string
 */
const char* rcl_error_string(rcl_ret_t ret) {
  switch(ret) {
    case RCL_RET_OK: return "OK";
    case RCL_RET_ERROR: return "Unspecified error";
    case RCL_RET_TIMEOUT: return "Timeout";
    case RCL_RET_BAD_ALLOC: return "Failed to allocate memory";
    case RCL_RET_INVALID_ARGUMENT: return "Invalid argument";
    case RCL_RET_UNSUPPORTED: return "Unsupported operation";
    case RCL_RET_ALREADY_INIT: return "Already initialized";
    case RCL_RET_NOT_INIT: return "Not initialized";
    case RCL_RET_MISMATCHED_RMW_ID: return "RMW implementation mismatch";
    case RCL_RET_TOPIC_NAME_INVALID: return "Invalid topic name";
    case RCL_RET_SERVICE_NAME_INVALID: return "Invalid service name";
    case RCL_RET_UNKNOWN_SUBSTITUTION: return "Unknown substitution";
    case RCL_RET_ALREADY_SHUTDOWN: return "Already shutdown";
    default: return "Unknown error";
  }
}

/**
 * Error handler that stops the lidar, displays error, and restarts ESP32
 */
void microros_error_handler(rcl_ret_t temp_rc, int line) {
  if(temp_rc == RCL_RET_ERROR){ // Ignore generic errors
    tft_printf(ST77XX_RED, "uROS Error\n%s\nline: %d\nContinuing...", rcl_error_string(temp_rc), line);
    buzzer->errorTune();
  }
  else {
#if defined(INCLUDE_LIDAR)
    lidar->stopScan();
#endif
    tft_printf(ST77XX_BLUE, "uROS Error\n%s\nline: %d\nRestarting...", rcl_error_string(temp_rc), line);
    buzzer->errorTune();
    delay(5000);
    buzzer->byeTune();
    delay(1000);
    ESP.restart(); 
  }
}

/**
 * Error warning that stops the lidar, displays error, and restarts ESP32
 */
void microros_warning_handler(rcl_ret_t temp_rc, int line) {
  if(temp_rc != RCL_RET_ERROR){ // Ignore generic errors
    tft_printf(ST77XX_RED, "uROS Warning\n%s\nline: %d\nContinuing...", rcl_error_string(temp_rc), line);
    buzzer->warningTune();
  } 
};  

void error_handler(int line) {
#if defined(INCLUDE_LIDAR)
    lidar->stopScan();
#endif
    DEBUG_PRINT("Fatal Error\nRestarting...\nline: %d\n", line);
    tft_printf(ST77XX_BLUE, "Fatal Error\nRestarting...\nline: %d", line);
    buzzer->errorTune();
    delay(5000);
    buzzer->byeTune();
    delay(1000);
    ESP.restart();
}

/**
 * Initialize SPI display
 */
void init_display(){
  // ESP32-S3 specific SPI initialization with explicit pins
  SPIClass *spiClass = new SPIClass(FSPI); // was HSPI
  
  // Initialize SPI with explicit pins for ESP32-S3
  spiClass->begin(
    DISPLAY_CLK_PIN,   // SCK
    -1,                // MISO (not used for display)
    DISPLAY_SDA_PIN,   // MOSI
    DISPLAY_CS_PIN     // SS
  );

  tft = new Adafruit_ST7735(
    spiClass, 
    DISPLAY_CS_PIN, 
    DISPLAY_RS_DC_PIN, 
    DISPLAY_RST_PIN
  );
  
  tft_prinft_begin(tft);

  tft->initR(INITR_GREENTAB);
  tft->fillScreen(ST77XX_BLACK);
  tft->setRotation(3);
  tft->setFont(&FreeSansBold9pt7b);
  tft->fillScreen(ST77XX_BLACK);
  tft->setTextColor(ST77XX_CYAN);
  tft->setTextSize(1);
  tft->setCursor(1, 22);
  tft->println("P3DX Control");

  DEBUG_PRINT("Display Ready\n"); 
}

/**
 * Convert snake_case to camelCase
 */
char* convertToCamelCase(const char *input) {
  static char output[64];
  if (input == nullptr) {
    output[0] = '\0';
    return output;
  }

  size_t out_index = 0;
  bool upper_next = false;
  for (size_t in_index = 0; input[in_index] != '\0' && out_index < (sizeof(output) - 1); ++in_index) {
    char current = input[in_index];
    if (current == '_') {
      upper_next = true;
      continue;
    }

    if (upper_next && current >= 'a' && current <= 'z') {
      current = static_cast<char>(current - ('a' - 'A'));
    }
    upper_next = false;
    output[out_index++] = current;
  }

  output[out_index] = '\0';
  return output;
}

#if 0
/**
 * Get synchronized time with ROS2 agent
 */
struct timespec getTime() {
  struct timespec tp = { 0 };
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}


/**
 * Synchronize ESP32 time with ROS2 agent
 */
void syncTime() {
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  time_offset = ros_time_ms - now;
}
#endif