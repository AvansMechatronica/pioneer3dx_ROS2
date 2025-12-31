#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>
#include <odometry.h>
#include <jointstate.h>
#if defined(INCLUDE_LIDAR)
#include <lds08_lidar.h>
#include <buzzer.h>
//#include <rplidar.h>
#endif
#if defined(INCLUDE_IMU)
#include <imu_mpu6050.h>
#endif

#include <p3dx_interfaces/msg/status.h>


#include "wifi_network_config.h"

#include "motor_controller.h"
#include "pins.h"

// Display libraries
#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/Tiny3x3a2pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
//#include <SPI.h>
#include "tft_printf.h"

#define DEBUG
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


#ifdef RPLIDAR_H
#define EXPRESS_LIDAR_MODE  false
#endif

// Encoder ticks per full wheel revolution
#define TICK_PER_REVOLUTION  19150

// Command to test robot movement:
//ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

// Robot physical parameters
#define WHEELS_Y_DISTANCE       (float)0.34      // Distance between wheels (meters)
#define WHEELS_RADIUS           (float)0.09765   // Wheel radius (meters)
#define WHEELS_CIRCUMFERENCE    (2 * 3.14 * WHEELS_RADIUS)

// Motor direction definitions
#define CLOCK_WISE            HIGH
#define COUNTER_CLOCK_WISE    LOW

#define THRESHOLD   0

// PID controller constants for left wheel
#define KP_L        (float)30
#define KI_L        (float)80
#define KD_L        (float)0.1

// PID controller constants for right wheel
#define KP_R        (float)30
#define KI_R       (float)80
#define KD_R        (float)0.1

// PWM channel assignments
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1

// ROS2 subscribers
rcl_subscription_t cmd_vel_subscriber;  // Subscribes to velocity commands
rcl_subscription_t reset_subscriber;    // Subscribes to reset commands

// ROS2 message objects
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// ROS2 publishers
rcl_publisher_t error_publisher;
rcl_publisher_t battery_voltage_publisher;
rcl_publisher_t status_publisher;

// Published message objects

std_msgs__msg__Bool reset_msg;
p3dx_interfaces__msg__Status  status_msg;

// Encoder values
int64_t encodervalue_l = 0;  // Left wheel encoder count
int64_t encodervalue_r = 0;  // Right wheel encoder count
bool motors_enabled = false;  // Motor enable state

// Previous RPM values for velocity tracking
float prev_rpm_l;
float prev_rpm_r;

// ROS2 timers
rcl_timer_t motorControlTimer;         // Controls motor PID loop
#if 0
rcl_timer_t odomPublisherTimer;        // Publishes odometry
rcl_timer_t jointstatePublisherTimer;  // Publishes joint states
rcl_timer_t statusPublisherTimer;      // Publishes system status
#if defined(INCLUDE_LIDAR)
rcl_timer_t lidarPublisherTimer;      // Publishes lidar data
#endif
#if defined(INCLUDE_IMU)
rcl_timer_t imuPublisherTimer;        // Publishes IMU data
#endif
#else
rcl_timer_t PublisherTimer;        // Publishes all high-frequency data
#endif

// Time synchronization
unsigned long long time_offset = 0;  // Offset between ESP32 and ROS time
unsigned long prev_cmd_time = 0;     // Last received cmd_vel timestamp
unsigned long prev_odom_update = 0;  // Last odometry update timestamp

// Robot component objects
Odometry *odometry;
Jointstate *jointstate;

#if defined(INCLUDE_LIDAR)
#ifdef RPLIDAR_H
rplidar *lidar;
#else
lds08_lidar *lidar;
#endif
#endif
#if defined(INCLUDE_IMU)
imu_mpu6050 *imu;
#endif

Buzzer *buzzer;

// Motor controller objects
MotorController *leftWheel;
MotorController *rightWheel;

// Display objects
SPIClass *spiClass;
Adafruit_ST7735 *tft;

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

// Macro to check ROS return codes and trigger error handler on failure
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      DEBUG_PRINT("Fatal Error: %s (code %d) at line %d\n", rcl_error_string(temp_rc), temp_rc, __LINE__); \
      microros_error_handler(temp_rc, __LINE__); } \
  }

// Soft check version 
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      DEBUG_PRINT("uROS Warning: %s (code %d) at line %d\n", rcl_error_string(temp_rc), temp_rc, __LINE__); \
      microros_warning_handler(temp_rc, __LINE__); } \
  }

/**
 * Error handler that stops the lidar, displays error, and restarts ESP32
 */
void microros_error_handler(rcl_ret_t temp_rc, int line) {
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
 * Callback for cmd_vel topic - receives velocity commands
 */
void cmd_vel_subscription_callback(const void* msgin) {
  prev_cmd_time = millis();

  const geometry_msgs__msg__Twist* msg = 
      static_cast<const geometry_msgs__msg__Twist*>(msgin);

  // Enable motors if no error and currently disabled
  if(status_msg.error == false){
    if(!motors_enabled){
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
      leftWheel->enable();
      rightWheel->enable();
      motors_enabled = true;
      tft_printf(ST77XX_MAGENTA, "Motors Enabled\n");
    }
  }
  twist_msg = *msg;  // Copy velocity command
}

/**
 * Reset error state by checking both bumpers are pressed
 */
void reset_p3dx(){
  tft_printf(ST77XX_BLUE, "Resetting...\n");
  leftWheel->reset();
  rightWheel->reset();
#if defined(HANDLE_BUMPERS)
//  if(digitalRead(BUMPER_FRONT_PIN)==HIGH && digitalRead(BUMPER_REAR_PIN)==HIGH){
  if(digitalRead(digitalRead(BUMPER_REAR_PIN)==HIGH)){
    digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
    status_msg.error = false; 
    delay(3000);
    tft_printf(ST77XX_MAGENTA, "Reset Done\nMotors Enabled\n");
  }
  else{
    tft_printf(ST77XX_RED, "Cannot Reset!\nBumpers\nNot Clear\n");
    delay(3000);
  }
#else
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
  status_msg.error = false; 
  delay(3000);
  tft_printf(ST77XX_MAGENTA, "Reset Done\nMotors Enabled\n");
#endif
}

/**
 * Callback for reset topic - clears error state
 */
void reset_subscription_callback(const void* msgin) {
  const std_msgs__msg__Bool* msg = 
      static_cast<const std_msgs__msg__Bool*>(msgin);
  if (msg->data == true) {
  reset_p3dx();
  }
}

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



// Voltage divider resistor values
#define R2 560000.0f // Resistor R2 value in ohms, according schematics
#define R3 100000.0f // Resistor R3 value in ohms, according schematics



/**
 * Read and publish the system state
 */

void statusPublisher(){
#if defined(HANDLE_BUMPERS)
  status_msg.bumpers.front = !digitalRead(BUMPER_FRONT_PIN);  // Active low
  status_msg.bumpers.rear = !digitalRead(BUMPER_REAR_PIN);    // Active low
#else
  status_msg.bumpers.front = false;
  status_msg.bumpers.rear = false;
#endif
  float adc_voltage = analogRead(BATTERY_VOLTAGE_PIN) * (3.3 / 1023.0);
  // Calculate actual battery voltage using voltage divider formula: Vout = Vin * R3/(R2+R3)
  // Rearranged: Vin = Vout * (R2+R3)/R3
  status_msg.battery_voltage = adc_voltage * ((R2 + R3) / R3) * 4; //??
  status_msg.motor_enable = motors_enabled;

#if defined(WIFI)
  int rssi = WiFi.RSSI();
  status_msg.wifi_rssi = rssi;
#else
  status_msg.wifi_rssi = 0;
#endif
  RCSOFTCHECK(rcl_publish(&status_publisher, &status_msg, NULL));
}




/**
 * Timer callback for high-frequency publishers (10Hz)
 * Publishes odometry, joint states, lidar data, and IMU data
 */
#if 1
void Publisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    RCSOFTCHECK(odometry->publish());
    delay(15);  // Small delay to avoid overwhelming the executor
    jointstate->update(leftWheel->getVelocity(), 
                      rightWheel->getVelocity(), 
                      encodervalue_l / (TICK_PER_REVOLUTION / (2.0 * M_PI)), 
                      encodervalue_r / (TICK_PER_REVOLUTION / (2.0 * M_PI)));
    RCSOFTCHECK(jointstate->publish());
    delay(15);  // Small delay to avoid overwhelming the executor

  #if defined(INCLUDE_LIDAR)
    RCSOFTCHECK(lidar->publish());
    delay(15);  // Small delay to avoid overwhelming the executor
  #endif

  #if defined(INCLUDE_IMU)
    RCSOFTCHECK(imu->publish());
    delay(15);  // Small delay to avoid overwhelming the executor
  #endif

    statusPublisher();
    delay(15);  // Small delay to avoid overwhelming the executor
  }
}
#else
void odomPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    //RCSOFTCHECK(odometry->publish());
  }
}

#if defined(INCLUDE_IMU)
void imuPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    RCSOFTCHECK(imu->publish());
  }
}
#endif
void statusPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    statusPublisher();
  }
}

#if defined(INCLUDE_LIDAR)
void lidarPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    //RCSOFTCHECK(lidar->publish());
  }
}
#endif

void jointstatePublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    jointstate->update(leftWheel->getVelocity(), 
                  rightWheel->getVelocity(), 
                  encodervalue_l / (TICK_PER_REVOLUTION / (2.0 * M_PI)), 
                  encodervalue_r / (TICK_PER_REVOLUTION / (2.0 * M_PI)));
    //RCSOFTCHECK(jointstate->publish());
  }
}
#endif
int display_interval_counter=0;  // Counter for display update rate limiting

/**
 * Motor control timer callback (50Hz)
 * Performs PID control, odometry calculation, and motor command execution
 */
void MotorControll_timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity;
  float angularVelocity;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    // Disable motors if no cmd_vel received for 100ms (safety timeout)
    if((millis() - prev_cmd_time) > 100) {
      if(motors_enabled) {
        twist_msg.linear.x = 0;
        twist_msg.angular.z = 0;
        tft_printf(ST77XX_MAGENTA, "Motor Stop\nNo cmd_vel\nReceived\n");
        digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
        motors_enabled = false;
        leftWheel->disable();
        rightWheel->disable();
      }  
    }

    // Get commanded velocities
    linearVelocity = twist_msg.linear.x;
    angularVelocity = twist_msg.angular.z;

    // Convert to differential drive wheel velocities
    float vL = (linearVelocity - ((WHEELS_Y_DISTANCE/2.0) * angularVelocity));
    float vR = (linearVelocity + ((WHEELS_Y_DISTANCE/2.0) * angularVelocity));

    // Apply PID control
    float actuating_signal_LW = leftWheel->pid(-vL);
    float actuating_signal_RW = rightWheel->pid(vR);

    // Update display every 10 cycles (200ms)
    if(display_interval_counter % 10 == 0 && motors_enabled){
      tft_printf(ST77XX_MAGENTA, "Linear: %.2f\nAngular: %.2f\nvL: %.2f\nvR: %.2f", linearVelocity, angularVelocity, vL, vR);
    }
    display_interval_counter++;

    // Send PWM commands to motors
    rightWheel->moveBase(actuating_signal_RW);
    leftWheel->moveBase(actuating_signal_LW);

    // Calculate actual wheel velocities and update odometry
    float currentRpmL = leftWheel->getVelocity();
    float currentRpmR = rightWheel->getVelocity();
    float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0;
    float linear_x = average_rps_x * WHEELS_CIRCUMFERENCE;  // m/s
    float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
    float angular_z = (average_rps_a * WHEELS_CIRCUMFERENCE) / (WHEELS_Y_DISTANCE / 2.0);  // rad/s
    float linear_y = 0;

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;

    odometry->update(vel_dt, linear_x, linear_y, angular_z);
  }
}

/**
 * Interrupt handler for left wheel encoder
 */
void updateEncoderL() {
  if (digitalRead(leftWheel->EncoderPinA) == digitalRead(leftWheel->EncoderPinB))
    encodervalue_l--;
  else
    encodervalue_l++;
}

/**
 * Interrupt handler for right wheel encoder
 */
void updateEncoderR() {
  if (digitalRead(rightWheel->EncoderPinA) == digitalRead(rightWheel->EncoderPinB))
    encodervalue_r--;
  else
    encodervalue_r++;
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

#if defined(HANDLE_BUMPERS)
/**
 * Interrupt handler for bumper collision
 */
void bumber_hit(){
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
  tft_printf(ST77XX_BLUE, "Bumper Hit!\nMotors Disabled\n");
  motors_enabled = false; 
  status_msg.error = true;
}
#endif

#define NODE_NAME "p3dx_controller"

bool errorLedState = false;


void reset_button_hit(){
  DEBUG_PRINT("Reset Button Hit!\n");
  tft_printf(ST77XX_BLUE, "Reset Button\nHit!\n");
  delay(3000);
  reset_p3dx();
}

void motors_button_hit(){
  DEBUG_PRINT("Motors Button Hit!\n");
  tft_printf(ST77XX_BLUE, "Motors Button\nHit!\n");
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
  tft->setRotation(1);
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
    int i, j;
    int len = strlen(input);
    char *output = (char *)malloc((len + 1) * sizeof(char));
    
    if(output == NULL) {
        DEBUG_PRINT("Error allocating memory\n");
        error_handler(__LINE__);
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


bool wifiUp = false;
/**
 * Setup function - initializes hardware and ROS2 components
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Pioneer 3DX Controller Starting...");

  init_display();

  pinMode(UCP_RESET_PIN, INPUT_PULLUP);
  // Use polling instead of interrupt to avoid accidental resets
  //attachInterrupt(digitalPinToInterrupt(UCP_RESET_PIN), reset_button_hit, FALLING);

  pinMode(UCP_MOTORS_PIN, INPUT_PULLUP);
  // Use polling instead of interrupt to avoid accidental motor enables
  //attachInterrupt(digitalPinToInterrupt(UCP_MOTORS_PIN), motors_button_hit, FALLING);

  buzzer = new Buzzer(UCP_BUZZER_PIN, UCP_BUZZER_PWM_CHANNEL);

#if defined(WIFI)
  const char *host_name = convertToCamelCase(NODE_NAME);
  DEBUG_PRINT("hostname :%s\n", host_name);
  WiFi.setHostname(host_name);
#if 0
  
  NETWORK_CONFIG networkConfig;
  wifiUp = configureNetwork(false, &networkConfig);
#if 0
  if(!wifiUp){
    tft_printf(ST77XX_MAGENTA, "Error configuring\nWiFi\nRestarting...\n");
    delay(5000);
    ESP.restart();
  };
#endif  
  
#else

  const char* ssid     = "BirdsBoven";
  const char* password = "Highway12!";
  NETWORK_CONFIG networkConfig;
  networkConfig.ssid = ssid;
  networkConfig.password = password;
  networkConfig.microros_agent_ip_address.fromString("192.168.2.150");
  networkConfig.microros_agent_port = 8888;
  wifiUp = true;

#endif

  set_microros_wifi_transports(const_cast<char*>(networkConfig.ssid.c_str()), 
                               const_cast<char*>(networkConfig.password.c_str()), 
                               networkConfig.microros_agent_ip_address,
                               networkConfig.microros_agent_port);
  tft_printf(ST77XX_MAGENTA, "WiFi Connected\n");
  delay(2000);
#else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
#endif
  // Initialize ADC for battery voltage reading
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReadResolution(10);


  tft_printf(ST77XX_MAGENTA, "Pioneer 3DX\nController\nStarted\n");

  // Initialize motor controllers
  leftWheel = new MotorController(PWM_CHANNEL_LEFT, L_PWM_PIN, L_DIR_PIN, L_ENCODER_PINA, L_ENCODER_PINB, &encodervalue_l, WHEELS_RADIUS);
  rightWheel = new MotorController(PWM_CHANNEL_RIGHT, R_PWM_PIN, R_DIR_PIN, R_ENCODER_PINA, R_ENCODER_PINB, &encodervalue_r, WHEELS_RADIUS);

  // Set PID parameters
  leftWheel->setPIDvalues(KP_L, KI_L, KD_L);
  rightWheel->setPIDvalues(KP_R, KI_R, KD_R);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(leftWheel->EncoderPinB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel->EncoderPinB), updateEncoderR, RISING);

  prev_rpm_l = leftWheel->getVelocity();
  prev_rpm_r = rightWheel->getVelocity();

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);

#if defined(HANDLE_BUMPERS)
  // Setup bumper interrupts
  pinMode(BUMPER_FRONT_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUMPER_FRONT_PIN), bumber_hit, FALLING);
  pinMode(BUMPER_REAR_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUMPER_REAR_PIN), bumber_hit, FALLING);
  
  // Check initial bumper state
//  if(digitalRead(BUMPER_FRONT_PIN)==HIGH && digitalRead(BUMPER_REAR_PIN)==HIGH){
  if(digitalRead(digitalRead(BUMPER_REAR_PIN)==HIGH)){
    status_msg.error = false;
  }
  else{
    status_msg.error = true;
  }
#else
  status_msg.error = false;
#endif

  // Ensure motors are disabled
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
  leftWheel->disable();
  rightWheel->disable();
  motors_enabled = false;

  delay(2000);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();

  if(rclc_support_init(&support, 0, NULL, &allocator)){
    tft_printf(ST77XX_BLUE, "microROS agent\nnot found\nRestarting...\n");
    delay(3000);
    ESP.restart();
  }


  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Initialize subscribers
  RCCHECK(rclc_subscription_init_default(&cmd_vel_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(&reset_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "p3dx/reset"));

  // Initialize publishers
  RCCHECK(rclc_publisher_init_default(&status_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(p3dx_interfaces, msg, Status), "p3dx/status"));

    
  // Initialize sensors
#if defined(INCLUDE_LIDAR)
#ifdef RPLIDAR_H
  lidar = new rplidar(&node, RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN, RPLIDAR_MOTOR_PIN);
  Serial.println("RPlidar A1M8 - Setup gestart");
#else
  lidar = new lds08_lidar(&node, RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN, RPLIDAR_MOTOR_PIN);
  Serial.println("LDS08 Lidar - Setup gestart");
#endif
#endif

  odometry = new Odometry(&node);
  jointstate = new Jointstate(&node);
#if defined(INCLUDE_IMU)
  imu = new imu_mpu6050(&node, I2C_SCL_PIN, I2C_SDA_PIN, IMU_INT_PIN);
#endif

  // Initialize timers
  const unsigned int samplingT = 20;  // 50Hz motor control
  RCCHECK(rclc_timer_init_default(&motorControlTimer, &support,
    RCL_MS_TO_NS(samplingT), MotorControll_timerCallback));

#if 1
  RCCHECK(rclc_timer_init_default(&PublisherTimer, &support,
    RCL_MS_TO_NS(200), Publisher_timerCallBack));  // 5Hz
#else
  RCCHECK(rclc_timer_init_default(&odomPublisherTimer, &support,
    RCL_MS_TO_NS(200), odomPublisher_timerCallBack));  // 5Hz

#if defined(INCLUDE_IMU)
  RCCHECK(rclc_timer_init_default(&imuPublisherTimer, &support,
    RCL_MS_TO_NS(200), imuPublisher_timerCallBack));  // 5Hz
#endif
  RCCHECK(rclc_timer_init_default(&statusPublisherTimer, &support,
    RCL_MS_TO_NS(1000), statusPublisher_timerCallBack));  // 1Hz
#if defined(INCLUDE_LIDAR)
  RCCHECK(rclc_timer_init_default(&lidarPublisherTimer, &support,
    RCL_MS_TO_NS(200), lidarPublisher_timerCallBack));  // 5Hz
#endif
  RCCHECK(rclc_timer_init_default(&jointstatePublisherTimer, &support,
    RCL_MS_TO_NS(200), jointstatePublisher_timerCallBack));  // 5Hz
#endif
  // Setup executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &reset_msg, &reset_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &motorControlTimer));
#if 1
  RCCHECK(rclc_executor_add_timer(&executor, &PublisherTimer));
#else
  RCCHECK(rclc_executor_add_timer(&executor, &odomPublisherTimer));
#if defined(INCLUDE_IMU)
  RCCHECK(rclc_executor_add_timer(&executor, &imuPublisherTimer));
#endif
  RCCHECK(rclc_executor_add_timer(&executor, &statusPublisherTimer));
#if defined(INCLUDE_LIDAR)
//  RCCHECK(rclc_executor_add_timer(&executor, &lidarPublisherTimer));
#endif
  RCCHECK(rclc_executor_add_timer(&executor, &jointstatePublisherTimer)); 
#endif
  // Start lidar
#if defined(INCLUDE_LIDAR)
#ifdef RPLIDAR_H
  lidar->startScan(EXPRESS_LIDAR_MODE, DEFAULT_LIDAR_MOTOR_PWM);
#else
  lidar->startScan(DEFAULT_LIDAR_MOTOR_PWM);
#endif
#endif
  
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n");
  buzzer->welcomeTune();

}

/**
 * Main loop - spins ROS2 executor
 */
void loop() {

#if defined(WIFI)
  if(!wifiUp){
    delay(1000);
    return;
  }
#endif

  vTaskDelay(20);
  buzzer->update();
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Poll reset button - edge detection
  static bool prev_reset_state = HIGH;
  bool current_reset_state = digitalRead(UCP_RESET_PIN);
  if(prev_reset_state == HIGH && current_reset_state == LOW){
    reset_button_hit();
  }
  prev_reset_state = current_reset_state;

  // Poll motors button - edge detection
  static bool prev_motors_state = HIGH;
  bool current_motors_state = digitalRead(UCP_MOTORS_PIN);
  if(prev_motors_state == HIGH && current_motors_state == LOW){
    motors_button_hit();
  }
  prev_motors_state = current_motors_state;

  // Poll bumpers - edge detection 
  // Poll bumpers - edge detection 
#if defined(HANDLE_BUMPERS)
  static bool prev_front_bumper_state = HIGH;
  static bool prev_rear_bumper_state = HIGH;
  
  bool current_front_bumper_state = digitalRead(BUMPER_FRONT_PIN);
  bool current_rear_bumper_state = digitalRead(BUMPER_REAR_PIN);
  
  if((prev_front_bumper_state == HIGH && current_front_bumper_state == LOW) ||
     (prev_rear_bumper_state == HIGH && current_rear_bumper_state == LOW)){
    bumber_hit();   
  }
  
  prev_front_bumper_state = current_front_bumper_state;
  prev_rear_bumper_state = current_rear_bumper_state;
#endif

}