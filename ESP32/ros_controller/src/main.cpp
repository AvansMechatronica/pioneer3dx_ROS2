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
#include <lds08_lidar.h>
//#include <rplidar.h>
#if defined(INCLUDE_IMU)
#include <imu_mpu6050.h>
#endif

#if defined(HANDLE_BUMPERS)
#include <p3dx_interfaces/msg/bumpers.h>
#endif

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
#if defined(HANDLE_BUMPERS)
rcl_publisher_t bumpers_publisher;
#endif

// Published message objects
std_msgs__msg__Bool error_msg;
std_msgs__msg__Bool reset_msg;
std_msgs__msg__Float32 battery_voltage_msg;

#if defined(HANDLE_BUMPERS)
p3dx_interfaces__msg__Bumpers  bumper_msg;
#endif

// Encoder values
int64_t encodervalue_l = 0;  // Left wheel encoder count
int64_t encodervalue_r = 0;  // Right wheel encoder count
bool motors_enabled = false;  // Motor enable state

// Previous RPM values for velocity tracking
float prev_rpm_l;
float prev_rpm_r;

// ROS2 timers
rcl_timer_t timer;
rcl_timer_t motorControlTimer;         // Controls motor PID loop
rcl_timer_t lowSpeedPublisherTimer;    // Publishes slow-rate data (battery, errors)
rcl_timer_t highSpeedPublisherTimer;   // Publishes fast-rate data (odometry, lidar)

// Time synchronization
unsigned long long time_offset = 0;  // Offset between ESP32 and ROS time
unsigned long prev_cmd_time = 0;     // Last received cmd_vel timestamp
unsigned long prev_odom_update = 0;  // Last odometry update timestamp

// Robot component objects
Odometry *odometry;
Jointstate *jointstate;

#ifdef RPLIDAR_H
rplidar *lidar;
#else
lds08_lidar *lidar;
#endif

#if defined(INCLUDE_IMU)
imu_mpu6050 *imu;
#endif

// Motor controller objects
MotorController *leftWheel;
MotorController *rightWheel;

// Display objects
SPIClass *spiClass;
Adafruit_ST7735 *tft;

// Macro to check ROS return codes and trigger error handler on failure
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { Serial.printf("Fatal Error, line %i, ", __LINE__); microros_error_handler(__LINE__); } \
  }

// Soft check version (same as RCCHECK in this implementation)
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { Serial.printf("Fatal Error, line %i, ", __LINE__);microros_error_handler(__LINE__); } \
  }

/**
 * Error handler that stops the lidar, displays error, and restarts ESP32
 */
void microros_error_handler(int line) {
    lidar->stopScan();
    Serial.printf("Restarting...\n");
    tft_printf(ST77XX_BLUE, "Fatal Error\nRestarting...\n,Line: %d", line);
    delay(5000);
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
  if(error_msg.data == false){
    if(!motors_enabled){
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
      leftWheel->enable();
      rightWheel->enable();
      motors_enabled = true;
      tft_printf(ST77XX_MAGENTA, "Motors Enabled\n");
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
    }
  }
  twist_msg = *msg;  // Copy velocity command
}

/**
 * Reset error state by checking both bumpers are pressed
 */
void reset_by_button(){
#if defined(HANDLE_BUMPERS)
    if(digitalRead(BUMPER_FRONT_PIN)==HIGH &&
       digitalRead(BUMPER_REAR_PIN)==HIGH){
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
      error_msg.data = false;
    }
#endif
}

/**
 * Callback for reset topic - clears error state
 */
void reset_subscription_callback(const void* msgin) {
  const std_msgs__msg__Bool* msg = 
      static_cast<const std_msgs__msg__Bool*>(msgin);
  if (msg->data == true) {
#if defined(HANDLE_BUMPERS)
  reset_by_button();
#else
    digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
    error_msg.data = false; 
#endif
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

/**
 * Publish error state
 */
void errorPublisher() {
    RCSOFTCHECK(rcl_publish(&error_publisher, &error_msg, NULL));
}

// Voltage divider resistor values
#define R1 330000.0f // Resistor R1 value in ohms
#define R2 100000.0f // Resistor R2 value in ohms

/**
 * Read and publish battery voltage
 */
void batteryPublisher(){
  float battery_voltage = analogRead(BATTERY_VOLTAGE_PIN) * (3.3 / 1023.0);  
  battery_voltage_msg.data = battery_voltage * 3.1 * 2; // Compensate for voltage divider
  RCSOFTCHECK(rcl_publish(&battery_voltage_publisher, &battery_voltage_msg, NULL));
}

#if defined(HANDLE_BUMPERS)
/**
 * Read and publish bumper states
 */
void bumpersPunblisher(){
  bumper_msg.front = !digitalRead(BUMPER_FRONT_PIN);  // Active low
  bumper_msg.rear = !digitalRead(BUMPER_REAR_PIN);    // Active low
  RCSOFTCHECK(rcl_publish(&bumpers_publisher, &bumper_msg, NULL));
}
#endif

/**
 * Timer callback for low-frequency publishers (1Hz)
 * Publishes battery voltage, error state, and bumper state
 */
void lowSpeedPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  batteryPublisher();
  errorPublisher();
#if defined(HANDLE_BUMPERS)
  bumpersPunblisher();
#endif
}

/**
 * Timer callback for high-frequency publishers (10Hz)
 * Publishes odometry, joint states, lidar data, and IMU data
 */
void highSpeedPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCSOFTCHECK(odometry->publish());
  jointstate->update(leftWheel->getVelocity(), 
                     rightWheel->getVelocity(), 
                     encodervalue_l / (TICK_PER_REVOLUTION / (2.0 * M_PI)), 
                     encodervalue_r / (TICK_PER_REVOLUTION / (2.0 * M_PI)));
  RCSOFTCHECK(lidar->publish());
  RCSOFTCHECK(jointstate->publish());
#if defined(INCLUDE_IMU)
  RCSOFTCHECK(imu->publish());
#endif
}

int display_interval_counter=0;  // Counter for display update rate limiting

/**
 * Motor control timer callback (50Hz)
 * Performs PID control, odometry calculation, and motor command execution
 */
void MotorControll_timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity;
  float angularVelocity;

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
  error_msg.data = true;
}
#endif

#define NODE_NAME "p3dx_controller"

bool errorLedState = false;

/**
 * Initialize SPI display
 */
void init_display(){
  SPIClass *spiClass = new SPIClass(HSPI);
  spiClass->begin(DISPLAY_CLK_PIN, -1, DISPLAY_SDA_PIN, DISPLAY_CS_PIN);

  tft = new Adafruit_ST7735(spiClass, DISPLAY_CS_PIN, DISPLAY_RS_DC_PIN, DISPLAY_RST_PIN);
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

  Serial.printf("Ready"); 
}

/**
 * Convert snake_case to camelCase
 */
char* convertToCamelCase(const char *input) {
    int i, j;
    int len = strlen(input);
    char *output = (char *)malloc((len + 1) * sizeof(char));
    
    if(output == NULL) {
        Serial.printf("Error allocating memory\n");
        microros_error_handler(__LINE__);
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



/**
 * Setup function - initializes hardware and ROS2 components
 */
void setup() {
  Serial.begin(115200);
  delay(500);
#ifdef RPLIDAR_H
  Serial.println("RPlidar A1M8 - Setup gestart");
#else
  Serial.println("LDS08 Lidar - Setup gestart");
#endif

  // Initialize ADC for battery voltage reading
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReadResolution(10);

  init_display();
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

#if defined(WIFI)
  WiFi.setHostname("p3dx_controller");
  
  NETWORK_CONFIG networkConfig;
  bool wifiUp = configureNetwork(false, &networkConfig);
  if(!wifiUp){
    tft_printf(ST77XX_MAGENTA, "Error configuring\nWiFi\n");
  };

  const char *host_name = convertToCamelCase(NODE_NAME);
  Serial.printf("hostname :%s\n", host_name);
  WiFi.setHostname(NODE_NAME);
  WiFi.setHostname("P3dxController");
  
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

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);

#if defined(HANDLE_BUMPERS)
  // Setup bumper interrupts
  pinMode(BUMPER_FRONT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUMPER_FRONT_PIN), bumber_hit, FALLING);
  pinMode(BUMPER_REAR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUMPER_REAR_PIN), bumber_hit, FALLING);
  
  // Check initial bumper state
  if(digitalRead(BUMPER_FRONT_PIN)==HIGH && digitalRead(BUMPER_REAR_PIN)==HIGH){
    error_msg.data = false;
  }
  else{
    error_msg.data = true;
  }
#else
  error_msg.data = false;
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "reset"));

  // Initialize publishers
#if defined(HANDLE_BUMPERS)
  RCCHECK(rclc_publisher_init_default(&bumpers_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(p3dx_interfaces, msg, Bumpers), "bumpers"));
#endif  

  RCCHECK(rclc_publisher_init_default(&error_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "error"));

  RCCHECK(rclc_publisher_init_default(&battery_voltage_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "battery_voltage"));
    
  // Initialize sensors
#ifdef RPLIDAR_H
  lidar = new rplidar(&node, RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN, RPLIDAR_MOTOR_PIN);
#else
  lidar = new lds08_lidar(&node, RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN, RPLIDAR_MOTOR_PIN);
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

  RCCHECK(rclc_timer_init_default(&lowSpeedPublisherTimer, &support,
    RCL_MS_TO_NS(1000), lowSpeedPublisher_timerCallBack));  // 1Hz

  RCCHECK(rclc_timer_init_default(&highSpeedPublisherTimer, &support,
    RCL_MS_TO_NS(100), highSpeedPublisher_timerCallBack));  // 10Hz

  // Setup executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &reset_msg, &reset_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &motorControlTimer));
  RCCHECK(rclc_executor_add_timer(&executor, &lowSpeedPublisherTimer));
  RCCHECK(rclc_executor_add_timer(&executor, &highSpeedPublisherTimer));
#if defined(HANDLE_BUMPERS)
  RCCHECK(rclc_executor_add_timer(&executor, &lowSpeedPublisherTimer));
#endif

  // Start lidar
#ifdef RPLIDAR_H
  lidar->startScan(EXPRESS_LIDAR_MODE, DEFAULT_LIDAR_MOTOR_PWM);
#else
  lidar->startScan(DEFAULT_LIDAR_MOTOR_PWM);
#endif
  
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n");
}

/**
 * Main loop - spins ROS2 executor
 */
void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
