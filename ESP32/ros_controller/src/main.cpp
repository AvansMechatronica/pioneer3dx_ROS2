#include "main.h"
#include <rmw_microros/time_sync.h>

#ifdef RPLIDAR_H
#define EXPRESS_LIDAR_MODE  false
#endif

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
//rcl_timer_t motorControlTimer;         // Controls motor PID loop
rcl_timer_t odomPublisherTimer;        // Publishes odometry
rcl_timer_t jointstatePublisherTimer;  // Publishes joint states
rcl_timer_t statusPublisherTimer;      // Publishes system status
#if defined(INCLUDE_LIDAR)
rcl_timer_t lidarPublisherTimer;      // Publishes lidar data
#endif


TaskHandle_t motorcontrolTaskHandle;
TaskHandle_t pollingTaskHandle;

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

  pinMode(UCP_MOTORS_PIN, INPUT_PULLUP);
  // Use polling instead of interrupt to avoid accidental motor enables
  //attachInterrupt(digitalPinToInterrupt(UCP_MOTORS_PIN), motors_button_hit, FALLING);

  pinMode(RPLIDAR_MOTOR_PIN, OUTPUT);
  digitalWrite(RPLIDAR_MOTOR_PIN, LOW);

  buzzer = new Buzzer(UCP_BUZZER_PIN, UCP_BUZZER_PWM_CHANNEL);

#if defined(WIFI)
  const char *host_name = convertToCamelCase(NODE_NAME);
  DEBUG_PRINT("hostname :%s\n", host_name);
  WiFi.setHostname(host_name);
  
  // UCP_MOTORS_PIN low to force WiFi configuration mode
  bool force_configure_wifi = digitalRead(UCP_MOTORS_PIN) == LOW;

  NETWORK_CONFIG networkConfig;
  wifiUp = configureNetwork(force_configure_wifi, &networkConfig);
  if(!wifiUp){
    tft_printf(ST77XX_MAGENTA, "Error configuring\nWiFi\nRestarting...\n");
    delay(5000);
    ESP.restart();
  };

  set_microros_wifi_transports(const_cast<char*>(networkConfig.ssid.c_str()), 
                               const_cast<char*>(networkConfig.password.c_str()), 
                               networkConfig.microros_agent_ip_address,
                               networkConfig.microros_agent_port);
  tft_printf(ST77XX_MAGENTA, "WiFi Connected\n");
  delay(2000);
#else // WIFI
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
#endif // 
  // Initialize ADC for battery voltage reading
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReadResolution(10);


  tft_printf(ST77XX_MAGENTA, "Pioneer 3DX\nController\nStarted\n");

  // Initialize motor controllers
  leftWheel = new MotorController(PWM_CHANNEL_LEFT, L_PWM_PIN, L_DIR_PIN, &encodervalue_l, WHEELS_RADIUS);
  rightWheel = new MotorController(PWM_CHANNEL_RIGHT, R_PWM_PIN, R_DIR_PIN, &encodervalue_r, WHEELS_RADIUS);

  // Set PID parameters
  leftWheel->setPIDvalues(KP_L, KI_L, KD_L);
  rightWheel->setPIDvalues(KP_R, KI_R, KD_R);

  pinMode(L_ENCODER_PINA, INPUT);
  pinMode(L_ENCODER_PINB, INPUT);
  pinMode(R_ENCODER_PINA, INPUT);
  pinMode(R_ENCODER_PINB, INPUT);
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PINB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_PINB), updateEncoderR, RISING);

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
#else // HANDLE_BUMPERS
  status_msg.error = false;
#endif // HANDLE_BUMPERS

  // Ensure motors are disabled
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
  leftWheel->disable();
  rightWheel->disable();
  motors_enabled = false;

  delay(2000);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();

  rcl_ret_t support_rc = rclc_support_init(&support, 0, NULL, &allocator);
  if(support_rc != RCL_RET_OK){
    Serial.printf("micro-ROS init failed rc=%d, free_heap=%u, free_psram=%u\n",
                  (int)support_rc,
                  ESP.getFreeHeap(),
                  ESP.getFreePsram());
    tft_printf(ST77XX_BLUE, "microROS init\nfailed rc=%d\nRestarting...\n", (int)support_rc);
    delay(3000);
    ESP.restart();
  }


  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  rmw_ret_t time_sync_rc = rmw_uros_sync_session(1000);
  if (time_sync_rc == RMW_RET_OK && rmw_uros_epoch_synchronized()) {
    DEBUG_PRINT("micro-ROS time synchronized\n");
  } else {
    DEBUG_PRINT("micro-ROS time sync failed rc=%d, fallback to millis timestamps\n", (int)time_sync_rc);
  }

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
#else // RPLIDAR_H
  lidar = new lds08_lidar(&node, RPLIDAR_COM_PORT, RPLIDAR_TX_PIN, RPLIDAR_RX_PIN, RPLIDAR_MOTOR_PIN);
  Serial.println("LDS08 Lidar - Setup gestart");
#endif // RPLIDAR_H
#endif // INCLUDE_LIDAR

  odometry = new Odometry(&node);
  jointstate = new Jointstate(&node);

#if defined(INCLUDE_IMU)
  imu = new imu_mpu6050(&node, I2C_SCL_PIN, I2C_SDA_PIN, IMU_INT_PIN);
#endif // INCLUDE_IMU

  // Initialize timers and executor

  int executer_count = 2; // cmd_vel + reset subscriptions

  // Initialize timers
  RCCHECK(rclc_timer_init_default(&statusPublisherTimer, &support,
    RCL_MS_TO_NS(1000), statusPublisher_timerCallBack));  // 1Hz
  executer_count++;

  RCCHECK(rclc_timer_init_default(&odomPublisherTimer, &support,
    RCL_MS_TO_NS(100), odomPublisher_timerCallBack));  // 5Hz, toggle between odometry and IMU publishing
  executer_count++;


#if defined(INCLUDE_LIDAR)
  RCCHECK(rclc_timer_init_default(&lidarPublisherTimer, &support,
    RCL_MS_TO_NS(200), lidarPublisher_timerCallBack));  // 5Hz
  executer_count++;
#endif // INCLUDE_LIDAR

  RCCHECK(rclc_timer_init_default(&jointstatePublisherTimer, &support,
    RCL_MS_TO_NS(200), jointstatePublisher_timerCallBack));  // 5Hz
  executer_count++;

  // Setup executor
  RCCHECK(rclc_executor_init(&executor, &support.context, executer_count, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &reset_msg, &reset_subscription_callback, ON_NEW_DATA));
//  RCCHECK(rclc_executor_add_timer(&executor, &motorControlTimer));

  RCCHECK(rclc_executor_add_timer(&executor, &statusPublisherTimer));
 
  RCCHECK(rclc_executor_add_timer(&executor, &jointstatePublisherTimer)); 

  RCCHECK(rclc_executor_add_timer(&executor, &odomPublisherTimer));

  // Start lidar
#if defined(INCLUDE_LIDAR)
  RCCHECK(rclc_executor_add_timer(&executor, &lidarPublisherTimer));
#endif // INCLUDE_LIDAR


#if defined(INCLUDE_LIDAR)
#ifdef RPLIDAR_H
  lidar->startScan(EXPRESS_LIDAR_MODE, DEFAULT_LIDAR_MOTOR_PWM);
#else // RPLIDAR_H
  lidar->startScan(DEFAULT_LIDAR_MOTOR_PWM);
  tft_printf(ST77XX_MAGENTA, "LDS08 Lidar\nWait for sync\n");
  while(!lidar->isSyncronized()){
    delay(100);
  }
  tft_printf(ST77XX_MAGENTA, "LDS08 Lidar\nSyncronized\n");
  delay(2000);
#endif // RPLIDAR_H
#endif // INCLUDE_LIDAR

  // Create FreeRTOS task for scanning
  const uint16_t stackSize = 8192; // Stack size in bytes
  const UBaseType_t priority = 1;   // Task priority
  BaseType_t result = xTaskCreate(
      motorcontrolTaskFunction,       // Task function
      "Motor_Control",          // Task name
      stackSize,             // Stack size
      nullptr,               // Task parameter (no context)
      priority,              // Priority
      &motorcontrolTaskHandle        // Task handle
  );
  if (result != pdPASS) {
      tft_printf(ST77XX_MAGENTA, "Error\ncreating\nmotor control\ntask\n");
      delay(5000);
      // Handle task creation error appropriately
  }  
  

  // Create FreeRTOS task for polling buttons and bumpers
  const uint16_t pollingStackSize = 4096; // Stack size in bytes
  const UBaseType_t pollingPriority = 1;   // Task priority
  result = xTaskCreate(
      pollingTaskFunction,       // Task function
      "Polling_Task",          // Task name
      pollingStackSize,             // Stack size
      nullptr,               // Task parameter (no context)
      pollingPriority,              // Priority
      nullptr        // Task handle
  );
  if (result != pdPASS) {
      tft_printf(ST77XX_MAGENTA, "Error\ncreating\npolling\ntask\n");
      delay(5000);
      // Handle task creation error appropriately
  } 
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n");
  buzzer->welcomeTune();

}

/**
 * Main loop - spins ROS2 executor
 */
void loop() {

  vTaskDelay(1 / portTICK_PERIOD_MS); // Allow other tasks to run
  buzzer->update();
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
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

#if defined(HANDLE_BUMPERS)
  if(digitalRead(BUMPER_FRONT_PIN) == HIGH && digitalRead(BUMPER_REAR_PIN) == HIGH){
    digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
    status_msg.error = false; 
    delay(1500);
    tft_printf(ST77XX_MAGENTA, "Reset Done\nMotors Enabled\n");
  }
  else{
    tft_printf(ST77XX_RED, "Cannot Reset!\nBumpers\nNot Clear\n");
    delay(1500);
  }
#else
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
  status_msg.error = false; 
  delay(1500);
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
  status_msg.battery_voltage = adc_voltage * ((R2 + R3) / R3) * 4; // Multiply by 4 to account for voltage divider scaling (since we want the actual battery voltage, not just the divided voltage)
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
bool odom_publish_toggle = false; // Toggle to alternate between odometry and imu publishing
void odomPublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){

    if(odom_publish_toggle){
      RCSOFTCHECK(odometry->publish());
    }
    else{
#if defined(INCLUDE_IMU)
      RCSOFTCHECK(imu->publish());
#endif
    }
    odom_publish_toggle = !odom_publish_toggle; // Alternate next time
  }
}



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
    RCSOFTCHECK(lidar->publish());
  }
}
#endif

void jointstatePublisher_timerCallBack(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    double left_wheel_rad_s = leftWheel->getVelocity() / WHEELS_RADIUS;
    double right_wheel_rad_s = rightWheel->getVelocity() / WHEELS_RADIUS;
    jointstate->update(left_wheel_rad_s, 
                  right_wheel_rad_s, 
                  encodervalue_l / (TICK_PER_REVOLUTION / (2.0 * M_PI)), 
                  encodervalue_r / (TICK_PER_REVOLUTION / (2.0 * M_PI)));
    RCSOFTCHECK(jointstate->publish());
  }
}

int display_interval_counter=0;  // Counter for display update rate limiting

/**
 * Motor control timer callback (50Hz)
 * Performs PID control, odometry calculation, and motor command execution
 */
//void MotorControll_timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
void motorcontrolTaskFunction(void* parameter){
  float linearVelocity;
  float angularVelocity;
//  RCLC_UNUSED(last_call_time);
//  if (timer != NULL){
  while(true){ 
    // Disable motors if no cmd_vel received for 500ms (safety timeout)
    if((millis() - prev_cmd_time) > 500) {
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

    // Update display every 25 cycles (500ms)
    if(display_interval_counter % 25 == 0 && motors_enabled){
      tft_printf(ST77XX_MAGENTA, "Linear: %.2f\nAngular: %.2f\nvL: %.2f\nvR: %.2f", linearVelocity, angularVelocity, vL, vR);
    }
    display_interval_counter++;

    // Send PWM commands to motors
    rightWheel->moveBase(actuating_signal_RW);
    leftWheel->moveBase(actuating_signal_LW);

    // Calculate actual wheel velocities and update odometry
    // getVelocity() already returns wheel linear speed in m/s.
    float currentVelL = leftWheel->getVelocity();
    float currentVelR = rightWheel->getVelocity();
    float linear_x = (-currentVelL + currentVelR) / 2.0f;   // m/s
    float angular_z = (currentVelL + currentVelR) / WHEELS_Y_DISTANCE;  // rad/s
    float linear_y = 0;

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;

    odometry->update(vel_dt, linear_x, linear_y, angular_z);
    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
  }
}

/**
 * Interrupt handler for left wheel encoder
 */
void updateEncoderL() {
  if (digitalRead(L_ENCODER_PINA) == digitalRead(L_ENCODER_PINB))
    encodervalue_l--;
  else
    encodervalue_l++;
}

/**
 * Interrupt handler for right wheel encoder
 */
void updateEncoderR() {
  if (digitalRead(R_ENCODER_PINA) == digitalRead(R_ENCODER_PINB))
    encodervalue_r--;
  else
    encodervalue_r++;
}



#if defined(HANDLE_BUMPERS)
/**
 * Interrupt handler for bumper collision
 */
void bumber_hit(){
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
  tft_printf(ST77XX_BLUE, "Bumper Hit!\nMotors Disabled\n");
  motors_enabled = false; 
  leftWheel->disable();
  rightWheel->disable();
  status_msg.error = true;
}
#endif

bool errorLedState = false;

void reset_button_hit(){
  DEBUG_PRINT("Reset Button Hit!\n");
  tft_printf(ST77XX_BLUE, "Reset Button\nHit!\n");
  delay(1500);
  reset_p3dx();
}

void motors_button_hit(){
  DEBUG_PRINT("Motors Button Hit!\n");
  tft_printf(ST77XX_BLUE, "Motors Button\nHit!\n");
}


void pollingTaskFunction(void* parameter){
  bool prev_reset_state = HIGH;
  bool prev_motors_state = HIGH;
#if defined(HANDLE_BUMPERS)
  bool prev_front_bumper_state = HIGH;
  bool prev_rear_bumper_state = HIGH;
#endif

  while(true){
    // Poll reset button - edge detection
    bool current_reset_state = digitalRead(UCP_RESET_PIN);
    if(prev_reset_state == HIGH && current_reset_state == LOW){
      reset_button_hit();
    }
    prev_reset_state = current_reset_state;

    // Poll motors button - edge detection
    bool current_motors_state = digitalRead(UCP_MOTORS_PIN);
    if(prev_motors_state == HIGH && current_motors_state == LOW){
      motors_button_hit();
    }
    prev_motors_state = current_motors_state;

#if defined(HANDLE_BUMPERS)
    // Poll bumpers - edge detection 
    bool current_front_bumper_state = digitalRead(BUMPER_FRONT_PIN);
    bool current_rear_bumper_state = digitalRead(BUMPER_REAR_PIN);
    
    if((prev_front_bumper_state == HIGH && current_front_bumper_state == LOW) ||
       (prev_rear_bumper_state == HIGH && current_rear_bumper_state == LOW)){
      bumber_hit();   
    }
    
    prev_front_bumper_state = current_front_bumper_state;
    prev_rear_bumper_state = current_rear_bumper_state;
#endif

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


