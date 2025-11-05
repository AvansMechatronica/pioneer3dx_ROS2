#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "rosidl_runtime_c/string_functions.h"  // Header for string assignment functions

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "motor_controller.h"
#include "pins.h"

#define R_CHANNEL         1

#if defined(R_CHANNEL)
#undef L_PWM_PIN        
#undef L_DIR_PIN        
#undef L_ENCODER_PINA   
#undef L_ENCODER_PINB

//#undef CLOCK_WISE
//#undef COUNTER_CLOCK_WISE

#define L_PWM_PIN        R_PWM_PIN
#define L_DIR_PIN        R_DIR_PIN
#define L_ENCODER_PINA   R_ENCODER_PINA
#define L_ENCODER_PINB   R_ENCODER_PINB

//#define CLOCK_WISE            HIGH
//#define COUNTER_CLOCK_WISE    LOW

#endif

#define TICK_PER_REVOLUTION  19150  //encoder tick per revolution

// No used
//#define L_ENA_PIN        25
//#define R_ENA_PIN         5

//parameters of the robot
#define WHEELS_Y_DISTANCE       (float)0.34 //in meters
#define WHEELS_RADIUS           (float)0.09765 //in meters
#define WHEELS_CIRCUMFERENCE    (2 * 3.14 * WHEELS_RADIUS)
//encoder value per revolution of left wheel and right wheel

#define CLOCK_WISE            HIGH
#define COUNTER_CLOCK_WISE    LOW

#define THRESHOLD   0
//pid constants of left wheel
#define KP_L        (float)30
#define KI_L        (float)80
#define KD_L        (float)0.1


//pwm parameters setup
//#define PWM_FRQUENCY 30000
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
//#define PWM_RESOLUTION 8
//#define MAX_PWM ((2^PWM_RESOLUTION)-1)



rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t ControlTimer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

int64_t encodervalue_l;

float setpoint = 0.0;

//creating objects for right wheel and left wheel
MotorController wheel(PWM_CHANNEL_LEFT, L_PWM_PIN, L_DIR_PIN, L_ENCODER_PINA, L_ENCODER_PINB, &encodervalue_l, WHEELS_RADIUS);

#define LED_PIN 2
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
}

struct timespec getTime() {
  struct timespec tp = { 0 };
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}


//interrupt function for left wheel encoder.
void updateEncoder() {
  if (digitalRead(wheel.EncoderPinA) == digitalRead(wheel.EncoderPinB))
    encodervalue_l--;
  else
    encodervalue_l++;
}

void syncTime() {
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}


bool errorLedState = false;

void executerTask(void *pvParameters);
void inputTask(void *pvParameters);


void setup() {
  // Configure serial transport
  Serial.begin(115200);


  //initializing the pid constants
  wheel.setPIDvalues(KP_L, KI_L, KD_L);
  wheel.enable();

  //initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(wheel.EncoderPinB), updateEncoder, RISING);

  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);
   // Create Task 1
  xTaskCreate(
    executerTask,                // Function that implements the task
    "executerTask",              // Name of the task (for debugging)
    2048,                 // Stack size in words (not bytes)
    NULL,                 // Task input parameter
    1,                    // Priority (higher = more important)
    NULL                  // Task handle
  );

  // Create Task 2
  xTaskCreatePinnedToCore(
    inputTask,                // Function that implements the task
    "inputTask",              // Name of the task
    2048,                 // Stack size
    NULL,                 // Parameter
    1,                    // Priority
    NULL,                 // Task handle
    1                     // Core ID (0 or 1)
  );
}

void loop() {
    delay(100);
  // put your main code here, to run repeatedly:
}

// Define Task 1
void executerTask(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay 10 milli seconds
    //pid controlled is used for generating the pwm signal
    float actuating_signal_LW = wheel.pid(setpoint);
    wheel.moveBase(actuating_signal_LW);
  }
}


#define V_MAX 0.8 // m/s
// Define Task 2
void inputTask(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    vTaskDelay(1500 / portTICK_PERIOD_MS);  // Delay 1.5 seconds


    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      int value = input.toInt();

      if (value < -100) value = -100;
      if (value > 100) value = 100;
      setpoint = (float)value/100.0 * V_MAX; // convert to ticks/s
      prev_cmd_time = millis();

      Serial.printf("Invoer: %d\n",  value);
    }
  }
}



