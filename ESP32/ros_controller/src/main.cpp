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
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

//#include <p3dx_interfaces/msg/bumper_state.h>


#include "motor_controller.h"
#include "pins.h"


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
#define KP_L        (float)1.8
#define KI_L        (float)5
#define KD_L        (float)0.1
//pid constants of right wheel
#define KP_R        (float)1.8
#define KI_R        (float)5
#define KD_R        (float)0.1

//pwm parameters setup
//#define PWM_FRQUENCY 30000
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
//#define PWM_RESOLUTION 8
//#define MAX_PWM ((2^PWM_RESOLUTION)-1)



rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t reset_subscriber;

geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t odom_publisher;
rcl_publisher_t error_publisher;
rcl_publisher_t battery_voltage_publisher;
//rcl_publisher_t bumper_state_publisher;
std_msgs__msg__Int32 encodervalue_l_msg;
std_msgs__msg__Int32 encodervalue_r_msg;
nav_msgs__msg__Odometry odom_msg;

std_msgs__msg__Bool error_msg;
std_msgs__msg__Bool reset_msg;
std_msgs__msg__Float32 battery_voltage_msg;
//p3dx_interfaces_msg_bumper_state bumper_state;


rcl_timer_t timer;
rcl_timer_t ControlTimer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;


//creating objects for right wheel and left wheel
MotorController leftWheel(PWM_CHANNEL_LEFT, L_PWM_PIN, L_DIR_PIN, L_ENCODER_PINA, L_ENCODER_PINB);
MotorController rightWheel(PWM_CHANNEL_RIGHT, R_PWM_PIN, R_DIR_PIN, R_ENCODER_PINA, R_ENCODER_PINB);

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


void cmd_vel_subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
}

void reset_subscription_callback(const void* msgin) {
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

//function which publishes wheel odometry.
void publishData() {
  // odometry data publish
  odom_msg = odometry.getData();

  struct timespec time_stamp = getTime();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  // error data publish
  error_msg.data = false;
  RCSOFTCHECK(rcl_publish(&error_publisher, &error_msg, NULL));

  // battery voltage publish
  float battery_voltage = 0;//analogRead(BATTERY_VOLTAGE_PIN) * (5.0 / 1023.0) * ((R1 + R2) / R2);  
  battery_voltage_msg.data = battery_voltage;
  RCSOFTCHECK(rcl_publish(&battery_voltage_publisher, &battery_voltage_msg, NULL));

}


//function which controlles the motor, callled every 10ms
void MotorControll_timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity;
  float angularVelocity;
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity = twist_msg.linear.x;
  angularVelocity = twist_msg.angular.z;
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
  float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;
  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = leftWheel.pid(vL);
  float actuating_signal_RW = rightWheel.pid(vR);
  if (vL == 0 && vR == 0) {
    leftWheel.stop();
    rightWheel.stop();
    actuating_signal_LW = 0;
    actuating_signal_RW = 0;
  } else {
    rightWheel.moveBase(actuating_signal_RW);
    leftWheel.moveBase(actuating_signal_LW);
  }
  //odometry
  //current wheel rpm is calculated
  float currentRpmL = leftWheel.getRpm();
  float currentRpmR = rightWheel.getRpm();
  float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0;  // RPM
  float linear_x = average_rps_x * WHEELS_CIRCUMFERENCE;                  // m/s
  float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
  float angular_z = (average_rps_a * WHEELS_CIRCUMFERENCE) / (WHEELS_Y_DISTANCE / 2.0);  //  rad/s
  float linear_y = 0;
  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(
    vel_dt,
    linear_x,
    linear_y,
    angular_z);
  publishData();
}

//interrupt function for left wheel encoder.
void updateEncoderL() {
  if (digitalRead(leftWheel.EncoderPinA) == digitalRead(leftWheel.EncoderPinB))
    leftWheel.EncoderCount.data--;
  else
    leftWheel.EncoderCount.data++;
  encodervalue_l_msg = leftWheel.EncoderCount;
}

//interrupt function for right wheel encoder
void updateEncoderR() {
  if (digitalRead(rightWheel.EncoderPinA) == digitalRead(rightWheel.EncoderPinB))
    rightWheel.EncoderCount.data++;
  else
    rightWheel.EncoderCount.data--;
  encodervalue_r_msg = rightWheel.EncoderCount;
}

void syncTime() {
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}



#define NODE_NAME "p3dx_controller"


bool errorLedState = false;



#if defined(WIFI)
  String wifiSSID = SSID;
  String wifiPass = SSID_PASSWORD;
#endif

void setup() {
  // Configure serial transport
  Serial.begin(115200);


  //initializing the pid constants
  leftWheel.setPIDvalues(KP_L, KI_L, KD_L);
  rightWheel.setPIDvalues(KP_R, KI_R, KD_R);

  //initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinA), updateEncoderR, RISING);

#if defined(WIFI)


  WiFi.setHostname("p3dx_controller");

  bool force_network_configure;
  force_network_configure = !digitalRead(SELECT_WIFI_CONFIG_MODE_PIN);

  NETWORK_CONFIG networkConfig;
  wifiUp = configureNetwork(force_network_configure, &networkConfig);
  if(!wifiUp){
    tft_printf(ST77XX_MAGENTA, "Error configuring\nWiFi\n");

  };

  set_microros_wifi_transports(wifiSSID, wifiPass, AGENT_IP_ADDRESS, (size_t)PORT);
#else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
#endif

  allocator = rcl_get_default_allocator();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create cmd_vel_subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create reset_subscriber for reset topic
  RCCHECK(rclc_subscription_init_default(
    &reset_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "reset"));
  
  //create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered"));

  //create a error publisher
  RCCHECK(rclc_publisher_init_default(
    &error_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "error"));

  //create a battery_voltage publisher
  RCCHECK(rclc_publisher_init_default(
    &battery_voltage_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "battery_voltage"));

  //timer function for controlling the motor base. At every samplingT time
  //MotorControll_timerCallback function is called
  //Here I had set SamplingT=10 Which means at every 10 milliseconds MotorControll_timerCallback function is called
  const unsigned int samplingT = 20;
  RCCHECK(rclc_timer_init_default(
    &ControlTimer,
    &support,
    RCL_MS_TO_NS(samplingT),
    MotorControll_timerCallback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmd_vel_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &reset_msg, &reset_subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

