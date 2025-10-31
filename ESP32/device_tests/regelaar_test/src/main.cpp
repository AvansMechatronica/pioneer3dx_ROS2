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



rcl_subscription_t subscriber;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
rcl_timer_t timer;
rcl_timer_t ControlTimer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;


//creating objects for right wheel and left wheel
MotorController wheel(PWM_CHANNEL_LEFT, L_PWM_PIN, L_DIR_PIN, L_ENCODER_PINA, L_ENCODER_PINB);

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


//function which controlles the motor, callled every 10ms
void MotorControll_timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity;
  float angularVelocity;
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity = twist_msg.linear.x;
  angularVelocity = twist_msg.angular.z;
  //linear and angular velocities are converted to wheel and rightwheel velocities
  float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = wheel.pid(vL);
  if (vL == 0) {
    wheel.stop();
    actuating_signal_LW = 0;
  } else {
    wheel.moveBase(actuating_signal_LW);
  }
}



//interrupt function for left wheel encoder.
void updateEncoder() {
  if (digitalRead(wheel.EncoderPinA) == digitalRead(wheel.EncoderPinB))
    wheel.EncoderCount.data--;
  else
    wheel.EncoderCount.data++;
  encodervalue_l = wheel.EncoderCount;
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

void setup() {
  // Configure serial transport
  Serial.begin(115200);


  //initializing the pid constants
  wheel.setPIDvalues(KP_L, KI_L, KD_L);

  //initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(wheel.EncoderPinB), updateEncoder, RISING);

  Serial.begin(115200);

  allocator = rcl_get_default_allocator();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


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
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

