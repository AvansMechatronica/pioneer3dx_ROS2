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
#include <sensor_msgs/msg/joint_state.h>

#if defined(HANDLE_BUMPERS)
#include <p3dx_interfaces/msg/bumpers.h>
#endif

#include "wifi_network_config.h"

#include "motor_controller.h"
#include "pins.h"

#include <Adafruit_GFX.h> // Core graphics library
#include <Fonts/FreeSansBold9pt7b.h>
//#include <Fonts/Tiny3x3a2pt7b.h>
#include <Adafruit_ST7735.h> // Hardware-specific library
//#include <SPI.h>
#include "tft_printf.h"

#define TICK_PER_REVOLUTION  19150  //encoder tick per revolution

//ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

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
//pid constants of right wheel
#define KP_R        (float)30
#define KI_R       (float)80
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
rcl_publisher_t joint_state_publisher;
rcl_publisher_t error_publisher;
rcl_publisher_t battery_voltage_publisher;
#if defined(HANDLE_BUMPERS)
rcl_publisher_t bumpers_publisher;
#endif

nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Bool error_msg;
std_msgs__msg__Bool reset_msg;
std_msgs__msg__Float32 battery_voltage_msg;
sensor_msgs__msg__JointState joint_state_msg;
#if defined(HANDLE_BUMPERS)
p3dx_interfaces__msg__Bumpers  bumper_msg;
#endif
int64_t encodervalue_l = 0;
int64_t encodervalue_r = 0;
bool motors_enabled = false;


float prev_rpm_l;
float prev_rpm_r;

rcl_timer_t timer;
rcl_timer_t ControlTimer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;


//creating objects for right wheel and left wheel
MotorController leftWheel(PWM_CHANNEL_LEFT, L_PWM_PIN, L_DIR_PIN, L_ENCODER_PINA, L_ENCODER_PINB, &encodervalue_l, WHEELS_RADIUS);
MotorController rightWheel(PWM_CHANNEL_RIGHT, R_PWM_PIN, R_DIR_PIN, R_ENCODER_PINA, R_ENCODER_PINB, &encodervalue_r, WHEELS_RADIUS);

Adafruit_ST7735 *tft;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { microros_error_handler(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { microros_error_handler(); } \
  }

void microros_error_handler() {
    tft_printf(ST77XX_BLUE, "Fatal Error\n\nRestarting...\n");
    delay(3000);
    ESP.restart();
}


void cmd_vel_subscription_callback(const void* msgin) {
  prev_cmd_time = millis();

  const geometry_msgs__msg__Twist* msg = 
      static_cast<const geometry_msgs__msg__Twist*>(msgin);

  if(error_msg.data == false){
    if(!motors_enabled){
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
      leftWheel.enable();
      rightWheel.enable();
      motors_enabled = true;
      tft_printf(ST77XX_MAGENTA, "Motors Enabled\n");
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
    }
  }
  twist_msg = *msg;  //  copy the data
}

void reset_by_button(){
#if defined(HANDLE_BUMPERS)
    if(digitalRead(BUMPER_FRONT_RIGHT_PIN)==HIGH &&
      digitalRead(BUMPER_FRONT_LEFT_PIN)==HIGH  &&
      digitalRead(BUMPER_REAR_RIGHT_PIN)==HIGH  &&
      digitalRead(BUMPER_REAR_LEFT_PIN)==HIGH){
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_ENABLE);
      error_msg.data = false;
    }
#endif
}

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
#if 0
  odom_msg = odometry.getData();

  struct timespec time_stamp = getTime();
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
#endif
  // error data publish
  RCSOFTCHECK(rcl_publish(&error_publisher, &error_msg, NULL));

#define R1 330000.0f // Resistor R1 value in ohms
#define R2 100000.0f  // Resistor R2 value in ohms

  // battery voltage publish

  float battery_voltage = analogRead(BATTERY_VOLTAGE_PIN) * (3.3 / 1023.0);  
  battery_voltage_msg.data = battery_voltage * 3.1 * 2; //3.1 is factor determined by P3dx voltage divider, R1 = R2 = 100k ohm
  RCSOFTCHECK(rcl_publish(&battery_voltage_publisher, &battery_voltage_msg, NULL));
#if 0

  float current_rpm_l = leftWheel.getVelocity();
  float current_rpm_r = rightWheel.getVelocity();

  double pos_left = encodervalue_l / (TICK_PER_REVOLUTION / (2.0 * M_PI));
  double pos_right = encodervalue_r / (TICK_PER_REVOLUTION / (2.0 * M_PI));
  // Update data

  joint_state_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_millis() / 1000);
  joint_state_msg.header.stamp.nanosec = (uint32_t)((rmw_uros_epoch_millis() % 1000) * 1000000);

  //Serial.printf("Pos L: %.2f, Pos R: %.2f, Vel L: %.2f, Vel R: %.2f\n", pos_left, pos_right, vel_left, vel_right);
  joint_state_msg.position.data[0] = pos_left;
  joint_state_msg.position.data[1] = -pos_right;

  joint_state_msg.velocity.data[0] = current_rpm_l;//vel_left;
  joint_state_msg.velocity.data[1] = -current_rpm_r;//vel_right;

  // Publish jointstates
  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
#endif

#if defined(HANDLE_BUMPERS)
  bumper_msg.front_right = !digitalRead(BUMPER_FRONT_RIGHT_PIN);
  bumper_msg.front_left = !digitalRead(BUMPER_FRONT_LEFT_PIN);
  bumper_msg.rear_right = !digitalRead(BUMPER_REAR_RIGHT_PIN);
  bumper_msg.rear_left = !digitalRead(BUMPER_REAR_LEFT_PIN);

  // Publish bumpers
  RCSOFTCHECK(rcl_publish(&bumpers_publisher, &bumper_msg, NULL));
#endif

}

void setup_joint_state_msg()
{
    // Initialize message memory
    sensor_msgs__msg__JointState__init(&joint_state_msg);

    joint_state_msg.header.frame_id.data = "p3dx_base";

    // Two joints: left_wheel and right_wheel
    joint_state_msg.name.size = 3;
    joint_state_msg.name.capacity = 3;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(3 * sizeof(rosidl_runtime_c__String));

    rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");

    rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");

    rosidl_runtime_c__String__init(&joint_state_msg.name.data[2]);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2], "caster_swivel_hubcap_joint");


    // Allocate arrays for positions, velocities, efforts
    joint_state_msg.position.size = 3;
    joint_state_msg.position.capacity = 3;
    joint_state_msg.position.data = (double*)malloc(3 * sizeof(double));

    joint_state_msg.velocity.size = 3;
    joint_state_msg.velocity.capacity = 3;
    joint_state_msg.velocity.data = (double*)malloc(3 * sizeof(double));

    joint_state_msg.effort.size = 3;
    joint_state_msg.effort.capacity = 3;
    joint_state_msg.effort.data = (double*)malloc(3 * sizeof(double));

    joint_state_msg.effort.data[0] = 0.0;
    joint_state_msg.effort.data[1] = 0.0;
    joint_state_msg.effort.data[2] = 0.0;

    joint_state_msg.position.data[2] = 0.0; // Caster joint position
    joint_state_msg.velocity.data[2] = 0.0; // Caster joint velocity

}


int display_interval_counter=0;


//function which controlles the motor, callled every 10ms
void MotorControll_timerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity;
  float angularVelocity;

  if((millis() - prev_cmd_time) > 100) {
    if(motors_enabled) {
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
      tft_printf(ST77XX_MAGENTA, "Motor Stop\nNo cmd_vel\nReceived\n");
      digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
      motors_enabled = false;
      leftWheel.disable();
      rightWheel.disable();
    }  
  }
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity = twist_msg.linear.x;
  angularVelocity = twist_msg.angular.z;
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = (linearVelocity - ((WHEELS_Y_DISTANCE/2.0) * angularVelocity));
  float vR = (linearVelocity + ((WHEELS_Y_DISTANCE/2.0) * angularVelocity));

  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = leftWheel.pid(-vL);
  float actuating_signal_RW = rightWheel.pid(vR);
  if(display_interval_counter % 10 == 0 && motors_enabled){
    tft_printf(ST77XX_MAGENTA, "Linear: %.2f\nAngular: %.2f\nvL: %.2f\nvR: %.2f", linearVelocity, angularVelocity, vL, vR);
  }
  display_interval_counter ++;

  rightWheel.moveBase(actuating_signal_RW);
  leftWheel.moveBase(actuating_signal_LW);


  //odometry
  //current wheel rpm is calculated
  float currentRpmL = leftWheel.getVelocity();
  float currentRpmR = rightWheel.getVelocity();
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
    encodervalue_l--;
  else
    encodervalue_l++;
}

//interrupt function for right wheel encoder
void updateEncoderR() {
  if (digitalRead(rightWheel.EncoderPinA) == digitalRead(rightWheel.EncoderPinB))
    encodervalue_r--;
  else
    encodervalue_r++;
}

void syncTime() {
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

#if defined(HANDLE_BUMPERS)
void bumber_hit(){
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
  tft_printf(ST77XX_BLUE, "Bumper Hit!\nMotors Disabled\n");
  motors_enabled = false; 
  error_msg.data = true;
}
#endif


#define NODE_NAME "p3dx_controller"


bool errorLedState = false;

void init_display(){

  #if 0
  Serial.printf("MOSI: %i\n", MOSI);
  Serial.printf("MISO: %i\n", MISO);
  Serial.printf("SCK: %i\n", SCK);
  Serial.printf("SS: %i\n", SS); 
  #endif

  tft = new Adafruit_ST7735(DISPLAY_CS_PIN, DISPLAY_RS_DC_PIN, DISPLAY_RST_PIN);
  tft_prinft_begin(tft);

  tft->initR(INITR_GREENTAB);
  tft->fillScreen(ST77XX_BLACK);

  tft->setRotation(1);
  tft->setFont(&FreeSansBold9pt7b);
  //tft->setFont(&Tiny3x3a2pt7b);
  tft->fillScreen(ST77XX_BLACK);
  tft->setTextColor(ST77XX_CYAN);
  tft->setTextSize(1);
  tft->setCursor(1, 22);
  tft->println("P3DX Control");

    Serial.printf("Ready"); 

}

char* convertToCamelCase(const char *input) {
    int i, j;
    int len = strlen(input);
    char *output = (char *)malloc((len + 1) * sizeof(char));
    
    if(output == NULL) {
        Serial.printf("Error allocating memory\n");
        microros_error_handler();
    }

    // Kopieer de originele string naar de uitvoerstring
    strcpy(output, input);

    // Loop door de uitvoerstring en converteer naar camel case
    for (i = 0; i < len; i++) {
        if (output[i] == '_') {
            // Verwijder de underscore
            for (j = i; j < len; j++) {
                output[j] = output[j + 1];
            }
            // Converteer het volgende teken naar hoofdletter
            output[i] = toupper(output[i]);
            // Verlaag de lengte van de string
            len--;
        }
    }
    return output;
}


#if defined(WIFI)
  String wifiWIFI_SSID = WIFI_SSID;
  String wifiPass = WIFI_PASSWORD;
#endif

void setup() {
  // Configure serial transport
  Serial.begin(115200);

  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReadResolution(10);

  init_display();

  tft_printf(ST77XX_MAGENTA, "Pioneer 3DX\nController\nStarted\n");

  //initializing the pid constants
  leftWheel.setPIDvalues(KP_L, KI_L, KD_L);
  rightWheel.setPIDvalues(KP_R, KI_R, KD_R);

  //initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinB), updateEncoderR, RISING);


  //prev_rpm_l = leftWheel.getVelocity();
  //prev_rpm_r = rightWheel.getVelocity();

#if defined(WIFI)
  WiFi.setHostname("p3dx_controller");

  bool force_network_configure;
  force_network_configure = !digitalRead(SELECT_WIFI_CONFIG_MODE_PIN);

  NETWORK_CONFIG networkConfig;
  bool wifiUp = configureNetwork(force_network_configure, &networkConfig);
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


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);

  // pinMode(RESET_ERROR_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(RESET_ERROR_PIN), reset_by_button, FALLING);

#if defined(HANDLE_BUMPERS)
  pinMode(BUMPER_FRONT_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUMPER_FRONT_RIGHT_PIN), bumber_hit, FALLING);
  pinMode(BUMPER_FRONT_LEFT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUMPER_FRONT_LEFT_PIN), bumber_hit, FALLING); 
  pinMode(BUMPER_REAR_RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUMPER_REAR_RIGHT_PIN), bumber_hit, FALLING);
  pinMode(BUMPER_REAR_LEFT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUMPER_REAR_LEFT_PIN), bumber_hit, FALLING);
  if(digitalRead(BUMPER_FRONT_RIGHT_PIN)==HIGH &&
     digitalRead(BUMPER_FRONT_LEFT_PIN)==HIGH  &&
     digitalRead(BUMPER_REAR_RIGHT_PIN)==HIGH  &&
     digitalRead(BUMPER_REAR_LEFT_PIN)==HIGH){
    error_msg.data = false;
  }
  else{
    error_msg.data = true;
  }
#else
    error_msg.data = false;

#endif

  digitalWrite(MOTOR_ENABLE_PIN, MOTOR_DISABLE);
  leftWheel.disable();
  rightWheel.disable();
  motors_enabled = false;

  delay(2000);

  allocator = rcl_get_default_allocator();


  //create init_options
  if(rclc_support_init(&support, 0, NULL, &allocator)){
    tft_printf(ST77XX_BLUE, "microROS agent\nnot found\nRestarting...\n");
    delay(3000);
    ESP.restart();
  }

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

  //create a bumper publisher
#if defined(HANDLE_BUMPERS)
  RCCHECK(rclc_publisher_init_default(
    &bumpers_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(p3dx_interfaces, msg, Bumpers),
    "bumpers"));
#endif  

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

    // Create publisher
  RCCHECK(rclc_publisher_init_default(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states"));

    // Prepare message memory
    setup_joint_state_msg();

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
  tft_printf(ST77XX_MAGENTA, "Controller\nReady\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

