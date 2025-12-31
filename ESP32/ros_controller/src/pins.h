#ifndef PINS_H
#define PINS_H

//pin declaration
//Left wheel
#define L_PWM_PIN        3
#define L_DIR_PIN        4
#define L_ENCODER_PINA   10 //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define L_ENCODER_PINB   7
//right wheel
#define R_PWM_PIN        5
#define R_DIR_PIN        6
#define R_ENCODER_PINA   9  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define R_ENCODER_PINB   8

#if defined(HANDLE_BUMPERS)
#define BUMPER_FRONT_PIN    46
#define BUMPER_REAR_PIN     45
#endif



#define MOTOR_ENABLE_PIN    2
#define MOTOR_ENABLE        LOW
#define MOTOR_DISABLE       HIGH 

//#define SELECT_WIFI_CONFIG_MODE_PIN 13

#define DISPLAY_CS_PIN    11
#define DISPLAY_RS_DC_PIN 14
#define DISPLAY_RST_PIN   13
#define DISPLAY_CLK_PIN   12
#define DISPLAY_SDA_PIN   17
//#define DISPLAY_MISO_PIN  19 // Not used

#define RPLIDAR_COM_PORT    2      
#define RPLIDAR_TX_PIN      18 // Lidar side

#define RPLIDAR_RX_PIN      21 // Lidar side
#define RPLIDAR_MOTOR_PIN   35 // optioneel: PWM of GPIO voor motor aan/uit

#define UCP_STATUS_PIN      39 // Pin to read UCP status
#define UCP_RESET_PIN       40 // Pin to reset UCP
#define UCP_MOTORS_PIN      41 // Pin to control UCP motor power
#define UCP_BUZZER_PIN      42 // Pin to control UCP buzzer
#define UCP_BUZZER_PWM_CHANNEL  1 // PWM channel for UCP buzzer

#define I2C_SDA_PIN  36 //  
#define I2C_SCL_PIN  37 // 
#define IMU_INT_PIN  38 // Interrupt pin from IMU

#define LED_PIN 2

#define BATTERY_VOLTAGE_PIN 1 // ADC pin to read battery voltage

#endif