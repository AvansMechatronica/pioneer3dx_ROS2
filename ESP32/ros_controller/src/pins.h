#ifndef PINS_H
#define PINS_H

//pin declaration
//Left wheel
#define L_PWM_PIN        26
#define L_DIR_PIN        27
#define L_ENCODER_PINA   39  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define L_ENCODER_PINB   21
//right wheel
#define R_PWM_PIN        33
#define R_DIR_PIN        32
#define R_ENCODER_PINA   36  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define R_ENCODER_PINB   15

#if defined(HANDLE_BUMPERS)
#define BUMPER_FRONT_RIGHT_PIN 34
#define BUMPER_FRONT_LEFT_PIN  35   
#define BUMPER_REAR_RIGHT_PIN 39
#define BUMPER_REAR_LEFT_PIN 36
#endif

#define RESET_ERROR_PIN 25


#define MOTOR_ENABLE_PIN 14
#define MOTOR_ENABLE        HIGH
#define MOTOR_DISABLE       LOW 

#define SELECT_WIFI_CONFIG_MODE_PIN 13

#define DISPLAY_CS_PIN    5
#define DISPLAY_RS_DC_PIN 12
#define DISPLAY_RST_PIN   13
#define DISPLAY_CLK_PIN   18
#define DISPLAY_SDA_PIN   23
#define DISPLAY_MISO_PIN  19 // Not used

#define RPLIDAR_COM_PORT    1      
#define RPLIDAR_TX_PIN      35// Never use pin 10 !!!! // Lidar side
#define RPLIDAR_RX_PIN      17// 9 // Lidar side
#define RPLIDAR_MOTOR_PIN 25   // optioneel: PWM of GPIO voor motor aan/uit


#define LED_PIN 2

#define BATTERY_VOLTAGE_PIN 34 // ADC pin to read battery voltage

#endif