#ifndef PINS_H
#define PINS_H

//pin declaration
//Left wheel
#define L_PWM_PIN        26
#define L_DIR_PIN        27
#define L_ENCODER_PINA   18  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define L_ENCODER_PINB   21
//right wheel
#define R_PWM_PIN        33
#define R_DIR_PIN        32
#define R_ENCODER_PINA   23  //Encoder Output of pin1 must connected with intreput pin of Esp32.
#define R_ENCODER_PINB   15

#define BUMPER_FRONT_RIGHT_PIN 34
#define BUMPER_FRONT_LEFT_PIN  35   
#define BUMPER_REAR_RIGHT_PIN 39
#define BUMPER_REAR_LEFT_PIN 36

#define MOTOR_ENABLE_PIN 14
#define MOTOR_ENABLE        HIGH
#define MOTOR_DISABLE       LOW 

#endif