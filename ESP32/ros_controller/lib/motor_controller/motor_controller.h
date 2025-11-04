#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "std_msgs/msg/int32.h"

// Define these constants if not already defined elsewhere
#ifndef PWM_FRQUENCY
#define PWM_FRQUENCY 20000
#endif

#ifndef PWM_RESOLUTION
#define PWM_RESOLUTION 8
#endif

#ifndef TICK_PER_REVOLUTION
#define TICK_PER_REVOLUTION 19150
#endif

#ifndef CLOCK_WISE
#define CLOCK_WISE HIGH
#endif

#ifndef COUNTER_CLOCK_WISE
#define COUNTER_CLOCK_WISE LOW
#endif

#ifndef THRESHOLD
#define THRESHOLD 0
#endif

#ifndef MAX_PWM
#define MAX_PWM ((1 << PWM_RESOLUTION) - 1)
#endif

class MotorController {
public:
  int8_t ledcChannel;
  int8_t Pwm;
  int8_t Dir;
  int8_t EncoderPinA;
  int8_t EncoderPinB;
  int64_t *encoderCount;
  volatile long CurrentPosition;
  volatile long PreviousPosition;
  volatile long CurrentTime;
  volatile long PreviousTime;
  volatile long CurrentTimeforError;
  volatile long PreviousTimeForError;
  float rpmFilt;
  float eintegral;
  float ederivative;
  float rpmPrev;
  float kp;
  float ki;
  float kd;
  float error;
  float previousError = 0;
  float wheel_radius;

  MotorController(int8_t ledcChannel,
                  int8_t PwmPin, 
                  int8_t DirPin, 
                  int8_t EncoderA, 
                  int8_t EncoderB,
                  int64_t *encoderCount,
                  float wheel_radius
                  );

  void setPIDvalues(float proportionalGain, float integralGain, float derivativeGain);
  float getVelocity();
  float pid(float setpoint);
  void moveBase(float ActuatingSignal);
  void stop();

private:
  float getVelocityInt();
  float current_velocity;
};

#endif // MOTOR_CONTROLLER_H
