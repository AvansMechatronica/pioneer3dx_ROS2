#include "motor_controller.h"

MotorController::MotorController(int8_t ledcChannel,
                                 int8_t PwmPin, 
                                 int8_t DirPin, 
                                 int8_t EncoderA, 
                                 int8_t EncoderB) {
  this->ledcChannel = ledcChannel;
  this->Pwm = PwmPin;
  this->Dir = DirPin;
  this->EncoderPinA = EncoderA;
  this->EncoderPinB = EncoderB;

  pinMode(Pwm, OUTPUT);
  pinMode(Dir, OUTPUT);
  pinMode(EncoderPinA, INPUT);
  pinMode(EncoderPinB, INPUT);

  // initializing PWM signal parameters
  ledcSetup(this->ledcChannel, PWM_FRQUENCY, PWM_RESOLUTION);
  ledcAttachPin(this->Pwm, this->ledcChannel);
  ledcWrite(this->ledcChannel, 0);
  digitalWrite(Dir, LOW);
}

void MotorController::setPIDvalues(float proportionalGain, float integralGain, float derivativeGain) {
  kp = proportionalGain;
  ki = integralGain;
  kd = derivativeGain;
}

float MotorController::getRpm() {
  CurrentPosition = EncoderCount.data;
  CurrentTime = millis();
  float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;
  float velocity = ((float)CurrentPosition - PreviousPosition) / delta1;
  float rpm = (velocity / TICK_PER_REVOLUTION) * 60;
  rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
  rpmPrev = rpm;
  PreviousPosition = CurrentPosition;
  PreviousTime = CurrentTime;
  return rpmFilt;
}

float MotorController::pid(float setpoint) {
  CurrentTimeforError = millis();
  float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
  error = setpoint - getRpm();
  eintegral = eintegral + (error * delta2);
  ederivative = (error - previousError) / delta2;
  float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

  previousError = error;
  PreviousTimeForError = CurrentTimeforError;
  return control_signal;
}

void MotorController::moveBase(float ActuatingSignal) {
  if (ActuatingSignal > 0) {
    digitalWrite(Dir, CLOCK_WISE);
  } else {
    digitalWrite(Dir, COUNTER_CLOCK_WISE);
  }
  int pwm = THRESHOLD + (int)fabs(ActuatingSignal);
  if (pwm > MAX_PWM)
    pwm = MAX_PWM;
  ledcWrite(this->ledcChannel, pwm);
}

void MotorController::stop() {
  ledcWrite(this->ledcChannel, 0);
  digitalWrite(Dir, LOW);
}
