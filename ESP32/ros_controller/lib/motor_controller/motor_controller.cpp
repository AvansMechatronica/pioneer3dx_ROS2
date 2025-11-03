#include "motor_controller.h"

MotorController::MotorController(int8_t ledcChannel,
                                 int8_t PwmPin, 
                                 int8_t DirPin, 
                                 int8_t EncoderA, 
                                 int8_t EncoderB,
                                 float wheel_radius) {
  this->ledcChannel = ledcChannel;
  this->Pwm = PwmPin;
  this->Dir = DirPin;
  this->EncoderPinA = EncoderA;
  this->EncoderPinB = EncoderB;
  this->wheel_radius = wheel_radius;

  this->previousError = 0;
  this->eintegral = 0;
  this->rpmFilt = 0;
  this->rpmPrev = 0;
  this->PreviousPosition = 0;
  this->CurrentPosition = 0;
  this->PreviousTime = millis();
  this->CurrentTime = millis();
  this->PreviousTimeForError = millis();
  this->CurrentTimeforError = millis();

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

float MotorController::getVelocity(){// returns linear velocity in m/s
  CurrentPosition = EncoderCount.data;
  CurrentTime = millis();
  float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;
  float velocity = ((float)CurrentPosition - PreviousPosition) / delta1; // in ticks per second

  float rpm = (velocity / TICK_PER_REVOLUTION) * 60; // convert to rpm
  rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev; // 
  rpmPrev = rpm; // store previous rpm value   
  PreviousPosition = CurrentPosition; //      
  PreviousTime = CurrentTime;
  return (rpmFilt * wheel_radius * 2 * 3.14) / 60;  // return linear velocity in m/s
}

int i = 0;

float MotorController::pid(float setpoint) {
  float current_rpm = getVelocity();
  if((i % 100) == 0){
    Serial.printf("%i, Setpoint: %.2f | RPM: %.2f\n", i /100, setpoint, current_rpm);
  }
  i++;
  CurrentTimeforError = millis();
  float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
  error = setpoint - current_rpm;
  eintegral = eintegral + (error * delta2);
  ederivative = (error - previousError) / delta2;
  float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

  previousError = error;
  PreviousTimeForError = CurrentTimeforError;
  if((i % 100) == 0){
    Serial.printf("** Error: %.2f | Integral: %.2f | Derivative: %.2f\n", error, eintegral, ederivative);
    Serial.printf("** control signal: %.2f\n", control_signal);
  }
  return control_signal;
}

void MotorController::moveBase(float ActuatingSignal) {
  static int i = 0;  // tel hoe vaak de functie wordt aangeroepen
  i++;

  // Beperk de waarde van het regelsignaal
  if (ActuatingSignal > 100.0f) ActuatingSignal = 100.0f;
  if (ActuatingSignal < -100.0f) ActuatingSignal = -100.0f;

  // Afdrukken om de 100 cycli
  if ((i % 100) == 0) {
    Serial.printf("-- 1. Actuating Signal: %.2f\n", ActuatingSignal);
  }

  // Richting bepalen en PWM duty cycle berekenen
  int dutyCycle = 0;
  if (ActuatingSignal > 0) {
    digitalWrite(Dir, CLOCK_WISE);  // zorg dat dit HIGH/LOW is
    dutyCycle = (int)(ActuatingSignal / 100.0f * MAX_PWM);
  } else {
    digitalWrite(Dir, COUNTER_CLOCK_WISE);
    dutyCycle = (int)(-ActuatingSignal / 100.0f * MAX_PWM);
  }

  // Debug print
  if ((i % 100) == 0) {
    Serial.printf("-- 1. Duty Cycle before limit: %d\n", dutyCycle);
  }

  // Veiligheidslimiet
  if (dutyCycle > MAX_PWM) dutyCycle = MAX_PWM;

  if ((i % 100) == 0) {
    Serial.printf("-- 2. PWM value after limit: %d\n", dutyCycle);
  }

  // Schrijf PWM-waarde uit naar de motor driver
  ledcWrite(this->ledcChannel, dutyCycle);
}


void MotorController::stop() {
  ledcWrite(this->ledcChannel, 0);
  digitalWrite(Dir, LOW);
}
