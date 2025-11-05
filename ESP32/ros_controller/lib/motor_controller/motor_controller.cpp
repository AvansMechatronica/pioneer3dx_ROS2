#include "motor_controller.h"



MotorController::MotorController(int8_t ledcChannel,
                                 int8_t PwmPin, 
                                 int8_t DirPin, 
                                 int8_t EncoderA, 
                                 int8_t EncoderB,
                                 int64_t *encoderCount,
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
  this->encoderCount = encoderCount;

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

float MotorController::getVelocityInt(){// returns linear velocity in m/s
  CurrentPosition = *encoderCount;
  CurrentTime = millis();
  float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3; //convert to seconds
  float velocity = ((float)CurrentPosition - PreviousPosition) / delta1; // in ticks per second

  float rpm = (velocity / TICK_PER_REVOLUTION) * 60; // convert to rpm
  rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev; // 
  rpmPrev = rpm; // store previous rpm value   
  PreviousPosition = CurrentPosition; //      
  PreviousTime = CurrentTime;
  return (rpmFilt * wheel_radius * 2 * 3.14) / 60;  // return linear velocity in m/s
}

float MotorController::getVelocity(){
  return current_velocity;
}

#ifdef DEBUG_MOTOR_CONTROLLER
int display_interval_counter = 0;
#endif

float MotorController::pid(float setpoint) {
  current_velocity = getVelocityInt();
#ifdef DEBUG_MOTOR_CONTROLLER
  if((display_interval_counter % 100) == 0){
    Serial.printf("%i: Setpoint: %.2f | RPM: %.2f\n",  display_interval_counter/100, setpoint, current_velocity);
  }
  display_interval_counter++;
#endif
  CurrentTimeforError = millis();
  if(!enabled){
    return 0.0f;
  }
  float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
  error = setpoint - current_velocity;
  eintegral = eintegral + (error * delta2);
  ederivative = (error - previousError) / delta2;
  float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

  previousError = error;
  PreviousTimeForError = CurrentTimeforError;

  #ifdef DEBUG_MOTOR_CONTROLLER
  if((display_interval_counter % 100) == 0){
    Serial.printf("-- Error: %.2f | Integral: %.2f | Derivative: %.2f\n", error, eintegral, ederivative);
    Serial.printf("-- control signal: %.2f\n", control_signal);
  }
#endif
  return control_signal;
}

void MotorController::moveBase(float ActuatingSignal) {

  // Beperk de waarde van het regelsignaal
  if(enabled){

    if (ActuatingSignal > 100.0f) ActuatingSignal = 100.0f;
    if (ActuatingSignal < -100.0f) ActuatingSignal = -100.0f;

    // Afdrukken om de 100 cycli
#ifdef DEBUG_MOTOR_CONTROLLER
    if ((display_interval_counter % 100) == 0) {
      Serial.printf("-- Actuating Signal: %.2f\n", ActuatingSignal);
    }
#endif
    // Richting bepalen en PWM duty cycle berekenen
    int dutyCycle = 0;
    if (ActuatingSignal > 0) {
      digitalWrite(Dir, CLOCK_WISE);  // zorg dat dit HIGH/LOW is
      dutyCycle = (int)(ActuatingSignal / 100.0f * MAX_PWM);
    } else {
      digitalWrite(Dir, COUNTER_CLOCK_WISE);
      dutyCycle = (int)(-ActuatingSignal / 100.0f * MAX_PWM);
    }

#ifdef DEBUG_MOTOR_CONTROLLER
    // Debug print
    if ((display_interval_counter % 100) == 0) {
      Serial.printf("-- Duty Cycle before limit: %d\n", dutyCycle);
    }
#endif
    // Veiligheidslimiet
    if (dutyCycle > MAX_PWM) dutyCycle = MAX_PWM;

#ifdef DEBUG_MOTOR_CONTROLLER
    if ((display_interval_counter % 100) == 0) {
      Serial.printf("-- PWM value after limit: %d\n", dutyCycle);
    }
#endif
    // Schrijf PWM-waarde uit naar de motor driver
    ledcWrite(this->ledcChannel, dutyCycle);
  }
}


void MotorController::disable() {
  ledcWrite(this->ledcChannel, 0);
  digitalWrite(Dir, LOW);
  enabled = false;
}

void MotorController::enable() {
  enabled = true;
}