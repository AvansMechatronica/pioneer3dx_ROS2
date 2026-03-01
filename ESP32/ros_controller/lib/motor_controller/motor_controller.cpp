#include "motor_controller.h"



MotorController::MotorController(int8_t ledcChannel,
                                 int8_t PwmPin, 
                                 int8_t DirPin, 
                                 int64_t *encoderCount,
                                 float wheel_radius) {
  this->ledcChannel = ledcChannel;
  this->Pwm = PwmPin;
  this->Dir = DirPin;

  this->wheel_radius = wheel_radius;

  this->previousError = 0;
  this->eintegral = 0;
  this->rpmFilt = 0;
  this->rpmPrev = 0;
  this->previous_encoder_count = 0;
  this->PreviousTime = millis();
  this->PreviousTimeForError = millis();
  this->encoderCount = encoderCount;

  pinMode(Pwm, OUTPUT);
  pinMode(Dir, OUTPUT);


  // initializing PWM signal parameters
  ledcSetup(this->ledcChannel, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(this->Pwm, this->ledcChannel);
  ledcWrite(this->ledcChannel, 0);
  digitalWrite(Dir, LOW);
}

void MotorController::setPIDvalues(float proportionalGain, float integralGain, float derivativeGain) {
  kp = proportionalGain;
  ki = integralGain;
  kd = derivativeGain;
}

float MotorController::getVelocityInternal(){// returns linear velocity in m/s
  int64_t current_encoder_count = *encoderCount;
  uint32_t currentTime = millis();
  uint32_t deltaMs = currentTime - PreviousTime;
  if (deltaMs == 0U) {
    // Avoid division by zero or negative time intervals; return current filtered velocity
    return (rpmFilt * wheel_radius * 2 * 3.14f) / 60.0f;  // linear velocity in m/s
  }
  float delta1 = ((float)deltaMs) / 1.0e3f; // convert to seconds
  float velocity = ((float)(current_encoder_count - previous_encoder_count)) / delta1; // in ticks per second

  float rpm = (velocity / TICK_PER_REVOLUTION) * 60; // convert to rpm
  rpmFilt = rpm;//0.854f * rpmFilt + 0.0728f * rpm + 0.0728f * rpmPrev; // 
  rpmPrev = rpm; // store previous rpm value   
  previous_encoder_count = current_encoder_count; //      
  PreviousTime = currentTime;
  return (rpmFilt * wheel_radius * 2 * 3.14f) / 60.0f;  // return linear velocity in m/s
}


float MotorController::getVelocity(){
  return current_velocity;
}

#ifdef DEBUG_MOTOR_CONTROLLER
int mc_display_interval_counter = 0;
#endif

float MotorController::pid(float setpoint) {
  current_velocity = getVelocityInternal();
#ifdef DEBUG_MOTOR_CONTROLLER
  if((mc_display_interval_counter % 20) == 0){
    Serial.printf("%i: Setpoint: %.2f | RPM: %.2f\n",  mc_display_interval_counter/100, setpoint, current_velocity);
  }
  mc_display_interval_counter++;
#endif
  long CurrentTimeForError = millis();
  if(!enabled){
    setpoint = 0.0f;
//    previousError = 0;
    eintegral = 0;
//    PreviousTimeForError = CurrentTimeForError;
//    return 0.0f;
  }
  float delta2 = ((float)CurrentTimeForError - PreviousTimeForError) / 1.0e3;
  error = setpoint - current_velocity;
  eintegral = eintegral + (error * delta2);
  ederivative = (error - previousError) / delta2;
  float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

  previousError = error;
  PreviousTimeForError = CurrentTimeForError;

#ifdef DEBUG_MOTOR_CONTROLLER
  if((mc_display_interval_counter % 20) == 0){
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
    if ((mc_display_interval_counter % 20) == 0) {
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
    if ((mc_display_interval_counter % 20) == 0) {
      Serial.printf("-- Duty Cycle before limit: %d\n", dutyCycle);
    }
#endif
    // Veiligheidslimiet
    if (dutyCycle > MAX_PWM) dutyCycle = MAX_PWM;

#ifdef DEBUG_MOTOR_CONTROLLER
    if ((mc_display_interval_counter % 20) == 0) {
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

