/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"
#include "control.h"
#include "pid_controller.h"

namespace Robotic_sys {
namespace control {

using Robotic_sys::common::Result_state;

void Motors::Init(){
  // set pin
  pinMode(motorSpeedPin1, OUTPUT);
  pinMode(motorDirectionPin1, OUTPUT);
  pinMode(motorSpeedPin2, OUTPUT);
  pinMode(motorDirectionPin2, OUTPUT);
}

void Motors::SetMontorPower(int l_pwm, int r_pwm){
  analogWrite(motorSpeedPin1, l_pwm);
  analogWrite(motorSpeedPin2, r_pwm);
}

const char* Control::Name() const { return "control"; }

void Control::ComputeLateralErrors(){
  
}

Result_state Control::Init(){
  Serial.println("control init, starting...");

  motor_.Init();
 
 return Result_state::State_Ok;
}

Result_state Control::Start(){

  Result_state status = ProduceControlCommand();

  motor_.SetMontorPower(30,30);

 return Result_state::State_Ok;
}

void Control::Stop(){

}

Result_state Control::ProduceControlCommand(){

  ComputeLateralErrors();
  return Result_state::State_Ok;
}



} // namespace control
} // namespace Robotic_sys
