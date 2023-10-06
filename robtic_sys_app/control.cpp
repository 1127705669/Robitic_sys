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

const char* Control::Name() const { return "control"; }

void Control::ComputeLateralErrors(){
  
}

Result_state Control::Init(){
  Serial.println("control init, starting...");

  // set pin
  pinMode(motorSpeedPin1, OUTPUT);
  pinMode(motorDirectionPin1, OUTPUT);
  pinMode(motorSpeedPin2, OUTPUT);
  pinMode(motorDirectionPin2, OUTPUT);
 
 return Result_state::State_Ok;
}

Result_state Control::Start(){

  Result_state status = ProduceControlCommand();
  
 digitalWrite(motorDirectionPin1, HIGH);
 digitalWrite(motorDirectionPin2, LOW);
 analogWrite(motorSpeedPin1, 100);
 analogWrite(motorSpeedPin2, 20);

// Serial.println("control strated..");

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
