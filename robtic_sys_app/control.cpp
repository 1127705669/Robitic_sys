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
  pinMode(motor_left_direction_pin_, OUTPUT);
  pinMode(motor_right_direction_pin_, OUTPUT);
  pinMode(motor_left_speed_pin_, OUTPUT);
  pinMode(motor_right_speed_pin_, OUTPUT);
}

void Motors::SetMontorPower(int left_pwm, int right_pwm){
  if(0 < left_pwm){
    digitalWrite(motor_left_direction_pin_, LOW);
  }else{
    digitalWrite(motor_left_direction_pin_, HIGH);
    left_pwm *= -1;
  }

  if(0 < right_pwm){
    digitalWrite(motor_right_direction_pin_, LOW);
  }else{
    digitalWrite(motor_right_direction_pin_, HIGH);
    right_pwm *= -1;
  }

  analogWrite(motor_left_speed_pin_, left_pwm);
  analogWrite(motor_right_speed_pin_, right_pwm);
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

 return Result_state::State_Ok;
}

void Control::BangBangControl(unsigned long* gray_scale){
  unsigned long gray_scale_max = 0;
  int sensor_nub = 0;
  for (int sensor_number = 0; sensor_number < 5; sensor_number++) {
    if(gray_scale[sensor_number] > gray_scale_max){
      gray_scale_max = gray_scale[sensor_number];
      sensor_nub = sensor_number;
    }
  }

  switch(sensor_nub){
    case 0:
      motor_.SetMontorPower(0,15);
      break;
    case 1:
      motor_.SetMontorPower(8,15);
      break;
    case 2:
      motor_.SetMontorPower(15,15);
      break;
    case 3:
      motor_.SetMontorPower(15,8);
      break;
    case 4:
      motor_.SetMontorPower(15,0);
      break;
    default:
      // nothing
      break;
  }
}

void Control::Stop(){

}

Result_state Control::ProduceControlCommand(){

  ComputeLateralErrors();
  return Result_state::State_Ok;
}

void Control::GoFixedSpeed(int left_pwm = 30, int right_pwm = 30){
  motor_.SetMontorPower(left_pwm, right_pwm);
}

} // namespace control
} // namespace Robotic_sys
