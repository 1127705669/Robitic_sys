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

void Control::Rotate(RotateType rotate_direction){
  switch (rotate_direction) {
    case CLOCKWISE:
      motor_.SetMontorPower(20,-20);
      delay(200);
      break;

    case ANTICLOCKWISE:
      motor_.SetMontorPower(-20,20);
      delay(200);
      break;

    default:
      break;
  }
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

void Control::BangBangControl(Robotic_sys::perception::Sensor* sensor_lists){
  int left_speed = BiasPWM + (sensor_lists[3].gray_scale_*MaxTurnPWM)/100;
  int right_speed = BiasPWM + (sensor_lists[1].gray_scale_*MaxTurnPWM)/100;

  motor_.SetMontorPower(left_speed, right_speed);
//  unsigned long gray_scale_max = 0;
//  int sensor_nub = 0;
//  for (int sensor_number = 0; sensor_number < 5; sensor_number++) {
//    if(gray_scale[sensor_number] > gray_scale_max){
//      gray_scale_max = gray_scale[sensor_number];
//      sensor_nub = sensor_number;
//    }
//  }
//
//  switch(sensor_nub){
//    case 0:
//      motor_.SetMontorPower(-25,25);
//      break;
//    case 1:
//      motor_.SetMontorPower(10,25);
//      break;
//    case 2:
//      motor_.SetMontorPower(25,25);
//      break;
//    case 3:
//      motor_.SetMontorPower(25,10);
//      break;
//    case 4:
//      motor_.SetMontorPower(25,-25);
//      break;
//    default:
//      // nothing
//      break;
//  }
  
}

void Control::Stop(){

}

Result_state Control::ProduceControlCommand(){

  ComputeLateralErrors();
  return Result_state::State_Ok;
}

void Control::GoFixedSpeed(int left_pwm = 20, int right_pwm = 20){
  motor_.SetMontorPower(left_pwm, right_pwm);
}

} // namespace control
} // namespace Robotic_sys
