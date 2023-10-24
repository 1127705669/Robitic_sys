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
      break;

    case ANTICLOCKWISE:
      motor_.SetMontorPower(-20,20);
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
  left_pid_controller_.Init();
  right_pid_controller_.Init();
 
 return Result_state::State_Ok;
}

Result_state Control::Start(){

  Result_state status = ProduceControlCommand();

 return Result_state::State_Ok;
}

void Control::BangBangControl(Robotic_sys::perception::Sensor* sensor_lists){
  int basic_speed = (sensor_lists[SENSOR_DN3].gray_scale_ - 80)*BiasPWM/200 + BiasPWM;
  int left_speed = basic_speed + (sensor_lists[3].gray_scale_*MaxTurnPWM)/100 -(sensor_lists[1].gray_scale_*MaxTurnPWM)/100;
  int right_speed = basic_speed + (sensor_lists[1].gray_scale_*MaxTurnPWM)/100 - (sensor_lists[3].gray_scale_*MaxTurnPWM)/100;
  motor_.SetMontorPower(left_speed, right_speed);
}

void Control::ComputeControlCmd(Robotic_sys::perception::Sensor* sensor_lists, const double dt){
  int sum = sensor_lists[SENSOR_DN2].gray_scale_ + sensor_lists[SENSOR_DN4].gray_scale_;
  double weighted_value = 2*((double)sensor_lists[SENSOR_DN4].gray_scale_/(double)sum) - 1;
//  Serial.println(weighted_value);
  double feedback_left = left_pid_controller_.Control(weighted_value, dt);
  double feedback_right = right_pid_controller_.Control(weighted_value, dt);
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
