/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"
#include "control.h"

namespace Robotic_sys {
namespace control {

using Robotic_sys::common::Result_state;

char Control::Name() const { return "control"; }

Result_state Control::Init(){
  Serial.println("control init, starting...");

  // 设置电机控制引脚为输出模式
  pinMode(motorSpeedPin1, OUTPUT);
  pinMode(motorDirectionPin1, OUTPUT);
  pinMode(motorSpeedPin2, OUTPUT);
  pinMode(motorDirectionPin2, OUTPUT);
 
 return Result_state::State_Ok;
}

Result_state Control::Start(){
 digitalWrite(motorDirectionPin1, HIGH); // 设置电机1正向
 digitalWrite(motorDirectionPin2, HIGH); // 设置电机2正向
 analogWrite(motorSpeedPin1, 50); // 设置电机1速度最大
 analogWrite(motorSpeedPin2, 50); // 设置电机2速度最大

 return Result_state::State_Ok;
}

void Control::Stop(){
 
}

} // namespace control
} // namespace Robotic_sys
