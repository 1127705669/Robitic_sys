/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common.h"
#include "localization.h"
#include "perception.h"
#include "control.h"
#include "encoders.h"

using Robotic_sys::common::Result_state;

#define INIT_COMPONENT()                                 \
  Robotic_sys::localization::Localization localization;  \
  Robotic_sys::perception::Perception perception;        \
  Robotic_sys::control::Control control;                 \
  Robotic_sys::common::StateMachine state_machine;       \

INIT_COMPONENT();

bool is_frist_time = true;
bool first_hit_ = true;

unsigned long prev_time_stamp;
unsigned long prev_pid_time_stamp;

void setup() {
  
  if(Result_state::State_Ok != localization.Init()){
    Serial.println("location init failed!");
  }
  
  if(Result_state::State_Ok != perception.Init()){
    Serial.println("perception init failed!");
  }
  
  if(Result_state::State_Ok != control.Init()){
    Serial.println("control init failed!");
  }

  Robotic_sys::common::BuzzleInit();
  
  setupEncoder0();
  
  setupEncoder1();

  Serial.begin(9600);

  // waiting for conection finish
  delay(1500);
  
  Serial.println("init done!");
}

void loop() {
  volatile unsigned long current_time = millis();

  static Robotic_sys::perception::Sensor sensor_lists[SENSOR_NUM];
  static Robotic_sys::perception::Bumper bumper_lists[BUMPER_NUM];

  unsigned long duration;

  if(is_frist_time){
    prev_time_stamp = current_time;
    is_frist_time = false;
  }else{
   duration = current_time - prev_time_stamp;
  }

  if(duration > 20){
    perception.GetGrayScale(sensor_lists);
    perception.CollisionDetect(bumper_lists);

    localization.CalculateSpeed(count_e0, count_e1, duration);
    localization.ComputePosition(duration);
    localization.ImuRead();
    
    prev_time_stamp = current_time;
  }

  Serial.print(bumper_lists[BL].bumper_time_);
  Serial.print("  ");
  Serial.print(bumper_lists[BR].bumper_time_);
  Serial.print("  ");
  Serial.print(bumper_lists[BL].min_bumper_time_);
  Serial.print("  ");
  Serial.println(bumper_lists[BR].min_bumper_time_);

  if((bumper_lists[BL].min_bumper_time_ < 230)|(bumper_lists[BR].min_bumper_time_ < 190)){
    control.GoFixedSpeed(0, 0);
  }else{
    unsigned long pid_duration;
  
    if(first_hit_){
      prev_pid_time_stamp = current_time;
      first_hit_ = false;
    }else{
      pid_duration = current_time - prev_pid_time_stamp;
    }
  
    if(pid_duration > 10) {
      control.ComputeControlCmd(localization.left_wheel_speed, localization.right_wheel_speed, pid_duration);
      prev_pid_time_stamp = current_time;
    }
  }
}
