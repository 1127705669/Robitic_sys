/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common.h"
#include "localization.h"
#include "perception.h"
#include "control.h"
#include "kinematics.h"
#include "encoders.h"

using Robotic_sys::common::Result_state;

#define INIT_COMPONENT()                                 \
  Robotic_sys::localization::Localization localization;  \
  Robotic_sys::perception::Perception perception;        \
  Robotic_sys::control::Control control;                 \
  Robotic_sys::common::StateMachine state_machine;       \
  Kinematics_c kinematic;                                \

INIT_COMPONENT();

bool is_frist_time = false;
unsigned long prev_time_stamp;
unsigned long prev_pid_time_stamp;

double theta1_yaw;
double theta2_yaw;

unsigned long turn_time_stamp;
bool first_hit_ = true;

long start_count_left;
long start_count_right;

double left_wheel_speed;
double right_wheel_speed;

double left_linear_distance = 0;
double right_linear_distance = 0;

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
  unsigned long current_time = millis();

  Robotic_sys::perception::Sensor sensor_lists[SENSOR_NUM];
  
  perception.GetGrayScale(sensor_lists);

  if(!is_frist_time){
    prev_time_stamp = current_time;
    is_frist_time = true;
  }

  unsigned long duration = current_time - prev_time_stamp;
  
  if(duration > 10){
    unsigned long duration_time = current_time - prev_time_stamp;
    
    long duration_count_right = count_e0 - start_count_right;
    right_linear_distance = -RADIUS*((double)duration_count_right/GEAR_RATIO*(2*PI));
    right_wheel_speed = right_linear_distance/((double)duration_time/1000);
    start_count_right = count_e0;

    long duration_count_left = count_e1 - start_count_left;
    left_linear_distance = -RADIUS*((double)duration_count_left/GEAR_RATIO*(2*PI));
    left_wheel_speed = left_linear_distance/((double)duration_time/1000);
    start_count_left = count_e1;
    
    kinematic.update(left_linear_distance, right_linear_distance, left_wheel_speed, right_wheel_speed, duration);
    prev_time_stamp = current_time;
  }

//  debug
//  for (int sensor_number = 0; sensor_number < SENSOR_NUM; sensor_number++) {
//    Serial.print(sensor_lists[sensor_number].sensor_time_);
//    Serial.print(" ");
//    Serial.print(sensor_lists[sensor_number].gray_scale_);
//    Serial.print("   ");
//  }
//  
//  Serial.print(left_wheel_speed);
//  Serial.print("   ");
//  Serial.println(right_wheel_speed);

  if(state_machine.Init == state_machine.state){
    control.GoFixedSpeed();
    if(sensor_lists[SENSOR_DN3].is_black_line_detected_){
      state_machine.is_black_frame_edge_detected_ = true;
    }
    
    if((state_machine.is_black_frame_edge_detected_)&&(perception.IsAllBlank())){
      state_machine.is_black_frame_edge_over_ = true;
    }

    if(state_machine.is_black_frame_edge_over_){
      state_machine.state = state_machine.JoinTheLine;
    }
  }

  if(state_machine.JoinTheLine == state_machine.state){
    if((sensor_lists[SENSOR_DN1].gray_scale_ > 90)&&(!state_machine.is_black_line_detected_)){
      state_machine.is_black_line_detected_ = true;
    }else if(state_machine.is_black_line_detected_){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
      if((sensor_lists[SENSOR_DN4].gray_scale_ > 70) && (sensor_lists[SENSOR_DN1].gray_scale_ < 50)){
        state_machine.state = state_machine.FollowTheLine;
      }
    }else{
      control.GoFixedSpeed();
    }
  }

  if(state_machine.FollowTheLine == state_machine.state){
    if(sensor_lists[SENSOR_DN1].is_black_line_detected_){
      state_machine.state = state_machine.NavigateCorners;
      turn_time_stamp = millis();
    }

    if((sensor_lists[SENSOR_DN5].is_black_line_detected_)&&(sensor_lists[SENSOR_DN3].gray_scale_ < 50)){
      state_machine.state = state_machine.NavigateCorners;
      turn_time_stamp = millis();
    }
    
    if((perception.IsAllBlank())&&(!state_machine.is_turning_back)){
      state_machine.is_turning_back = true;
      if((current_time - turn_time_stamp) > 2000){
        state_machine.state = state_machine.DetermineEnd;
      }
    }else if(state_machine.is_turning_back){
      control.Rotate(Robotic_sys::control::Control::CLOCKWISE);
      if(sensor_lists[SENSOR_DN2].gray_scale_ > 60){
        state_machine.is_turning_back = false;
      }
    }else{
      control.BangBangControl(sensor_lists);
    }
  }

  if(state_machine.NavigateCorners == state_machine.state){
    if((sensor_lists[SENSOR_DN1].is_black_line_detected_)&&(!state_machine.is_turning_left_)){
      state_machine.is_turning_left_ = true;
    }else if(state_machine.is_turning_left_){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
      if(sensor_lists[SENSOR_DN3].gray_scale_ > 60){
        state_machine.state = state_machine.FollowTheLine;
        state_machine.is_turning_left_ = false;
      }
      state_machine.is_turning_right_ = false;
    }
    
    if((sensor_lists[SENSOR_DN5].is_black_line_detected_)&&(!state_machine.is_turning_right_)&&(sensor_lists[SENSOR_DN3].gray_scale_ < 50)){
      state_machine.is_turning_right_ = true;
    }else if(state_machine.is_turning_right_ ){
      control.Rotate(Robotic_sys::control::Control::CLOCKWISE);
      if(sensor_lists[SENSOR_DN1].is_black_line_detected_){
        state_machine.is_turning_left_ = true;
        state_machine.is_turning_right_ = false;
      }else{
        if(sensor_lists[SENSOR_DN3].gray_scale_ > 80){
          state_machine.state = state_machine.FollowTheLine;
          state_machine.is_turning_right_ = false;
        }
      }
    }
  }

  if(state_machine.DetermineEnd == state_machine.state){
    if(!state_machine.is_return_yaw_recoreded_){
      theta2_yaw = kinematic.yaw;
      theta1_yaw = atan2(kinematic.position_y_, kinematic.position_x_);
      state_machine.is_return_yaw_recoreded_ = true;
    }

    double yaw_duration = theta2_yaw - kinematic.yaw;
    double value = PI + theta2_yaw - theta1_yaw;
    
    if(yaw_duration > 0.95*value){
      state_machine.state = state_machine.ReturnHome;
    }else{
      control.Rotate(Robotic_sys::control::Control::CLOCKWISE);
    }
  }

  if(state_machine.ReturnHome == state_machine.state){
    unsigned long duration = current_time - prev_pid_time_stamp;
    double target_yaw = PI - atan2(kinematic.position_y_, kinematic.position_x_);
    if (first_hit_) {
      first_hit_ = false;
      prev_pid_time_stamp = current_time;
    } else if(duration > 20) {
      if(
         ((kinematic.position_x_>-10)&&(kinematic.position_x_<10))|
         ((kinematic.position_y_>-10)&&(kinematic.position_y_<10))
        ){
       control.GoFixedSpeed(0,0);
       }else{
          control.ComputeHeadingCmd(target_yaw, kinematic.yaw, duration);
          prev_pid_time_stamp = current_time;
       }
      
    }
    
//    if(kinematic.position_x_<-10){
//      Robotic_sys::common::BuzzlePlayTone(300);
//    }
//
//    if(kinematic.position_y_<-10){
//      Robotic_sys::common::BuzzlePlayTone(300,500);
//    }
  }
}
