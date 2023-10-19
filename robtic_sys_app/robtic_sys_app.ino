

#include "common.h"
#include "localization.h"
#include "perception.h"
#include "control.h"

using Robotic_sys::common::Result_state;

#define INIT_COMPONENT()                                 \
  Robotic_sys::localization::Localization localization;  \
  Robotic_sys::perception::Perception perception;        \
  Robotic_sys::control::Control control;                 \
  Robotic_sys::common::StateMachine state_machine;       \

INIT_COMPONENT();

void setup() {
  Serial.begin(9600);

  // waiting for conection finish
  delay(3000);

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
  
  Robotic_sys::common::BuzzlePlayTone(300);
  
  Serial.println("init done!");
}

void loop() {
  Result_state state = Result_state::State_Failed;

  Robotic_sys::perception::Sensor sensor_lists[SENSOR_NUM];
  
  state = perception.GetGrayScale(sensor_lists);

//  debug
  for (int sensor_number = 0; sensor_number < SENSOR_NUM; sensor_number++) {
    Serial.print(sensor_lists[sensor_number].sensor_time_);
    Serial.print(" ");
    Serial.print(sensor_lists[sensor_number].gray_scale_);
    Serial.print("   ");
  }
  
  Serial.println("   ");

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
      Robotic_sys::common::BuzzlePlayTone(300);
    }
  }

  if(state_machine.JoinTheLine == state_machine.state){
    if((sensor_lists[SENSOR_DN1].gray_scale_ > 90)&&(!state_machine.is_black_line_detected_)){
      state_machine.is_black_line_detected_ = true;
    }else if(state_machine.is_black_line_detected_){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
      if((sensor_lists[SENSOR_DN4].gray_scale_ > 90) && (sensor_lists[SENSOR_DN1].gray_scale_ < 50)){
        state_machine.state = state_machine.FollowTheLine;
      }
    }else{
      control.GoFixedSpeed();
    }
  }

  if(state_machine.FollowTheLine == state_machine.state){
    if((sensor_lists[SENSOR_DN1].is_black_line_detected_) || 
       ((sensor_lists[SENSOR_DN5].is_black_line_detected_)&&(sensor_lists[SENSOR_DN3].gray_scale_ < 50))){
      state_machine.state = state_machine.NavigateCorners;
    }
    
    if((perception.IsAllBlank())&&(!state_machine.is_turning_back)){
      state_machine.is_turning_back = true;
    }else if(state_machine.is_turning_back){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
      if(sensor_lists[SENSOR_DN2].gray_scale_ > 80){
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
      if(sensor_lists[SENSOR_DN3].gray_scale_ > 80){
        state_machine.state = state_machine.FollowTheLine;
        state_machine.is_turning_left_ = false;
      }
      state_machine.is_turning_right_ = false;
    }
    
    if((sensor_lists[SENSOR_DN5].is_black_line_detected_)&&(!state_machine.is_turning_right_)){
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
}
