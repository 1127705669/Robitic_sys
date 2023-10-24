

#include "common.h"
#include "localization.h"
#include "perception.h"
#include "control.h"
#include "kinematics.h"
#include "encoders.h"
#include "task.h"

using Robotic_sys::common::Result_state;

#define INIT_COMPONENT()                                 \
  Robotic_sys::localization::Localization localization;  \
  Robotic_sys::perception::Perception perception;        \
  Robotic_sys::control::Control control;                 \
  Robotic_sys::common::StateMachine state_machine;       \
  Kinematics_c kinematic;                                \

INIT_COMPONENT();
bool is_frist_time = false;
unsigned long last_time;

double left_speed;
double right_speed;

double last_yaw;
double return_yaw;

unsigned long turn_time;

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
  
//  Robotic_sys::common::BuzzleInit();

  setupEncoder0();
  
  setupEncoder1();

  setupTimer3();
  
//  Robotic_sys::common::BuzzlePlayTone(300);

  Serial.begin(9600);

  // waiting for conection finish
  delay(1500);
  
  Serial.println("init done!");
}

void loop() {
  unsigned long current_time = micros();
  
  if(!is_frist_time){
    last_time = micros();
    is_frist_time = true;
  }

  unsigned long duration = current_time - last_time;

  if((left_speed != left_wheel_speed)||(right_speed != right_wheel_speed)){
    kinematic.update(left_wheel_speed, right_wheel_speed, duration);
    left_speed = left_wheel_speed;
    right_speed = right_wheel_speed;
    last_time = current_time;
  }
  
  Result_state state = Result_state::State_Failed;

  Robotic_sys::perception::Sensor sensor_lists[SENSOR_NUM];
  
  state = perception.GetGrayScale(sensor_lists);

  control.ComputeControlCmd(sensor_lists, 0);

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
//      Robotic_sys::common::BuzzlePlayTone(300);
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
    if(sensor_lists[SENSOR_DN1].is_black_line_detected_){
      state_machine.state = state_machine.NavigateCorners;
      turn_time = micros();
    }

    if((sensor_lists[SENSOR_DN5].is_black_line_detected_)&&(sensor_lists[SENSOR_DN3].gray_scale_ < 50)){
      state_machine.state = state_machine.NavigateCorners;
      turn_time = micros();
    }
    
    if((perception.IsAllBlank())&&(!state_machine.is_turning_back)){
      state_machine.is_turning_back = true;
      if((current_time - turn_time) > 2000000){
        state_machine.state = state_machine.ReturnHome;
      }
    }else if(state_machine.is_turning_back){
      control.Rotate(Robotic_sys::control::Control::CLOCKWISE);
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

  if(state_machine.ReturnHome == state_machine.state){
    if(!state_machine.is_return_yaw_recoreded_){
      last_yaw = kinematic.yaw;
      state_machine.is_return_yaw_recoreded_ = true;
      return_yaw = atan2(kinematic.position_y_, kinematic.position_x_);
    }
    control.Rotate(Robotic_sys::control::Control::CLOCKWISE);

    if((kinematic.yaw - last_yaw) < -(-last_yaw + PI+ return_yaw)){
      control.GoFixedSpeed();
    }
    
    
    double distance = sqrt(kinematic.position_x_*kinematic.position_x_ + kinematic.position_y_*kinematic.position_y_);
  }
}
