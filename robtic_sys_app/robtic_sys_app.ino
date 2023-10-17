/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

#include "common.h"
#include "localization.h"
#include "perception.h"
#include "control.h"

#define SENSOR_NUM 5

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

  int max_sensor_index = perception.GetMaxSensor();

//  debug
  for (int sensor_number = 0; sensor_number < SENSOR_NUM; sensor_number++) {
    Serial.print(sensor_lists[sensor_number].sensor_time_);
    Serial.print(" ");
    Serial.print(sensor_lists[sensor_number].is_black_line_detected);
    Serial.print("   ");
  }
  Serial.print(sensor_lists[max_sensor_index].sensor_time_);
  
  Serial.println("   ");

  if(state_machine.Init == state_machine.state){
    control.GoFixedSpeed();
    delay(1000);
    state_machine.state = state_machine.JoinTheLine;
  }

  if(state_machine.JoinTheLine == state_machine.state){
    
    if((!perception.IsAllBlank())&&(!state_machine.is_black_line_detected_)){
      state_machine.is_black_line_detected_ = true;
      Robotic_sys::common::BuzzlePlayTone(300);
    }else if(true == state_machine.is_black_line_detected_){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
    }else{
      control.GoFixedSpeed();
    }
    
    if(sensor_lists[2].is_black_line_detected){
      state_machine.state = state_machine.FollowTheLine;
    }
  }

  if(state_machine.FollowTheLine == state_machine.state){

    if(sensor_lists[0].is_black_line_detected || sensor_lists[4].is_black_line_detected){
      state_machine.state = state_machine.NavigateCorners;
    }
    
    unsigned long gray_scale[5];
    for (int sensor_number = 0; sensor_number < SENSOR_NUM; sensor_number++) {
      gray_scale[sensor_number] = sensor_lists[sensor_number].sensor_time_;
    }

    if((perception.IsAllBlank())&&(!state_machine.is_turning_back)){
      state_machine.is_turning_back = true;
      control.GoFixedSpeed(15, 15);
    }else if(state_machine.is_turning_back){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
      if(sensor_lists[2].is_black_line_detected){
        state_machine.is_turning_back = false;
      }
    }else{
      control.BangBangControl(gray_scale);
    }
  }

  if(state_machine.NavigateCorners == state_machine.state){
    if(sensor_lists[0].is_black_line_detected){
      control.Rotate(Robotic_sys::control::Control::ANTICLOCKWISE);
      if(sensor_lists[2].is_black_line_detected){
        state_machine.state = state_machine.FollowTheLine;
      }
    }
  }
}
