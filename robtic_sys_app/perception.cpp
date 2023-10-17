/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"
#include "perception.h"

namespace Robotic_sys {
namespace perception {

using Robotic_sys::common::Result_state;

Sensor::Init(int SensorPin) {
  // Set line sensor pin to input
  pinMode(SensorPin, INPUT);
  pin = SensorPin;
}

void Perception::Reset(){
  for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
    sensor_lists_[sensor_number].is_updated_ = false;
  }
}

Result_state Perception::GetGrayScale(unsigned long* gray_scale){
  unsigned long max_gray_scale = 0;
  
  for(int sensor_number = 0; sensor_number < sensor_number_; sensor_number++){
    pinMode(sensor_lists_[sensor_number].pin, OUTPUT);
    digitalWrite(sensor_lists_[sensor_number].pin, HIGH);
  }
  
  delayMicroseconds(sensor_charge_time_);
  
  for(int sensor_number = 0; sensor_number < sensor_number_; sensor_number++){
    pinMode(sensor_lists_[sensor_number].pin, INPUT);
  }
  
  unsigned long start_time = micros();
  
  while((digitalRead(sensor_lists_[0].pin) == HIGH) ||
        (digitalRead(sensor_lists_[1].pin) == HIGH) ||
        (digitalRead(sensor_lists_[2].pin) == HIGH) ||
        (digitalRead(sensor_lists_[3].pin) == HIGH) ||
        (digitalRead(sensor_lists_[4].pin) == HIGH)) {
    for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
      if((false == sensor_lists_[sensor_number].is_updated_)&&(LOW == (digitalRead(sensor_lists_[sensor_number].pin)))){
        unsigned long current_time = micros();
        sensor_lists_[sensor_number].sensor_time_ = current_time - start_time;
        sensor_lists_[sensor_number].is_updated_ = true;
        if(max_gray_scale < sensor_lists_[sensor_number].sensor_time_){
          max_gray_scale = sensor_lists_[sensor_number].sensor_time_;
        }
      }
    }
  }

  for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
    gray_scale[sensor_number] = sensor_lists_[sensor_number].sensor_time_;
  }

  Reset();
  
  return Result_state::State_Ok;
}

const char* Perception::Name() const {return "perception";}

Result_state Perception::Init(){
  Serial.println("Perception init, starting...");

  // Set some initial pin modes and states
  pinMode(emitPin, INPUT); // Set EMIT as an input (off)

  sensor_lists_[0].Init(LineSensorPin_1);
  sensor_lists_[1].Init(LineSensorPin_2);
  sensor_lists_[2].Init(LineSensorPin_3);
  sensor_lists_[3].Init(LineSensorPin_4);
  sensor_lists_[4].Init(LineSensorPin_5);

  pinMode(emitPin, OUTPUT);
  digitalWrite(emitPin, HIGH );

  Serial.println("perception agent init done!");
  
  return Result_state::State_Ok;
}

unsigned long Perception::GetMaxScale(){
  return max_gray_scale_;
}

bool Perception::IsBlank(){
  bool is_black = false;

  if(
     gray_scale_[0] < 1100 &&
     gray_scale_[1] < 800 &&
     gray_scale_[2] < 700 &&
     gray_scale_[3] < 800 &&
     gray_scale_[4] < 1100
    ){
    delay(200);
  }

  if(
     gray_scale_[0] < 1100 &&
     gray_scale_[1] < 800 &&
     gray_scale_[2] < 700 &&
     gray_scale_[3] < 800 &&
     gray_scale_[4] < 1100
    ){
    is_black = true;
  }
  return is_black;
}

void Perception::Stop(){

}

} // namespace perception
} // namespace Robotic_sys
