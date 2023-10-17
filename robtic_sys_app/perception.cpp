/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"
#include "perception.h"

namespace Robotic_sys {
namespace perception {

using Robotic_sys::common::Result_state;

Sensor::Init(int SensorPin, unsigned long threshold) {
  // Set line sensor pin to input
  pinMode(SensorPin, INPUT);
  pin = SensorPin;
  threshold_ = threshold;
}

void Perception::Reset(){
  for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
    sensor_lists_[sensor_number].is_updated_ = false;
    sensor_lists_[sensor_number].is_black_line_detected = false;
  }
}

Result_state Perception::GetGrayScale(Sensor* sensor_lists){

  // for find max sensor
  unsigned long max_gray_scale = 0;

  // set output of sensor
  for(int sensor_number = 0; sensor_number < sensor_number_; sensor_number++){
    pinMode(sensor_lists_[sensor_number].pin, OUTPUT);
    digitalWrite(sensor_lists_[sensor_number].pin, HIGH);
  }
  
  delayMicroseconds(sensor_charge_time_);

  // recieve data 
  for(int sensor_number = 0; sensor_number < sensor_number_; sensor_number++){
    pinMode(sensor_lists_[sensor_number].pin, INPUT);
  }
  
  unsigned long start_time = micros();
  
  while(
        ((false == sensor_lists_[0].is_updated_)) ||
        ((false == sensor_lists_[1].is_updated_)) ||
        ((false == sensor_lists_[2].is_updated_)) ||
        ((false == sensor_lists_[3].is_updated_)) ||
        ((false == sensor_lists_[4].is_updated_))
        ) {
    for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
      if((false == sensor_lists_[sensor_number].is_updated_)&&(LOW == (digitalRead(sensor_lists_[sensor_number].pin)))){
        unsigned long current_time = micros();
        sensor_lists_[sensor_number].sensor_time_ = current_time - start_time;
        sensor_lists_[sensor_number].is_updated_ = true;
        
        if(sensor_lists_[sensor_number].sensor_time_ > max_gray_scale){
          max_gray_scale = sensor_lists_[sensor_number].sensor_time_;
          max_sensor_index = sensor_number;
        }
      }
    }
  }

  for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
    if(sensor_lists_[sensor_number].sensor_time_ > sensor_lists_[sensor_number].threshold_){
      sensor_lists_[sensor_number].is_black_line_detected = true; 
    }
    
    sensor_lists[sensor_number] = sensor_lists_[sensor_number];
  }

  Reset();
  
  return Result_state::State_Ok;
}

const char* Perception::Name() const {return "perception";}

Result_state Perception::Init(){
  Serial.println("Perception init, starting...");

  // Set some initial pin modes and states
  pinMode(emitPin, INPUT); // Set EMIT as an input (off)

  sensor_lists_[0].Init(LineSensorPin_1, line_sensor_threshold_1_);
  sensor_lists_[1].Init(LineSensorPin_2, line_sensor_threshold_2_);
  sensor_lists_[2].Init(LineSensorPin_3, line_sensor_threshold_3_);
  sensor_lists_[3].Init(LineSensorPin_4, line_sensor_threshold_4_);
  sensor_lists_[4].Init(LineSensorPin_5, line_sensor_threshold_5_);

  pinMode(emitPin, OUTPUT);
  digitalWrite(emitPin, HIGH );

  Serial.println("perception agent init done!");
  
  return Result_state::State_Ok;
}

int Perception::GetMaxSensor(){
  return max_sensor_index;
}

bool Perception::IsAllBlank(){
  bool is_black = false;

  if(
     sensor_lists_[0].is_black_line_detected == true &&
     sensor_lists_[1].is_black_line_detected == true &&
     sensor_lists_[2].is_black_line_detected == true &&
     sensor_lists_[3].is_black_line_detected == true &&
     sensor_lists_[4].is_black_line_detected == true
    ){
    is_black = true;
  }
  return is_black;
}

void Perception::Stop(){

}

} // namespace perception
} // namespace Robotic_sys
