/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"
#include "perception.h"

namespace Robotic_sys {
namespace perception {

using Robotic_sys::common::Result_state;

Sensor::Init(int SensorPin, unsigned long blank_threshold, unsigned long black_line_threshold) {
  // Set line sensor pin to input
  pinMode(SensorPin, INPUT);
  pin = SensorPin;
  blank_threshold_ = blank_threshold;
  black_line_threshold_ = black_line_threshold;
}

Bumper::Init(int bumper_pin){
  pin_ = bumper_pin;
  pinMode(pin_, INPUT);
}

void Perception::Reset(){
  for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
    sensor_lists_[sensor_number].is_updated_ = false;
    sensor_lists_[sensor_number].is_black_line_detected_ = false;
  }
}

void Perception::GetGrayScale(Sensor* sensor_lists){

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

        sensor_lists_[sensor_number].gray_scale_ = (int)100*((float)(sensor_lists_[sensor_number].sensor_time_ - sensor_lists_[sensor_number].blank_threshold_)/
        (float)(sensor_lists_[sensor_number].black_line_threshold_ - sensor_lists_[sensor_number].blank_threshold_));

        if(sensor_lists_[sensor_number].black_line_threshold_ < sensor_lists_[sensor_number].sensor_time_){
          sensor_lists_[sensor_number].gray_scale_ = 100;   
        }

        if(sensor_lists_[sensor_number].blank_threshold_ > sensor_lists_[sensor_number].sensor_time_){
          sensor_lists_[sensor_number].gray_scale_ = 0;   
        }
        
        if(sensor_lists_[sensor_number].sensor_time_ > max_gray_scale){
          max_gray_scale = sensor_lists_[sensor_number].sensor_time_;
          max_sensor_index = sensor_number;
        }
      }
    }
  }

  for (int sensor_number = 0; sensor_number < sensor_number_; sensor_number++) {
    if(sensor_lists_[sensor_number].sensor_time_ > sensor_lists_[sensor_number].black_line_threshold_){
      sensor_lists_[sensor_number].is_black_line_detected_ = true; 
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

  sensor_lists_[SENSOR_DN1].Init(LineSensorPin_1, blank_threshold_1_, black_line_threshold_1_);
  sensor_lists_[SENSOR_DN2].Init(LineSensorPin_2, blank_threshold_2_, black_line_threshold_2_);
  sensor_lists_[SENSOR_DN3].Init(LineSensorPin_3, blank_threshold_3_, black_line_threshold_3_);
  sensor_lists_[SENSOR_DN4].Init(LineSensorPin_4, blank_threshold_4_, black_line_threshold_4_);
  sensor_lists_[SENSOR_DN5].Init(LineSensorPin_5, blank_threshold_5_, black_line_threshold_5_);

  bumper_lists_[BL].Init(bumper_left_pin_);
  bumper_lists_[BR].Init(bumper_right_pin_);

  pinMode(emitPin, OUTPUT);
  digitalWrite(emitPin, HIGH);

  Serial.println("perception agent init done!");
  
  return Result_state::State_Ok;
}

int Perception::GetMaxSensor(){
  return 0;
}

bool Perception::IsAllBlank(){
  bool is_black = false;

  if(
     sensor_lists_[SENSOR_DN1].gray_scale_ == 0 &&
     sensor_lists_[SENSOR_DN2].gray_scale_ == 0 &&
     sensor_lists_[SENSOR_DN3].gray_scale_ == 0 &&
     sensor_lists_[SENSOR_DN4].gray_scale_ == 0 &&
     sensor_lists_[SENSOR_DN5].gray_scale_ == 0
    ){
    is_black = true;
  }
  return is_black;
}

void Perception::CollisionDetect(Bumper* bumper_lists){
  // set output of bumper sensor
  for(int sensor_number = 0; sensor_number < bumper_number_; sensor_number++){
    pinMode(bumper_lists_[sensor_number].pin_, OUTPUT);
    digitalWrite(bumper_lists_[sensor_number].pin_, HIGH);
  }

  delayMicroseconds(bumper_charge_time_);

  // recieve data 
  for(int sensor_number = 0; sensor_number < bumper_number_; sensor_number++){
    pinMode(bumper_lists_[sensor_number].pin_, INPUT);
  }
  
  volatile unsigned long start_time = micros();

  while(((false == bumper_lists_[BL].is_updated_)) || ((false == bumper_lists_[BR].is_updated_))){
    for (int sensor_number = 0; sensor_number < bumper_number_; sensor_number++) {
      if((false == bumper_lists_[sensor_number].is_updated_)&&(LOW == (digitalRead(bumper_lists_[sensor_number].pin_)))){
        volatile unsigned long current_time = micros();
        bumper_lists_[sensor_number].bumper_time_ = current_time - start_time;
        bumper_lists_[sensor_number].is_updated_ = true;
      }
    }        
  }

  for (int sensor_number = 0; sensor_number < bumper_number_; sensor_number++) {
    bumper_lists_[sensor_number].is_updated_ = false;
    bumper_lists[sensor_number] = bumper_lists_[sensor_number];
  }
}

void Perception::Stop(){

}

} // namespace perception
} // namespace Robotic_sys
