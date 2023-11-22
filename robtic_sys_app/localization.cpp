/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common.h"
#include "localization.h"

namespace Robotic_sys {
namespace localization {

using Robotic_sys::common::Result_state;

 const char* Localization::Name() const {return "localization";}

Result_state Localization::Init(){
 Serial.println("localization init, starting...");
 
 return Result_state::State_Ok;
}

Result_state Localization::Start(){
   

 return Result_state::State_Ok;
}

void Localization::Stop(){
 
}

void Localization::CalculateSpeed(long count_e0, long count_e1, long duration){
  long duration_count_right = count_e0 - start_count_right;
  right_linear_distance = -RADIUS*((double)duration_count_right/GEAR_RATIO*(2*PI));
  right_wheel_speed = right_linear_distance/((double)duration/1000);
  start_count_right = count_e0;

  long duration_count_left = count_e1 - start_count_left;
  left_linear_distance = -RADIUS*((double)duration_count_left/GEAR_RATIO*(2*PI));
  left_wheel_speed = left_linear_distance/((double)duration/1000);
  start_count_left = count_e1;

//  Serial.print(left_wheel_speed);
//  Serial.print("  ");
//  Serial.println(right_wheel_speed);
  
//  Serial.print(left_linear_distance);
//  Serial.print("   ");
//  Serial.println(right_linear_distance);
}

void Localization::ComputePosition(long duration){
  kinematic.update(left_linear_distance, right_linear_distance, left_wheel_speed, right_wheel_speed, duration);
}

} // namespace localization
} // namespace Robotic_sys
