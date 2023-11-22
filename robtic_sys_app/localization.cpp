/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "localization.h"

namespace Robotic_sys {
namespace localization {

using Robotic_sys::common::Result_state;

 const char* Localization::Name() const {return "localization";}

Result_state Localization::Init(){
 Serial.println("localization init, starting...");
 Wire.begin();

 // Check the IMU initialised ok.
  if (!imu.init() ) {  // no..? :(

    // Since we failed to communicate with the
    // IMU, we put the robot into an infinite
    // while loop and report the error.
    while (1) {
      Serial.println("Failed to detect and initialize IMU!");
      delay(1000);
    }
  }

  // IMU initialise ok!
  // Set the IMU with default settings.
  imu.enableDefault();
  
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

void Localization::ImuRead(){
  // Make a read of the sensor.
  imu.read();

  double converted_raw_acceleration_x = imu.a.x*conversion_factor_accelerometer_/1000*GRAVITY;
  double converted_raw_acceleration_y = imu.a.y*conversion_factor_accelerometer_/1000*GRAVITY;
  double converted_raw_acceleration_z = imu.a.z*conversion_factor_accelerometer_/1000*GRAVITY;

  if(imu_first_hit){
    prev_acceleration_x_ = converted_raw_acceleration_x;
    prev_acceleration_y_ = converted_raw_acceleration_y;
    prev_acceleration_z_ = converted_raw_acceleration_z;
    imu_first_hit = false;
  }

  filted_acceleration_x_ = Robotic_sys::common::LowPassFilter(prev_acceleration_x_, converted_raw_acceleration_x);
  filted_acceleration_y_ = Robotic_sys::common::LowPassFilter(prev_acceleration_y_, converted_raw_acceleration_y);
  filted_acceleration_z_ = Robotic_sys::common::LowPassFilter(prev_acceleration_z_, converted_raw_acceleration_z);

  prev_acceleration_x_ = filted_acceleration_x_;
  prev_acceleration_y_ = filted_acceleration_y_;
  prev_acceleration_z_ = filted_acceleration_z_;
}

} // namespace localization
} // namespace Robotic_sys
