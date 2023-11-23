/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <Arduino.h>

#include "common.h"

namespace Robotic_sys {
namespace perception {

using Robotic_sys::common::Result_state;

class Sensor{
 public:

  Init(int SensorPin, unsigned long blank_threshold, unsigned long black_line_threshold);

  int pin;
  bool is_updated_ = false;
  bool is_black_line_detected_ = false;
  unsigned long sensor_time_ = 0;
  unsigned long black_line_threshold_;
  unsigned long blank_threshold_;
  int gray_scale_;
};

class Bumper{
 public: 
  Init(int bumper_pin);

  int pin_;
  bool is_first_hit_ = true;
  unsigned long min_bumper_time_ = 0;
  unsigned long prev_bumper_time_ = 0;
  unsigned long bumper_time_ = 0;
  bool is_updated_ = false;
  int collision_intensity;
};

class Perception{
 public:

  const char* Name() const;

  Result_state Init();

  void Reset();

  void Stop();

  void GetGrayScale(Sensor* sensor_lists);

  int GetMaxSensor();

  bool IsAllBlank();

  void CollisionDetect(Bumper* bumper_lists);

 private:

  const int emitPin = 11;
  
  // line sensor data
  static const int sensor_number_ = 5;
  const int sensor_charge_time_ = 10;
  int max_sensor_index;

  const int LineSensorPin_1 = 12;
  const int LineSensorPin_2 = A0;
  const int LineSensorPin_3 = A2;
  const int LineSensorPin_4 = A3;
  const int LineSensorPin_5 = A4;

  const unsigned long blank_threshold_1_ = 1050;
  const unsigned long blank_threshold_2_ = 800;
  const unsigned long blank_threshold_3_ = 700;
  const unsigned long blank_threshold_4_ = 800;
  const unsigned long blank_threshold_5_ = 1050;

  const unsigned long black_line_threshold_1_ = 2500;
  const unsigned long black_line_threshold_2_ = 2600;
  const unsigned long black_line_threshold_3_ = 1650;
  const unsigned long black_line_threshold_4_ = 2200;
  const unsigned long black_line_threshold_5_ = 2500;
  
  Sensor sensor_lists_[sensor_number_];

  // bumper sensor data
  static const int bumper_number_ = 2;
  const int bumper_charge_time_ = 10;  
  
  const int bumper_left_pin_ = 4;
  const int bumper_right_pin_ = 5;
  
  Bumper bumper_lists_[bumper_number_];
};

} // namespace perception
} // namespace Robotic_sys
