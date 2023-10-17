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

  Init(int SensorPin, unsigned long threshold);

  int pin;
  bool is_updated_ = false;
  bool is_black_line_detected = false;
  unsigned long sensor_time_ = 0;
  unsigned long threshold_;
};

class Perception{
 public:

  const char* Name() const;

  Result_state Init();

  void Reset();

  void Stop();

  Result_state GetGrayScale(Sensor* sensor_lists);

  Sensor GetMaxSensor();

  bool IsAllBlank();

 private:
  static const int sensor_number_ = 5;
  const int sensor_charge_time_ = 10;
  
  const int emitPin = 11;
  
  const int LineSensorPin_1 = 12;
  const int LineSensorPin_2 = A0;
  const int LineSensorPin_3 = A2;
  const int LineSensorPin_4 = A3;
  const int LineSensorPin_5 = A4;

  const unsigned long line_sensor_threshold_1_ = 1050;
  const unsigned long line_sensor_threshold_2_ = 800;
  const unsigned long line_sensor_threshold_3_ = 700;
  const unsigned long line_sensor_threshold_4_ = 800;
  const unsigned long line_sensor_threshold_5_ = 1050;
  
  int max_sensor_index = 0;
  Sensor sensor_lists_[sensor_number_];
  
};

} // namespace perception
} // namespace Robotic_sys
