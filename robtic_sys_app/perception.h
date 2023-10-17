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

  Init(int SensorPin);

  int pin;
  bool is_updated_ = false;
  unsigned long sensor_time_ = 0;
  char sensor_name;
};

class Perception{
 public:

  const char* Name() const;

  Result_state Init();

  void Reset();

  void Stop();

  Result_state GetGrayScale(unsigned long* gray_scale);

  unsigned long GetMaxScale();

  bool IsBlank();

 private:
  static const int sensor_number_ = 5;
  
  const int emitPin = 11;
  const int threshold = 2000;
  
  const int LineSensorPin_1 = 12;
  const int LineSensorPin_2 = A0;
  const int LineSensorPin_3 = A2;
  const int LineSensorPin_4 = A3;
  const int LineSensorPin_5 = A4;
  
  const int sensor_charge_time_ = 10;

  unsigned long max_gray_scale_;
  unsigned long gray_scale_[sensor_number_];
  Sensor sensor_lists_[sensor_number_];
  
};

} // namespace perception
} // namespace Robotic_sys
