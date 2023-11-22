/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common.h"

#include "kinematics.h"

namespace Robotic_sys {
namespace localization {

using Robotic_sys::common::Result_state;

class Localization {
 public:

  const char* Name() const;

  Result_state Init();

  Result_state Start();

  void Stop();

  void CalculateSpeed(long count_e0, long count_e1, long duration);

  void ComputePosition(long duration);

  virtual ~Localization() = default;

  double left_wheel_speed;
  double right_wheel_speed;

 private:
  Kinematics_c kinematic;
  
  long start_count_left;
  long start_count_right;

  double left_linear_distance = 0;
  double right_linear_distance = 0;

};

} // namespace localization
} // namespace Robotic_sys
