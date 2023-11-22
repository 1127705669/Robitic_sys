/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

# include <Wire.h>
# include <LSM6.h>

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

  void ImuRead();

  virtual ~Localization() = default;

  double left_wheel_speed;
  double right_wheel_speed;

  // imu
  double filted_acceleration_x_;
  double filted_acceleration_y_;
  double filted_acceleration_z_;

 private:
  Kinematics_c kinematic;
  
  long start_count_left;
  long start_count_right;

  double left_linear_distance = 0;
  double right_linear_distance = 0;

  LSM6 imu;

  bool imu_first_hit = true;
  
  const double conversion_factor_accelerometer_ = 0.061;
  const double conversion_factor_gyroscope_ = 8.75;

  double prev_acceleration_x_;
  double prev_acceleration_y_;
  double prev_acceleration_z_;
  
  double prev_gyroscope_x_;
  double prev_gyroscope_y_;
  double prev_gyroscope_z_;
};

} // namespace localization
} // namespace Robotic_sys
