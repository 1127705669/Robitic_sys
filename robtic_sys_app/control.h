/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common.h"

namespace Robotic_sys {
namespace control {

using Robotic_sys::common::Result_state;

class Motors{
 public:
  void Init();
  
  void SetMontorPower(int left_pwm, int right_pwm);

 private:
  // the setup function runs once when you press reset or power the board
  const int motor_left_direction_pin_ = 16; // 电机1方向控制引脚
  const int motor_right_direction_pin_ = 15; // 电机2方向控制引脚
  const int motor_left_speed_pin_ = 10; // 电机2速度控制引脚
  const int motor_right_speed_pin_ = 9; // 电机1速度控制引脚
};

class Control{
 public:

  enum RotateType {
    CLOCKWISE,
    ANTICLOCKWISE,
  };

  const char* Name() const;

  Result_state Init();

  Result_state Start();

  void BangBangControl(unsigned long* gray_scale);

  void GoFixedSpeed(int left_pwm = 20, int right_pwm = 20);

  void Rotate(RotateType rotate_direction);

  void Stop();

  virtual ~Control() = default;

 private:

  Motors motor_;

  Result_state ProduceControlCommand();

 protected:
  void ComputeLateralErrors();

};

} // namespace control
} // namespace Robotic_sys
