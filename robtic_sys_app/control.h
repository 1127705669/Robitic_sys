/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common.h"
#include "perception.h"
#include "pid_controller.h"

namespace Robotic_sys {
namespace control {

using Robotic_sys::common::Result_state;

class Motors{
 public:
  void Init();
  
  void SetMontorPower(int left_pwm, int right_pwm);

 private:
  // the setup function runs once when you press reset or power the board
  const int motor_left_direction_pin_ = 16;
  const int motor_right_direction_pin_ = 15;
  const int motor_left_speed_pin_ = 10;
  const int motor_right_speed_pin_ = 9;
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

  void BangBangControl(Robotic_sys::perception::Sensor* sensor_lists);

  void ComputeControlCmd(Robotic_sys::perception::Sensor* sensor_lists, const double dt);

  void GoFixedSpeed(int left_pwm = 20, int right_pwm = 20);

  void Rotate(RotateType rotate_direction);

  void Stop();

  virtual ~Control() = default;

 private:

  Motors motor_;

  PIDController left_pid_controller_;
  PIDController right_pid_controller_;

  Result_state ProduceControlCommand();

  const int BiasPWM = 22;

  const int MaxTurnPWM = 15;

 protected:
  void ComputeLateralErrors();

};

} // namespace control
} // namespace Robotic_sys
