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
  
  void SetMontorPower(int l_pwm, int r_pwm);

 private:
  // the setup function runs once when you press reset or power the board
  const int motorSpeedPin1 = 9; // 电机1速度控制引脚
  const int motorDirectionPin1 = 8; // 电机1方向控制引脚
  const int motorSpeedPin2 = 10; // 电机2速度控制引脚
  const int motorDirectionPin2 = 11; // 电机2方向控制引脚
};

class Control : public Robotic_sys::common::ConponentBase{
 public:

  const char* Name() const override;

  Result_state Init() override;

  Result_state Start() override;

  void Stop() override;

  virtual ~Control() = default;

 private:

  Motors motor_;

  Result_state ProduceControlCommand();

 protected:
  void ComputeLateralErrors();

};

} // namespace control
} // namespace Robotic_sys
