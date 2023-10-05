/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <Arduino.h>

#include "common.h"

namespace Robotic_sys {
namespace perception {

using Robotic_sys::common::Result_state;

class Perception : public Robotic_sys::common::ConponentBase{
 public:

  char Name() const override;

  Result_state Init() override;

  Result_state Start() override;

  void Stop() override;

  virtual ~Perception() = default;

 private:
  
  const int sensorPin = 0xA0;
  const int threshold = 500; // 调整此阈值以适应你的传感器和环境

//  Result_state Send_result();

};

} // namespace perception
} // namespace Robotic_sys
