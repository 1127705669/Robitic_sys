/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <Arduino.h>
#include <string.h>

namespace Robotic_sys {
namespace common {

enum Result_state {
  State_Ok,
  State_Running,
  State_Failed
};

class ConponentBase {
 public:

  virtual Result_state Init() = 0;

  virtual Result_state Start() = 0;

  virtual void Stop() = 0;

  virtual const char* Name() const = 0;

};

void BuzzleInit();

void BuzzlePlayTone(int duration, int frequency = 1000);

} // namespace common
} // namespace Robotic_sys
