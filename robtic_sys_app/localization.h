/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common.h"

namespace Robotic_sys {
namespace localization {

using Robotic_sys::common::Result_state;

class Localization {
 public:

  const char* Name() const;

  Result_state Init();

  Result_state Start();

  void Stop();

  virtual ~Localization() = default;

 private:

};

} // namespace localization
} // namespace Robotic_sys
