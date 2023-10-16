/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common.h"

namespace Robotic_sys {
namespace localization {

using Robotic_sys::common::Result_state;

class Localization : public Robotic_sys::common::ConponentBase{
 public:

  const char* Name() const override;

  Result_state Init() override;

  Result_state Start() override;

  void Stop() override;

  virtual ~Localization() = default;

 private:

};

} // namespace localization
} // namespace Robotic_sys
