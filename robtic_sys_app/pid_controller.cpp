/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "pid_controller.h"

namespace Robotic_sys {
namespace control {

double PIDController::Control(const double error, const double dt) {
  double output = 0;

  return output;
}

void PIDController::Init() {
  SetPID();
}

void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
  output_saturation_status_ = 0;
}

void PIDController::SetPID() {

}

} // namespace control
} // namespace Robotic_sys
