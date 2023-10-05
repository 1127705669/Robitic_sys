/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"
#include "perception.h"

namespace Robotic_sys {
namespace perception {

using Robotic_sys::common::Result_state;

char Perception::Name() const { return "perception"; }

Result_state Perception::Init(){
 Serial.println("Perception init, starting...");
 
 return Result_state::State_Ok;
}

Result_state Perception::Start(){
   

 return Result_state::State_Ok;
}

void Perception::Stop(){
 
}

} // namespace perception
} // namespace Robotic_sys
