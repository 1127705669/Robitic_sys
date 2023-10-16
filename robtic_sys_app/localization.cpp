/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include "common.h"
#include "localization.h"

namespace Robotic_sys {
namespace localization {

using Robotic_sys::common::Result_state;

 const char* Localization::Name() const {return "localization";}

Result_state Localization::Init(){
 Serial.println("localization init, starting...");
 
 return Result_state::State_Ok;
}

Result_state Localization::Start(){
   

 return Result_state::State_Ok;
}

void Localization::Stop(){
 
}

} // namespace localization
} // namespace Robotic_sys
