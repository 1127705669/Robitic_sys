/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <Arduino.h>
#include <string.h>

namespace Robotic_sys {
namespace common {

class StateMachine {
 public: 
  int state = Init;

  enum StateType{
    Init,
    JoinTheLine,
    FollowTheLine,
    NavigateCorners,
    NavigateIntersection,
    DetermineEnd,
    ReturnHome
  };

  bool is_black_frame_edge_detected_ = false;
  bool is_black_frame_edge_over_ = false;
  bool is_black_line_detected_ = false;
  bool is_turning_back = false;
};

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

void BuzzlePlayTone(int duration = 200, int frequency = 800);

} // namespace common
} // namespace Robotic_sys
