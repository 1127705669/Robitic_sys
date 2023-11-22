/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <Arduino.h>
#include <string.h>

#define SENSOR_NUM 5
#define SENSOR_DN1 0
#define SENSOR_DN2 1
#define SENSOR_DN3 2
#define SENSOR_DN4 3
#define SENSOR_DN5 4

#define BUMPER_NUM 2
#define BL 0
#define BR 1

#define GEAR_RATIO 358.3
#define RADIUS 16
#define AXIS_LENGTH 42.5

#define GRAVITY 9.8

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
  bool is_turning_left_ = false;
  bool is_turning_right_ = false;
  bool is_return_yaw_recoreded_ = false;
};

enum Result_state {
  State_Ok,
  State_Running,
  State_Failed
};

void BuzzleInit();

void BuzzlePlayTone(int duration = 200, int frequency = 800);

template <typename T>
T LowPassFilter(T prev_value, T current_value ,double weight = 0.8){
  T output;
  output = weight*current_value + (1.0 - weight)*prev_value;

  return output;
}

} // namespace common
} // namespace Robotic_sys
