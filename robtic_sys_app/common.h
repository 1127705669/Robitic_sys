/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef EDRIVE_APP_EDRIVE_H_
#define EDRIVE_APP_EDRIVE_H_

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

  virtual char Name() const = 0;

};

//class ComponentAgent(){
// public:
//  virtual Result_state Init();
//
//  virtual Reslut_state Start();
//}

class BuzzleAgent {
 public:
 
  void Play(int play_time) {
  tone(buzzerPin, 1000);
  delay(play_time);
  noTone(buzzerPin); }

  Result_state Init(){ pinMode(buzzerPin, OUTPUT); };

 private: 
  const int buzzerPin = 9;
};

} // namespace common
} // namespace Robotic_sys

#endif
