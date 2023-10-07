/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#include <Arduino.h>

#include "common.h"

namespace Robotic_sys {
namespace common {

void BuzzleInit() {
  const int buzzerPin = 6;
  pinMode(buzzerPin, OUTPUT);
}

void BuzzlePlayTone(int duration, int frequency = 1000) {
  const int buzzerPin = 6;
  tone(buzzerPin, 1000); // 发出1000Hz的声音
  delay(duration);             // 持续500毫秒
  noTone(buzzerPin);      // 停止声音
}

} // namespace common
} // namespace Robotic_sys
