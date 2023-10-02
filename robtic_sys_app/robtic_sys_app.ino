/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

#include "common.h"
#include "controller.h"

// the setup function runs once when you press reset or power the board
int motorSpeedPin1 = 9; // 电机1速度控制引脚
int motorDirectionPin1 = 8; // 电机1方向控制引脚
int motorSpeedPin2 = 10; // 电机2速度控制引脚
int motorDirectionPin2 = 11; // 电机2方向控制引脚

void setup() {
  Robotic_sys::common::Result_state state;
  
  // 设置电机控制引脚为输出模式
  pinMode(motorSpeedPin1, OUTPUT);
  pinMode(motorDirectionPin1, OUTPUT);
  pinMode(motorSpeedPin2, OUTPUT);
  pinMode(motorDirectionPin2, OUTPUT);
}

void loop() {
  // 向前移动
  Serial.println("Debug: Code is running.");
  digitalWrite(motorDirectionPin1, HIGH); // 设置电机1正向
  digitalWrite(motorDirectionPin2, HIGH); // 设置电机2正向
  analogWrite(motorSpeedPin1, 0); // 设置电机1速度最大
  analogWrite(motorSpeedPin2, 0); // 设置电机2速度最大

  // 在这里可以添加延迟或其他控制逻辑
}
