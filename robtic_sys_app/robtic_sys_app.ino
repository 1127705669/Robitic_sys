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
#include "perception.h"
#include "control.h"

#define INIT_COMPONENT()                                       \
  static Robotic_sys::perception::Perception perception;       \
  static Robotic_sys::control::Control control;                \
  static Robotic_sys::common::BuzzleAgent buzzle;              \

INIT_COMPONENT();

void setup() {
  Serial.begin(9600);

  delay(1000);

  if(Robotic_sys::common::Result_state::State_Ok != buzzle.Init()){
    
  }
  
  if(Robotic_sys::common::Result_state::State_Ok != perception.Init()){
    
  }

  if(Robotic_sys::common::Result_state::State_Ok != control.Init()){
    
  }

  buzzle.Play(500);
  
  
}

void loop() {

  auto a = perception.Start();

  auto b = control.Start();
  
}
