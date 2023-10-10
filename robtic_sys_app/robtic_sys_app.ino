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
#include "localization.h"
#include "perception.h"
#include "control.h"

using Robotic_sys::common::Result_state;

#define INIT_COMPONENT()                                       \
  static Robotic_sys::localization::Localization localization; \
  static Robotic_sys::perception::Perception perception;       \
  static Robotic_sys::control::Control control;                \

INIT_COMPONENT();

void setup() {
  Serial.begin(9600);

  // waiting for conection finish
  delay(3000);

  if(Result_state::State_Ok != localization.Init()){
    Serial.println("location init failed!");
  }
  
  if(Result_state::State_Ok != perception.Init()){
    Serial.println("perception init failed!");
  }

  if(Result_state::State_Ok != control.Init()){
    Serial.println("control init failed!");
  }

  Robotic_sys::common::BuzzleInit();

  Robotic_sys::common::BuzzlePlayTone(300);
  
  Serial.println("init done!");

}

void loop() {
  Result_state state;
  state = perception.Start();
}
