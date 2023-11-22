// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

//#include "encoders.h"
#include "common.h"

// Class to track robot position.
class Kinematics_c {
  public:
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double speed_ = 0.0;
    double yaw = 0.0;
    double yaw_rate = 0.0;
    
    // Constructor, must exist.
    Kinematics_c() {

    }

    // Use this function to update
    // your kinematics
    void update(double left_distance, double right_distance, double left_speed, double right_speed, unsigned long duration) {
      double dt = duration / 1000.0;
      double linear_distance = (left_distance + right_distance) / 2;

      speed_ = (left_speed + right_speed) / 2;

      // 更新速度和角速度

      yaw_rate = (right_speed - left_speed) / (2 * AXIS_LENGTH);

      // 更新位置和方向
      position_x_ += linear_distance * cos(yaw + yaw_rate * dt / 2);
      position_y_ += linear_distance * sin(yaw + yaw_rate * dt / 2);

      yaw += yaw_rate * dt;

//      Serial.print(position_x_);
//
//      Serial.print("   ");
//
//      Serial.println(position_y_);
//
//      Serial.print("   ");
//
//      Serial.println(yaw);
    }

};

#endif
