// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "encoders.h"

// Class to track robot position.
class Kinematics_c {
  public:
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double speed_ = 0.0;
    double yaw = 0.0;
    double yaw_rate = 0.0;
    int counter = 0;
    
    // Constructor, must exist.
    Kinematics_c() {

    }

    // Use this function to update
    // your kinematics
    void update(double left_speed, double right_speed, unsigned long duration) {
      speed_ = left_speed/2 + right_speed/2;
      yaw_rate = right_speed/(2*AXIS_LENGTH) - left_speed/(2*AXIS_LENGTH);
      yaw += yaw_rate*duration/1000000;
      position_x_ = position_x_ + (cos(yaw)*speed_)*duration/1000000;
      position_y_ += (sin(yaw)*speed_)*duration/1000000;

//      Serial.println(speed_);

      Serial.println(yaw);
      
//      Serial.println(position_x_);
      
//      Serial.println(position_y_);
    }

};

#endif
