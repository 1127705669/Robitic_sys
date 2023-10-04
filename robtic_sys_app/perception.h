/******************************************************************************
 * Copyright 2023 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common.h"

namespace Robotic_sys {
namespace perception {

class Perception : public ComponentBase{
 public:

  String Name() const override;

  Result_state Init() override;

  Result_state Start() override;

  void Stop() override;

  virtual ~Control() = default;

 private:

 // Watch dog timer
  void OnTimer(const ros::TimerEvent &);

  EDrive::Result_state CheckInput();

  void SendCmd(::control::CarlaEgoVehicleControl *control_command);

  Result_state ProduceControlCommand(::control::CarlaEgoVehicleControl *control_command);
  
  ros::Timer timer_;
  ros::Time init_time_;
  ControllerAgent controller_agent_;
  ControlConf control_conf_;
  planning::ADCTrajectory trajectory_;

  std::string root_path;
  std::string adapter_conf_file = "/src/control/conf/adapter.conf";
  std::string control_conf_file = "/src/control/conf/control.conf";
  
};

} // namespace perception
} // namespace Robotic_sys
