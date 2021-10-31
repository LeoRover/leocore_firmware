#pragma once

#include <vector>

#include <ros.h>

#include "firmware/diff_drive_controller.hpp"
#include "firmware/motor_controller.hpp"
#include "firmware/wheel_controller.hpp"

struct Odom {
  float vel_lin;
  float vel_ang;
  float pose_x;
  float pose_y;
  float pose_yaw;
};

struct DiffDriveConfiguration {
  WheelConfiguration wheel_FL_conf;
  WheelConfiguration wheel_RL_conf;
  WheelConfiguration wheel_FR_conf;
  WheelConfiguration wheel_RR_conf;
};

class DiffDriveController {
 public:
  DiffDriveController(const DiffDriveConfiguration& dd_conf);

  void init();
  void setSpeed(float linear, float angular);
  Odom getOdom();
  void resetOdom();
  void updateWheelStates();
  void update(uint32_t dt_ms);

  double positions[4];
  double velocities[4];
  double efforts[4];

 private:
  // void inputWatchdog();

  WheelController wheel_FL_;
  WheelController wheel_RL_;
  WheelController wheel_FR_;
  WheelController wheel_RR_;

  Odom odom_;

  uint64_t last_update_;
};
