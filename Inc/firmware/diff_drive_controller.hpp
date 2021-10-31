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

  /**
   * Initialize the Diff Drive Controller.
   * Should be called after all ROS parameters are loaded.
   * Initializes all Wheel Controllers.
   */
  void init();

  /**
   * Set the target speed of the robot.
   * @param linear The linear speed of the robot in m/s
   * @param angular The angular speed of the robot in rad/s
   */
  void setSpeed(float linear, float angular);

  /**
   * Get the current odometry.
   */
  Odom getOdom();

  /**
   * Reset the odometry position.
   */
  void resetOdom();

  /**
   * Retrieve the wheel states and populate the positions, velocities and
   * efforts fields with the new values.
   */
  void updateWheelStates();

  /**
   * Perform an update routine.
   */
  void update(uint32_t dt_ms);

  double positions[4];
  double velocities[4];
  double efforts[4];

 private:
  WheelController wheel_FL_;
  WheelController wheel_RL_;
  WheelController wheel_FR_;
  WheelController wheel_RR_;

  Odom odom_;
};
