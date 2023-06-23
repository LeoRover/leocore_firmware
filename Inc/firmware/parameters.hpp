#pragma once

#include <ros.h>

#include <diff_drive_lib/diff_drive_controller.hpp>

struct Parameters : diff_drive_lib::RobotParams {
  // Override inherited parameters
  Parameters() {
    // Motor
    wheel_encoder_resolution = 878.4F;
    wheel_torque_constant = 1.17647F;
    wheel_pid_p = 0.0F;
    wheel_pid_i = 0.005F;
    wheel_pid_d = 0.0F;
    wheel_pwm_duty_limit = 100.0F;

    robot_wheel_radius = 0.0625F;
    robot_wheel_separation = 0.33F;
    robot_wheel_base = 0.3052F;
    robot_angular_velocity_multiplier = 1.91F;
    robot_input_timeout = 500;
  }

  float battery_min_voltage = 10.0;

  bool mecanum_wheels = false;

  void load(ros::NodeHandle &nh);
};
