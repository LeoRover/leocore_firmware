#pragma once

#include <ros.h>

#include <diff_drive_controller.hpp>

struct Parameters : DiffDriveParams {
  // Override inherited parameters
  Parameters() {
    // Motor
    wheel_encoder_resolution = 878.4F;
    wheel_torque_constant = 1.17647F;
    wheel_pid_p = 0.0F;
    wheel_pid_i = 0.005F;
    wheel_pid_d = 0.0F;
    wheel_pwm_duty_limit = 100.0F;

    // Differential drive
    dd_wheel_radius = 0.0625F;
    dd_wheel_separation = 0.33F;
    dd_angular_velocity_multiplier = 1.91F;
    dd_input_timeout = 500;
  }

  float battery_min_voltage = 10.0;

  void load(ros::NodeHandle &nh);
};
