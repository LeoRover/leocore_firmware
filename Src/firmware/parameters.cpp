#include <string>

#include "firmware/parameters.hpp"

static constexpr int TIMEOUT = 1000;

Parameters params;

void Parameters::load(ros::NodeHandle &nh) {
  nh.getParam("leo_hat/motors/encoder_resolution", &motor_encoder_resolution, 1,
              TIMEOUT);
  nh.getParam("leo_hat/motors/torque_constant", &motor_torque_constant, 1,
              TIMEOUT);
  nh.getParam("leo_hat/motors/pid/p", &motor_pid_p, 1, TIMEOUT);
  nh.getParam("leo_hat/motors/pid/i", &motor_pid_i, 1, TIMEOUT);
  nh.getParam("leo_hat/motors/pid/d", &motor_pid_d, 1, TIMEOUT);
  nh.getParam("leo_hat/motors/power_limit", &motor_power_limit, 1, TIMEOUT);

  nh.getParam("leo_hat/diff_drive/wheel_radius", &dd_wheel_radius, 1, TIMEOUT);
  nh.getParam("leo_hat/diff_drive/wheel_separation", &dd_wheel_separation, 1,
              TIMEOUT);
  nh.getParam("leo_hat/diff_drive/angular_velocity_multiplier",
              &dd_angular_velocity_multiplier, 1, TIMEOUT);
  nh.getParam("leo_hat/diff_drive/input_timeout", &dd_input_timeout, 1,
              TIMEOUT);
}
