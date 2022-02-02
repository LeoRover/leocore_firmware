#include <string>

#include "firmware/parameters.hpp"

static constexpr int TIMEOUT = 1000;

void Parameters::load(ros::NodeHandle &nh) {
  nh.getParam("firmware/wheels/encoder_resolution", &wheel_encoder_resolution, 1,
              TIMEOUT);
  nh.getParam("firmware/wheels/torque_constant", &wheel_torque_constant, 1,
              TIMEOUT);
  nh.getParam("firmware/wheels/pid/p", &wheel_pid_p, 1, TIMEOUT);
  nh.getParam("firmware/wheels/pid/i", &wheel_pid_i, 1, TIMEOUT);
  nh.getParam("firmware/wheels/pid/d", &wheel_pid_d, 1, TIMEOUT);
  nh.getParam("firmware/wheels/pwm_duty_limit", &wheel_pwm_duty_limit, 1, TIMEOUT);

  nh.getParam("firmware/diff_drive/wheel_radius", &dd_wheel_radius, 1, TIMEOUT);
  nh.getParam("firmware/diff_drive/wheel_separation", &dd_wheel_separation, 1,
              TIMEOUT);
  nh.getParam("firmware/diff_drive/angular_velocity_multiplier",
              &dd_angular_velocity_multiplier, 1, TIMEOUT);
  nh.getParam("firmware/diff_drive/input_timeout", &dd_input_timeout, 1,
              TIMEOUT);
  nh.getParam("firmware/battery_min_voltage", &battery_min_voltage, 1, TIMEOUT);
}
