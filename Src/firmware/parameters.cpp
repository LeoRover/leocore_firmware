#include <string>

#include "firmware/parameters.hpp"

static constexpr int TIMEOUT = 1000;

void Parameters::load(ros::NodeHandle &nh) {
  nh.getParam("firmware/mecanum_wheels", &mecanum_wheels, 1, TIMEOUT);
  if(mecanum_wheels) {
    robot_wheel_radius = 0.0635F;
    robot_wheel_separation = 0.405F;
    robot_wheel_base = 0.3052F;
    robot_angular_velocity_multiplier = 1.0F;
    robot_input_timeout = 500;
    nh.getParam("firmware/mecanum_drive/wheel_radius", &robot_wheel_radius, 1, TIMEOUT);
    nh.getParam("firmware/mecanum_drive/wheel_separation", &robot_wheel_separation, 1,
              TIMEOUT);
    nh.getParam("firmware/mecanum_drive/wheel_base", &robot_wheel_base, 1, TIMEOUT);
    nh.getParam("firmware/mecanum_drive/angular_velocity_multiplier",
              &robot_angular_velocity_multiplier, 1, TIMEOUT);
    nh.getParam("firmware/mecanum_drive/input_timeout", &robot_input_timeout, 1,
              TIMEOUT);
  } else {
    nh.getParam("firmware/diff_drive/wheel_radius", &robot_wheel_radius, 1, TIMEOUT);
    nh.getParam("firmware/diff_drive/wheel_separation", &robot_wheel_separation, 1,
              TIMEOUT);
    nh.getParam("firmware/diff_drive/angular_velocity_multiplier",
              &robot_angular_velocity_multiplier, 1, TIMEOUT);
    nh.getParam("firmware/diff_drive/input_timeout", &robot_input_timeout, 1,
              TIMEOUT);
  }
  
  nh.getParam("firmware/wheels/encoder_resolution", &wheel_encoder_resolution, 1,
              TIMEOUT);
  nh.getParam("firmware/wheels/torque_constant", &wheel_torque_constant, 1,
              TIMEOUT);
  nh.getParam("firmware/wheels/pid/p", &wheel_pid_p, 1, TIMEOUT);
  nh.getParam("firmware/wheels/pid/i", &wheel_pid_i, 1, TIMEOUT);
  nh.getParam("firmware/wheels/pid/d", &wheel_pid_d, 1, TIMEOUT);
  nh.getParam("firmware/wheels/pwm_duty_limit", &wheel_pwm_duty_limit, 1, TIMEOUT);
  nh.getParam("firmware/battery_min_voltage", &battery_min_voltage, 1, TIMEOUT);
}
