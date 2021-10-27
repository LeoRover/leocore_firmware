#pragma once

#include <ros.h>

struct Parameters {
  // TF frames
  char robot_frame_id[50] = "base_link";
  char odom_frame_id[50] = "odom";
  char imu_frame_id[50] = "imu";
  char gps_frame_id[50] = "gpu";

  // Motor
  float motor_encoder_resolution = (878.4F);
  float motor_max_speed = 800.0F;
  float motor_pid_p = 0.0F;
  float motor_pid_i = 0.005F;
  float motor_pid_d = 0.0F;
  float motor_power_limit = 1000.0F;
  float motor_torque_limit = 1000.0F;

  // Differential drive
  float dd_wheel_radius = 0.0625F;
  float dd_wheel_separation = 0.33F;
  float dd_angular_velocity_multiplier = 1.91F;
  float dd_input_timeout = 500.0F;

  void load(ros::NodeHandle &nh);
};

extern Parameters params;