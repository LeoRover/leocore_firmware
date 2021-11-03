#include <cmath>
#include <cstdio>

#include "firmware/diff_drive_controller.hpp"
#include "firmware/parameters.hpp"
#include "firmware/utils.hpp"

static constexpr float PI = 3.141592653F;

DiffDriveController::DiffDriveController(const DiffDriveConfiguration& dd_conf)
    : wheel_FL_(dd_conf.wheel_FL_conf),
      wheel_RL_(dd_conf.wheel_RL_conf),
      wheel_FR_(dd_conf.wheel_FR_conf),
      wheel_RR_(dd_conf.wheel_RR_conf) {}

void DiffDriveController::init() {
  wheel_FL_.init();
  wheel_RL_.init();
  wheel_FR_.init();
  wheel_RR_.init();
}

void DiffDriveController::setSpeed(const float linear, const float angular) {
  const float angular_multiplied =
      angular * params.dd_angular_velocity_multiplier;
  const float wheel_L_lin_vel =
      linear - (angular_multiplied * params.dd_wheel_separation / 2.0F);
  const float wheel_R_lin_vel =
      linear + (angular_multiplied * params.dd_wheel_separation / 2.0F);
  const float wheel_L_ang_vel = wheel_L_lin_vel / params.dd_wheel_radius;
  const float wheel_R_ang_vel = wheel_R_lin_vel / params.dd_wheel_radius;

  wheel_FL_.setTargetVelocity(wheel_L_ang_vel);
  wheel_RL_.setTargetVelocity(wheel_L_ang_vel);
  wheel_FR_.setTargetVelocity(wheel_R_ang_vel);
  wheel_RR_.setTargetVelocity(wheel_R_ang_vel);
}

Odom DiffDriveController::getOdom() { return odom_; }

void DiffDriveController::resetOdom() {
  odom_.pose_x = 0.0F;
  odom_.pose_y = 0.0F;
  odom_.pose_yaw = 0.0F;
}

void DiffDriveController::updateWheelStates() {
  positions[0] = static_cast<double>(wheel_FL_.getDistance());
  positions[1] = static_cast<double>(wheel_RL_.getDistance());
  positions[2] = static_cast<double>(wheel_FR_.getDistance());
  positions[3] = static_cast<double>(wheel_RR_.getDistance());

  velocities[0] = static_cast<double>(wheel_FL_.getVelocity());
  velocities[1] = static_cast<double>(wheel_RL_.getVelocity());
  velocities[2] = static_cast<double>(wheel_FR_.getVelocity());
  velocities[3] = static_cast<double>(wheel_RR_.getVelocity());

  efforts[0] = static_cast<double>(wheel_FL_.getTorque());
  efforts[1] = static_cast<double>(wheel_RL_.getTorque());
  efforts[2] = static_cast<double>(wheel_FR_.getTorque());
  efforts[3] = static_cast<double>(wheel_RR_.getTorque());
}

void DiffDriveController::update(uint32_t dt_ms) {
  wheel_FL_.update(dt_ms);
  wheel_RL_.update(dt_ms);
  wheel_FR_.update(dt_ms);
  wheel_RR_.update(dt_ms);

  // velocity in radians per second
  const float FL_ang_vel = wheel_FL_.getVelocity();
  const float RL_ang_vel = wheel_RL_.getVelocity();
  const float FR_ang_vel = wheel_FR_.getVelocity();
  const float RR_ang_vel = wheel_RR_.getVelocity();

  const float L_ang_vel = (FL_ang_vel + RL_ang_vel) / 2.0F;
  const float R_ang_vel = (FR_ang_vel + RR_ang_vel) / 2.0F;

  // velocity in meters per second
  const float L_lin_vel = L_ang_vel * params.dd_wheel_radius;
  const float R_lin_vel = R_ang_vel * params.dd_wheel_radius;

  const float dt_s = static_cast<float>(dt_ms) * 0.001F;

  // linear (m/s) and angular (r/s) velocities of the robot
  odom_.vel_lin = (L_lin_vel + R_lin_vel) / 2.0F;
  odom_.vel_ang = (R_lin_vel - L_lin_vel) / params.dd_wheel_separation;

  odom_.vel_ang /= params.dd_angular_velocity_multiplier;

  // Integrate the velocity using the rectangle rule
  odom_.pose_yaw += odom_.vel_ang * dt_s;
  if (odom_.pose_yaw > 2.0F * PI)
    odom_.pose_yaw -= 2.0F * PI;
  else if (odom_.pose_yaw < 0.0F)
    odom_.pose_yaw += 2.0F * PI;

  odom_.pose_x += odom_.vel_lin * std::cos(odom_.pose_yaw) * dt_s;
  odom_.pose_y += odom_.vel_lin * std::sin(odom_.pose_yaw) * dt_s;
}
