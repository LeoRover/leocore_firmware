#include <cmath>
#include <cstdio>

#include "firmware/diff_drive_controller.hpp"
#include "firmware/parameters.hpp"
#include "firmware/utils.hpp"

static constexpr float PI = 3.141592653F;

DiffDriveController::DiffDriveController(const DiffDriveConfiguration& dd_conf)
    : wheel_FL(dd_conf.wheel_FL_conf),
      wheel_RL(dd_conf.wheel_RL_conf),
      wheel_FR(dd_conf.wheel_FR_conf),
      wheel_RR(dd_conf.wheel_RR_conf) {}

void DiffDriveController::init() {
  wheel_FL.init();
  wheel_RL.init();
  wheel_FR.init();
  wheel_RR.init();
}

void DiffDriveController::enable() {
  wheel_FL.enable();
  wheel_RL.enable();
  wheel_FR.enable();
  wheel_RR.enable();
  enabled_ = true;
}

void DiffDriveController::disable() {
  wheel_FL.disable();
  wheel_RL.disable();
  wheel_FR.disable();
  wheel_RR.disable();
  enabled_ = false;
}

void DiffDriveController::setSpeed(const float linear, const float angular) {
  if (!enabled_) enable();
  last_command_time_ = time();

  const float angular_multiplied =
      angular * params.dd_angular_velocity_multiplier;
  const float wheel_L_lin_vel =
      linear - (angular_multiplied * params.dd_wheel_separation / 2.0F);
  const float wheel_R_lin_vel =
      linear + (angular_multiplied * params.dd_wheel_separation / 2.0F);
  const float wheel_L_ang_vel = wheel_L_lin_vel / params.dd_wheel_radius;
  const float wheel_R_ang_vel = wheel_R_lin_vel / params.dd_wheel_radius;

  wheel_FL.setTargetVelocity(wheel_L_ang_vel);
  wheel_RL.setTargetVelocity(wheel_L_ang_vel);
  wheel_FR.setTargetVelocity(wheel_R_ang_vel);
  wheel_RR.setTargetVelocity(wheel_R_ang_vel);
}

Odom DiffDriveController::getOdom() { return odom_; }

void DiffDriveController::resetOdom() {
  odom_.pose_x = 0.0F;
  odom_.pose_y = 0.0F;
  odom_.pose_yaw = 0.0F;
}

void DiffDriveController::updateWheelStates() {
  positions[0] = static_cast<double>(wheel_FL.getDistance());
  positions[1] = static_cast<double>(wheel_RL.getDistance());
  positions[2] = static_cast<double>(wheel_FR.getDistance());
  positions[3] = static_cast<double>(wheel_RR.getDistance());

  velocities[0] = static_cast<double>(wheel_FL.getVelocity());
  velocities[1] = static_cast<double>(wheel_RL.getVelocity());
  velocities[2] = static_cast<double>(wheel_FR.getVelocity());
  velocities[3] = static_cast<double>(wheel_RR.getVelocity());

  efforts[0] = static_cast<double>(wheel_FL.getTorque());
  efforts[1] = static_cast<double>(wheel_RL.getTorque());
  efforts[2] = static_cast<double>(wheel_FR.getTorque());
  efforts[3] = static_cast<double>(wheel_RR.getTorque());
}

void DiffDriveController::update(uint32_t dt_ms) {
  if (enabled_ && time() - last_command_time_ >
                      static_cast<uint32_t>(params.dd_input_timeout))
    disable();

  wheel_FL.update(dt_ms);
  wheel_RL.update(dt_ms);
  wheel_FR.update(dt_ms);
  wheel_RR.update(dt_ms);

  // velocity in radians per second
  const float FL_ang_vel = wheel_FL.getVelocity();
  const float RL_ang_vel = wheel_RL.getVelocity();
  const float FR_ang_vel = wheel_FR.getVelocity();
  const float RR_ang_vel = wheel_RR.getVelocity();

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
