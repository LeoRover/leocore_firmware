#include <algorithm>

#include "firmware/configuration.hpp"
#include "firmware/motor_controller.hpp"
#include "firmware/parameters.hpp"
#include "firmware/utils.hpp"
#include "firmware/wheel_controller.hpp"

static constexpr float PI = 3.141592653F;

WheelController::WheelController(const WheelConfiguration& wheel_conf)
    : motor_(wheel_conf.motor_conf), encoder_buffer_(ENCODER_BUFFER_SIZE) {
  if (wheel_conf.reverse_polarity) {
    motor_.setMotorPolarity(Polarity::Reversed);
    motor_.setEncoderPolarity(Polarity::Reversed);
  }
}

void WheelController::init() {
  v_reg_.setCoeffs(params.motor_pid_p, params.motor_pid_i, params.motor_pid_d);
  v_reg_.setRange(
      std::min(static_cast<float>(PWM_RANGE), params.motor_power_limit));
  motor_.init();
  enable();
}

void WheelController::update(const uint32_t dt_ms) {
  int32_t ticks_prev = ticks_now_;
  ticks_now_ = motor_.getEncoderCnt();

  int32_t new_ticks = ticks_now_ - ticks_prev;

  std::pair<int32_t, uint32_t> encoder_old =
      encoder_buffer_.push_back(std::pair<int32_t, uint32_t>(new_ticks, dt_ms));

  ticks_sum_ += new_ticks;
  dt_sum_ += dt_ms;

  ticks_sum_ -= encoder_old.first;
  dt_sum_ -= encoder_old.second;

  v_now_ = static_cast<float>(ticks_sum_) / (dt_sum_ * 0.001F);

  if (enabled_) {
    float v_err = v_now_ - v_target_;
    power_ = v_reg_.update(v_err, dt_ms);
    motor_.setPower(power_);
  }
}

void WheelController::setTargetVelocity(const float speed) {
  v_target_ = (speed / (2.0F * PI)) * params.motor_encoder_resolution;
}

float WheelController::getVelocity() {
  return (v_now_ / params.motor_encoder_resolution) * (2.0F * PI);
}

int16_t WheelController::getPower() { return power_; }

int32_t WheelController::getDistance() {
  return (ticks_now_ / params.motor_encoder_resolution) * (2.0F * PI);
}

void WheelController::resetDistance() {
  motor_.resetEncoderCnt();
  ticks_now_ = 0;
}

void WheelController::enable() {
  if (!enabled_) {
    v_reg_.reset();
    enabled_ = true;
  }
}

void WheelController::disable() {
  enabled_ = false;
  power_ = 0;
  motor_.setPower(0);
}
