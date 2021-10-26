#include "firmware/motor_controller.hpp"

void MotorController::init() {
  gpio_set(config_.nsleep);  // Wake up the driver
  gpio_set(config_.mode);    // Turn on Slow-decay mode
}

void MotorController::setPower(int16_t power) {
  if (motor_polarity_ == Polarity::Reversed) power *= -1;

  if (power >= 0) {
    gpio_set(config_.phase);
    *config_.pwm_ccr = static_cast<uint32_t>(power);
  } else {
    gpio_reset(config_.phase);
    *config_.pwm_ccr = static_cast<uint32_t>(-power);
  }
}

int32_t MotorController::getEncoderCnt(void) {
  // TODO handle over and underflow of 16 bit timers
  int16_t ticks = *config_.enc_cnt;
  if (encoder_polarity_ == Polarity::Reversed) ticks *= -1;
  return ticks;
}

void MotorController::resetEncoderCnt(void) { *config_.enc_cnt = 0; }

void MotorController::setMotorPolarity(Polarity polarity) {
  motor_polarity_ = polarity;
}

void MotorController::setEncoderPolarity(Polarity polarity) {
  encoder_polarity_ = polarity;
}