#include "firmware/motor_controller.hpp"
#include "firmware/configuration.hpp"

void MotorController::init() {
  gpio_set(config_.nsleep);  // Wake up the driver
  gpio_set(config_.mode);    // Turn on Slow-decay mode
}

void MotorController::setPWMDutyCycle(float pwm_duty) {
  pwm_duty_ = clamp(pwm_duty, 100.0F);

  int16_t power = static_cast<int16_t>((pwm_duty_ / 100.0F) *
                                       static_cast<float>(PWM_RANGE));

  if (motor_polarity_ == Polarity::Reversed) power *= -1;

  if (power >= 0) {
    gpio_set(config_.phase);
    *config_.pwm_ccr = static_cast<uint32_t>(power);
  } else {
    gpio_reset(config_.phase);
    *config_.pwm_ccr = static_cast<uint32_t>(-power);
  }
}

float MotorController::getPWMDutyCycle() { return pwm_duty_; }

int32_t MotorController::getEncoderCnt() {
  uint16_t ticks_timer = *config_.enc_cnt;
  uint8_t ticks_quarter = ticks_timer >> 14;

  if (ticks_prev_quarter_ == 3 && ticks_quarter == 0)
    ticks_offset_ += (1 << 16);
  if (ticks_prev_quarter_ == 0 && ticks_quarter == 3)
    ticks_offset_ -= (1 << 16);

  ticks_prev_quarter_ = ticks_quarter;

  int32_t ticks = ticks_offset_ + ticks_timer;
  if (encoder_polarity_ == Polarity::Reversed) ticks *= -1;
  return ticks;
}

void MotorController::resetEncoderCnt() {
  ticks_prev_quarter_ = 0;
  ticks_offset_ = 0;
  *config_.enc_cnt = 0;
}

float MotorController::getWindingCurrent() {
  return static_cast<float>(*config_.vpropi_adc) * VPROPI_ADC_TO_CURRENT;
}

void MotorController::setMotorPolarity(Polarity polarity) {
  motor_polarity_ = polarity;
}

void MotorController::setEncoderPolarity(Polarity polarity) {
  encoder_polarity_ = polarity;
}