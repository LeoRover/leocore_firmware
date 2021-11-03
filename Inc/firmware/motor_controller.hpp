#pragma once

#include <stdint.h>

#include "firmware/gpio_compat.hpp"

enum class Polarity { Normal = 0, Reversed = 1 };

struct MotorConfiguration {
  GPIO nsleep, phase, mode, fault;
  volatile uint32_t *enc_cnt;
  volatile uint32_t *pwm_ccr;
  volatile uint16_t *vpropi_adc;
};

class MotorController {
 public:
  MotorController(const MotorConfiguration &config) : config_(config){};

  /**
   * Initialize the Motor Controller.
   * Configures the motor driver by setting gpio states.
   */
  void init();

  /**
   * Set the power to the motor.
   * @param power value between -PWM_RANGE and PWM_RANGE
   */
  void setPower(int16_t power);

  /**
   * Get the number of encoder ticks.
   * Must be called at least once per (2^14) encoder ticks
   * @return encoder ticks
   */
  int32_t getEncoderCnt();

  /**
   * Set the number of encoder ticks to 0.
   */
  void resetEncoderCnt();

  /**
   * Get motor winding current in Amperes
   */
  float getWindingCurrent();

  /**
   * Set motor polarity.
   * @param polarity motor polarity
   */
  void setMotorPolarity(Polarity polarity);

  /**
   * Set encoder polarity.
   * @param polarity encoder polarity
   */
  void setEncoderPolarity(Polarity polarity);

 private:
  MotorConfiguration config_;
  Polarity motor_polarity_ = Polarity::Normal;
  Polarity encoder_polarity_ = Polarity::Normal;

  uint8_t ticks_prev_quarter_ = 0;
  int32_t ticks_offset_ = 0;
};
