#pragma once

#include <stdint.h>

#include "firmware/gpio_compat.h"

enum class Polarity { Normal = 0, Reversed = 1 };

struct MotorConfiguration {
  GPIO nsleep, phase, mode, fault;
  volatile uint32_t *enc_cnt;
  volatile uint32_t *pwm_ccr;
};

class MotorController {
 public:
  MotorController(const MotorConfiguration &config) : config_(config){};

  /**
   * @brief Initialize the motor
   */
  void init();

  /**
   * @brief Set the power to the motor
   * @param power value between -PWM_RANGE and PWM_RANGE
   */
  void setPower(int16_t power);

  /**
   * @brief Get the number of encoder ticks.
   * @return encoder ticks
   */
  int32_t getEncoderCnt();

  /**
   * @brief Set the number of encoder ticks to 0
   */
  void resetEncoderCnt();

  /**
   * @brief Set motor polarity
   * @param polarity motor polarity
   */
  void setMotorPolarity(Polarity polarity);

  /**
   * @brief Set encoder polarity
   * @param polarity encoder polarity
   */
  void setEncoderPolarity(Polarity polarity);

 private:
  MotorConfiguration config_;
  Polarity motor_polarity_ = Polarity::Normal;
  Polarity encoder_polarity_ = Polarity::Normal;
};
