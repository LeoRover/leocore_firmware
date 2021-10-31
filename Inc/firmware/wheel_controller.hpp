#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "firmware/motor_controller.hpp"
#include "firmware/pid_regulator.hpp"
#include "firmware/utils.hpp"

struct WheelConfiguration {
  MotorConfiguration motor_conf;
  bool reverse_polarity;
};

class WheelController {
 public:
  WheelController(const WheelConfiguration& wheel_conf);

  /**
   * @brief Initialize and enable the wheel controller
   */
  void init();

  /**
   * @brief Perform an update routine
   * @param dt_ms Time elapsed since the last call to update function
   */
  void update(uint32_t dt_ms);

  /**
   * @brief Set the target velocity of the wheel in rad/s
   */
  void setTargetVelocity(float speed);

  /**
   * @brief Get the current velocity of the motor in rad/s
   */
  float getVelocity();

  /**
   * @brief Get the current power (PWM Duty) applied to the motor
   */
  int16_t getPower();

  /**
   * @brief Get the current distance traversed by the wheel in radians
   */
  int32_t getDistance();

  /**
   * @brief Reset the distance traversed by the wheel
   */
  void resetDistance();

  void enable();
  void disable();

 private:
  MotorController motor_;
  PIDRegulator v_reg_;
  CircularBuffer<std::pair<int32_t, uint32_t>> encoder_buffer_;

  int16_t power_ = 0;

  bool enabled_ = false;

  int32_t ticks_now_ = 0;
  int32_t ticks_sum_ = 0;
  uint32_t dt_sum_ = 0;

  float v_target_ = 0.0F;
  float v_now_ = 0.0F;
};
