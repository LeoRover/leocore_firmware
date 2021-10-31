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
   * Initialize the Wheel Controller.
   * Should be called after all ROS parameters are loaded.
   * Sets the PID regulator parameters and initializes the Motor Controller.
   */
  void init();

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function
   */
  void update(uint32_t dt_ms);

  /**
   * Set the target velocity of the wheel in rad/s.
   */
  void setTargetVelocity(float speed);

  /**
   * Get the current velocity of the motor in rad/s.
   */
  float getVelocity();

  /**
   * Get the current power (PWM Duty) applied to the motor.
   */
  int16_t getPower();

  /**
   * Get the current distance traversed by the wheel in radians.
   */
  int32_t getDistance();

  /**
   * Reset the distance traversed by the wheel.
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
