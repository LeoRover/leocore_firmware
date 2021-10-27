#pragma once

#include <cstdint>

class PIDRegulator {
 public:
  PIDRegulator() {}
  PIDRegulator(float Kp, float Ki, float Kd, float range);

  /**
   * @brief Set the PID coefficients
   */
  void setCoeffs(float Kp, float Ki, float Kd);

  /**
   * @brief Set the range of the regulator output
   */
  void setRange(float range);

  /**
   * @brief Reset the regulator to initial state
   */
  void reset();

  /**
   * @brief Perform an update routine of the regulator
   * @param error Current error
   * @param dt_ms Time elapsed since the last call to update function
   * @return Output of the regulator
   */ 
  float update(float error, uint16_t dt_ms);

 private:
  float Kp_ = 1.0F, Ki_ = 0.0F, Kd_ = 0.0F;
  float range_ = 1000.0;
  float isum_ = 0.0F;
  float last_error_ = 0.0F;
  bool has_last_error_ = false;
};
