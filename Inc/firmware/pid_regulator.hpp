#pragma once

#include <cstdint>

class PIDRegulator {
 public:
  PIDRegulator() = delete;
  PIDRegulator(float Kp, float Ki, float Kd, float range);

  void reset();

  float update(float error, uint16_t dt_ms);

 private:
  float Kp_, Ki_, Kd_, range_;
  float isum_;
  float last_error_;
  bool has_last_error_;
};
