#include "firmware/pid_regulator.hpp"
#include "firmware/utils.hpp"

PIDRegulator::PIDRegulator(float Kp, float Ki, float Kd, float range)
    : Kp_(Kp),
      Ki_(Ki),
      Kd_(Kd),
      range_(range),
      isum_(0),
      last_error_(0),
      has_last_error_(false) {}

void PIDRegulator::reset() {
  isum_ = 0;
  last_error_ = 0;
  has_last_error_ = false;
}

float PIDRegulator::update(float error, uint16_t dt_ms) {
  float cur_err;
  if (has_last_error_) {
    cur_err = error - last_error_;
  } else {
    cur_err = 0;
    has_last_error_ = true;
  }
  last_error_ = error;

  isum_ += Ki_ * error * static_cast<float>(dt_ms);

  // Anti-windup
  isum_ = clamp(isum_, range_);

  float val = Kp_ * error + isum_ + Kd_ * cur_err / static_cast<float>(dt_ms);
  val = clamp(val, range_);

  return val;
}
