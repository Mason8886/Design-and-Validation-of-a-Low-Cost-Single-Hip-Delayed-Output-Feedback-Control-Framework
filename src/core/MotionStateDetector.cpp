#include "dofc/core/MotionStateDetector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dofc {

MotionStateDetector::MotionStateDetector(MotionStateConfig config) : config_(config) {
  if (config_.velocity_on_threshold_rad_s <= config_.velocity_off_threshold_rad_s) {
    throw std::invalid_argument("walking on threshold must be above off threshold");
  }
  if (config_.ema_time_constant_s <= 0.0 ||
      config_.ramp_up_rate_per_s <= 0.0 ||
      config_.ramp_down_rate_per_s <= 0.0) {
    throw std::invalid_argument("motion detector time constants and ramp rates must be positive");
  }
}

void MotionStateDetector::reset() {
  state_ = {};
  has_time_ = false;
  last_time_s_ = 0.0;
}

MotionState MotionStateDetector::update(double time_s, double hip_velocity_rad_s) {
  if (!std::isfinite(time_s) || !std::isfinite(hip_velocity_rad_s)) {
    throw std::invalid_argument("motion detector inputs must be finite");
  }

  const double abs_velocity = std::abs(hip_velocity_rad_s);
  if (!has_time_) {
    state_.filtered_abs_velocity_rad_s = abs_velocity;
    has_time_ = true;
    last_time_s_ = time_s;
    return state_;
  }

  const double dt = std::max(0.0, time_s - last_time_s_);
  last_time_s_ = time_s;

  const double alpha = 1.0 - std::exp(-dt / config_.ema_time_constant_s);
  state_.filtered_abs_velocity_rad_s +=
      alpha * (abs_velocity - state_.filtered_abs_velocity_rad_s);

  if (state_.filtered_abs_velocity_rad_s > config_.velocity_on_threshold_rad_s) {
    state_.walking = true;
  } else if (state_.filtered_abs_velocity_rad_s < config_.velocity_off_threshold_rad_s) {
    state_.walking = false;
  }

  const double target = state_.walking ? 1.0 : 0.0;
  const double rate = target > state_.assistance_scale
                          ? config_.ramp_up_rate_per_s
                          : config_.ramp_down_rate_per_s;
  const double max_step = rate * dt;

  if (target > state_.assistance_scale) {
    state_.assistance_scale = std::min(target, state_.assistance_scale + max_step);
  } else {
    state_.assistance_scale = std::max(target, state_.assistance_scale - max_step);
  }

  return state_;
}

MotionState MotionStateDetector::state() const noexcept {
  return state_;
}

}  // namespace dofc
