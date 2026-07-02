#include "dofc/core/SafetyLimiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dofc {

namespace {

double clampAbs(double value, double max_abs) {
  return std::clamp(value, -max_abs, max_abs);
}

}  // namespace

TorqueLimiter::TorqueLimiter(TorqueLimitConfig config) : config_(config) {
  if (!std::isfinite(config_.max_abs_torque_nm) ||
      !std::isfinite(config_.max_abs_torque_rate_nm_per_s) ||
      config_.max_abs_torque_nm <= 0.0 ||
      config_.max_abs_torque_rate_nm_per_s <= 0.0) {
    throw std::invalid_argument("torque limits must be positive and finite");
  }
}

void TorqueLimiter::reset(double output_nm) {
  last_output_nm_ = clampAbs(output_nm, config_.max_abs_torque_nm);
  last_time_s_.reset();
}

TorqueLimitResult TorqueLimiter::apply(double requested_nm, double time_s) {
  if (!std::isfinite(requested_nm) || !std::isfinite(time_s)) {
    throw std::invalid_argument("torque limiter inputs must be finite");
  }

  TorqueLimitResult result;
  result.requested_nm = requested_nm;
  result.after_magnitude_limit_nm = clampAbs(requested_nm, config_.max_abs_torque_nm);
  result.saturated = result.after_magnitude_limit_nm != requested_nm;

  double output = result.after_magnitude_limit_nm;
  if (last_time_s_.has_value()) {
    const double dt = std::max(0.0, time_s - *last_time_s_);
    const double max_delta = config_.max_abs_torque_rate_nm_per_s * dt;
    const double delta = output - last_output_nm_;
    const double limited_delta = clampAbs(delta, max_delta);
    output = last_output_nm_ + limited_delta;
    result.rate_limited = limited_delta != delta;
  }

  last_output_nm_ = output;
  last_time_s_ = time_s;
  result.output_nm = output;
  return result;
}

TorqueLimitConfig TorqueLimiter::config() const noexcept {
  return config_;
}

}  // namespace dofc
