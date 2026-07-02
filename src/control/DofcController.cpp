#include "dofc/control/DofcController.hpp"

#include <cmath>
#include <stdexcept>

namespace dofc {

DofcController::DofcController(DofcConfig config)
    : config_(config), delay_buffer_(config.max_supported_delay_s) {
  if (!std::isfinite(config_.feedback_gain) ||
      !std::isfinite(config_.delay_s) ||
      config_.delay_s < 0.0 ||
      config_.delay_s > config_.max_supported_delay_s) {
    throw std::invalid_argument("invalid DOFC gain or delay");
  }
}

void DofcController::reset() {
  delay_buffer_.clear();
}

DofcOutput DofcController::update(double time_s,
                                  double hip_angle_rad,
                                  double hip_velocity_rad_s) {
  if (!std::isfinite(time_s) ||
      !std::isfinite(hip_angle_rad) ||
      !std::isfinite(hip_velocity_rad_s)) {
    throw std::invalid_argument("DOFC inputs must be finite");
  }

  const double current_signal = selectSignal(hip_angle_rad, hip_velocity_rad_s);
  delay_buffer_.push(time_s, current_signal);

  DofcOutput output;
  output.current_signal = current_signal;

  const auto delayed = delay_buffer_.delayedValue(time_s, config_.delay_s);
  if (!delayed.has_value()) {
    return output;
  }

  output.ready = true;
  output.delayed_signal = *delayed;

  switch (config_.law) {
    case FeedbackLaw::PureDelay:
      output.raw_torque_nm = config_.feedback_gain * output.delayed_signal;
      break;
    case FeedbackLaw::PyragasDifference:
      output.raw_torque_nm =
          config_.feedback_gain * (output.current_signal - output.delayed_signal);
      break;
  }

  if (config_.invert_output) {
    output.raw_torque_nm = -output.raw_torque_nm;
  }

  return output;
}

DofcConfig DofcController::config() const noexcept {
  return config_;
}

double DofcController::selectSignal(double hip_angle_rad, double hip_velocity_rad_s) const {
  switch (config_.signal) {
    case FeedbackSignal::Angle:
      return hip_angle_rad;
    case FeedbackSignal::Velocity:
      return hip_velocity_rad_s;
  }
  return hip_angle_rad;
}

}  // namespace dofc
