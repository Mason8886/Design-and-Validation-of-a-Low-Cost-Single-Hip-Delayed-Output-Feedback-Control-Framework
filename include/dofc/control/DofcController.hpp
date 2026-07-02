#pragma once

#include "dofc/core/DelayBuffer.hpp"

namespace dofc {

enum class FeedbackSignal {
  Angle,
  Velocity
};

enum class FeedbackLaw {
  PureDelay,
  PyragasDifference
};

struct DofcConfig {
  double feedback_gain{10.0};
  double delay_s{0.25};
  double max_supported_delay_s{1.5};
  FeedbackSignal signal{FeedbackSignal::Angle};
  FeedbackLaw law{FeedbackLaw::PureDelay};
  bool invert_output{false};
};

struct DofcOutput {
  double raw_torque_nm{0.0};
  bool ready{false};
  double current_signal{0.0};
  double delayed_signal{0.0};
};

class DofcController {
public:
  explicit DofcController(DofcConfig config = {});

  void reset();
  DofcOutput update(double time_s, double hip_angle_rad, double hip_velocity_rad_s);
  DofcConfig config() const noexcept;

private:
  double selectSignal(double hip_angle_rad, double hip_velocity_rad_s) const;

  DofcConfig config_;
  DelayBuffer delay_buffer_;
};

}  // namespace dofc
