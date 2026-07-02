#pragma once

namespace dofc {

struct MotionStateConfig {
  double velocity_on_threshold_rad_s{0.35};
  double velocity_off_threshold_rad_s{0.12};
  double ema_time_constant_s{0.25};
  double ramp_up_rate_per_s{0.6};
  double ramp_down_rate_per_s{1.8};
};

struct MotionState {
  bool walking{false};
  double filtered_abs_velocity_rad_s{0.0};
  double assistance_scale{0.0};
};

class MotionStateDetector {
public:
  explicit MotionStateDetector(MotionStateConfig config = {});

  void reset();
  MotionState update(double time_s, double hip_velocity_rad_s);
  MotionState state() const noexcept;

private:
  MotionStateConfig config_;
  MotionState state_;
  bool has_time_{false};
  double last_time_s_{0.0};
};

}  // namespace dofc
