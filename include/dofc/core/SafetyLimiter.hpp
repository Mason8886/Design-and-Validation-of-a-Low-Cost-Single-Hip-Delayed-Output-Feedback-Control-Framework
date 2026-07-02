#pragma once

#include <optional>

namespace dofc {

struct TorqueLimitConfig {
  double max_abs_torque_nm{6.0};
  double max_abs_torque_rate_nm_per_s{20.0};
};

struct TorqueLimitResult {
  double requested_nm{0.0};
  double after_magnitude_limit_nm{0.0};
  double output_nm{0.0};
  bool saturated{false};
  bool rate_limited{false};
};

class TorqueLimiter {
public:
  explicit TorqueLimiter(TorqueLimitConfig config = {});

  void reset(double output_nm = 0.0);
  TorqueLimitResult apply(double requested_nm, double time_s);
  TorqueLimitConfig config() const noexcept;

private:
  TorqueLimitConfig config_;
  std::optional<double> last_time_s_;
  double last_output_nm_{0.0};
};

}  // namespace dofc
