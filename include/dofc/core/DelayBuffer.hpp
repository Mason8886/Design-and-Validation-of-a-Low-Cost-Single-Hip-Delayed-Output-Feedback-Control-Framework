#pragma once

#include <cstddef>
#include <deque>
#include <optional>

namespace dofc {

struct TimedSample {
  double time_s{};
  double value{};
};

class DelayBuffer {
public:
  explicit DelayBuffer(double max_delay_s);

  void clear();
  void push(double time_s, double value);

  std::optional<double> valueAt(double query_time_s) const;
  std::optional<double> delayedValue(double current_time_s, double delay_s) const;

  std::size_t size() const noexcept;
  double maxDelay() const noexcept;

private:
  double max_delay_s_;
  std::deque<TimedSample> samples_;
};

}  // namespace dofc
