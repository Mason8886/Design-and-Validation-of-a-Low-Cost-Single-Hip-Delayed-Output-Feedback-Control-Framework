#include "dofc/core/DelayBuffer.hpp"

#include <cmath>
#include <stdexcept>

namespace dofc {

DelayBuffer::DelayBuffer(double max_delay_s) : max_delay_s_(max_delay_s) {
  if (!std::isfinite(max_delay_s_) || max_delay_s_ <= 0.0) {
    throw std::invalid_argument("max_delay_s must be positive and finite");
  }
}

void DelayBuffer::clear() {
  samples_.clear();
}

void DelayBuffer::push(double time_s, double value) {
  if (!std::isfinite(time_s) || !std::isfinite(value)) {
    throw std::invalid_argument("DelayBuffer samples must be finite");
  }

  if (!samples_.empty()) {
    const double last_time = samples_.back().time_s;
    if (time_s < last_time) {
      throw std::invalid_argument("DelayBuffer time must be nondecreasing");
    }
    if (time_s == last_time) {
      samples_.back().value = value;
      return;
    }
  }

  samples_.push_back({time_s, value});

  const double keep_after_s = time_s - max_delay_s_ - 1.0;
  while (samples_.size() > 2 && samples_[1].time_s < keep_after_s) {
    samples_.pop_front();
  }
}

std::optional<double> DelayBuffer::valueAt(double query_time_s) const {
  if (samples_.empty() || !std::isfinite(query_time_s)) {
    return std::nullopt;
  }

  constexpr double eps = 1e-12;
  if (query_time_s < samples_.front().time_s - eps ||
      query_time_s > samples_.back().time_s + eps) {
    return std::nullopt;
  }

  if (query_time_s <= samples_.front().time_s + eps) {
    return samples_.front().value;
  }
  if (query_time_s >= samples_.back().time_s - eps) {
    return samples_.back().value;
  }

  for (std::size_t i = 1; i < samples_.size(); ++i) {
    const auto& previous = samples_[i - 1];
    const auto& next = samples_[i];
    if (query_time_s <= next.time_s + eps) {
      const double span = next.time_s - previous.time_s;
      if (span <= eps) {
        return next.value;
      }
      const double alpha = (query_time_s - previous.time_s) / span;
      return previous.value + alpha * (next.value - previous.value);
    }
  }

  return std::nullopt;
}

std::optional<double> DelayBuffer::delayedValue(double current_time_s, double delay_s) const {
  if (!std::isfinite(delay_s) || delay_s < 0.0 || delay_s > max_delay_s_) {
    return std::nullopt;
  }
  return valueAt(current_time_s - delay_s);
}

std::size_t DelayBuffer::size() const noexcept {
  return samples_.size();
}

double DelayBuffer::maxDelay() const noexcept {
  return max_delay_s_;
}

}  // namespace dofc
