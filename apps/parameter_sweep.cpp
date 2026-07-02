#include "dofc/control/DofcController.hpp"
#include "dofc/core/MotionStateDetector.hpp"
#include "dofc/core/SafetyLimiter.hpp"
#include "dofc/io/CsvLogger.hpp"
#include "dofc/model/SingleHipModel.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDurationS = 16.0;
constexpr double kDtS = 0.002;
constexpr double kCadenceHz = 1.0;
constexpr double kBiologicalHipTorqueAmplitudeNm = 60.0;
constexpr double kPartialAssistTorqueLimitNm = 6.0;
constexpr double kTorqueRateLimitNmPerS = 20.0;
constexpr double kMaxStableHipAngleRad = 1.0;
constexpr double kMaxStableRmsTorqueRateNmPerS = kTorqueRateLimitNmPerS;
constexpr double kMaxStableSaturationFraction = 0.20;
constexpr double kMaxPeakAssistToHumanRatio = 0.12;

struct Metrics {
  double max_abs_angle_rad{0.0};
  double peak_abs_torque_nm{0.0};
  double peak_assist_to_human_ratio{0.0};
  double rms_torque_rate_nm_s{0.0};
  double saturation_fraction{0.0};
  bool stable{false};
};

double humanTorque(double time_s) {
  const bool walking = time_s < 8.0 || time_s >= 12.0;
  if (!walking) {
    return 0.0;
  }
  // 60 Nm represents the biological hip torque amplitude in the full-scale simulation.
  return kBiologicalHipTorqueAmplitudeNm * std::sin(2.0 * kPi * kCadenceHz * time_s);
}

Metrics runScenario(double gain, double delay_s) {
  dofc::SingleHipModel model;
  dofc::DofcController controller({
      gain,
      delay_s,
      1.5,
      dofc::FeedbackSignal::Angle,
      dofc::FeedbackLaw::PureDelay,
      false,
  });
  dofc::MotionStateDetector motion_detector;
  // 6 Nm is a conservative partial-assistance actuator limit for a low-cost prototype.
  dofc::TorqueLimiter limiter({kPartialAssistTorqueLimitNm, kTorqueRateLimitNmPerS});

  Metrics metrics;
  int samples = 0;
  int saturated_samples = 0;
  int torque_rate_samples = 0;
  double sum_torque_rate_sq = 0.0;
  std::optional<double> previous_torque;

  for (double time_s = 0.0; time_s <= kDurationS; time_s += kDtS) {
    const auto state = model.state();
    const auto controller_output =
        controller.update(time_s, state.angle_rad, state.velocity_rad_s);
    const auto motion_state = motion_detector.update(time_s, state.velocity_rad_s);
    const auto limited_torque =
        limiter.apply(controller_output.raw_torque_nm * motion_state.assistance_scale,
                      time_s);

    metrics.max_abs_angle_rad =
        std::max(metrics.max_abs_angle_rad, std::abs(state.angle_rad));
    metrics.peak_abs_torque_nm =
        std::max(metrics.peak_abs_torque_nm, std::abs(limited_torque.output_nm));

    if (limited_torque.saturated) {
      ++saturated_samples;
    }
    if (previous_torque.has_value()) {
      const double torque_rate = (limited_torque.output_nm - *previous_torque) / kDtS;
      sum_torque_rate_sq += torque_rate * torque_rate;
      ++torque_rate_samples;
    }
    previous_torque = limited_torque.output_nm;

    model.step(kDtS, humanTorque(time_s), limited_torque.output_nm);
    ++samples;
  }

  metrics.rms_torque_rate_nm_s =
      torque_rate_samples > 0 ? std::sqrt(sum_torque_rate_sq / torque_rate_samples) : 0.0;
  metrics.saturation_fraction =
      samples > 0 ? static_cast<double>(saturated_samples) / static_cast<double>(samples) : 0.0;
  metrics.peak_assist_to_human_ratio =
      metrics.peak_abs_torque_nm / kBiologicalHipTorqueAmplitudeNm;

  metrics.stable =
      metrics.max_abs_angle_rad <= kMaxStableHipAngleRad &&
      metrics.rms_torque_rate_nm_s <= kMaxStableRmsTorqueRateNmPerS &&
      metrics.saturation_fraction <= kMaxStableSaturationFraction &&
      metrics.peak_assist_to_human_ratio <= kMaxPeakAssistToHumanRatio;

  return metrics;
}

}  // namespace

int main() {
  dofc::CsvLogger logger("data/parameter_sweep.csv", {
      "gain",
      "delay_s",
      "stable",
      "max_abs_angle_rad",
      "peak_abs_torque_nm",
      "peak_assist_to_human_ratio",
      "rms_torque_rate_nm_s",
      "saturation_fraction",
  });

  for (double delay_s = 0.05; delay_s <= 0.60; delay_s += 0.05) {
    for (double gain = -4.0; gain <= 14.0; gain += 0.5) {
      const auto metrics = runScenario(gain, delay_s);
      logger.writeRow({
          gain,
          delay_s,
          metrics.stable ? 1.0 : 0.0,
          metrics.max_abs_angle_rad,
          metrics.peak_abs_torque_nm,
          metrics.peak_assist_to_human_ratio,
          metrics.rms_torque_rate_nm_s,
          metrics.saturation_fraction,
      });
    }
  }

  std::cout << "Parameter sweep complete\n";
  std::cout << "Output CSV: data/parameter_sweep.csv\n";
  return 0;
}
