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

double readDoubleArg(const std::vector<std::string>& args,
                     const std::string& name,
                     double default_value) {
  for (std::size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == name) {
      return std::stod(args[i + 1]);
    }
  }
  return default_value;
}

std::string readStringArg(const std::vector<std::string>& args,
                          const std::string& name,
                          const std::string& default_value) {
  for (std::size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == name) {
      return args[i + 1];
    }
  }
  return default_value;
}

bool isWalkingWindow(double time_s) {
  return time_s < 8.0 || time_s >= 12.0;
}

double humanWalkingTorque(double time_s, double amplitude_nm, double cadence_hz) {
  if (!isWalkingWindow(time_s)) {
    return 0.0;
  }
  return amplitude_nm * std::sin(2.0 * kPi * cadence_hz * time_s);
}

dofc::FeedbackLaw parseLaw(const std::string& value) {
  if (value == "pyragas" || value == "difference") {
    return dofc::FeedbackLaw::PyragasDifference;
  }
  return dofc::FeedbackLaw::PureDelay;
}

dofc::FeedbackSignal parseSignal(const std::string& value) {
  if (value == "velocity") {
    return dofc::FeedbackSignal::Velocity;
  }
  return dofc::FeedbackSignal::Angle;
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);

  const double duration_s = readDoubleArg(args, "--duration", 16.0);
  const double dt_s = readDoubleArg(args, "--dt", 0.002);
  const double gain = readDoubleArg(args, "--gain", 4.0);
  const double delay_s = readDoubleArg(args, "--delay", 0.25);
  const double max_torque_nm = readDoubleArg(args, "--max-torque", 6.0);
  const double max_rate_nm_s = readDoubleArg(args, "--max-rate", 20.0);
  const double human_torque_amp_nm = readDoubleArg(args, "--human-torque", 2.5);
  const double cadence_hz = readDoubleArg(args, "--cadence", 1.0);
  const std::string output_path =
      readStringArg(args, "--output", "data/simulation_output.csv");
  const std::string law = readStringArg(args, "--law", "pure-delay");
  const std::string signal = readStringArg(args, "--signal", "angle");

  dofc::SingleHipModel model;
  dofc::DofcController controller({
      gain,
      delay_s,
      1.5,
      parseSignal(signal),
      parseLaw(law),
      false,
  });
  dofc::MotionStateDetector motion_detector;
  dofc::TorqueLimiter limiter({max_torque_nm, max_rate_nm_s});

  dofc::CsvLogger logger(output_path, {
      "time_s",
      "hip_angle_rad",
      "hip_velocity_rad_s",
      "human_torque_nm",
      "dofc_raw_torque_nm",
      "assistance_scale",
      "exo_torque_nm",
      "delayed_signal",
      "controller_ready",
      "saturated",
      "rate_limited",
      "walking_window",
  });

  double peak_abs_torque = 0.0;
  double max_abs_angle = 0.0;
  double sum_torque_rate_sq = 0.0;
  int torque_rate_samples = 0;
  std::optional<double> previous_torque;

  for (double time_s = 0.0; time_s <= duration_s; time_s += dt_s) {
    const auto state = model.state();
    const double human_torque =
        humanWalkingTorque(time_s, human_torque_amp_nm, cadence_hz);

    const auto controller_output =
        controller.update(time_s, state.angle_rad, state.velocity_rad_s);
    const auto motion_state = motion_detector.update(time_s, state.velocity_rad_s);
    const double gated_torque =
        controller_output.raw_torque_nm * motion_state.assistance_scale;
    const auto limited_torque = limiter.apply(gated_torque, time_s);

    logger.writeRow({
        time_s,
        state.angle_rad,
        state.velocity_rad_s,
        human_torque,
        controller_output.raw_torque_nm,
        motion_state.assistance_scale,
        limited_torque.output_nm,
        controller_output.delayed_signal,
        controller_output.ready ? 1.0 : 0.0,
        limited_torque.saturated ? 1.0 : 0.0,
        limited_torque.rate_limited ? 1.0 : 0.0,
        isWalkingWindow(time_s) ? 1.0 : 0.0,
    });

    peak_abs_torque = std::max(peak_abs_torque, std::abs(limited_torque.output_nm));
    max_abs_angle = std::max(max_abs_angle, std::abs(state.angle_rad));
    if (previous_torque.has_value()) {
      const double torque_rate = (limited_torque.output_nm - *previous_torque) / dt_s;
      sum_torque_rate_sq += torque_rate * torque_rate;
      ++torque_rate_samples;
    }
    previous_torque = limited_torque.output_nm;

    model.step(dt_s, human_torque, limited_torque.output_nm);
  }

  const double rms_torque_rate =
      torque_rate_samples > 0 ? std::sqrt(sum_torque_rate_sq / torque_rate_samples) : 0.0;

  std::cout << "Simulation complete\n";
  std::cout << "Output CSV: " << output_path << "\n";
  std::cout << "Peak absolute exoskeleton torque: " << peak_abs_torque << " Nm\n";
  std::cout << "Maximum absolute hip angle: " << max_abs_angle << " rad\n";
  std::cout << "RMS torque rate: " << rms_torque_rate << " Nm/s\n";

  return 0;
}
