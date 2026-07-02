#include "dofc/estimation/SeriesElasticEstimator.hpp"

#include <cmath>
#include <stdexcept>

namespace dofc {

SeriesElasticEstimator::SeriesElasticEstimator(SeriesElasticConfig config) : config_(config) {
  if (!std::isfinite(config_.stiffness_nm_per_rad) ||
      !std::isfinite(config_.gear_ratio) ||
      config_.stiffness_nm_per_rad <= 0.0 ||
      config_.gear_ratio <= 0.0) {
    throw std::invalid_argument("series elastic stiffness and gear ratio must be positive");
  }
}

void SeriesElasticEstimator::setZeroFromCurrentPose(double motor_encoder_rad,
                                                    double output_encoder_rad) {
  config_.motor_zero_rad = motor_encoder_rad;
  config_.output_zero_rad = output_encoder_rad;
}

TorqueEstimate SeriesElasticEstimator::estimate(SeriesElasticMeasurement measurement) const {
  if (!std::isfinite(measurement.motor_encoder_rad) ||
      !std::isfinite(measurement.output_encoder_rad)) {
    throw std::invalid_argument("encoder measurements must be finite");
  }

  const double motor_output_side_rad =
      (measurement.motor_encoder_rad - config_.motor_zero_rad) / config_.gear_ratio;
  const double output_rad = measurement.output_encoder_rad - config_.output_zero_rad;
  const double spring_deflection_rad = motor_output_side_rad - output_rad;

  return {
      spring_deflection_rad,
      config_.sign * config_.stiffness_nm_per_rad * spring_deflection_rad};
}

SeriesElasticConfig SeriesElasticEstimator::config() const noexcept {
  return config_;
}

}  // namespace dofc
