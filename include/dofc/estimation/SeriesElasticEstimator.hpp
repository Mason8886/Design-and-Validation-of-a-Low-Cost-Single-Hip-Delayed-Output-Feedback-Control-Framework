#pragma once

namespace dofc {

struct SeriesElasticConfig {
  double stiffness_nm_per_rad{35.0};
  double gear_ratio{7.0};
  double motor_zero_rad{0.0};
  double output_zero_rad{0.0};
  double sign{1.0};
};

struct SeriesElasticMeasurement {
  double motor_encoder_rad{0.0};
  double output_encoder_rad{0.0};
};

struct TorqueEstimate {
  double spring_deflection_rad{0.0};
  double torque_nm{0.0};
};

class SeriesElasticEstimator {
public:
  explicit SeriesElasticEstimator(SeriesElasticConfig config = {});

  void setZeroFromCurrentPose(double motor_encoder_rad, double output_encoder_rad);
  TorqueEstimate estimate(SeriesElasticMeasurement measurement) const;
  SeriesElasticConfig config() const noexcept;

private:
  SeriesElasticConfig config_;
};

}  // namespace dofc
