#pragma once

namespace dofc {

struct SingleHipModelConfig {
  double inertia_kg_m2{0.08};
  double damping_nm_s_per_rad{0.25};
  double passive_stiffness_nm_per_rad{2.0};
};

struct SingleHipState {
  double angle_rad{0.0};
  double velocity_rad_s{0.0};
};

class SingleHipModel {
public:
  explicit SingleHipModel(SingleHipModelConfig config = {});

  void reset(SingleHipState state = {});
  void step(double dt_s, double human_torque_nm, double exoskeleton_torque_nm);

  SingleHipState state() const noexcept;
  SingleHipModelConfig config() const noexcept;

private:
  double acceleration(double angle_rad,
                      double velocity_rad_s,
                      double total_torque_nm) const;

  SingleHipModelConfig config_;
  SingleHipState state_;
};

}  // namespace dofc
