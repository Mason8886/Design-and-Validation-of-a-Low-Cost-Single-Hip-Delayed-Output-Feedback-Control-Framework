#include "dofc/model/SingleHipModel.hpp"

#include <cmath>
#include <stdexcept>

namespace dofc {

SingleHipModel::SingleHipModel(SingleHipModelConfig config) : config_(config) {
  if (!std::isfinite(config_.inertia_kg_m2) ||
      !std::isfinite(config_.damping_nm_s_per_rad) ||
      !std::isfinite(config_.passive_stiffness_nm_per_rad) ||
      config_.inertia_kg_m2 <= 0.0 ||
      config_.damping_nm_s_per_rad < 0.0 ||
      config_.passive_stiffness_nm_per_rad < 0.0) {
    throw std::invalid_argument("invalid single hip model parameters");
  }
}

void SingleHipModel::reset(SingleHipState state) {
  state_ = state;
}

void SingleHipModel::step(double dt_s, double human_torque_nm, double exoskeleton_torque_nm) {
  if (!std::isfinite(dt_s) ||
      !std::isfinite(human_torque_nm) ||
      !std::isfinite(exoskeleton_torque_nm) ||
      dt_s <= 0.0) {
    throw std::invalid_argument("invalid model step input");
  }

  const double total_torque = human_torque_nm + exoskeleton_torque_nm;

  const double k1_angle = state_.velocity_rad_s;
  const double k1_velocity =
      acceleration(state_.angle_rad, state_.velocity_rad_s, total_torque);

  const double k2_angle = state_.velocity_rad_s + 0.5 * dt_s * k1_velocity;
  const double k2_velocity =
      acceleration(state_.angle_rad + 0.5 * dt_s * k1_angle,
                   state_.velocity_rad_s + 0.5 * dt_s * k1_velocity,
                   total_torque);

  const double k3_angle = state_.velocity_rad_s + 0.5 * dt_s * k2_velocity;
  const double k3_velocity =
      acceleration(state_.angle_rad + 0.5 * dt_s * k2_angle,
                   state_.velocity_rad_s + 0.5 * dt_s * k2_velocity,
                   total_torque);

  const double k4_angle = state_.velocity_rad_s + dt_s * k3_velocity;
  const double k4_velocity =
      acceleration(state_.angle_rad + dt_s * k3_angle,
                   state_.velocity_rad_s + dt_s * k3_velocity,
                   total_torque);

  state_.angle_rad += dt_s * (k1_angle + 2.0 * k2_angle + 2.0 * k3_angle + k4_angle) / 6.0;
  state_.velocity_rad_s +=
      dt_s * (k1_velocity + 2.0 * k2_velocity + 2.0 * k3_velocity + k4_velocity) / 6.0;
}

SingleHipState SingleHipModel::state() const noexcept {
  return state_;
}

SingleHipModelConfig SingleHipModel::config() const noexcept {
  return config_;
}

double SingleHipModel::acceleration(double angle_rad,
                                    double velocity_rad_s,
                                    double total_torque_nm) const {
  const double passive_torque =
      config_.damping_nm_s_per_rad * velocity_rad_s +
      config_.passive_stiffness_nm_per_rad * angle_rad;
  return (total_torque_nm - passive_torque) / config_.inertia_kg_m2;
}

}  // namespace dofc
