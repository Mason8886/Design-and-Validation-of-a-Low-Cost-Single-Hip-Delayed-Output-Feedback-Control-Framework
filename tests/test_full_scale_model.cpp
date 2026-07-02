#include "dofc/model/SingleHipModel.hpp"
#include "test_harness.hpp"

#include <algorithm>
#include <cmath>

void testFullScaleHumanTorqueModelResponse() {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kHumanTorqueAmplitudeNm = 60.0;
  constexpr double kCadenceHz = 1.0;
  constexpr double kDurationS = 16.0;
  constexpr double kDtS = 0.002;

  dofc::SingleHipModel model;

  double max_abs_angle_rad = 0.0;
  for (double time_s = 0.0; time_s <= kDurationS; time_s += kDtS) {
    const bool walking = time_s < 8.0 || time_s >= 12.0;
    const double human_torque_nm = walking
                                       ? kHumanTorqueAmplitudeNm *
                                             std::sin(2.0 * kPi * kCadenceHz * time_s)
                                       : 0.0;
    const auto state = model.state();
    max_abs_angle_rad = std::max(max_abs_angle_rad, std::abs(state.angle_rad));
    model.step(kDtS, human_torque_nm, 0.0);
  }

  test::expectTrue(max_abs_angle_rad > 0.45,
                   "full-scale model should not be over-damped into a negligible response");
  test::expectTrue(max_abs_angle_rad < 0.80,
                   "full-scale model should stay inside a reasonable walking hip range");
}
