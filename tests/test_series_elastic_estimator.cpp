#include "dofc/estimation/SeriesElasticEstimator.hpp"
#include "test_harness.hpp"

void testSeriesElasticEstimator() {
  dofc::SeriesElasticEstimator estimator({35.0, 7.0, 0.0, 0.0, 1.0});
  const auto estimate = estimator.estimate({0.7, 0.05});

  test::expectNear(estimate.spring_deflection_rad,
                   0.05,
                   1e-9,
                   "series elastic spring deflection");
  test::expectNear(estimate.torque_nm, 1.75, 1e-9, "series elastic torque");
}
