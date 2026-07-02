#include "test_harness.hpp"

#include <iostream>

void testDelayBufferInterpolation();
void testDofcPureDelay();
void testFullScaleHumanTorqueModelResponse();
void testTorqueLimiterMagnitudeAndRate();
void testSeriesElasticEstimator();

int main() {
  testDelayBufferInterpolation();
  testDofcPureDelay();
  testFullScaleHumanTorqueModelResponse();
  testTorqueLimiterMagnitudeAndRate();
  testSeriesElasticEstimator();

  if (test::failures == 0) {
    std::cout << "All tests passed\n";
    return 0;
  }

  std::cerr << test::failures << " test(s) failed\n";
  return 1;
}
