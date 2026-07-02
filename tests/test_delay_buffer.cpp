#include "dofc/core/DelayBuffer.hpp"
#include "test_harness.hpp"

void testDelayBufferInterpolation() {
  dofc::DelayBuffer buffer(2.0);
  buffer.push(0.0, 0.0);
  buffer.push(1.0, 10.0);
  buffer.push(2.0, 20.0);

  const auto mid = buffer.valueAt(0.5);
  test::expectTrue(mid.has_value(), "delay buffer should return interpolated value");
  test::expectNear(*mid, 5.0, 1e-9, "delay buffer interpolation");

  const auto delayed = buffer.delayedValue(2.0, 0.5);
  test::expectTrue(delayed.has_value(), "delay buffer should return delayed value");
  test::expectNear(*delayed, 15.0, 1e-9, "delay buffer delayed lookup");
}
