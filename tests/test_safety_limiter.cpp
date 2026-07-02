#include "dofc/core/SafetyLimiter.hpp"
#include "test_harness.hpp"

void testTorqueLimiterMagnitudeAndRate() {
  dofc::TorqueLimiter limiter({5.0, 10.0});

  auto initial = limiter.apply(100.0, 0.0);
  test::expectTrue(initial.saturated, "torque limiter should detect saturation");
  test::expectNear(initial.output_nm, 5.0, 1e-9, "torque limiter magnitude clamp");

  auto rate_limited = limiter.apply(-5.0, 0.1);
  test::expectTrue(rate_limited.rate_limited, "torque limiter should detect rate limiting");
  test::expectNear(rate_limited.output_nm, 4.0, 1e-9, "torque limiter rate clamp");
}
