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

void testDefaultPartialAssistanceTorqueLimit() {
  dofc::TorqueLimiter limiter({6.0, 20.0});

  auto saturated = limiter.apply(60.0, 0.0);
  test::expectTrue(saturated.saturated,
                   "default partial-assistance limiter should report saturation");
  test::expectNear(saturated.output_nm,
                   6.0,
                   1e-9,
                   "default partial-assistance limiter should clamp to 6 Nm");

  auto rate_limited = limiter.apply(-6.0, 0.1);
  test::expectTrue(rate_limited.rate_limited,
                   "default partial-assistance limiter should report rate limiting");
  test::expectNear(rate_limited.output_nm,
                   4.0,
                   1e-9,
                   "default partial-assistance limiter should clamp torque rate");
}
