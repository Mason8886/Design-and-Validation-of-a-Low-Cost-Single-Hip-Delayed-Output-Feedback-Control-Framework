#include "dofc/control/DofcController.hpp"
#include "test_harness.hpp"

void testDofcPureDelay() {
  dofc::DofcController controller({
      2.0,
      0.5,
      1.0,
      dofc::FeedbackSignal::Angle,
      dofc::FeedbackLaw::PureDelay,
      false,
  });

  auto first = controller.update(0.0, 1.0, 0.0);
  test::expectTrue(!first.ready, "DOFC should not be ready before delay history exists");

  controller.update(0.5, 2.0, 0.0);
  auto second = controller.update(1.0, 3.0, 0.0);
  test::expectTrue(second.ready, "DOFC should be ready after delay history exists");
  test::expectNear(second.delayed_signal, 2.0, 1e-9, "DOFC delayed signal");
  test::expectNear(second.raw_torque_nm, 4.0, 1e-9, "DOFC pure delay torque");
}
