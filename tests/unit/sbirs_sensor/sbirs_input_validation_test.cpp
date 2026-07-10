#include <gtest/gtest.h>

#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

TEST(SbirsInputValidationTest, RejectsMissingSatellitePosition) {
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .Build();

  const sbirs_sensor::session::ValidationIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, AcceptsMinimalValidScene) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 10U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 1200.0f;
  target.projected_area_m2 = 10.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::ValidationIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

}  // namespace
