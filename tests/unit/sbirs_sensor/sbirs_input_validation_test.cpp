#include <gtest/gtest.h>

#include <limits>
#include <vector>

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

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
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

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsFiniteDomainFlagIdAndEnvironmentMatrix) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 10U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 1200.0f;
  target.projected_area_m2 = 10.0f;
  const sbirs_sensor::session::SbirsCycleInput valid =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  std::vector<sbirs_sensor::session::SbirsCycleInput> invalid_inputs;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().satellite_position_ecef_m = Vector(0.0, 0.0, 0.0);
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().temperature_k =
      std::numeric_limits<float>::quiet_NaN();
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().emissivity = 1.1f;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().projected_area_m2 =
      std::numeric_limits<float>::infinity();
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().target_id = 0U;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.push_back(invalid_inputs.back().scene.front());
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().velocity_ecef_m_per_s.x = 1.0;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().has_velocity_ecef_m_per_s = true;
  invalid_inputs.back().scene.front().velocity_ecef_m_per_s.y =
      std::numeric_limits<double>::quiet_NaN();

  for (std::size_t i = 0; i < invalid_inputs.size(); ++i) {
    const sbirs_sensor::session::SbirsIssueList issues =
        sbirs_sensor::session::ValidateSbirsCycleInput(invalid_inputs[i], 10.0f);
    EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues)) << "case " << i;
  }
}

TEST(SbirsInputValidationTest, AcceptsDtSecWithinFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 10/10 = 1.0s; dt_sec=0.5 is within range.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 1200.0f;
  target.projected_area_m2 = 10.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(0.5f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsDtSecExceedingFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 10/10 = 1.0s; dt_sec=2.0 exceeds the bound.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 1200.0f;
  target.projected_area_m2 = 10.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(2.0f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, AcceptsDtSecAtExactFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 10/10 = 1.0s; dt_sec=1.0 is exactly at the boundary.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
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

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsDtSecJustAboveFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 1.0s; dt_sec=1.001 is just above.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 1200.0f;
  target.projected_area_m2 = 10.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.001f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, HigherFrameRateAllowsTighterDtSec) {
  // frame_rate_hz=20 → max_dt = 10/20 = 0.5s; dt_sec=0.8 should fail.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.temperature_k = 1200.0f;
  target.projected_area_m2 = 10.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(0.8f)
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .AddTarget(target)
          .Build();

  // At 20Hz, max_dt=0.5s → 0.8s should fail.
  const sbirs_sensor::session::SbirsIssueList issues_20hz =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 20.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues_20hz));

  // At 10Hz, max_dt=1.0s → 0.8s should pass.
  const sbirs_sensor::session::SbirsIssueList issues_10hz =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues_10hz));
}

}  // namespace
