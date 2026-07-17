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
  invalid_inputs.push_back(valid);
  invalid_inputs.back().environment.has_environment_override = true;
  invalid_inputs.back().environment.environment.relative_humidity_percent = 101.0f;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().environment.has_environment_override = true;
  invalid_inputs.back().environment.environment.weather_type =
      static_cast<sbirs_sensor::config::SbirsWeatherType>(99);
  invalid_inputs.push_back(valid);
  invalid_inputs.back().environment.has_environment_override = true;
  invalid_inputs.back().environment.environment.base_atmospheric_transmittance =
      std::numeric_limits<float>::quiet_NaN();

  for (std::size_t i = 0; i < invalid_inputs.size(); ++i) {
    const sbirs_sensor::session::ValidationIssueList issues =
        sbirs_sensor::session::ValidateSbirsCycleInput(invalid_inputs[i]);
    EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues)) << "case " << i;
  }
}

}  // namespace
