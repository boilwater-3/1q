/**
 * @file eos_environment_model_unit_test.cpp
 * @brief 验证 EOS 环境模型因子派生行为。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/environment/EosEnvironmentModel.h"

namespace electro_optical_sensor {
namespace environment {
namespace {

TEST(EosEnvironmentModelTest, CycleEnvironmentAlwaysModifiesPresetBaseline) {
  EnvironmentModelInputs baseline_inputs;
  const EnvironmentModelResult baseline_result =
      ResolveEnvironmentFactors(baseline_inputs);

  EnvironmentModelInputs observed_inputs;
  observed_inputs.platform_altitude_m = 5000.0f;
  observed_inputs.cloud_coverage_ratio = 0.7f;
  observed_inputs.wind_speed_mps = 25.0f;
  const EnvironmentModelResult observed_result =
      ResolveEnvironmentFactors(observed_inputs);

  EXPECT_GT(observed_result.aerosol_density_factor,
            baseline_result.aerosol_density_factor);
  EXPECT_GT(observed_result.turbulence_factor,
            baseline_result.turbulence_factor);
  EXPECT_GT(observed_result.path_radiance_scale_bias, 1.0f);
}

TEST(EosEnvironmentModelTest, EnabledAtmosphericPhysicsAppliesHumidityAndTemperature) {
  EnvironmentModelInputs baseline_inputs;
  EnvironmentModelInputs physical_inputs = baseline_inputs;
  physical_inputs.atmospheric_physics.enable_physical_model = true;
  physical_inputs.atmospheric_physics.relative_humidity = 0.8f;
  physical_inputs.atmospheric_physics.temperature_k = 318.15f;

  const auto baseline_result = ResolveEnvironmentFactors(baseline_inputs);
  const auto physical_result = ResolveEnvironmentFactors(physical_inputs);

  EXPECT_GT(physical_result.aerosol_density_factor,
            baseline_result.aerosol_density_factor);
  EXPECT_GT(physical_result.turbulence_factor,
            baseline_result.turbulence_factor);
}

TEST(EosEnvironmentModelTest, InvalidOrNegativeInputsAreClampedToSafeRange) {
  EnvironmentModelInputs inputs;
  inputs.platform_altitude_m = -1000.0f;
  inputs.cloud_coverage_ratio = 5.0f;
  inputs.wind_speed_mps = -15.0f;

  const EnvironmentModelResult result = ResolveEnvironmentFactors(inputs);

  EXPECT_GE(result.aerosol_density_factor, 1.0f);
  EXPECT_GE(result.turbulence_factor, 1.0f);
  EXPECT_GE(result.path_radiance_scale_bias, 1.0f);
}

}  // namespace
}  // namespace environment
}  // namespace electro_optical_sensor
