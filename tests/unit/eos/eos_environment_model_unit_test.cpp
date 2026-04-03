/**
 * @file eos_environment_model_unit_test.cpp
 * @brief 验证 EOS 环境模型因子派生行为。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/environment/EosEnvironmentModel.h"

namespace electro_optical_sensor {
namespace environment {
namespace {

TEST(EosEnvironmentModelTest, AdvancedModelAmplifiesAerosolAndTurbulenceFactors) {
  EosEnvironmentModelInputs simplified_inputs;
  simplified_inputs.model_type = EosEnvironmentModelType::kSimplified;
  simplified_inputs.platform_altitude_m = 5000.0f;
  simplified_inputs.cloud_coverage_ratio = 0.7f;
  simplified_inputs.wind_speed_mps = 25.0f;
  simplified_inputs.base_aerosol_density_factor = 1.2f;
  simplified_inputs.base_turbulence_factor = 1.1f;

  EosEnvironmentModelInputs advanced_inputs = simplified_inputs;
  advanced_inputs.model_type = EosEnvironmentModelType::kAdvanced;

  const EosEnvironmentModelResult simplified_result =
      ResolveEnvironmentFactors(simplified_inputs);
  const EosEnvironmentModelResult advanced_result =
      ResolveEnvironmentFactors(advanced_inputs);

  EXPECT_GT(advanced_result.aerosol_density_factor,
            simplified_result.aerosol_density_factor);
  EXPECT_GT(advanced_result.turbulence_factor,
            simplified_result.turbulence_factor);
  EXPECT_GT(advanced_result.path_radiance_scale_bias, 1.0f);
}

TEST(EosEnvironmentModelTest, InvalidOrNegativeInputsAreClampedToSafeRange) {
  EosEnvironmentModelInputs inputs;
  inputs.model_type = EosEnvironmentModelType::kAdvanced;
  inputs.platform_altitude_m = -1000.0f;
  inputs.cloud_coverage_ratio = 5.0f;
  inputs.wind_speed_mps = -15.0f;
  inputs.base_aerosol_density_factor = 0.0f;
  inputs.base_turbulence_factor = -1.0f;

  const EosEnvironmentModelResult result = ResolveEnvironmentFactors(inputs);

  EXPECT_GE(result.aerosol_density_factor, 1.0f);
  EXPECT_GE(result.turbulence_factor, 1.0f);
  EXPECT_GE(result.path_radiance_scale_bias, 1.0f);
}

}  // namespace
}  // namespace environment
}  // namespace electro_optical_sensor
