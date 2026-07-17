/**
 * @file eos_session_composition_root_test.cpp
 * @brief 验证 EOS 会话装配根的依赖组合与配置同步契约。
 * @note 管线与环境服务已完全内部化，不再支持外部注入。
 */

#include <gtest/gtest.h>

#include <memory>

#include "electro_optical_sensor/runtime/EosController.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

config::EosSessionConfig MakeSessionConfig() {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  config.mission.scan_rate_deg_per_sec = 9.0f;
  config.mission.frame_rate_hz = 15.0f;
  config.policy.detection.minimum_snr_db = 60.0f;
  config.policy.detection.detection_sensitivity_w = 2.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 1000.0f;
  config.environment.scenario_config.model_type = config::EosEnvironmentModelType::kAdvanced;
  config.environment.scenario_config.preset = config::EosEnvironmentPreset::kDusty;
  return config;
}

TEST(EosSessionCompositionRootTest, ComposeDefaultBuildsOwnedGraphAndRuntimeAssembly) {
  const config::EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition = EosSessionCompositionRoot::ComposeDefault(config);

  ASSERT_NE(composition.owned_pipeline, nullptr);
  ASSERT_NE(composition.owned_controller, nullptr);
  EXPECT_EQ(composition.internal_config.scan.work_mode, config.mission.work_mode);
  EXPECT_FLOAT_EQ(composition.internal_config.detection.minimum_snr_db, 60.0f);
}

TEST(EosEnvironmentConfigMapperTest, MapsEveryPresetToItsExactBaseline) {
  struct ExpectedPreset {
    config::EosEnvironmentPreset preset;
    config::RadiativeTransferModel radiative_model;
    float aerosol_factor;
    float turbulence_factor;
  };
  const ExpectedPreset cases[] = {
      {config::EosEnvironmentPreset::kStandard,
       config::RadiativeTransferModel::kDerivedBeerLambert, 1.0f, 1.0f},
      {config::EosEnvironmentPreset::kHumid,
       config::RadiativeTransferModel::kHumidityWeighted, 1.1f, 1.1f},
      {config::EosEnvironmentPreset::kDusty,
       config::RadiativeTransferModel::kAdaptivePathRadiance, 2.0f, 1.2f},
      {config::EosEnvironmentPreset::kTurbulent,
       config::RadiativeTransferModel::kAdaptivePathRadiance, 1.3f, 1.8f},
      {config::EosEnvironmentPreset::kMaritime,
       config::RadiativeTransferModel::kHumidityWeighted, 1.5f, 1.4f},
  };

  for (const ExpectedPreset& expected : cases) {
    config::EosEnvironmentScenarioConfig scenario;
    scenario.model_type = config::EosEnvironmentModelType::kAdvanced;
    scenario.preset = expected.preset;

    const config::EosEnvironmentModelConfig mapped =
        config::BuildModelConfigFromScenario(scenario);

    EXPECT_EQ(mapped.model_type, config::EosEnvironmentModelType::kAdvanced);
    EXPECT_EQ(mapped.radiative_transfer_model, expected.radiative_model);
    EXPECT_FLOAT_EQ(mapped.aerosol_density_factor, expected.aerosol_factor);
    EXPECT_FLOAT_EQ(mapped.turbulence_factor, expected.turbulence_factor);
  }
}

TEST(EosEnvironmentConfigMapperTest, EnabledCustomOverridesReplaceWholePresetGroup) {
  config::EosEnvironmentScenarioConfig scenario;
  scenario.preset = config::EosEnvironmentPreset::kDusty;
  scenario.has_custom_overrides = true;
  scenario.custom_overrides.radiative_transfer_model =
      config::RadiativeTransferModel::kHumidityWeighted;
  scenario.custom_overrides.aerosol_density_factor = 3.25f;
  scenario.custom_overrides.turbulence_factor = 2.75f;

  const config::EosEnvironmentModelConfig mapped =
      config::BuildModelConfigFromScenario(scenario);

  EXPECT_EQ(mapped.radiative_transfer_model,
            config::RadiativeTransferModel::kHumidityWeighted);
  EXPECT_FLOAT_EQ(mapped.aerosol_density_factor, 3.25f);
  EXPECT_FLOAT_EQ(mapped.turbulence_factor, 2.75f);
}

TEST(EosEnvironmentConfigMapperTest, DisabledCustomOverridesAreIgnored) {
  config::EosEnvironmentScenarioConfig scenario;
  scenario.preset = config::EosEnvironmentPreset::kHumid;
  scenario.has_custom_overrides = false;
  scenario.custom_overrides.radiative_transfer_model =
      config::RadiativeTransferModel::kAdaptivePathRadiance;
  scenario.custom_overrides.aerosol_density_factor = 99.0f;
  scenario.custom_overrides.turbulence_factor = 88.0f;

  const config::EosEnvironmentModelConfig mapped =
      config::BuildModelConfigFromScenario(scenario);

  EXPECT_EQ(mapped.radiative_transfer_model,
            config::RadiativeTransferModel::kHumidityWeighted);
  EXPECT_FLOAT_EQ(mapped.aerosol_density_factor, 1.1f);
  EXPECT_FLOAT_EQ(mapped.turbulence_factor, 1.1f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
