/**
 * @file eos_session_composition_root_test.cpp
 * @brief 验证 EOS 会话装配根的依赖组合与配置同步契约。
 * @note 管线已完全内部化，不再支持外部注入。
 */

#include <gtest/gtest.h>

#include <memory>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

class CountingEnvironmentService final : public environment::IEosEnvironmentService {
 public:
  environment::EosEnvironmentModelResult ResolveFactors(
      const environment::EosEnvironmentModelInputs& inputs) const override {
    ++resolve_count;
    last_inputs = inputs;
    environment::EosEnvironmentModelResult result;
    result.aerosol_density_factor = 1.0f + 0.1f * inputs.cloud_coverage_ratio;
    result.turbulence_factor = 1.0f + 0.02f * inputs.wind_speed_mps;
    result.path_radiance_scale_bias = 1.0f;
    return result;
  }

  mutable std::size_t resolve_count{0U};
  mutable environment::EosEnvironmentModelInputs last_inputs{};
};

config::EosSessionConfig MakeSessionConfig() {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  config.mission.scan_rate_deg_per_sec = 9.0f;
  config.mission.frame_rate_hz = 15.0f;
  config.policy.detection.minimum_snr_db = 60.0f;
  config.policy.detection.detection_sensitivity_w = 2.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 1000.0f;
  config.environment.scenario_config.model_type = environment::EosEnvironmentModelType::kAdvanced;
  config.environment.scenario_config.preset = environment::EosEnvironmentPreset::kDusty;
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

TEST(EosSessionCompositionRootTest, ComposeWithEnvironmentServiceBuildsOwnedPipeline) {
  CountingEnvironmentService environment_service;
  const config::EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition =
      EosSessionCompositionRoot::ComposeWithEnvironmentService(config, environment_service);

  ASSERT_NE(composition.owned_pipeline, nullptr);
  ASSERT_NE(composition.owned_controller, nullptr);
  EXPECT_FLOAT_EQ(composition.internal_config.environment.aerosol_density_factor, 2.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
