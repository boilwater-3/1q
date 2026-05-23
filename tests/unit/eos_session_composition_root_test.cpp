/**
 * @file eos_session_composition_root_test.cpp
 * @brief 验证 EOS 会话装配根的依赖组合与配置同步契约。
 */

#include <gtest/gtest.h>

#include <memory>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

class CountingPipeline final : public extension::IEosPipeline {
 public:
  void UpdateConfig(const extension::EosPipelineConfig& config,
                    bool reset_scan_phase) override {
    ++update_count;
    last_config = config;
    last_reset_scan_phase = reset_scan_phase;
  }

  extension::EosPipelineExecuteResult RunCycle(const EosCycleInput& input) override {
    extension::EosPipelineExecuteResult result;
    result.executed_this_cycle = true;
    result.abort_reason = extension::EosPipelineAbortReason::kNone;
    return result;
  }

  extension::EosPipelineRuntimeState CaptureRuntimeState() const override {
    extension::EosPipelineRuntimeState state;
    state.owner_identity = this;
    state.schema_version = 1U;
    state.current_scan_azimuth_deg = 0.0f;
    return state;
  }

  bool RestoreRuntimeState(const extension::EosPipelineRuntimeState& state) override {
    (void)state;
    return true;
  }

  std::size_t update_count{0U};
  bool last_reset_scan_phase{false};
  extension::EosPipelineConfig last_config{};
};

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

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  config.mission.scan_rate_deg_per_sec = 9.0f;
  config.mission.frame_rate_hz = 15.0f;
  config.policy.detection.profile = config::EosDetectionProfile::kConservative;
  config.environment.scenario_config.model_type = environment::EosEnvironmentModelType::kAdvanced;
  config.environment.scenario_config.preset = config::EosEnvironmentPreset::kDusty;
  return config;
}

TEST(EosSessionCompositionRootTest, ComposeWithPipelineSyncsInjectedPipelineAndController) {
  CountingPipeline pipeline;
  const EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition =
      EosSessionCompositionRoot::ComposeWithPipeline(config, pipeline);

  EXPECT_EQ(composition.pipeline, &pipeline);
  EXPECT_NE(composition.controller, nullptr);
  EXPECT_EQ(composition.owned_pipeline, nullptr);
  EXPECT_NE(composition.owned_controller, nullptr);
  EXPECT_EQ(pipeline.update_count, 1U);
  EXPECT_TRUE(pipeline.last_reset_scan_phase);
  EXPECT_FLOAT_EQ(pipeline.last_config.scan_rate_deg_per_sec, config.mission.scan_rate_deg_per_sec);
  EXPECT_EQ(composition.runtime_config.mission.work_mode, config.mission.work_mode);
}

TEST(EosSessionCompositionRootTest, ComposeWithControllerSyncsProvidedControllerPipeline) {
  CountingPipeline pipeline;
  extension::EosController controller(pipeline);
  const EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition =
      EosSessionCompositionRoot::ComposeWithController(config, controller);

  EXPECT_EQ(composition.pipeline, &pipeline);
  EXPECT_EQ(composition.controller, &controller);
  EXPECT_EQ(composition.owned_pipeline, nullptr);
  EXPECT_EQ(composition.owned_controller, nullptr);
  EXPECT_EQ(pipeline.update_count, 1U);
  EXPECT_TRUE(pipeline.last_reset_scan_phase);
  EXPECT_FLOAT_EQ(pipeline.last_config.frame_rate_hz, config.mission.frame_rate_hz);
}

TEST(EosSessionCompositionRootTest, ComposeDefaultBuildsOwnedGraphAndRuntimeAssembly) {
  const EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition = EosSessionCompositionRoot::ComposeDefault(config);

  ASSERT_NE(composition.owned_pipeline, nullptr);
  ASSERT_NE(composition.owned_controller, nullptr);
  EXPECT_EQ(composition.pipeline, composition.owned_pipeline.get());
  EXPECT_EQ(composition.controller, composition.owned_controller.get());
  EXPECT_EQ(composition.runtime_config.mission.work_mode, config.mission.work_mode);
  EXPECT_FLOAT_EQ(composition.pipeline_config.minimum_snr_db, 60.0f);
}

TEST(EosSessionCompositionRootTest, ComposeWithEnvironmentServiceBuildsOwnedPipeline) {
  CountingEnvironmentService environment_service;
  const EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition =
      EosSessionCompositionRoot::ComposeWithEnvironmentService(config, environment_service);

  ASSERT_NE(composition.owned_pipeline, nullptr);
  ASSERT_NE(composition.owned_controller, nullptr);
  EXPECT_EQ(composition.pipeline, composition.owned_pipeline.get());
  EXPECT_EQ(composition.controller, composition.owned_controller.get());
  EXPECT_FLOAT_EQ(composition.pipeline_config.aerosol_density_factor, 2.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
