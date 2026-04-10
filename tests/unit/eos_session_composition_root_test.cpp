/**
 * @file eos_session_composition_root_test.cpp
 * @brief 验证 EOS 会话装配根的依赖组合与配置同步契约。
 */

#include <gtest/gtest.h>

#include <memory>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
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

  common::EosOutputFrame Execute(const EosCycleInput& input) override {
    common::EosOutputFrame frame;
    frame.cycle_index = input.cycle_index;
    return frame;
  }

  std::size_t update_count{0U};
  bool last_reset_scan_phase{false};
  extension::EosPipelineConfig last_config{};
};

class CountingEnvironmentService final : public extension::IEosEnvironmentService {
 public:
  environment::EosEnvironmentModelResult ResolveFactors(
      const environment::EosEnvironmentModelInputs& inputs) const override {
    ++resolve_count;
    last_inputs = inputs;
    environment::EosEnvironmentModelResult result;
    result.aerosol_density_factor = inputs.base_aerosol_density_factor;
    result.turbulence_factor = inputs.base_turbulence_factor;
    result.path_radiance_scale_bias = 1.0f;
    return result;
  }

  mutable std::size_t resolve_count{0U};
  mutable environment::EosEnvironmentModelInputs last_inputs{};
};

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.work_mode = EosWorkMode::kVisibleOnly;
  config.scan_rate_deg_per_sec = 9.0f;
  config.frame_rate_hz = 15.0f;
  config.minimum_snr_db = 3.0f;
  config.environment_default_config.model_type = environment::EosEnvironmentModelType::kAdvanced;
  config.environment_default_config.aerosol_density_factor = 1.8f;
  config.environment_default_config.turbulence_factor = 1.4f;
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
  EXPECT_FLOAT_EQ(pipeline.last_config.scan_rate_deg_per_sec, config.scan_rate_deg_per_sec);
  EXPECT_EQ(composition.runtime_config.work_mode, config.work_mode);
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
  EXPECT_FLOAT_EQ(pipeline.last_config.frame_rate_hz, config.frame_rate_hz);
}

TEST(EosSessionCompositionRootTest, ComposeDefaultBuildsOwnedGraphAndRuntimeAssembly) {
  const EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition = EosSessionCompositionRoot::ComposeDefault(config);

  ASSERT_NE(composition.owned_pipeline, nullptr);
  ASSERT_NE(composition.owned_controller, nullptr);
  EXPECT_EQ(composition.pipeline, composition.owned_pipeline.get());
  EXPECT_EQ(composition.controller, composition.owned_controller.get());
  EXPECT_EQ(composition.runtime_config.work_mode, config.work_mode);
  EXPECT_FLOAT_EQ(composition.pipeline_config.minimum_snr_db, config.minimum_snr_db);
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
  EXPECT_FLOAT_EQ(composition.pipeline_config.aerosol_density_factor,
                  config.environment_default_config.aerosol_density_factor);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
