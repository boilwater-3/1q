/**
 * @file eos_session_factory_test.cpp
 * @brief 验证 EOS 会话工厂的外部注入与装配路径契约。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace {

class CountingPipeline final : public extension::IEosPipeline {
 public:
  void UpdateConfig(const extension::EosPipelineConfig& config,
                    bool reset_scan_phase) override {
    ++update_count;
    last_config = config;
    last_reset_scan_phase = reset_scan_phase;
  }

  extension::EosPipelineExecuteResult Execute(const EosCycleInput& input) override {
    ++execute_count;
    extension::EosPipelineExecuteResult result;
    result.scan_azimuth_deg = 42.0f;
    output::EosDetectionRecord detection;
    detection.target_id = 99U;
    detection.detected = true;
    detection.fused_snr_linear = 12.5f;
    result.detections.push_back(detection);
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
  std::size_t execute_count{0U};
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
    result.aerosol_density_factor = 1.0f + 0.2f * inputs.cloud_coverage_ratio;
    result.turbulence_factor = 1.0f + 0.01f * inputs.wind_speed_mps;
    result.path_radiance_scale_bias = 1.0f;
    return result;
  }

  mutable std::size_t resolve_count{0U};
  mutable environment::EosEnvironmentModelInputs last_inputs{};
};

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = config::EosDetectionProfile::kAggressive;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 4.0f;
  return config;
}

EosCycleInput MakeValidInput(std::uint32_t cycle_index) {
  EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.environment.solar_irradiance_w_m2 = 800.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.cloud_coverage_ratio = 0.2f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  return input;
}

TEST(EosSessionFactoryTest, CreateWithPipelineUsesInjectedPipeline) {
  CountingPipeline pipeline;
  EosSession session = EosSessionFactory::CreateWithPipeline(MakeSessionConfig(), pipeline);

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(MakeValidInput(8U));

  EXPECT_EQ(pipeline.update_count, 2U);
  EXPECT_EQ(pipeline.execute_count, 1U);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 8U);
  EXPECT_EQ(result.output_frame.detections.size(), 1U);
  EXPECT_EQ(result.output_frame.detections.front().target_id, 99U);
}

TEST(EosSessionFactoryTest, CreateWithEnvironmentServiceUsesInjectedService) {
  CountingEnvironmentService environment_service;
  EosSession session = EosSessionFactory::CreateWithEnvironmentService(MakeSessionConfig(),
                                                                       environment_service);

  EosCycleInput input = MakeValidInput(9U);
  input.scene.clear();
  EosSceneTarget target;
  target.target_id = 7U;
  target.range_m = 1000.0f;
  target.azimuth_deg = -55.0f;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 500.0f;
  target.appearance.emissivity = 0.9f;
  target.appearance.reflectance = 0.2f;
  target.appearance.projected_area_m2 = 4.0f;
  input.scene.push_back(target);

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_EQ(environment_service.resolve_count, 1U);
  EXPECT_FLOAT_EQ(environment_service.last_inputs.cloud_coverage_ratio, 0.2f);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 9U);
}

TEST(EosSessionFactoryTest, CreateUsesDefaultPipelineAndProducesResult) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());

  EosCycleInput input = MakeValidInput(10U);
  input.scene.clear();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 10U);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosSessionFactoryTest, CreateWithControllerReusesProvidedController) {
  CountingPipeline pipeline;
  extension::EosController controller(pipeline);
  EosSession session = EosSessionFactory::CreateWithController(MakeSessionConfig(), controller);

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(MakeValidInput(11U));

  EXPECT_EQ(pipeline.update_count, 2U);
  EXPECT_EQ(pipeline.execute_count, 1U);
  EXPECT_TRUE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 11U);
}

TEST(EosSessionFactoryTest, CreateWithControllerSessionMoveKeepsExternalControllerPath) {
  CountingPipeline pipeline;
  extension::EosController controller(pipeline);
  EosSession session = EosSessionFactory::CreateWithController(MakeSessionConfig(), controller);

  EosSession moved_session(std::move(session));
  const ::electro_optical_sensor::session::EosCycleResult result = moved_session.StepWithResult(MakeValidInput(12U));

  EXPECT_EQ(pipeline.update_count, 2U);
  EXPECT_EQ(pipeline.execute_count, 1U);
  EXPECT_TRUE(controller.ExecutedLatestCycle());
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 12U);
}

TEST(EosSessionFactoryTest, ApplyRuntimeConfigUpdatesInjectedControllerPipeline) {
  CountingPipeline pipeline;
  extension::EosController controller(pipeline);
  EosSession session = EosSessionFactory::CreateWithController(MakeSessionConfig(), controller);

  EXPECT_EQ(pipeline.update_count, 2U);

  EosRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 9.0f;
  session.ApplyRuntimeConfig(patch);

  EXPECT_EQ(pipeline.update_count, 3U);
  EXPECT_TRUE(pipeline.last_reset_scan_phase);
}

TEST(EosSessionFactoryTest, InvalidRuntimeConfigDoesNotUpdateInjectedControllerPipeline) {
  CountingPipeline pipeline;
  extension::EosController controller(pipeline);
  EosSession session = EosSessionFactory::CreateWithController(MakeSessionConfig(), controller);

  EXPECT_EQ(pipeline.update_count, 2U);

  EosRuntimeConfigPatch patch;
  patch.has_frame_rate_hz = true;
  patch.frame_rate_hz = 0.0f;
  session.ApplyRuntimeConfig(patch);

  EXPECT_EQ(pipeline.update_count, 2U);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
