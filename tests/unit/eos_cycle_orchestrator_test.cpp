/**
 * @file eos_cycle_orchestrator_test.cpp
 * @brief 验证 EOS 运行期周期编排器的运行时配置提交与周期执行契约。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "electro_optical_sensor/runtime/EosCycleOrchestrator.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

namespace eos_config = ::electro_optical_sensor::config;

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.work_mode = EosWorkMode::kInfraredOnly;
  config.minimum_snr_db = -120.0f;
  config.scan_start_az_deg = -10.0f;
  config.scan_end_az_deg = 10.0f;
  config.scan_rate_deg_per_sec = 5.0f;
  config.horizontal_fov_deg = 20.0f;
  config.vertical_fov_deg = 4.0f;
  return config;
}

EosCycleInput MakeCycleInput(std::uint32_t cycle_index, float dt_sec) {
  EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = dt_sec;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = DayNightType::kDay;
  return input;
}

::electro_optical_sensor::extension::EosPipelineConfig BuildPipelineConfig(
    const EosSessionConfig& config) {
  ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config;
  pipeline_config.wavelength_lower_um = config.wavelength_lower_um;
  pipeline_config.wavelength_upper_um = config.wavelength_upper_um;
  pipeline_config.optical_aperture_m = config.optical_aperture_m;
  pipeline_config.focal_length_m = config.focal_length_m;
  pipeline_config.work_mode =
      ::electro_optical_sensor::extension::EosPipelineWorkMode::kInfraredOnly;
  pipeline_config.horizontal_fov_deg = config.horizontal_fov_deg;
  pipeline_config.vertical_fov_deg = config.vertical_fov_deg;
  pipeline_config.scan_rate_deg_per_sec = config.scan_rate_deg_per_sec;
  pipeline_config.frame_rate_hz = config.frame_rate_hz;
  pipeline_config.minimum_snr_db = config.minimum_snr_db;
  pipeline_config.detection_sensitivity_w = config.detection_sensitivity_w;
  pipeline_config.scan_start_az_deg = config.scan_start_az_deg;
  pipeline_config.scan_end_az_deg = config.scan_end_az_deg;
  pipeline_config.scan_center_el_deg = config.scan_center_el_deg;
  pipeline_config.boresight_depression_deg = config.boresight_depression_deg;
  pipeline_config.min_detection_depression_deg = config.min_detection_depression_deg;
  pipeline_config.max_detection_depression_deg = config.max_detection_depression_deg;
  pipeline_config.visible_reference_irradiance_w_m2 = config.visible_reference_irradiance_w_m2;
  pipeline_config.enable_straylight_filter = config.enable_straylight_filter;
  pipeline_config.hood_inner_half_angle_deg = config.hood_inner_half_angle_deg;
  pipeline_config.hood_outer_half_angle_deg = config.hood_outer_half_angle_deg;
  pipeline_config.hood_min_suppression_ratio = config.hood_min_suppression_ratio;
  pipeline_config.hood_max_suppression_ratio = config.hood_max_suppression_ratio;
  pipeline_config.radiative_transfer_model =
      config.environment_default_config.radiative_transfer_model;
  pipeline_config.aerosol_density_factor =
      config.environment_default_config.aerosol_density_factor;
  pipeline_config.turbulence_factor = config.environment_default_config.turbulence_factor;
  pipeline_config.environment_model_type =
      ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType::kSimplified;
  return pipeline_config;
}

TEST(EosCycleOrchestratorTest, StepProducesOutputAndPreservesCycleIndex) {
  const EosSessionConfig config = MakeSessionConfig();
  const ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config =
      BuildPipelineConfig(config);
  core::pipeline::EosPipeline pipeline(pipeline_config);
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline_config, true, pipeline, controller);

  const EosCycleResult result = orchestrator.Step(MakeCycleInput(7U, 1.0f));

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
}

TEST(EosCycleOrchestratorTest, ValidRuntimePatchTakesEffectOnNextStep) {
  const EosSessionConfig config = MakeSessionConfig();
  const ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config =
      BuildPipelineConfig(config);
  core::pipeline::EosPipeline pipeline(pipeline_config);
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline_config, true, pipeline, controller);

  const EosCycleResult baseline = orchestrator.Step(MakeCycleInput(1U, 1.0f));
  const float baseline_scan_azimuth = baseline.output_frame.scan_azimuth_deg;

  orchestrator.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(9.0f).Build());

  const EosCycleResult patched = orchestrator.Step(MakeCycleInput(2U, 1.0f));
  EXPECT_FALSE(patched.has_validation_error);
  EXPECT_TRUE(patched.executed_this_cycle);
  EXPECT_NEAR(patched.output_frame.scan_azimuth_deg, -1.0f, 1.0e-6f);
  EXPECT_NE(baseline_scan_azimuth, patched.output_frame.scan_azimuth_deg);
}

TEST(EosCycleOrchestratorTest, InvalidRuntimePatchDoesNotChangeUpdateBehavior) {
  const EosSessionConfig config = MakeSessionConfig();
  const ::electro_optical_sensor::extension::EosPipelineConfig pipeline_config =
      BuildPipelineConfig(config);
  core::pipeline::EosPipeline pipeline(pipeline_config);
  extension::EosController controller(pipeline);
  EosCycleOrchestrator orchestrator(config, pipeline_config, true, pipeline, controller);

  orchestrator.ApplyRuntimeConfig(
      eos_config::EosRuntimeConfigBuilder().WithFrameRateHz(0.0f).Build());

  const EosCycleResult result = orchestrator.Step(MakeCycleInput(3U, 1.0f));
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_NEAR(result.output_frame.scan_azimuth_deg, -5.0f, 1.0e-6f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
