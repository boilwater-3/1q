/**
 * @file eos_pipeline_test.cpp
 * @brief 验证 EOS core/pipeline 的扫描递推与视场判定策略。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/core/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace core {
namespace pipeline {
namespace {

context::EosTargetState MakeTarget(std::uint64_t id, float azimuth_deg, float range_m = 1800.0f,
                                   float projected_area_m2 = 2.0f) {
  context::EosTargetState target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 330.0f;
  target.emissivity = 0.92f;
  target.reflectance = 0.38f;
  target.projected_area_m2 = projected_area_m2;
  return target;
}

context::EosCycleInput MakeCycleInput(float dt_sec = 1.0f) {
  context::EosCycleInput input;
  input.dt_sec = dt_sec;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = context::DayNightType::kDay;
  input.platform_pose.position_m.z = 1200.0f;
  return input;
}

EosPipelineConfig MakePipelineConfig() {
  EosPipelineConfig config;
  config.work_mode = EosPipelineWorkMode::kFused;
  config.minimum_snr_db = 0.0f;
  config.scan_start_az_deg = -10.0f;
  config.scan_end_az_deg = 10.0f;
  config.scan_rate_deg_per_sec = 5.0f;
  config.horizontal_fov_deg = 6.0f;
  config.vertical_fov_deg = 4.0f;
  return config;
}

TEST(EosPipelineTest, ScanAngleAdvancesAndWrapsInsideRange) {
  EosPipeline pipeline(MakePipelineConfig());
  context::EosCycleInput input = MakeCycleInput(5.0f);  // advance by 25 deg, wrap inside [-10, 10]
  input.cycle_index = 1U;
  input.scene_targets.push_back(MakeTarget(1U, -5.0f));

  const common::EosOutputFrame frame = pipeline.Execute(input);

  EXPECT_GE(frame.scan_azimuth_deg, -10.0f);
  EXPECT_LE(frame.scan_azimuth_deg, 10.0f);
}

TEST(EosPipelineTest, InFovTargetIsDetectedAndOutOfFovTargetIsFiltered) {
  EosPipeline pipeline(MakePipelineConfig());
  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 3U;
  input.scene_targets.push_back(MakeTarget(101U, -5.0f));   // in fov after first advance
  input.scene_targets.push_back(MakeTarget(102U, 35.0f));   // out of fov

  const common::EosOutputFrame frame = pipeline.Execute(input);

  ASSERT_EQ(frame.detections.size(), 1U);
  EXPECT_EQ(frame.detections[0].target_id, 101U);
}

TEST(EosPipelineTest, OutOfRangeTargetIsMarkedUndetected) {
  EosPipeline pipeline(MakePipelineConfig());
  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 4U;
  input.scene_targets.push_back(MakeTarget(201U, -5.0f, 8000.0f));

  const common::EosOutputFrame frame = pipeline.Execute(input);

  ASSERT_EQ(frame.detections.size(), 1U);
  EXPECT_FALSE(frame.detections[0].detected);
}

TEST(EosPipelineTest, LargerTargetAreaHasHigherFusedSnrAtSameGeometry) {
  EosPipelineConfig config = MakePipelineConfig();
  config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  EosPipeline pipeline(config);
  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 5U;
  input.background_temperature_k = 240.0f;
  input.cloud_coverage_ratio = 0.0f;
  context::EosTargetState small_target = MakeTarget(301U, -5.0f, 600.0f, 1.0f);
  small_target.apparent_temperature_k = 900.0f;
  small_target.emissivity = 0.98f;
  context::EosTargetState large_target = MakeTarget(302U, -5.0f, 600.0f, 36.0f);
  large_target.apparent_temperature_k = 900.0f;
  large_target.emissivity = 0.98f;
  input.scene_targets.push_back(small_target);
  input.scene_targets.push_back(large_target);

  const common::EosOutputFrame frame = pipeline.Execute(input);

  ASSERT_EQ(frame.detections.size(), 2U);
  EXPECT_GT(frame.detections[0].fused_snr_linear, 0.0f);
  EXPECT_GT(frame.detections[1].fused_snr_linear, frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, AdaptiveRadiativeTransferModelProducesLowerSnrInSameScene) {
  EosPipelineConfig baseline_config = MakePipelineConfig();
  baseline_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  baseline_config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert;

  EosPipelineConfig adaptive_config = baseline_config;
  adaptive_config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  adaptive_config.aerosol_density_factor = 1.6f;
  adaptive_config.turbulence_factor = 1.5f;

  EosPipeline baseline_pipeline(baseline_config);
  EosPipeline adaptive_pipeline(adaptive_config);

  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 6U;
  input.background_temperature_k = 240.0f;
  input.cloud_coverage_ratio = 0.5f;
  context::EosTargetState target = MakeTarget(401U, -5.0f, 1200.0f, 12.0f);
  target.apparent_temperature_k = 850.0f;
  target.emissivity = 0.98f;
  input.scene_targets.push_back(target);

  const common::EosOutputFrame baseline_frame = baseline_pipeline.Execute(input);
  const common::EosOutputFrame adaptive_frame = adaptive_pipeline.Execute(input);

  ASSERT_EQ(baseline_frame.detections.size(), 1U);
  ASSERT_EQ(adaptive_frame.detections.size(), 1U);
  EXPECT_GT(baseline_frame.detections[0].fused_snr_linear, 0.0f);
  EXPECT_GT(baseline_frame.detections[0].fused_snr_linear,
            adaptive_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, AdvancedEnvironmentModelLowersSnrInHighWindScene) {
  EosPipelineConfig simplified_config = MakePipelineConfig();
  simplified_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  simplified_config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  simplified_config.environment_model_type = EosPipelineEnvironmentModelType::kSimplified;
  simplified_config.aerosol_density_factor = 1.2f;
  simplified_config.turbulence_factor = 1.1f;

  EosPipelineConfig advanced_config = simplified_config;
  advanced_config.environment_model_type = EosPipelineEnvironmentModelType::kAdvanced;

  EosPipeline simplified_pipeline(simplified_config);
  EosPipeline advanced_pipeline(advanced_config);

  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 7U;
  input.background_temperature_k = 240.0f;
  input.cloud_coverage_ratio = 0.6f;
  input.ambient_wind_speed_mps = 120.0f;
  context::EosTargetState target = MakeTarget(501U, -5.0f, 1200.0f, 10.0f);
  target.apparent_temperature_k = 880.0f;
  target.emissivity = 0.98f;
  input.scene_targets.push_back(target);

  const common::EosOutputFrame simplified_frame = simplified_pipeline.Execute(input);
  const common::EosOutputFrame advanced_frame = advanced_pipeline.Execute(input);

  ASSERT_EQ(simplified_frame.detections.size(), 1U);
  ASSERT_EQ(advanced_frame.detections.size(), 1U);
  EXPECT_GT(simplified_frame.detections[0].fused_snr_linear,
            advanced_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, PlatformVelocityDoesNotAffectEnvironmentPenaltyWhenWindFixed) {
  EosPipelineConfig config = MakePipelineConfig();
  config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  config.environment_model_type = EosPipelineEnvironmentModelType::kAdvanced;
  config.aerosol_density_factor = 1.2f;
  config.turbulence_factor = 1.1f;

  EosPipeline low_speed_pipeline(config);
  EosPipeline high_speed_pipeline(config);

  context::EosCycleInput low_speed_input = MakeCycleInput(1.0f);
  low_speed_input.cycle_index = 8U;
  low_speed_input.background_temperature_k = 240.0f;
  low_speed_input.cloud_coverage_ratio = 0.6f;
  low_speed_input.ambient_wind_speed_mps = 35.0f;
  low_speed_input.platform_pose.velocity_mps.x = 10.0f;
  context::EosTargetState low_speed_target = MakeTarget(601U, -5.0f, 1200.0f, 10.0f);
  low_speed_target.apparent_temperature_k = 880.0f;
  low_speed_target.emissivity = 0.98f;
  low_speed_input.scene_targets.push_back(low_speed_target);

  context::EosCycleInput high_speed_input = low_speed_input;
  high_speed_input.cycle_index = 9U;
  high_speed_input.platform_pose.velocity_mps.x = 250.0f;
  high_speed_input.platform_pose.velocity_mps.y = -90.0f;
  high_speed_input.platform_pose.velocity_mps.z = 20.0f;

  const common::EosOutputFrame low_speed_frame = low_speed_pipeline.Execute(low_speed_input);
  const common::EosOutputFrame high_speed_frame = high_speed_pipeline.Execute(high_speed_input);

  ASSERT_EQ(low_speed_frame.detections.size(), 1U);
  ASSERT_EQ(high_speed_frame.detections.size(), 1U);
  EXPECT_NEAR(low_speed_frame.detections[0].fused_snr_linear,
              high_speed_frame.detections[0].fused_snr_linear, 1.0e-8f);
}

}  // namespace
}  // namespace pipeline
}  // namespace core
}  // namespace electro_optical_sensor
