/**
 * @file eos_pipeline_unit_test.cpp
 * @brief 验证 EOS core/pipeline 的扫描递推与视场判定策略。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {
namespace {

namespace context = ::electro_optical_sensor::session;
namespace output = ::electro_optical_sensor::output;

session::EosSceneTarget MakeTarget(std::uint64_t id, float azimuth_deg, float range_m = 1800.0f,
                                   float projected_area_m2 = 2.0f) {
  session::EosSceneTarget target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 330.0f;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.38f;
  target.appearance.projected_area_m2 = projected_area_m2;
  return target;
}

::electro_optical_sensor::session::EosCycleInput MakeCycleInput(float dt_sec = 1.0f) {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = dt_sec;
  input.environment.solar_irradiance_w_m2 = 850.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.cloud_coverage_ratio = 0.2f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  input.platform_altitude_m = 1200.0f;
  input.platform_pose.position_m.z = 0.0f;
  return input;
}

extension::EosPipelineConfig MakePipelineConfig() {
  extension::EosPipelineConfig config;
  config.work_mode = EosPipelineWorkMode::kFused;
  config.minimum_snr_db = 0.0f;
  config.scan_start_az_deg = -10.0f;
  config.scan_end_az_deg = 10.0f;
  config.scan_rate_deg_per_sec = 5.0f;
  config.horizontal_fov_deg = 6.0f;
  config.vertical_fov_deg = 4.0f;
  return config;
}

float ResolveFirstCycleScanAzimuthDeg(const extension::EosPipelineConfig& config, float dt_sec) {
  const float scan_width_deg = config.scan_end_az_deg - config.scan_start_az_deg;
  if (scan_width_deg <= 0.0f) {
    return config.scan_start_az_deg;
  }

  float wrapped_offset_deg = config.scan_rate_deg_per_sec * dt_sec;
  while (wrapped_offset_deg >= scan_width_deg) {
    wrapped_offset_deg -= scan_width_deg;
  }
  while (wrapped_offset_deg < 0.0f) {
    wrapped_offset_deg += scan_width_deg;
  }
  return config.scan_start_az_deg + wrapped_offset_deg;
}

TEST(EosPipelineTest, ScanAngleAdvancesAndWrapsInsideRange) {
  EosPipeline pipeline(MakePipelineConfig());
  ::electro_optical_sensor::session::EosCycleInput input =
      MakeCycleInput(5.0f);  // advance by 25 deg, wrap inside [-10, 10]
  input.cycle_index = 1U;
  input.scene.push_back(MakeTarget(1U, -5.0f));

  const auto frame = pipeline.RunCycle(input);

  EXPECT_GE(frame.scan_azimuth_deg, -10.0f);
  EXPECT_LE(frame.scan_azimuth_deg, 10.0f);
}

TEST(EosPipelineTest, InFovTargetIsDetectedAndOutOfFovTargetIsFiltered) {
  EosPipeline pipeline(MakePipelineConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 3U;
  input.scene.push_back(MakeTarget(101U, -5.0f));  // in fov after first advance
  input.scene.push_back(MakeTarget(102U, 35.0f));  // out of fov

  const auto frame = pipeline.RunCycle(input);

  ASSERT_EQ(frame.detections.size(), 1U);
  EXPECT_EQ(frame.detections[0].target_id, 101U);
}

TEST(EosPipelineTest, OutOfRangeTargetIsMarkedUndetected) {
  EosPipeline pipeline(MakePipelineConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 4U;
  input.scene.push_back(MakeTarget(201U, -5.0f, 8000.0f));

  const auto frame = pipeline.RunCycle(input);

  ASSERT_EQ(frame.detections.size(), 1U);
  EXPECT_FALSE(frame.detections[0].detected);
}

TEST(EosPipelineTest, LargerTargetAreaHasHigherFusedSnrAtSameGeometry) {
  extension::EosPipelineConfig config = MakePipelineConfig();
  config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  EosPipeline pipeline(config);
  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 5U;
  input.environment.background_temperature_k = 240.0f;
  input.environment.cloud_coverage_ratio = 0.0f;
  session::EosSceneTarget small_target = MakeTarget(301U, -5.0f, 600.0f, 1.0f);
  small_target.appearance.apparent_temperature_k = 900.0f;
  small_target.appearance.emissivity = 0.98f;
  session::EosSceneTarget large_target = MakeTarget(302U, -5.0f, 600.0f, 36.0f);
  large_target.appearance.apparent_temperature_k = 900.0f;
  large_target.appearance.emissivity = 0.98f;
  input.scene.push_back(small_target);
  input.scene.push_back(large_target);

  const auto frame = pipeline.RunCycle(input);

  ASSERT_EQ(frame.detections.size(), 2U);
  EXPECT_GT(frame.detections[0].fused_snr_linear, 0.0f);
  EXPECT_GT(frame.detections[1].fused_snr_linear, frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, VisibleChainAppliesProjectedAreaOnce) {
  extension::EosPipelineConfig config = MakePipelineConfig();
  config.work_mode = EosPipelineWorkMode::kVisibleOnly;
  config.minimum_snr_db = -120.0f;
  EosPipeline pipeline(config);
  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 6U;
  input.environment.cloud_coverage_ratio = 0.0f;
  const float scan_azimuth = ResolveFirstCycleScanAzimuthDeg(config, input.dt_sec);
  input.scene.push_back(MakeTarget(401U, scan_azimuth, 800.0f, 4.0f));
  input.scene.push_back(MakeTarget(402U, scan_azimuth, 800.0f, 8.0f));

  const auto frame = pipeline.RunCycle(input);
  ASSERT_EQ(frame.detections.size(), 2U);
  const float area_gain = frame.detections[1].fused_snr_linear /
                          std::max(frame.detections[0].fused_snr_linear, 1.0e-12f);
  EXPECT_GT(area_gain, 1.2f);
  EXPECT_LT(area_gain, 3.0f);
}

TEST(EosPipelineTest, AdaptiveRadiativeTransferModelProducesLowerSnrInSameScene) {
  extension::EosPipelineConfig baseline_config = MakePipelineConfig();
  baseline_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  baseline_config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert;

  extension::EosPipelineConfig adaptive_config = baseline_config;
  adaptive_config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  adaptive_config.aerosol_density_factor = 1.6f;
  adaptive_config.turbulence_factor = 1.5f;

  EosPipeline baseline_pipeline(baseline_config);
  EosPipeline adaptive_pipeline(adaptive_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 6U;
  input.environment.background_temperature_k = 240.0f;
  input.environment.cloud_coverage_ratio = 0.5f;
  session::EosSceneTarget target = MakeTarget(401U, -5.0f, 1200.0f, 12.0f);
  target.appearance.apparent_temperature_k = 850.0f;
  target.appearance.emissivity = 0.98f;
  input.scene.push_back(target);

  const auto baseline_frame = baseline_pipeline.RunCycle(input);
  const auto adaptive_frame = adaptive_pipeline.RunCycle(input);

  ASSERT_EQ(baseline_frame.detections.size(), 1U);
  ASSERT_EQ(adaptive_frame.detections.size(), 1U);
  EXPECT_GT(baseline_frame.detections[0].fused_snr_linear, 0.0f);
  EXPECT_GT(baseline_frame.detections[0].fused_snr_linear,
            adaptive_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, AdvancedEnvironmentModelLowersSnrInHighWindScene) {
  extension::EosPipelineConfig simplified_config = MakePipelineConfig();
  simplified_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  simplified_config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  simplified_config.environment_model_type = EosPipelineEnvironmentModelType::kSimplified;
  simplified_config.aerosol_density_factor = 1.2f;
  simplified_config.turbulence_factor = 1.1f;

  extension::EosPipelineConfig advanced_config = simplified_config;
  advanced_config.environment_model_type = EosPipelineEnvironmentModelType::kAdvanced;

  EosPipeline simplified_pipeline(simplified_config);
  EosPipeline advanced_pipeline(advanced_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 7U;
  input.environment.background_temperature_k = 240.0f;
  input.environment.cloud_coverage_ratio = 0.6f;
  input.environment.ambient_wind_speed_mps = 120.0f;
  session::EosSceneTarget target = MakeTarget(501U, -5.0f, 1200.0f, 10.0f);
  target.appearance.apparent_temperature_k = 880.0f;
  target.appearance.emissivity = 0.98f;
  input.scene.push_back(target);

  const auto simplified_frame = simplified_pipeline.RunCycle(input);
  const auto advanced_frame = advanced_pipeline.RunCycle(input);

  ASSERT_EQ(simplified_frame.detections.size(), 1U);
  ASSERT_EQ(advanced_frame.detections.size(), 1U);
  EXPECT_GT(simplified_frame.detections[0].fused_snr_linear,
            advanced_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, PlatformVelocityDoesNotAffectEnvironmentPenaltyWhenWindFixed) {
  extension::EosPipelineConfig config = MakePipelineConfig();
  config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  config.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  config.environment_model_type = EosPipelineEnvironmentModelType::kAdvanced;
  config.aerosol_density_factor = 1.2f;
  config.turbulence_factor = 1.1f;

  EosPipeline low_speed_pipeline(config);
  EosPipeline high_speed_pipeline(config);

  ::electro_optical_sensor::session::EosCycleInput low_speed_input = MakeCycleInput(1.0f);
  low_speed_input.cycle_index = 8U;
  low_speed_input.environment.background_temperature_k = 240.0f;
  low_speed_input.environment.cloud_coverage_ratio = 0.6f;
  low_speed_input.environment.ambient_wind_speed_mps = 35.0f;
  low_speed_input.platform_pose.velocity_mps.x = 10.0f;
  session::EosSceneTarget low_speed_target = MakeTarget(601U, -5.0f, 1200.0f, 10.0f);
  low_speed_target.appearance.apparent_temperature_k = 880.0f;
  low_speed_target.appearance.emissivity = 0.98f;
  low_speed_input.scene.push_back(low_speed_target);

  ::electro_optical_sensor::session::EosCycleInput high_speed_input = low_speed_input;
  high_speed_input.cycle_index = 9U;
  high_speed_input.platform_pose.velocity_mps.x = 250.0f;
  high_speed_input.platform_pose.velocity_mps.y = -90.0f;
  high_speed_input.platform_pose.velocity_mps.z = 20.0f;

  const auto low_speed_frame = low_speed_pipeline.RunCycle(low_speed_input);
  const auto high_speed_frame = high_speed_pipeline.RunCycle(high_speed_input);

  ASSERT_EQ(low_speed_frame.detections.size(), 1U);
  ASSERT_EQ(high_speed_frame.detections.size(), 1U);
  EXPECT_NEAR(low_speed_frame.detections[0].fused_snr_linear,
              high_speed_frame.detections[0].fused_snr_linear, 1.0e-8f);
}

TEST(EosPipelineTest, LowerFrameRateProducesHigherSnrWithLongerIntegrationWindow) {
  extension::EosPipelineConfig low_rate_config = MakePipelineConfig();
  low_rate_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  low_rate_config.frame_rate_hz = 5.0f;
  low_rate_config.scan_rate_deg_per_sec = 5.0f;

  extension::EosPipelineConfig high_rate_config = low_rate_config;
  high_rate_config.frame_rate_hz = 120.0f;

  EosPipeline low_rate_pipeline(low_rate_config);
  EosPipeline high_rate_pipeline(high_rate_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(0.1f);
  input.cycle_index = 10U;
  input.environment.background_temperature_k = 240.0f;
  input.environment.cloud_coverage_ratio = 0.1f;
  session::EosSceneTarget target = MakeTarget(
      701U, ResolveFirstCycleScanAzimuthDeg(low_rate_config, input.dt_sec), 1000.0f, 8.0f);
  target.appearance.apparent_temperature_k = 860.0f;
  target.appearance.emissivity = 0.98f;
  input.scene.push_back(target);

  const auto low_rate_frame = low_rate_pipeline.RunCycle(input);
  const auto high_rate_frame = high_rate_pipeline.RunCycle(input);

  ASSERT_EQ(low_rate_frame.detections.size(), 1U);
  ASSERT_EQ(high_rate_frame.detections.size(), 1U);
  EXPECT_GT(low_rate_frame.detections[0].fused_snr_linear,
            high_rate_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, VisibleReferenceIrradianceAffectsVisibleSnrThroughNoiseModel) {
  extension::EosPipelineConfig matched_reference_config = MakePipelineConfig();
  matched_reference_config.work_mode = EosPipelineWorkMode::kVisibleOnly;
  matched_reference_config.visible_reference_irradiance_w_m2 = 400.0f;

  extension::EosPipelineConfig mismatched_reference_config = matched_reference_config;
  mismatched_reference_config.visible_reference_irradiance_w_m2 = 2000.0f;

  EosPipeline matched_reference_pipeline(matched_reference_config);
  EosPipeline mismatched_reference_pipeline(mismatched_reference_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 11U;
  input.environment.solar_irradiance_w_m2 = 400.0f;
  input.environment.cloud_coverage_ratio = 0.6f;
  session::EosSceneTarget target = MakeTarget(801U, -5.0f, 1200.0f, 4.0f);
  input.scene.push_back(target);

  const auto matched_frame = matched_reference_pipeline.RunCycle(input);
  const auto mismatched_frame = mismatched_reference_pipeline.RunCycle(input);

  ASSERT_EQ(matched_frame.detections.size(), 1U);
  ASSERT_EQ(mismatched_frame.detections.size(), 1U);
  EXPECT_GT(matched_frame.detections[0].fused_snr_linear,
            mismatched_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, BetterDetectionSensitivityProducesHigherSnr) {
  extension::EosPipelineConfig better_sensitivity_config = MakePipelineConfig();
  better_sensitivity_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  better_sensitivity_config.detection_sensitivity_w = 5.0e-13f;

  extension::EosPipelineConfig worse_sensitivity_config = better_sensitivity_config;
  worse_sensitivity_config.detection_sensitivity_w = 5.0e-12f;

  EosPipeline better_sensitivity_pipeline(better_sensitivity_config);
  EosPipeline worse_sensitivity_pipeline(worse_sensitivity_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(0.5f);
  input.cycle_index = 12U;
  input.environment.background_temperature_k = 240.0f;
  input.environment.cloud_coverage_ratio = 0.1f;
  session::EosSceneTarget target = MakeTarget(
      901U, ResolveFirstCycleScanAzimuthDeg(better_sensitivity_config, input.dt_sec), 900.0f, 9.0f);
  target.appearance.apparent_temperature_k = 860.0f;
  target.appearance.emissivity = 0.98f;
  input.scene.push_back(target);

  const auto better_frame = better_sensitivity_pipeline.RunCycle(input);
  const auto worse_frame = worse_sensitivity_pipeline.RunCycle(input);

  ASSERT_EQ(better_frame.detections.size(), 1U);
  ASSERT_EQ(worse_frame.detections.size(), 1U);
  EXPECT_GT(better_frame.detections[0].fused_snr_linear,
            worse_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, InfraredBandwidthIncreaseRaisesSnrAtFixedCenterWavelength) {
  extension::EosPipelineConfig narrow_band_config = MakePipelineConfig();
  narrow_band_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  narrow_band_config.wavelength_lower_um = 3.5f;
  narrow_band_config.wavelength_upper_um = 4.5f;
  narrow_band_config.minimum_snr_db = -120.0f;

  extension::EosPipelineConfig wide_band_config = narrow_band_config;
  wide_band_config.wavelength_lower_um = 2.0f;
  wide_band_config.wavelength_upper_um = 6.0f;

  EosPipeline narrow_band_pipeline(narrow_band_config);
  EosPipeline wide_band_pipeline(wide_band_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 13U;
  input.environment.background_temperature_k = 240.0f;
  input.environment.cloud_coverage_ratio = 0.1f;
  session::EosSceneTarget target = MakeTarget(
      1001U, ResolveFirstCycleScanAzimuthDeg(narrow_band_config, input.dt_sec), 900.0f, 8.0f);
  target.appearance.apparent_temperature_k = 860.0f;
  target.appearance.emissivity = 0.98f;
  input.scene.push_back(target);

  const auto narrow_band_frame = narrow_band_pipeline.RunCycle(input);
  const auto wide_band_frame = wide_band_pipeline.RunCycle(input);
  ASSERT_EQ(narrow_band_frame.detections.size(), 1U);
  ASSERT_EQ(wide_band_frame.detections.size(), 1U);
  EXPECT_GT(wide_band_frame.detections[0].fused_snr_linear,
            narrow_band_frame.detections[0].fused_snr_linear);
}

TEST(EosPipelineTest, FusedWeightShiftsTowardVisibleInDayAndInfraredAtNight) {
  extension::EosPipelineConfig fused_config = MakePipelineConfig();
  fused_config.work_mode = EosPipelineWorkMode::kFused;
  fused_config.minimum_snr_db = -120.0f;
  extension::EosPipelineConfig infrared_config = fused_config;
  infrared_config.work_mode = EosPipelineWorkMode::kInfraredOnly;
  extension::EosPipelineConfig visible_config = fused_config;
  visible_config.work_mode = EosPipelineWorkMode::kVisibleOnly;

  EosPipeline fused_pipeline(fused_config);
  EosPipeline infrared_pipeline(infrared_config);
  EosPipeline visible_pipeline(visible_config);

  ::electro_optical_sensor::session::EosCycleInput day_input = MakeCycleInput(1.0f);
  day_input.cycle_index = 12U;
  day_input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  day_input.environment.cloud_coverage_ratio = 0.0f;
  day_input.environment.background_temperature_k = 240.0f;
  const float day_scan_azimuth = ResolveFirstCycleScanAzimuthDeg(fused_config, day_input.dt_sec);
  day_input.scene.push_back(MakeTarget(1101U, day_scan_azimuth, 800.0f, 10.0f));

  const auto day_fused_frame = fused_pipeline.RunCycle(day_input);
  const auto day_infrared_frame = infrared_pipeline.RunCycle(day_input);
  const auto day_visible_frame = visible_pipeline.RunCycle(day_input);
  ASSERT_EQ(day_fused_frame.detections.size(), 1U);
  ASSERT_EQ(day_infrared_frame.detections.size(), 1U);
  ASSERT_EQ(day_visible_frame.detections.size(), 1U);
  const float day_distance_to_infrared =
      std::fabs(day_fused_frame.detections[0].fused_snr_linear -
                day_infrared_frame.detections[0].fused_snr_linear);
  const float day_distance_to_visible = std::fabs(day_fused_frame.detections[0].fused_snr_linear -
                                                  day_visible_frame.detections[0].fused_snr_linear);
  EXPECT_LT(day_distance_to_visible, day_distance_to_infrared);

  EosPipeline night_fused_pipeline(fused_config);
  EosPipeline night_infrared_pipeline(infrared_config);
  EosPipeline night_visible_pipeline(visible_config);

  ::electro_optical_sensor::session::EosCycleInput night_input = day_input;
  night_input.cycle_index = 13U;
  night_input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kNight;
  const auto night_fused_frame = night_fused_pipeline.RunCycle(night_input);
  const auto night_infrared_frame = night_infrared_pipeline.RunCycle(night_input);
  const auto night_visible_frame = night_visible_pipeline.RunCycle(night_input);
  ASSERT_EQ(night_fused_frame.detections.size(), 1U);
  ASSERT_EQ(night_infrared_frame.detections.size(), 1U);
  ASSERT_EQ(night_visible_frame.detections.size(), 1U);
  const float night_distance_to_infrared =
      std::fabs(night_fused_frame.detections[0].fused_snr_linear -
                night_infrared_frame.detections[0].fused_snr_linear);
  const float night_distance_to_visible =
      std::fabs(night_fused_frame.detections[0].fused_snr_linear -
                night_visible_frame.detections[0].fused_snr_linear);
  EXPECT_LT(night_distance_to_infrared, night_distance_to_visible);
}

}  // namespace
}  // namespace pipeline
}  // namespace signal
}  // namespace electro_optical_sensor
