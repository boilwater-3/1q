// Copyright 2026. All Rights Reserved.
//
// @file eos_public_api_convenience_test.cpp
// @brief 验证 EOS 对外易用性增强 API 的默认语义与集成行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigValidation.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace tests {

namespace {

bool ContainsEosIssueCode(const std::vector<session::ValidationIssue>& issues,
                          session::ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

session::EosSceneTarget MakeTarget(std::uint64_t id, float range_m, float az_deg, float el_deg,
                                   float temp_k, float emissivity, float reflectance,
                                   float area_m2) {
  session::EosSceneTarget target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = az_deg;
  target.elevation_deg = el_deg;
  target.appearance.apparent_temperature_k = temp_k;
  target.appearance.emissivity = emissivity;
  target.appearance.reflectance = reflectance;
  target.appearance.projected_area_m2 = area_m2;
  return target;
}

}  // namespace

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderDefaultsMatchEosSessionConfig) {
  const config::EosSessionConfig built = config::EosSessionConfigBuilder().Build();
  const config::EosSessionConfig default_config;

  EXPECT_EQ(built.mission.work_mode, default_config.mission.work_mode);
  EXPECT_NEAR(built.hardware.wavelength_lower_um, default_config.hardware.wavelength_lower_um,
              1e-5f);
  EXPECT_NEAR(built.hardware.wavelength_upper_um, default_config.hardware.wavelength_upper_um,
              1e-5f);
  EXPECT_FLOAT_EQ(built.policy.detection.minimum_snr_db,
                  default_config.policy.detection.minimum_snr_db);
}

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderOverridesSemanticFields) {
  config::EosSessionConfig config =
      config::EosSessionConfigBuilder()
          .Mission()
          .WithMissionProfile(config::EosMissionProfile::kLongRangeSurveillance)
          .End()
          .Environment()
          .WithEnvironmentModelType(config::EosEnvironmentModelType::kAdvanced)
          .WithEnvironmentPreset(config::EosEnvironmentPreset::kDusty)
          .End()
          .Build();
  config.policy.detection.minimum_snr_db = 60.0f;
  config.policy.detection.detection_sensitivity_w = 2.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 1000.0f;
  config.policy.stray_light.enable_straylight_filter = true;
  config.policy.stray_light.hood_inner_half_angle_deg = 8.0f;
  config.policy.stray_light.hood_outer_half_angle_deg = 55.0f;
  config.policy.stray_light.hood_min_suppression_ratio = 0.35f;
  config.policy.stray_light.hood_max_suppression_ratio = 0.95f;

  EXPECT_EQ(config.mission.work_mode, config::EosWorkMode::kInfraredOnly);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 60.0f);
  EXPECT_TRUE(config.policy.stray_light.enable_straylight_filter);
  EXPECT_EQ(config.environment.scenario_config.model_type,
            config::EosEnvironmentModelType::kAdvanced);
  EXPECT_EQ(config.environment.scenario_config.preset, config::EosEnvironmentPreset::kDusty);
}

TEST(EosPublicApiConvenienceTest, DetailedSessionConfigBuilderOverridesDomainAndLeafFields) {
  config::EosSessionConfig config{};
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.mission.scan_rate_deg_per_sec = 40.0f;
  config.mission.frame_rate_hz = 60.0f;
  config.policy.detection.minimum_snr_db = 60.0f;
  config.policy.detection.detection_sensitivity_w = 2.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 1000.0f;
  config.policy.stray_light.enable_straylight_filter = true;
  config.policy.stray_light.hood_inner_half_angle_deg = 8.0f;
  config.policy.stray_light.hood_outer_half_angle_deg = 55.0f;
  config.policy.stray_light.hood_min_suppression_ratio = 0.35f;
  config.policy.stray_light.hood_max_suppression_ratio = 0.95f;
  config.environment.scenario_config.model_type = config::EosEnvironmentModelType::kAdvanced;
  config.environment.scenario_config.preset = config::EosEnvironmentPreset::kDusty;

  EXPECT_EQ(config.mission.work_mode, config::EosWorkMode::kInfraredOnly);
  EXPECT_NEAR(config.mission.scan_rate_deg_per_sec, 40.0f, 1e-5f);
  EXPECT_NEAR(config.mission.frame_rate_hz, 60.0f, 1e-5f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 60.0f);
  EXPECT_TRUE(config.policy.stray_light.enable_straylight_filter);
  EXPECT_EQ(config.environment.scenario_config.model_type,
            config::EosEnvironmentModelType::kAdvanced);
  EXPECT_EQ(config.environment.scenario_config.preset, config::EosEnvironmentPreset::kDusty);
}

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderPreservesPreconfiguredSessionConfig) {
  config::EosSessionConfig base;
  base.hardware.wavelength_lower_um = 8.0f;
  base.hardware.wavelength_upper_um = 12.0f;
  base.policy.detection.minimum_snr_db = 4.5f;
  base.policy.detection.detection_sensitivity_w = 0.8e-12f;
  base.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

  const config::EosSessionConfig config = config::EosSessionConfigBuilder(base).Build();

  EXPECT_NEAR(config.hardware.wavelength_lower_um, 8.0f, 1e-5f);
  EXPECT_NEAR(config.hardware.wavelength_upper_um, 12.0f, 1e-5f);
  EXPECT_FLOAT_EQ(config.policy.detection.minimum_snr_db, 4.5f);
}

TEST(EosPublicApiConvenienceTest, SessionConfigValidatorReportsFinalConfigIssues) {
  config::EosSessionConfig invalid_config;
  invalid_config.mission.horizontal_fov_deg = 0.0f;
  invalid_config.mission.vertical_fov_deg = -1.0f;
  invalid_config.mission.scan_rate_deg_per_sec = 0.0f;
  invalid_config.mission.frame_rate_hz = -5.0f;
  invalid_config.mission.scan_start_az_deg = 10.0f;
  invalid_config.mission.scan_end_az_deg = 10.0f;

  const config::ValidationIssueList issues = config::ValidateEosSessionConfig(invalid_config);

  ASSERT_EQ(issues.size(), 5U);
  EXPECT_EQ(issues[0].code, config::ConfigValidationCode::kHorizontalFovNotPositive);
  EXPECT_EQ(issues[1].code, config::ConfigValidationCode::kVerticalFovNotPositive);
  EXPECT_EQ(issues[2].code, config::ConfigValidationCode::kScanRateNotPositive);
  EXPECT_EQ(issues[3].code, config::ConfigValidationCode::kFrameRateNotPositive);
  EXPECT_EQ(issues[4].code, config::ConfigValidationCode::kScanRangeAzSwapped);
}

TEST(EosPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const config::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_deg_per_sec);
  EXPECT_FALSE(patch.has_frame_rate_hz);
}

TEST(EosPublicApiConvenienceTest, RuntimeConfigBuilderSetsAllFields) {
  config::EosEnvironmentScenarioConfig env_config;
  env_config.model_type = config::EosEnvironmentModelType::kAdvanced;
  env_config.has_custom_overrides = true;
  env_config.custom_overrides.radiative_transfer_model =
      config::RadiativeTransferModel::kAdaptivePathRadiance;
  env_config.custom_overrides.aerosol_density_factor = 1.3f;
  env_config.custom_overrides.turbulence_factor = 1.8f;

  const config::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder()
                                                  .WithWorkMode(config::EosWorkMode::kVisibleOnly)
                                                  .WithScanRateDegPerSec(50.0f)
                                                  .WithFrameRateHz(120.0f)
                                                  .WithMinimumSnrDb(60.0f)
                                                  .WithDetectionSensitivityW(2.0e-12f)
                                                  .WithVisibleReferenceIrradianceWM2(1000.0f)
                                                  .WithEnableStraylightFilter(true)
                                                  .WithHoodInnerHalfAngleDeg(8.0f)
                                                  .WithHoodOuterHalfAngleDeg(55.0f)
                                                  .WithHoodMinSuppressionRatio(0.35f)
                                                  .WithHoodMaxSuppressionRatio(0.95f)
                                                  .WithEnvironmentScenarioConfig(env_config)
                                                  .Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, config::EosWorkMode::kVisibleOnly);
  EXPECT_TRUE(patch.has_scan_rate_deg_per_sec);
  EXPECT_NEAR(patch.scan_rate_deg_per_sec, 50.0f, 1e-5f);
  EXPECT_TRUE(patch.has_frame_rate_hz);
  EXPECT_NEAR(patch.frame_rate_hz, 120.0f, 1e-5f);
  EXPECT_TRUE(patch.has_policy);
  EXPECT_FLOAT_EQ(patch.policy.detection.minimum_snr_db, 60.0f);
  EXPECT_TRUE(patch.policy.stray_light.enable_straylight_filter);
  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_scenario_config);
  EXPECT_EQ(patch.environment.scenario_config.model_type,
            config::EosEnvironmentModelType::kAdvanced);
  EXPECT_TRUE(patch.environment.scenario_config.has_custom_overrides);
  EXPECT_EQ(patch.environment.scenario_config.custom_overrides.radiative_transfer_model,
            config::RadiativeTransferModel::kAdaptivePathRadiance);
  EXPECT_NEAR(patch.environment.scenario_config.custom_overrides.aerosol_density_factor, 1.3f,
              1e-5f);
  EXPECT_NEAR(patch.environment.scenario_config.custom_overrides.turbulence_factor, 1.8f, 1e-5f);
}

TEST(EosPublicApiConvenienceTest, InputValidationReportsErrorsForCommonBoundaryCases) {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = 0.0f;
  input.environment.solar_irradiance_w_m2 = -1.0f;
  input.environment.cloud_coverage_ratio = -0.1f;
  input.environment.ambient_wind_speed_mps = -5.0f;
  input.environment.background_temperature_k = 0.0f;
  input.environment.solar_altitude_deg = 100.0f;

  session::EosSceneTarget invalid_target;
  invalid_target.target_id = 0U;
  invalid_target.range_m = -1.0f;
  invalid_target.appearance.apparent_temperature_k = 0.0f;
  invalid_target.appearance.emissivity = 1.5f;
  invalid_target.appearance.reflectance = 1.5f;
  invalid_target.appearance.projected_area_m2 = -1.0f;
  input.scene.push_back(invalid_target);

  const session::ValidationIssueList issues = session::ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidSolarIrradiance));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidCloudCoverageRatio));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidAmbientWindSpeed));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidBackgroundTemperature));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidTargetId));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidTargetRange));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidTargetTemperature));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidTargetEmissivity));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidTargetReflectance));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidTargetProjectedArea));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kInvalidSolarAltitudeRange));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationFlagsNonFiniteDtAndTargetFields) {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  session::EosSceneTarget nan_target;
  nan_target.target_id = 100U;
  nan_target.range_m = std::numeric_limits<float>::infinity();
  input.scene.push_back(nan_target);

  const session::ValidationIssueList issues = session::ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsEosIssueCode(issues, session::ValidationCode::kNonFiniteTargetNumericField));
  EXPECT_TRUE(session::HasValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationFlagsEnergyBalanceInconsistency) {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = 1.0f;

  session::EosSceneTarget unbalanced;
  unbalanced.target_id = 200U;
  unbalanced.range_m = 1000.0f;
  unbalanced.appearance.apparent_temperature_k = 300.0f;
  unbalanced.appearance.emissivity = 0.8f;
  unbalanced.appearance.reflectance = 0.3f;
  unbalanced.appearance.projected_area_m2 = 2.0f;
  input.scene.push_back(unbalanced);

  const session::ValidationIssueList issues = session::ValidateEosCycleInput(input);

  EXPECT_TRUE(
      ContainsEosIssueCode(issues, session::ValidationCode::kInconsistentTargetEnergyBalance));
  EXPECT_FALSE(session::HasValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationPassesForValidInput) {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.scene.push_back(MakeTarget(300U, 1500.0f, 5.0f, -2.0f, 320.0f, 0.9f, 0.1f, 2.0f));

  const session::ValidationIssueList issues = session::ValidateEosCycleInput(input);

  EXPECT_FALSE(session::HasValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, CoordinateUtilsBuildsPoseFromExternalKinematics) {
  oneq::coordinate::LocalFrameReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;
  reference.frame_attitude_deg.yaw_deg = 0.0f;
  reference.frame_attitude_deg.pitch_deg = 0.0f;
  reference.frame_attitude_deg.roll_deg = 0.0f;

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));

  oneq::coordinate::EulerAnglesDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 5.0f;

  session::EosExternalPoseInput input;
  input.platform_position_ecef_m = target_ecef;
  input.platform_velocity_mps.x_mps = 0.0f;   // ECEF X → ENU Up at lat=0,lon=0
  input.platform_velocity_mps.y_mps = 10.0f;  // ECEF Y → ENU East at lat=0,lon=0
  input.platform_velocity_mps.z_mps = 0.0f;   // ECEF Z → ENU North at lat=0,lon=0
  input.platform_attitude_deg = platform_attitude_deg;

  oneq::foundation::PoseState pose;
  ASSERT_TRUE(session::TryMakeEosPoseFromExternalKinematics(input, reference, &pose));
  EXPECT_GT(pose.position_m.x, 100.0f);
  EXPECT_NEAR(pose.position_m.y, 0.0f, 1.0e-2f);
  EXPECT_NEAR(pose.position_m.z, 0.0f, 1.0e-2f);
  EXPECT_NEAR(pose.attitude_deg.yaw_deg, 5.0f, 1.0e-5f);
}

TEST(EosPublicApiConvenienceTest, CoordinateUtilsBuildsTargetFromEcefAndLla) {
  oneq::coordinate::LocalFrameReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::foundation::PoseState platform_pose;
  platform_pose.position_m.x = 0.0f;
  platform_pose.position_m.y = 0.0f;
  platform_pose.position_m.z = 1000.0f;

  session::EosTargetAppearance appearance;
  appearance.apparent_temperature_k = 320.0f;
  appearance.emissivity = 0.9f;
  appearance.reflectance = 0.1f;
  appearance.projected_area_m2 = 2.0f;

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  session::EosExternalTargetInput lla_input;
  lla_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  lla_input.kinematics.position_lla_deg_m = target_lla;
  lla_input.appearance = appearance;
  session::EosSceneTarget target_from_lla;
  ASSERT_TRUE(session::TryMakeEosSceneTargetFromExternalInput(401U, lla_input, reference,
                                                              platform_pose, &target_from_lla));
  EXPECT_EQ(target_from_lla.target_id, 401U);
  EXPECT_GT(target_from_lla.range_m, 0.0f);
  EXPECT_NEAR(target_from_lla.appearance.apparent_temperature_k, 320.0f, 1e-5f);

  oneq::coordinate::EcefPositionM target_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(target_lla, &target_ecef));
  session::EosExternalTargetInput ecef_input;
  ecef_input.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  ecef_input.kinematics.position_ecef_m = target_ecef;
  ecef_input.appearance = appearance;
  session::EosSceneTarget target_from_ecef;
  ASSERT_TRUE(session::TryMakeEosSceneTargetFromExternalInput(402U, ecef_input, reference,
                                                              platform_pose, &target_from_ecef));
  EXPECT_NEAR(target_from_ecef.range_m, target_from_lla.range_m, 1.0f);
}

TEST(EosPublicApiConvenienceTest, RadiativeTransferEvaluatesModels) {
  foundation::radiative_transfer::RadiativeTransferInputs beer_lambert;
  beer_lambert.model = config::RadiativeTransferModel::kDerivedBeerLambert;
  beer_lambert.base_transmittance = 0.85f;
  beer_lambert.path_length_m = 1000.0f;
  const foundation::radiative_transfer::RadiativeTransferResult bl =
      foundation::radiative_transfer::EvaluateRadiativeTransfer(beer_lambert);
  EXPECT_GT(bl.transmittance, 0.0f);
  EXPECT_LE(bl.transmittance, 1.0f);

  foundation::radiative_transfer::RadiativeTransferInputs adaptive;
  adaptive.model = config::RadiativeTransferModel::kAdaptivePathRadiance;
  adaptive.base_transmittance = 0.8f;
  adaptive.cloud_coverage_ratio = 0.3f;
  adaptive.path_length_m = 5000.0f;
  adaptive.aerosol_density_factor = 2.0f;
  const foundation::radiative_transfer::RadiativeTransferResult ar =
      foundation::radiative_transfer::EvaluateRadiativeTransfer(adaptive);
  EXPECT_GT(ar.transmittance, 0.0f);
  EXPECT_LT(ar.transmittance, bl.transmittance);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepProducesDetectionOutput) {
  config::EosSessionConfig config;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

  session::EosSession session = session::EosSession::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 0U;
  input.dt_sec = 1.0f;
  input.scene.push_back(MakeTarget(501U, 1500.0f, 0.0f, 0.0f, 350.0f, 0.9f, 0.1f, 3.0f));

  const session::EosOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.cycle_index, 0U);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepWithResultAggregatesOutputAndValidation) {
  config::EosSessionConfig config;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

  session::EosSession session = session::EosSession::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.scene.push_back(MakeTarget(502U, 2000.0f, 5.0f, -3.0f, 330.0f, 0.85f, 0.15f, 2.5f));

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(result.validation_issues.empty());
  EXPECT_EQ(result.output_frame.cycle_index, 1U);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepWithResultSurfacesValidationErrors) {
  config::EosSessionConfig session_config;
  session::EosSession session = session::EosSession::Create(session_config);

  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  session::EosSceneTarget invalid_target;
  invalid_target.target_id = 0U;
  input.scene.push_back(invalid_target);

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(ContainsEosIssueCode(result.validation_issues,
                                   session::ValidationCode::kNonFiniteCycleDeltaTime));
}

TEST(EosPublicApiConvenienceTest, EosSessionAppliesRuntimeConfigPatch) {
  config::EosSessionConfig session_config;
  session::EosSession session = session::EosSession::Create(session_config);

  const config::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder()
                                                  .WithWorkMode(config::EosWorkMode::kInfraredOnly)
                                                  .WithFrameRateHz(15.0f)
                                                  .WithEnableStraylightFilter(true)
                                                  .WithHoodInnerHalfAngleDeg(8.0f)
                                                  .WithHoodOuterHalfAngleDeg(55.0f)
                                                  .WithHoodMinSuppressionRatio(0.35f)
                                                  .WithHoodMaxSuppressionRatio(0.95f)
                                                  .Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.scene.push_back(MakeTarget(600U, 1000.0f, 0.0f, 0.0f, 310.0f, 0.9f, 0.1f, 1.5f));

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(EosPublicApiConvenienceTest, EosSessionMultiCycleProducesProgressiveCycleIndices) {
  config::EosSessionConfig config;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;

  session::EosSession session = session::EosSession::Create(config);

  for (std::uint32_t i = 0U; i < 3U; ++i) {
    ::electro_optical_sensor::session::EosCycleInput input;
    input.cycle_index = i;
    input.dt_sec = 1.0f;
    input.scene.push_back(MakeTarget(700U + i, 1500.0f, 0.0f, 0.0f, 320.0f, 0.9f, 0.1f, 2.0f));

    const session::EosOutputFrame frame = session.Step(input);
    EXPECT_EQ(frame.cycle_index, i);
  }
}

TEST(EosPublicApiConvenienceTest, EosEnvironmentDefaultConfigHasReasonableDefaults) {
  const config::EosEnvironmentConfig default_config;
  EXPECT_EQ(default_config.scenario_config.model_type,
            config::EosEnvironmentModelType::kSimplified);
}

TEST(EosPublicApiConvenienceTest, EosRuntimeConfigBuilderWithEnvironmentPolicies) {
  config::EosEnvironmentScenarioConfig env_config;
  env_config.model_type = config::EosEnvironmentModelType::kAdvanced;
  env_config.has_custom_overrides = true;
  env_config.custom_overrides.radiative_transfer_model =
      config::RadiativeTransferModel::kAdaptivePathRadiance;
  env_config.custom_overrides.aerosol_density_factor = 1.3f;
  env_config.custom_overrides.turbulence_factor = 1.8f;

  const config::EosRuntimeConfigPatch patch =
      config::EosRuntimeConfigBuilder().WithEnvironmentScenarioConfig(env_config).Build();

  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_scenario_config);
  EXPECT_EQ(patch.environment.scenario_config.model_type,
            config::EosEnvironmentModelType::kAdvanced);
  EXPECT_TRUE(patch.environment.scenario_config.has_custom_overrides);
  EXPECT_EQ(patch.environment.scenario_config.custom_overrides.radiative_transfer_model,
            config::RadiativeTransferModel::kAdaptivePathRadiance);
  EXPECT_NEAR(patch.environment.scenario_config.custom_overrides.aerosol_density_factor, 1.3f,
              1e-5f);
  EXPECT_NEAR(patch.environment.scenario_config.custom_overrides.turbulence_factor, 1.8f, 1e-5f);
}

}  // namespace tests
}  // namespace electro_optical_sensor
