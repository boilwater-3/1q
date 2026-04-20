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

#include "1q/electro_optical_sensor/config/EosDetailedSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/electro_optical_sensor/model/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/foundation/coordinate_transform.h"

namespace rt = ::electro_optical_sensor::foundation::radiative_transfer;

namespace electro_optical_sensor {
namespace tests {

namespace {

static_assert(
  std::is_same<config::EosEnvironmentConfig,
         environment::EosEnvironmentDefaultConfig>::value,
  "config::EosEnvironmentConfig must alias environment::EosEnvironmentDefaultConfig");

bool ContainsEosIssueCode(const std::vector<model::EosValidationIssue>& issues,
                          model::EosValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

session::EosTargetState MakeTarget(std::uint64_t id, float range_m, float az_deg, float el_deg,
                                   float temp_k, float emissivity, float reflectance,
                                   float area_m2) {
  session::EosTargetState target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = az_deg;
  target.elevation_deg = el_deg;
  target.apparent_temperature_k = temp_k;
  target.emissivity = emissivity;
  target.reflectance = reflectance;
  target.projected_area_m2 = area_m2;
  return target;
}

class FixedEnvironmentService final : public environment::IEosEnvironmentService {
 public:
  environment::EosEnvironmentModelResult ResolveFactors(
      const environment::EosEnvironmentModelInputs& inputs) const override {
    (void)inputs;
    environment::EosEnvironmentModelResult result;
    result.aerosol_density_factor = 1.0f;
    result.turbulence_factor = 1.0f;
    result.path_radiance_scale_bias = 1.0f;
    return result;
  }
};

}  // namespace

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderDefaultsMatchEosSessionConfig) {
  const session::EosSessionConfig built = config::EosSessionConfigBuilder().Build();
  const session::EosSessionConfig default_config;

  EXPECT_EQ(built.mission.work_mode, default_config.mission.work_mode);
  EXPECT_NEAR(built.hardware.wavelength_lower_um, default_config.hardware.wavelength_lower_um, 1e-5f);
  EXPECT_NEAR(built.hardware.wavelength_upper_um, default_config.hardware.wavelength_upper_um, 1e-5f);
  EXPECT_EQ(built.policy.detection.profile, default_config.policy.detection.profile);
}

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderOverridesSemanticFields) {
  const session::EosSessionConfig config =
      config::EosSessionConfigBuilder()
          .WithWorkMode(session::EosWorkMode::kInfraredOnly)
          .WithDetectionProfile(config::EosDetectionProfile::kConservative)
          .WithStrayLightProfile(config::EosStrayLightProfile::kEnhancedHood)
          .WithEnvironmentModelType(environment::EosEnvironmentModelType::kAdvanced)
          .WithEnvironmentPreset(config::EosEnvironmentPreset::kDusty)
          .Build();

  EXPECT_EQ(config.mission.work_mode, session::EosWorkMode::kInfraredOnly);
  EXPECT_EQ(config.policy.detection.profile, config::EosDetectionProfile::kConservative);
  EXPECT_EQ(config.policy.stray_light.profile, config::EosStrayLightProfile::kEnhancedHood);
  EXPECT_EQ(config.environment.scenario_config.model_type,
            environment::EosEnvironmentModelType::kAdvanced);
  EXPECT_EQ(config.environment.scenario_config.preset, config::EosEnvironmentPreset::kDusty);
}

TEST(EosPublicApiConvenienceTest, DetailedSessionConfigBuilderOverridesDomainAndLeafFields) {
  const session::EosSessionConfig config =
      config::EosDetailedSessionConfigBuilder()
          .WithWorkMode(session::EosWorkMode::kInfraredOnly)
          .WithScanRateDegPerSec(40.0f)
          .WithFrameRateHz(60.0f)
          .WithDetectionProfile(config::EosDetectionProfile::kConservative)
          .WithStrayLightProfile(config::EosStrayLightProfile::kEnhancedHood)
          .WithEnvironmentModelType(environment::EosEnvironmentModelType::kAdvanced)
          .WithEnvironmentPreset(config::EosEnvironmentPreset::kDusty)
          .Build();

  EXPECT_EQ(config.mission.work_mode, session::EosWorkMode::kInfraredOnly);
  EXPECT_NEAR(config.mission.scan_rate_deg_per_sec, 40.0f, 1e-5f);
  EXPECT_NEAR(config.mission.frame_rate_hz, 60.0f, 1e-5f);
  EXPECT_EQ(config.policy.detection.profile, config::EosDetectionProfile::kConservative);
  EXPECT_EQ(config.policy.stray_light.profile, config::EosStrayLightProfile::kEnhancedHood);
  EXPECT_EQ(config.environment.scenario_config.model_type,
            environment::EosEnvironmentModelType::kAdvanced);
  EXPECT_EQ(config.environment.scenario_config.preset, config::EosEnvironmentPreset::kDusty);
}

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderPreservesPreconfiguredSessionConfig) {
  session::EosSessionConfig base;
  base.hardware.wavelength_lower_um = 8.0f;
  base.hardware.wavelength_upper_um = 12.0f;

  const session::EosSessionConfig config =
      config::EosSessionConfigBuilder(base)
          .WithDetectionProfile(config::EosDetectionProfile::kAggressive)
          .Build();

  EXPECT_NEAR(config.hardware.wavelength_lower_um, 8.0f, 1e-5f);
  EXPECT_NEAR(config.hardware.wavelength_upper_um, 12.0f, 1e-5f);
  EXPECT_EQ(config.policy.detection.profile, config::EosDetectionProfile::kAggressive);
}

TEST(EosPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const session::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_mission);
  EXPECT_FALSE(patch.has_policy);
  EXPECT_FALSE(patch.has_environment);
  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_deg_per_sec);
  EXPECT_FALSE(patch.has_frame_rate_hz);
}

TEST(EosPublicApiConvenienceTest, RuntimeConfigBuilderSetsAllFields) {
  const session::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder()
                                                   .WithWorkMode(session::EosWorkMode::kVisibleOnly)
                                                   .WithScanRateDegPerSec(50.0f)
                                                   .WithFrameRateHz(120.0f)
                                                   .WithDetectionProfile(
                                                       config::EosDetectionProfile::kConservative)
                                                   .WithStrayLightProfile(
                                                       config::EosStrayLightProfile::kEnhancedHood)
                                                   .WithEnvironmentModelType(
                                                       environment::EosEnvironmentModelType::kAdvanced)
                           .WithEnvironmentDetails(
                             rt::RadiativeTransferModel::kAdaptivePathRadiance,
                             1.3f,
                             1.8f)
                                                   .Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, session::EosWorkMode::kVisibleOnly);
  EXPECT_TRUE(patch.has_scan_rate_deg_per_sec);
  EXPECT_NEAR(patch.scan_rate_deg_per_sec, 50.0f, 1e-5f);
  EXPECT_TRUE(patch.has_frame_rate_hz);
  EXPECT_NEAR(patch.frame_rate_hz, 120.0f, 1e-5f);
  EXPECT_TRUE(patch.has_policy);
  EXPECT_EQ(patch.policy.detection.profile, config::EosDetectionProfile::kConservative);
  EXPECT_EQ(patch.policy.stray_light.profile, config::EosStrayLightProfile::kEnhancedHood);
  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_model_type);
  EXPECT_EQ(patch.environment.model_type, environment::EosEnvironmentModelType::kAdvanced);
  EXPECT_TRUE(patch.environment.has_radiative_transfer_model);
  EXPECT_EQ(patch.environment.radiative_transfer_model,
            rt::RadiativeTransferModel::kAdaptivePathRadiance);
  EXPECT_TRUE(patch.environment.has_aerosol_density_factor);
  EXPECT_NEAR(patch.environment.aerosol_density_factor, 1.3f, 1e-5f);
  EXPECT_TRUE(patch.environment.has_turbulence_factor);
  EXPECT_NEAR(patch.environment.turbulence_factor, 1.8f, 1e-5f);
}

TEST(EosPublicApiConvenienceTest, InputValidationReportsErrorsForCommonBoundaryCases) {
  session::EosCycleInput input;
  input.dt_sec = 0.0f;
  input.solar_irradiance_w_m2 = -1.0f;
  input.cloud_coverage_ratio = -0.1f;
  input.ambient_wind_speed_mps = -5.0f;
  input.background_temperature_k = 0.0f;
  input.solar_altitude_deg = 100.0f;

  session::EosTargetState invalid_target;
  invalid_target.target_id = 0U;
  invalid_target.range_m = -1.0f;
  invalid_target.apparent_temperature_k = 0.0f;
  invalid_target.emissivity = 1.5f;
  invalid_target.reflectance = 1.5f;
  invalid_target.projected_area_m2 = -1.0f;
  input.scene_targets.push_back(invalid_target);

  const model::EosValidationIssueList issues = model::ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidSolarIrradiance));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidCloudCoverageRatio));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidAmbientWindSpeed));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidBackgroundTemperature));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidTargetId));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidTargetRange));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidTargetTemperature));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidTargetEmissivity));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidTargetReflectance));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidTargetProjectedArea));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kInvalidSolarAltitudeRange));
  EXPECT_TRUE(model::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationFlagsNonFiniteDtAndTargetFields) {
  session::EosCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  session::EosTargetState nan_target;
  nan_target.target_id = 100U;
  nan_target.range_m = std::numeric_limits<float>::infinity();
  input.scene_targets.push_back(nan_target);

  const model::EosValidationIssueList issues = model::ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(ContainsEosIssueCode(issues, model::EosValidationCode::kNonFiniteTargetNumericField));
  EXPECT_TRUE(model::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationFlagsEnergyBalanceInconsistency) {
  session::EosCycleInput input;
  input.dt_sec = 1.0f;

  session::EosTargetState unbalanced;
  unbalanced.target_id = 200U;
  unbalanced.range_m = 1000.0f;
  unbalanced.apparent_temperature_k = 300.0f;
  unbalanced.emissivity = 0.8f;
  unbalanced.reflectance = 0.3f;
  unbalanced.projected_area_m2 = 2.0f;
  input.scene_targets.push_back(unbalanced);

  const model::EosValidationIssueList issues = model::ValidateEosCycleInput(input);

  EXPECT_TRUE(
      ContainsEosIssueCode(issues, model::EosValidationCode::kInconsistentTargetEnergyBalance));
  EXPECT_FALSE(model::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationPassesForValidInput) {
  session::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(300U, 1500.0f, 5.0f, -2.0f, 320.0f, 0.9f, 0.1f, 2.0f));

  const model::EosValidationIssueList issues = model::ValidateEosCycleInput(input);

  EXPECT_FALSE(model::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, CoordinateUtilsBuildsPoseFromExternalKinematics) {
  session::EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;
  reference.frame_attitude_deg.yaw_deg = 0.0f;
  reference.frame_attitude_deg.pitch_deg = 0.0f;
  reference.frame_attitude_deg.roll_deg = 0.0f;

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;
  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));

  oneq::foundation::EulerAnglesDeg platform_attitude_deg;
  platform_attitude_deg.yaw_deg = 5.0f;

  session::EosExternalPoseInput input_enu;
  input_enu.platform_position_ecef_m = target_ecef;
  input_enu.platform_velocity_frame = session::EosVelocityFrame::kEnu;
  input_enu.platform_velocity_mps.x = 10.0f;
  input_enu.platform_velocity_mps.y = 0.0f;
  input_enu.platform_velocity_mps.z = 0.0f;
  input_enu.platform_attitude_deg = platform_attitude_deg;

  oneq::foundation::PoseState pose_from_enu;
  ASSERT_TRUE(session::TryMakeEosPoseFromExternalKinematics(input_enu, reference, &pose_from_enu));
  EXPECT_GT(pose_from_enu.position_m.x, 100.0f);
  EXPECT_NEAR(pose_from_enu.position_m.y, 0.0f, 1.0e-2f);
  EXPECT_NEAR(pose_from_enu.position_m.z, 0.0f, 1.0e-2f);
  EXPECT_NEAR(pose_from_enu.velocity_mps.x, 10.0f, 1.0e-5f);
  EXPECT_NEAR(pose_from_enu.attitude_deg.yaw_deg, 5.0f, 1.0e-5f);

  session::EosExternalPoseInput input_ecef = input_enu;
  input_ecef.platform_velocity_frame = session::EosVelocityFrame::kEcef;
  input_ecef.platform_velocity_mps.x = 0.0f;
  input_ecef.platform_velocity_mps.y = 10.0f;
  input_ecef.platform_velocity_mps.z = 0.0f;

  oneq::foundation::PoseState pose_from_ecef;
  ASSERT_TRUE(
      session::TryMakeEosPoseFromExternalKinematics(input_ecef, reference, &pose_from_ecef));
  EXPECT_NEAR(pose_from_ecef.position_m.x, pose_from_enu.position_m.x, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.position_m.y, pose_from_enu.position_m.y, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.position_m.z, pose_from_enu.position_m.z, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.velocity_mps.x, pose_from_enu.velocity_mps.x, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.velocity_mps.y, pose_from_enu.velocity_mps.y, 1.0e-3f);
  EXPECT_NEAR(pose_from_ecef.velocity_mps.z, pose_from_enu.velocity_mps.z, 1.0e-3f);
}

TEST(EosPublicApiConvenienceTest, CoordinateUtilsBuildsTargetFromEcefAndLla) {
  session::EosCoordinateReference reference;
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

  oneq::foundation::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  session::EosTargetState target_from_lla;
  ASSERT_TRUE(session::TryMakeEosTargetFromLla(401U, target_lla, reference, platform_pose,
                                               appearance, &target_from_lla));
  EXPECT_EQ(target_from_lla.target_id, 401U);
  EXPECT_GT(target_from_lla.range_m, 0.0f);
  EXPECT_NEAR(target_from_lla.apparent_temperature_k, 320.0f, 1e-5f);

  oneq::foundation::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::foundation::TryLlaToEcef(target_lla, &target_ecef));
  session::EosTargetState target_from_ecef;
  ASSERT_TRUE(session::TryMakeEosTargetFromEcef(402U, target_ecef, reference, platform_pose,
                                                appearance, &target_from_ecef));
  EXPECT_NEAR(target_from_ecef.range_m, target_from_lla.range_m, 1.0f);
}

TEST(EosPublicApiConvenienceTest, RadiativeTransferEvaluatesModels) {
  foundation::radiative_transfer::RadiativeTransferInputs beer_lambert;
  beer_lambert.model = foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert;
  beer_lambert.base_transmittance = 0.85f;
  beer_lambert.path_length_m = 1000.0f;
  const foundation::radiative_transfer::RadiativeTransferResult bl =
      foundation::radiative_transfer::EvaluateRadiativeTransfer(beer_lambert);
  EXPECT_GT(bl.transmittance, 0.0f);
  EXPECT_LE(bl.transmittance, 1.0f);

  foundation::radiative_transfer::RadiativeTransferInputs adaptive;
  adaptive.model = foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
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
  session::EosSessionConfig config;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.policy.detection.profile = config::EosDetectionProfile::kAggressive;

  session::EosSession session = session::EosSessionFactory::Create(config);

  session::EosCycleInput input;
  input.cycle_index = 0U;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(501U, 1500.0f, 0.0f, 0.0f, 350.0f, 0.9f, 0.1f, 3.0f));

  const output::EosOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.cycle_index, 0U);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepWithResultAggregatesOutputAndValidation) {
  session::EosSessionConfig config;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.policy.detection.profile = config::EosDetectionProfile::kAggressive;

  session::EosSession session = session::EosSessionFactory::Create(config);

  session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(502U, 2000.0f, 5.0f, -3.0f, 330.0f, 0.85f, 0.15f, 2.5f));

  const model::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(result.validation_issues.empty());
  EXPECT_EQ(result.output_frame.cycle_index, 1U);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepWithResultSurfacesValidationErrors) {
  session::EosSessionConfig session_config;
  session::EosSession session = session::EosSessionFactory::Create(session_config);

  session::EosCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  session::EosTargetState invalid_target;
  invalid_target.target_id = 0U;
  input.scene_targets.push_back(invalid_target);

  const model::EosCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(ContainsEosIssueCode(result.validation_issues,
                                   model::EosValidationCode::kNonFiniteCycleDeltaTime));
}

TEST(EosPublicApiConvenienceTest, EosSessionAppliesRuntimeConfigPatch) {
  session::EosSessionConfig session_config;
  session::EosSession session = session::EosSessionFactory::Create(session_config);

  const session::EosRuntimeConfigPatch patch =
      config::EosRuntimeConfigBuilder()
          .WithWorkMode(session::EosWorkMode::kInfraredOnly)
          .WithFrameRateHz(15.0f)
          .WithStrayLightProfile(config::EosStrayLightProfile::kEnhancedHood)
          .Build();
  session.ApplyRuntimeConfig(patch);

  session::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(600U, 1000.0f, 0.0f, 0.0f, 310.0f, 0.9f, 0.1f, 1.5f));

  const model::EosCycleResult result = session.StepWithResult(input);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(EosPublicApiConvenienceTest, EosSessionFactoryCanInjectEnvironmentService) {
  FixedEnvironmentService environment_service;
  session::EosSession session = session::EosSessionFactory::CreateWithEnvironmentService(
      session::EosSessionConfig{}, environment_service);

  session::EosCycleInput input;
  input.cycle_index = 7U;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(601U, 1000.0f, 0.0f, 0.0f, 310.0f, 0.9f, 0.1f, 1.5f));

  const model::EosCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.output_frame.cycle_index, 7U);
}

TEST(EosPublicApiConvenienceTest, EosSessionMultiCycleProducesProgressiveCycleIndices) {
  session::EosSessionConfig config;
  config.mission.scan_start_az_deg = -20.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.policy.detection.profile = config::EosDetectionProfile::kAggressive;

  session::EosSession session = session::EosSessionFactory::Create(config);

  for (std::uint32_t i = 0U; i < 3U; ++i) {
    session::EosCycleInput input;
    input.cycle_index = i;
    input.dt_sec = 1.0f;
    input.scene_targets.push_back(
        MakeTarget(700U + i, 1500.0f, 0.0f, 0.0f, 320.0f, 0.9f, 0.1f, 2.0f));

    const output::EosOutputFrame frame = session.Step(input);
    EXPECT_EQ(frame.cycle_index, i);
  }
}

TEST(EosPublicApiConvenienceTest, EosEnvironmentDefaultConfigHasReasonableDefaults) {
  const environment::EosEnvironmentDefaultConfig default_config;
  EXPECT_EQ(default_config.scenario_config.model_type,
            environment::EosEnvironmentModelType::kSimplified);
}

TEST(EosPublicApiConvenienceTest, EosRuntimeConfigBuilderWithEnvironmentPolicies) {
  const session::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder()
                                                   .WithEnvironmentModelType(
                                                       environment::EosEnvironmentModelType::kAdvanced)
                                                   .WithEnvironmentDetails(
                                                       rt::RadiativeTransferModel::kAdaptivePathRadiance,
                                                       1.3f,
                                                       1.8f)
                                                   .Build();

  EXPECT_TRUE(patch.has_environment);
  EXPECT_TRUE(patch.environment.has_model_type);
  EXPECT_EQ(patch.environment.model_type, environment::EosEnvironmentModelType::kAdvanced);
  EXPECT_TRUE(patch.environment.has_radiative_transfer_model);
  EXPECT_EQ(patch.environment.radiative_transfer_model,
            rt::RadiativeTransferModel::kAdaptivePathRadiance);
  EXPECT_TRUE(patch.environment.has_aerosol_density_factor);
  EXPECT_NEAR(patch.environment.aerosol_density_factor, 1.3f, 1e-5f);
  EXPECT_TRUE(patch.environment.has_turbulence_factor);
  EXPECT_NEAR(patch.environment.turbulence_factor, 1.8f, 1e-5f);
}

}  // namespace tests
}  // namespace electro_optical_sensor
