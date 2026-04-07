// Copyright 2026. All Rights Reserved.
//
// @file eos_public_api_convenience_test.cpp
// @brief 验证 EOS 对外易用性增强 API 的默认语义与集成行为。

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "1q/common/coordinate_transform.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/core/context/EosCoordinateUtils.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/core/session/EosSession.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace tests {

namespace {

bool ContainsEosIssueCode(const std::vector<core::context::EosValidationIssue>& issues,
                          core::context::EosValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

core::context::EosTargetState MakeTarget(std::uint64_t id, float range_m, float az_deg,
                                         float el_deg, float temp_k, float emissivity,
                                         float reflectance, float area_m2) {
  core::context::EosTargetState target;
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

}  // namespace

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderDefaultsMatchEosSessionConfig) {
  const core::session::EosSessionConfig built = config::EosSessionConfigBuilder().Build();
  const core::session::EosSessionConfig default_config;

  EXPECT_EQ(built.work_mode, default_config.work_mode);
  EXPECT_NEAR(built.wavelength_lower_um, default_config.wavelength_lower_um, 1e-5f);
  EXPECT_NEAR(built.wavelength_upper_um, default_config.wavelength_upper_um, 1e-5f);
  EXPECT_NEAR(built.minimum_snr_db, default_config.minimum_snr_db, 1e-5f);
}

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderOverridesAllFields) {
  const core::session::EosSessionConfig config =
      config::EosSessionConfigBuilder()
          .WithWorkMode(core::session::EosWorkMode::kInfraredOnly)
          .WithScanRateDegPerSec(40.0f)
          .WithFrameRateHz(60.0f)
          .WithMinimumSnrDb(10.0f)
          .EnableStraylightFilter(true)
          .WithVisibleReferenceIrradianceWm2(1000.0f)
          .WithEnvironmentModelType(environment::EosEnvironmentModelType::kAdvanced)
          .WithRadiativeTransferModel(
              foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance)
          .WithAerosolDensityFactor(2.0f)
          .WithTurbulenceFactor(1.5f)
          .Build();

  EXPECT_EQ(config.work_mode, core::session::EosWorkMode::kInfraredOnly);
  EXPECT_NEAR(config.scan_rate_deg_per_sec, 40.0f, 1e-5f);
  EXPECT_NEAR(config.frame_rate_hz, 60.0f, 1e-5f);
  EXPECT_NEAR(config.minimum_snr_db, 10.0f, 1e-5f);
  EXPECT_TRUE(config.enable_straylight_filter);
  EXPECT_NEAR(config.visible_reference_irradiance_w_m2, 1000.0f, 1e-5f);
  EXPECT_EQ(config.environment_default_config.model_type,
            environment::EosEnvironmentModelType::kAdvanced);
}

TEST(EosPublicApiConvenienceTest, SessionConfigBuilderPreservesPreconfiguredSessionConfig) {
  core::session::EosSessionConfig base;
  base.wavelength_lower_um = 8.0f;
  base.wavelength_upper_um = 12.0f;

  const core::session::EosSessionConfig config =
      config::EosSessionConfigBuilder(base).WithMinimumSnrDb(3.0f).Build();

  EXPECT_NEAR(config.wavelength_lower_um, 8.0f, 1e-5f);
  EXPECT_NEAR(config.wavelength_upper_um, 12.0f, 1e-5f);
  EXPECT_NEAR(config.minimum_snr_db, 3.0f, 1e-5f);
}

TEST(EosPublicApiConvenienceTest, RuntimeConfigBuilderDefaultsAreUnset) {
  const core::session::EosRuntimeConfigPatch patch = config::EosRuntimeConfigBuilder().Build();

  EXPECT_FALSE(patch.has_work_mode);
  EXPECT_FALSE(patch.has_scan_rate_deg_per_sec);
  EXPECT_FALSE(patch.has_frame_rate_hz);
  EXPECT_FALSE(patch.has_minimum_snr_db);
  EXPECT_FALSE(patch.has_enable_straylight_filter);
  EXPECT_FALSE(patch.has_visible_reference_irradiance_w_m2);
  EXPECT_FALSE(patch.has_environment_runtime_config);
}

TEST(EosPublicApiConvenienceTest, RuntimeConfigBuilderSetsAllFields) {
  const core::session::EosRuntimeConfigPatch patch =
      config::EosRuntimeConfigBuilder()
          .WithWorkMode(core::session::EosWorkMode::kVisibleOnly)
          .WithScanRateDegPerSec(50.0f)
          .WithFrameRateHz(120.0f)
          .WithMinimumSnrDb(15.0f)
          .EnableStraylightFilter(true)
          .WithVisibleReferenceIrradianceWm2(1200.0f)
          .Build();

  EXPECT_TRUE(patch.has_work_mode);
  EXPECT_EQ(patch.work_mode, core::session::EosWorkMode::kVisibleOnly);
  EXPECT_TRUE(patch.has_scan_rate_deg_per_sec);
  EXPECT_NEAR(patch.scan_rate_deg_per_sec, 50.0f, 1e-5f);
  EXPECT_TRUE(patch.has_frame_rate_hz);
  EXPECT_NEAR(patch.frame_rate_hz, 120.0f, 1e-5f);
  EXPECT_TRUE(patch.has_minimum_snr_db);
  EXPECT_NEAR(patch.minimum_snr_db, 15.0f, 1e-5f);
  EXPECT_TRUE(patch.has_enable_straylight_filter);
  EXPECT_TRUE(patch.enable_straylight_filter);
  EXPECT_TRUE(patch.has_visible_reference_irradiance_w_m2);
  EXPECT_NEAR(patch.visible_reference_irradiance_w_m2, 1200.0f, 1e-5f);
}

TEST(EosPublicApiConvenienceTest, InputValidationReportsErrorsForCommonBoundaryCases) {
  core::context::EosCycleInput input;
  input.dt_sec = 0.0f;
  input.solar_irradiance_w_m2 = -1.0f;
  input.atmospheric_transmittance = 1.5f;
  input.cloud_coverage_ratio = -0.1f;
  input.ambient_wind_speed_mps = -5.0f;
  input.background_temperature_k = 0.0f;
  input.solar_altitude_deg = 100.0f;

  core::context::EosTargetState invalid_target;
  invalid_target.target_id = 0U;
  invalid_target.range_m = -1.0f;
  invalid_target.apparent_temperature_k = 0.0f;
  invalid_target.emissivity = 1.5f;
  invalid_target.reflectance = 1.5f;
  invalid_target.projected_area_m2 = -1.0f;
  input.scene_targets.push_back(invalid_target);

  const core::context::EosValidationIssueList issues = core::context::ValidateEosCycleInput(input);

  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidSolarIrradiance));
  EXPECT_TRUE(ContainsEosIssueCode(
      issues, core::context::EosValidationCode::kInvalidAtmosphericTransmittance));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidCloudCoverageRatio));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidAmbientWindSpeed));
  EXPECT_TRUE(ContainsEosIssueCode(
      issues, core::context::EosValidationCode::kInvalidBackgroundTemperature));
  EXPECT_TRUE(ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidTargetId));
  EXPECT_TRUE(ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidTargetRange));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidTargetTemperature));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidTargetEmissivity));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidTargetReflectance));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidTargetProjectedArea));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kInvalidSolarAltitudeRange));
  EXPECT_TRUE(core::context::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationFlagsNonFiniteDtAndTargetFields) {
  core::context::EosCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  core::context::EosTargetState nan_target;
  nan_target.target_id = 100U;
  nan_target.range_m = std::numeric_limits<float>::infinity();
  input.scene_targets.push_back(nan_target);

  const core::context::EosValidationIssueList issues = core::context::ValidateEosCycleInput(input);

  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kNonFiniteCycleDeltaTime));
  EXPECT_TRUE(
      ContainsEosIssueCode(issues, core::context::EosValidationCode::kNonFiniteTargetNumericField));
  EXPECT_TRUE(core::context::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationFlagsEnergyBalanceInconsistency) {
  core::context::EosCycleInput input;
  input.dt_sec = 1.0f;

  core::context::EosTargetState unbalanced;
  unbalanced.target_id = 200U;
  unbalanced.range_m = 1000.0f;
  unbalanced.apparent_temperature_k = 300.0f;
  unbalanced.emissivity = 0.8f;
  unbalanced.reflectance = 0.3f;
  unbalanced.projected_area_m2 = 2.0f;
  input.scene_targets.push_back(unbalanced);

  const core::context::EosValidationIssueList issues = core::context::ValidateEosCycleInput(input);

  EXPECT_TRUE(ContainsEosIssueCode(
      issues, core::context::EosValidationCode::kInconsistentTargetEnergyBalance));
  EXPECT_TRUE(core::context::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, InputValidationPassesForValidInput) {
  core::context::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(300U, 1500.0f, 5.0f, -2.0f, 320.0f, 0.9f, 0.1f, 2.0f));

  const core::context::EosValidationIssueList issues = core::context::ValidateEosCycleInput(input);

  EXPECT_FALSE(core::context::HasEosValidationError(issues));
}

TEST(EosPublicApiConvenienceTest, CoordinateUtilsConvertsLlaAndEcefToEosLocal) {
  core::context::EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;
  reference.frame_attitude_deg.yaw_deg = 0.0f;
  reference.frame_attitude_deg.pitch_deg = 0.0f;
  reference.frame_attitude_deg.roll_deg = 0.0f;

  oneq::common::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  oneq::common::Vector3f local_from_lla;
  ASSERT_TRUE(core::context::TryConvertLlaToEosLocal(target_lla, reference, &local_from_lla));
  EXPECT_GT(local_from_lla.x, 100.0f);
  EXPECT_NEAR(local_from_lla.y, 0.0f, 1.0e-2f);
  EXPECT_NEAR(local_from_lla.z, 0.0f, 1.0e-2f);

  oneq::common::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(target_lla, &target_ecef));
  oneq::common::Vector3f local_from_ecef;
  ASSERT_TRUE(core::context::TryConvertEcefToEosLocal(target_ecef, reference, &local_from_ecef));
  EXPECT_NEAR(local_from_ecef.x, local_from_lla.x, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.y, local_from_lla.y, 1.0e-3f);
  EXPECT_NEAR(local_from_ecef.z, local_from_lla.z, 1.0e-3f);
}

TEST(EosPublicApiConvenienceTest, CoordinateUtilsBuildsTargetFromEcefAndLla) {
  core::context::EosCoordinateReference reference;
  reference.origin_lla.latitude_deg = 0.0;
  reference.origin_lla.longitude_deg = 0.0;
  reference.origin_lla.altitude_m = 0.0;

  oneq::common::PoseState platform_pose;
  platform_pose.position_m.x = 0.0f;
  platform_pose.position_m.y = 0.0f;
  platform_pose.position_m.z = 1000.0f;

  core::context::EosTargetAppearance appearance;
  appearance.apparent_temperature_k = 320.0f;
  appearance.emissivity = 0.9f;
  appearance.reflectance = 0.1f;
  appearance.projected_area_m2 = 2.0f;

  oneq::common::LlaCoordinateDegM target_lla;
  target_lla.latitude_deg = 0.0;
  target_lla.longitude_deg = 0.001;
  target_lla.altitude_m = 0.0;

  core::context::EosTargetState target_from_lla;
  ASSERT_TRUE(core::context::TryMakeEosTargetFromLla(401U, target_lla, reference, platform_pose,
                                                     appearance, &target_from_lla));
  EXPECT_EQ(target_from_lla.target_id, 401U);
  EXPECT_GT(target_from_lla.range_m, 0.0f);
  EXPECT_NEAR(target_from_lla.apparent_temperature_k, 320.0f, 1e-5f);

  oneq::common::EcefCoordinateM target_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(target_lla, &target_ecef));
  core::context::EosTargetState target_from_ecef;
  ASSERT_TRUE(core::context::TryMakeEosTargetFromEcef(402U, target_ecef, reference, platform_pose,
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
  core::session::EosSessionConfig config;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.minimum_snr_db = 0.0f;

  core::session::EosSession session(config);

  core::context::EosCycleInput input;
  input.cycle_index = 0U;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(501U, 1500.0f, 0.0f, 0.0f, 350.0f, 0.9f, 0.1f, 3.0f));

  const common::EosOutputFrame frame = session.Step(input);
  EXPECT_EQ(frame.cycle_index, 0U);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepWithResultAggregatesOutputAndValidation) {
  core::session::EosSessionConfig config;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.minimum_snr_db = 0.0f;

  core::session::EosSession session(config);

  core::context::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(502U, 2000.0f, 5.0f, -3.0f, 330.0f, 0.85f, 0.15f, 2.5f));

  const core::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.validation_issues.empty());
  EXPECT_EQ(result.output_frame.cycle_index, 1U);
}

TEST(EosPublicApiConvenienceTest, EosSessionStepWithResultSurfacesValidationErrors) {
  core::session::EosSessionConfig session_config;
  core::session::EosSession session(session_config);

  core::context::EosCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  core::context::EosTargetState invalid_target;
  invalid_target.target_id = 0U;
  input.scene_targets.push_back(invalid_target);

  const core::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(result.has_validation_error);
  EXPECT_TRUE(ContainsEosIssueCode(result.validation_issues,
                                   core::context::EosValidationCode::kNonFiniteCycleDeltaTime));
}

TEST(EosPublicApiConvenienceTest, EosSessionAppliesRuntimeConfigPatch) {
  core::session::EosSessionConfig session_config;
  core::session::EosSession session(session_config);

  const core::session::EosRuntimeConfigPatch patch =
      config::EosRuntimeConfigBuilder()
          .WithWorkMode(core::session::EosWorkMode::kInfraredOnly)
          .WithFrameRateHz(15.0f)
          .EnableStraylightFilter(true)
          .Build();
  session.ApplyRuntimeConfig(patch);

  core::context::EosCycleInput input;
  input.dt_sec = 1.0f;
  input.scene_targets.push_back(MakeTarget(600U, 1000.0f, 0.0f, 0.0f, 310.0f, 0.9f, 0.1f, 1.5f));

  const core::session::EosCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(EosPublicApiConvenienceTest, EosSessionMultiCycleProducesProgressiveCycleIndices) {
  core::session::EosSessionConfig config;
  config.scan_start_az_deg = -20.0f;
  config.scan_end_az_deg = 20.0f;
  config.minimum_snr_db = 0.0f;

  core::session::EosSession session(config);

  for (std::uint32_t i = 0U; i < 3U; ++i) {
    core::context::EosCycleInput input;
    input.cycle_index = i;
    input.dt_sec = 1.0f;
    input.scene_targets.push_back(
        MakeTarget(700U + i, 1500.0f, 0.0f, 0.0f, 320.0f, 0.9f, 0.1f, 2.0f));

    const common::EosOutputFrame frame = session.Step(input);
    EXPECT_EQ(frame.cycle_index, i);
  }
}

TEST(EosPublicApiConvenienceTest, EosEnvironmentDefaultConfigHasReasonableDefaults) {
  const environment::EosEnvironmentDefaultConfig default_config;
  EXPECT_EQ(default_config.model_type, environment::EosEnvironmentModelType::kSimplified);
  EXPECT_NEAR(default_config.aerosol_density_factor, 1.0f, 1e-5f);
  EXPECT_NEAR(default_config.turbulence_factor, 1.0f, 1e-5f);
}

TEST(EosPublicApiConvenienceTest, EosRuntimeConfigBuilderWithEnvironmentRuntimePatch) {
  environment::EosEnvironmentRuntimeConfigPatch env_patch;
  env_patch.has_aerosol_density_factor = true;
  env_patch.aerosol_density_factor = 3.0f;
  env_patch.has_turbulence_factor = true;
  env_patch.turbulence_factor = 2.5f;

  const core::session::EosRuntimeConfigPatch patch =
      config::EosRuntimeConfigBuilder().WithEnvironmentRuntimeConfigPatch(env_patch).Build();

  EXPECT_TRUE(patch.has_environment_runtime_config);
  EXPECT_TRUE(patch.environment_runtime_config.has_aerosol_density_factor);
  EXPECT_NEAR(patch.environment_runtime_config.aerosol_density_factor, 3.0f, 1e-5f);
  EXPECT_TRUE(patch.environment_runtime_config.has_turbulence_factor);
  EXPECT_NEAR(patch.environment_runtime_config.turbulence_factor, 2.5f, 1e-5f);
}

}  // namespace tests
}  // namespace electro_optical_sensor
