/**
 * @file airborne_radar_session_usage.cpp
 * @brief Demonstrates recommended AR session configuration and runtime usage.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/coordinate/types.h"

namespace ar = airborne_radar;
namespace ar_config = airborne_radar::config;
namespace ar_env = airborne_radar::environment;
namespace ar_model = airborne_radar::model;
namespace ar_session = airborne_radar::session;

namespace {

ar_model::AzimuthElevationDeg MakeAzEl(float az_deg, float el_deg) {
  ar_model::AzimuthElevationDeg value;
  value.az_deg = az_deg;
  value.el_deg = el_deg;
  return value;
}

ar_model::CommandedBeamwidthDeg MakeBeamwidth(float az_deg, float el_deg) {
  ar_model::CommandedBeamwidthDeg value;
  value.commanded_az_beamwidth_deg = az_deg;
  value.commanded_el_beamwidth_deg = el_deg;
  return value;
}

ar_session::RadarSessionConfig MakeWideAreaSearchConfig() {
  return ar_config::RadarSessionConfigBuilder()
      .Detection()
      .EnablePhysicsDetection(false)
      .WithHardwareProfile(ar_config::profiles::RadarHardwareProfile::kLongRangeHighPower)
      .WithDetectionIntentProfile(ar_config::profiles::DetectionIntentProfile::kDetectionPriority)
      .WithAntennaPatternProfile(ar_config::profiles::AntennaPatternProfile::kStandard)
      .End()
      .Beam()
      .WithRadarWorkSubMode(ar_model::RadarWorkSubMode::kTas)
      .WithScanCenterDeg(MakeAzEl(0.0f, 0.0f))
      .End()
      .Tracking()
      .EnableTrackingFilter(true)
      .WithTrackingPolicyProfile(ar_config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(ar_config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Environment()
      .WithJammingSensitivityProfile(ar_env::JammingSensitivityProfile::kBalanced)
      .End()
      .Build();
}

ar_session::RadarSessionConfig MakeStableTrackConfig() {
  return ar_config::RadarSessionConfigBuilder()
      .Detection()
      .EnablePhysicsDetection(false)
      .WithDetectionIntentProfile(
          ar_config::profiles::DetectionIntentProfile::kTrackStabilityPriority)
      .WithRcsFusionProfile(ar_config::profiles::RcsFusionProfile::kEnhanced)
      .End()
      .Beam()
      .WithRadarWorkSubMode(ar_model::RadarWorkSubMode::kStt)
      .WithScanCenterDeg(MakeAzEl(6.0f, 1.5f))
      .EnableCommandedBeamwidth(true)
      .WithCommandedBeamwidthDeg(MakeBeamwidth(2.0f, 2.0f))
      .End()
      .Tracking()
      .EnableTrackingFilter(true)
      .WithTrackingPolicyProfile(ar_config::profiles::TrackingPolicyProfile::kBalanced)
      .End()
      .Lifecycle()
      .EnableImmFusion(true)
      .WithLifecyclePolicyProfile(ar_config::profiles::LifecyclePolicyProfile::kBalanced)
      .End()
      .Environment()
      .WithJammingSensitivityProfile(ar_env::JammingSensitivityProfile::kBalanced)
      .End()
      .Build();
}

ar_session::RadarSessionConfig MakeDenseJammingConfig() {
  return ar_config::RadarSessionConfigBuilder()
      .Detection()
      .EnablePhysicsDetection(false)
      .WithHardwareProfile(ar_config::profiles::RadarHardwareProfile::kLongRangeHighPower)
      .WithDetectionIntentProfile(
          ar_config::profiles::DetectionIntentProfile::kTrackStabilityPriority)
      .WithAntennaPatternProfile(ar_config::profiles::AntennaPatternProfile::kLowSidelobe)
      .End()
      .Beam()
      .WithRadarWorkSubMode(ar_model::RadarWorkSubMode::kTas)
      .WithScanCenterDeg(MakeAzEl(0.0f, 0.0f))
      .End()
      .Tracking()
      .EnableTrackingFilter(true)
      .WithTrackingPolicyProfile(ar_config::profiles::TrackingPolicyProfile::kRobustAntiJamming)
      .End()
      .Lifecycle()
      .EnableImmFusion(true)
      .WithLifecyclePolicyProfile(ar_config::profiles::LifecyclePolicyProfile::kHighPersistence)
      .End()
      .Environment()
      .WithJammingSensitivityProfile(ar_env::JammingSensitivityProfile::kStrict)
      .End()
      .Build();
}

ar_session::RadarSessionConfig MakeLowAltitudeClutterConfig() {
  ar_env::EnvironmentDefaultConfig environment_default;
  environment_default.scenario_config.atmospheric_physics.enable_physical_model = true;
  environment_default.scenario_config.atmospheric_physics.pressure_hpa = 1005.0f;
  environment_default.scenario_config.atmospheric_physics.temperature_k = 294.0f;
  environment_default.scenario_config.atmospheric_physics.relative_humidity = 0.75f;
  environment_default.scenario_config.vegetation_scatter_physics.cover_profile =
      ar_env::VegetationCoverProfile::kSparseWoodland;
  environment_default.scenario_config.vegetation_scatter_physics.enable_physical_model = true;

  return ar_config::RadarSessionConfigBuilder()
      .Detection()
      .EnablePhysicsDetection(true)
      .WithDetectionIntentProfile(ar_config::profiles::DetectionIntentProfile::kBalanced)
      .WithRcsFusionProfile(ar_config::profiles::RcsFusionProfile::kEnhanced)
      .End()
      .Beam()
      .WithRadarWorkSubMode(ar_model::RadarWorkSubMode::kTas)
      .WithScanCenterDeg(MakeAzEl(0.0f, -2.0f))
      .End()
      .Tracking()
      .EnableTrackingFilter(true)
      .WithTrackingPolicyProfile(ar_config::profiles::TrackingPolicyProfile::kBalanced)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(ar_config::profiles::LifecyclePolicyProfile::kHighPersistence)
      .End()
      .Environment()
      .WithEnvironmentDefault(environment_default)
      .WithJammingSensitivityProfile(ar_env::JammingSensitivityProfile::kRelaxed)
      .End()
      .Build();
}

ar_session::RadarSession CreateWideAreaSearchSession() {
  return ar_session::RadarSessionFactory::Create(MakeWideAreaSearchConfig());
}

ar_session::RadarSession CreateStableTrackSession() {
  return ar_session::RadarSessionFactory::Create(MakeStableTrackConfig());
}

ar_session::RadarSession CreateDenseJammingSession() {
  return ar_session::RadarSessionFactory::Create(MakeDenseJammingConfig());
}

ar_session::RadarSceneTarget MakeAirTarget(std::uint64_t id, float x_m, float y_m, float z_m,
                                           float vx_mps, float vy_mps, float vz_mps, float rcs_m2) {
  ar_session::RadarSceneTarget target;
  target.external_target_id = id;
  target.position_x = x_m;
  target.position_y = y_m;
  target.position_z = z_m;
  target.velocity_x = vx_mps;
  target.velocity_y = vy_mps;
  target.velocity_z = vz_mps;
  target.rcs = rcs_m2;
  target.range_m = std::sqrt(x_m * x_m + y_m * y_m + z_m * z_m);
  return target;
}

ar_env::JammerEmitterState MakeNoiseJammer(float az_deg, float js_db) {
  ar_env::JammerEmitterState jammer;
  jammer.technique = ar_env::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 14.0f;
  jammer.js_db = js_db;
  jammer.has_direction_deg = true;
  jammer.azimuth_deg = az_deg;
  jammer.elevation_deg = 0.0f;
  jammer.angular_span_deg = 12.0f;
  jammer.confidence = 0.9f;
  return jammer;
}

ar_session::RadarEnvironmentInput MakeInitialEnvironmentInput() {
  ar_session::RadarEnvironmentInput environment;
  environment.atmospheric_observation.enable_physical_model = true;
  environment.atmospheric_observation.pressure_hpa = 1010.0f;
  environment.atmospheric_observation.temperature_k = 290.0f;
  environment.atmospheric_observation.relative_humidity = 0.45f;
  environment.atmospheric_context.has_simulation_unix_seconds = true;
  environment.atmospheric_context.simulation_unix_seconds = 1770000000;
  environment.atmospheric_context.solar_flux_f107a = 145.0f;
  environment.atmospheric_context.solar_flux_f107 = 148.0f;
  environment.atmospheric_context.geomagnetic_ap = 5.0f;
  environment.surface_observation.cover_profile = ar_env::VegetationCoverProfile::kOpenGrassland;
  environment.surface_observation.enable_physical_model = true;
  return environment;
}

void ApplyJammingChange(ar_session::RadarEnvironmentInputState* environment_state) {
  ar_session::RadarEnvironmentInputPatch patch;
  patch.has_jammer_sources = true;
  patch.jammer_sources.push_back(MakeNoiseJammer(18.0f, 9.0f));
  environment_state->Update(patch);
}

ar_session::RadarCycleInput MakeLocalCycleInput(
    std::uint32_t cycle_index, const ar_session::RadarEnvironmentInput& environment_snapshot) {
  ar_session::RadarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.x = 0.0f;
  input.platform_pose.position_m.y = 0.0f;
  input.platform_pose.position_m.z = 9000.0f;
  input.platform_pose.velocity_mps.x = 230.0f;
  input.platform_pose.attitude_deg.yaw_deg = 0.0f;
  input.environment = environment_snapshot;
  input.scene.push_back(
      MakeAirTarget(1001U, 18000.0f, 2500.0f, 1200.0f, -120.0f, 8.0f, 0.0f, 2.2f));
  input.scene.push_back(
      MakeAirTarget(1002U, 24000.0f, -4000.0f, 2000.0f, -90.0f, -12.0f, 0.0f, 1.4f));
  return input;
}

bool BuildCycleFromExternalKinematics(ar_session::RadarCycleInput* output) {
  ar_session::RadarExternalPoseInput platform;
  platform.platform_position_ecef_m.x_m = 6378137.0;
  platform.platform_position_ecef_m.y_m = 0.0;
  platform.platform_position_ecef_m.z_m = 0.0;
  platform.platform_velocity_mps.y_mps = 230.0;
  platform.platform_attitude_deg.yaw_deg = 90.0;

  ar_session::TargetExternalKinematics target;
  target.target_position_ecef_m.x_m = 6378137.0;
  target.target_position_ecef_m.y_m = 18000.0;
  target.target_position_ecef_m.z_m = 1400.0;
  target.target_velocity_mps.y_mps = 110.0;
  target.rcs = 2.0f;

  std::vector<ar_session::TargetExternalKinematics> targets;
  targets.push_back(target);
  return ar_session::RadarCycleInputBuilder::Build(platform, targets, 1.0f, output);
}

void PrintResult(const char* label, const ar_session::RadarCycleResult& result) {
  std::cout << label << ": input_cycle=" << result.input_cycle_index
            << " output_cycle=" << result.track_output_frame.cycle_index
            << " tracks=" << result.track_output_frame.tracks.size() << " confirmed="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_model::TrackStatus::kConfirmed)
            << " commands=" << result.submitted_commands.size()
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

bool RunRecommendedScenario() {
  ar_session::RadarSession session = CreateWideAreaSearchSession();
  ar_session::RadarEnvironmentInputState environment_state(MakeInitialEnvironmentInput());

  ar_session::RadarCycleInput first_input = MakeLocalCycleInput(1U, environment_state.Snapshot());
  std::vector<ar_session::ValidationIssue> issues =
      ar_session::ValidateRadarCycleInput(first_input);
  if (ar_session::HasValidationError(issues)) {
    std::cerr << "first cycle input is invalid: " << issues.size() << " issues\n";
    return false;
  }
  PrintResult("wide-search", session.StepWithResult(first_input));

  ApplyJammingChange(&environment_state);
  ar_config::RadarRuntimeConfigPatch patch =
      ar_config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(ar_model::RadarWorkSubMode::kStt)
          .WithDwellCenterDeg(MakeAzEl(6.0f, 1.0f))
          .WithCommandedBeamwidthDeg(MakeBeamwidth(2.0f, 2.0f))
          .EnableCommandedBeamwidth(true)
          .WithJammingSensitivityProfile(ar_env::JammingSensitivityProfile::kStrict)
          .Build();
  session.ApplyRuntimeConfig(patch);

  ar_session::RadarCycleInput jammed_input = MakeLocalCycleInput(2U, environment_state.Snapshot());
  PrintResult("jammed-stt", session.StepWithResult(jammed_input));

  const ar_session::TrackOutputFrame output_only =
      session.Step(MakeLocalCycleInput(3U, environment_state.Snapshot()));
  std::cout << "output-only step tracks=" << output_only.tracks.size() << "\n";

  ar_session::RadarCycleInput external_input;
  if (BuildCycleFromExternalKinematics(&external_input)) {
    external_input.cycle_index = 4U;
    external_input.environment = environment_state.Snapshot();
    PrintResult("external-adapter", session.StepWithResult(external_input));
  }

  ar_session::RadarSession stable_track_session = CreateStableTrackSession();
  PrintResult("stable-track", stable_track_session.StepWithResult(
                                  MakeLocalCycleInput(10U, environment_state.Snapshot())));

  ar_session::RadarSession robust_session = CreateDenseJammingSession();
  PrintResult("dense-jamming", robust_session.StepWithResult(
                                   MakeLocalCycleInput(20U, environment_state.Snapshot())));

  ar_session::RadarSession low_altitude_session =
      ar_session::RadarSessionFactory::Create(MakeLowAltitudeClutterConfig());
  PrintResult("low-altitude-clutter", low_altitude_session.StepWithResult(
                                          MakeLocalCycleInput(30U, environment_state.Snapshot())));

  return true;
}

}  // namespace

int main() {
  if (!RunRecommendedScenario()) {
    return 1;
  }
  return 0;
}
