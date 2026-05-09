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

ar_session::RadarSessionConfig MakeWideAreaSearchConfig() {
  return ar_config::RadarSessionConfigBuilder()
      .Detection()
      .EnablePhysicsDetection(false)
      .WithHardwareProfile(ar_config::profiles::RadarHardwareProfile::kLongRangeHighPower)
      .WithDetectionIntentProfile(ar_config::profiles::DetectionIntentProfile::kDetectionPriority)
      .WithAntennaPatternProfile(ar_config::profiles::AntennaPatternProfile::kStandard)
      .End()
      .Mission()
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

ar_session::RadarSession CreateWideAreaSearchSession() {
  return ar_session::RadarSessionFactory::Create(MakeWideAreaSearchConfig());
}

ar_session::RadarExternalPoseInput MakePlatformPose(
    const oneq::coordinate::EcefPositionM& pos,
    const oneq::coordinate::EcefVelocityMps& vel) {
  ar_session::RadarExternalPoseInput platform;
  platform.platform_position_ecef_m = pos;
  platform.platform_velocity_mps = vel;
  platform.platform_attitude_deg.yaw_deg = 0.0;
  platform.platform_attitude_deg.pitch_deg = 0.0;
  platform.platform_attitude_deg.roll_deg = 0.0;
  platform.radar_mount_angles_deg.yaw_deg = 0.0;
  platform.radar_mount_angles_deg.pitch_deg = 0.0;
  platform.radar_mount_angles_deg.roll_deg = 0.0;
  return platform;
}

ar_session::TargetExternalKinematics MakeTargetKinematics(
    const oneq::coordinate::EcefPositionM& pos,
    const oneq::coordinate::EcefVelocityMps& vel,
    float rcs) {
  ar_session::TargetExternalKinematics target;
  target.target_position_ecef_m = pos;
  target.target_velocity_mps = vel;
  target.rcs = rcs;
  target.swerling_type = 0;
  return target;
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

void PrintResult(const char* label, const ar_session::RadarCycleResult& result) {
  std::cout << label << ": cycle=" << result.input_cycle_index
            << " tracks=" << result.track_output_frame.tracks.size() << " confirmed="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_model::TrackStatus::kConfirmed)
            << " tentative="
            << ar_session::CountTracksByStatus(result.track_output_frame,
                                               ar_model::TrackStatus::kTentative)
            << " commands=" << result.submitted_commands.size()
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

struct MovingAirTarget {
  oneq::coordinate::EcefPositionM pos;
  oneq::coordinate::EcefVelocityMps vel;
  float rcs;
};

bool RunMovingTargetsScenario() {
  ar_session::RadarSession session = CreateWideAreaSearchSession();
  ar_session::RadarEnvironmentInputState environment_state(MakeInitialEnvironmentInput());

  oneq::coordinate::EcefPositionM platform_pos;
  platform_pos.x_m = -2289512.0;
  platform_pos.y_m = 4909946.0;
  platform_pos.z_m = 3640982.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  platform_vel.x_mps = 120.0;
  platform_vel.y_mps = -80.0;
  platform_vel.z_mps = 30.0;

  std::vector<MovingAirTarget> targets = {
    {{-2289512.0 + 18000.0, 4909946.0 + 2500.0, 3640982.0 + 1200.0},
     {-120.0, 8.0, 0.0}, 2.2f},
    {{-2289512.0 + 24000.0, 4909946.0 - 4000.0, 3640982.0 + 2000.0},
     {-90.0, -12.0, 0.0}, 1.4f},
    {{-2289512.0 + 30000.0, 4909946.0 + 1000.0, 3640982.0 + 1500.0},
     {-150.0, 0.0, -5.0}, 3.0f},
  };

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_tracks = 0;
  std::uint32_t min_tracks = 100;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    ar_session::RadarExternalPoseInput platform = MakePlatformPose(platform_pos, platform_vel);

    std::vector<ar_session::TargetExternalKinematics> target_kinematics;
    target_kinematics.reserve(targets.size());
    for (const auto& mt : targets) {
      target_kinematics.push_back(
          MakeTargetKinematics(mt.pos, mt.vel, mt.rcs));
    }

    ar_session::RadarCycleInput input;
    if (!ar_session::RadarCycleInputBuilder::Build(
            platform, target_kinematics, 1.0f,
            environment_state.Snapshot(), &input)) {
      std::cerr << "ar-moving: cycle " << (i + 1) << " build failed\n";
      return false;
    }
    input.cycle_index = i + 1;

    const float dt = input.dt_sec;
    for (auto& mt : targets) {
      mt.pos.x_m += mt.vel.x_mps * dt;
      mt.pos.y_m += mt.vel.y_mps * dt;
      mt.pos.z_m += mt.vel.z_mps * dt;
    }

    ar_session::RadarCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) {
      ++validation_error_count;
    }
    std::size_t ntracks = result.track_output_frame.tracks.size();
    if (ntracks > max_tracks) max_tracks = ntracks;
    if (ntracks < min_tracks) min_tracks = ntracks;
    PrintResult("ar-moving", result);
  }

  std::cout << "\n=== AR Summary ===\n"
            << "cycles=" << num_cycles
            << " min_tracks=" << min_tracks
            << " max_tracks=" << max_tracks
            << " validation_errors=" << validation_error_count << "\n";
  return validation_error_count == 0;
}

}  // namespace

int main() { return RunMovingTargetsScenario() ? 0 : 1; }
