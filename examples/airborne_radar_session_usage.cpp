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
  std::uint64_t id;
  float x, y, z;
  float vx, vy, vz;
  float rcs;
};

bool RunMovingTargetsScenario() {
  ar_session::RadarSession session = CreateWideAreaSearchSession();
  ar_session::RadarEnvironmentInputState environment_state(MakeInitialEnvironmentInput());

  std::vector<MovingAirTarget> targets = {
    {1001U, 18000.0f, 2500.0f, 1200.0f, -120.0f, 8.0f, 0.0f, 2.2f},
    {1002U, 24000.0f, -4000.0f, 2000.0f, -90.0f, -12.0f, 0.0f, 1.4f},
    {1003U, 30000.0f, 1000.0f, 1500.0f, -150.0f, 0.0f, -5.0f, 3.0f},
  };

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_tracks = 0;
  std::uint32_t min_tracks = 100;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    ar_session::RadarCycleInput input;
    input.cycle_index = i + 1;
    input.dt_sec = 1.0f;
    input.platform_pose.position_m.x = 0.0f;
    input.platform_pose.position_m.y = 0.0f;
    input.platform_pose.position_m.z = 9000.0f;
    input.platform_pose.velocity_mps.x = 230.0f;
    input.platform_pose.attitude_deg.yaw_deg = 0.0f;
    input.environment = environment_state.Snapshot();

    for (const auto& mt : targets) {
      input.scene.push_back(
          MakeAirTarget(mt.id, mt.x, mt.y, mt.z, mt.vx, mt.vy, mt.vz, mt.rcs));
    }

    const float dt = input.dt_sec;
    for (auto& mt : targets) {
      mt.x += mt.vx * dt;
      mt.y += mt.vy * dt;
      mt.z += mt.vz * dt;
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
