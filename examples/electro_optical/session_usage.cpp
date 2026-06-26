/**
 * @file electro_optical_sensor_session_usage.cpp
 * @brief Demonstrates recommended EOS session configuration and runtime usage.
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <cmath>

#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "config_loader.h"

namespace eos_config = electro_optical_sensor::config;
namespace eos_env = electro_optical_sensor::session;
namespace eos_session = electro_optical_sensor::session;

namespace {

eos_config::EosSessionConfig LoadConfigFromFile() {
  eos_config::EosSessionConfig config;
  std::string error;
  if (!examples::LoadEosSessionConfigFromFile("configs/electro_optical.json",
                                               &config, &error)) {
    std::cerr << "Failed to load EOS config: " << error << "\n";
    std::exit(1);
  }
  return config;
}

eos_session::EosSession CreateFusedSearchSession() {
  return eos_session::EosSession::Create(LoadConfigFromFile());
}

eos_session::EosExternalTargetInput MakeTargetLlaWithVelocity(
    double lat_deg, double lon_deg, double alt_m,
    double vel_east_mps, double vel_north_mps, double vel_up_mps,
    float temperature_k, float area_m2) {
  eos_session::EosExternalTargetInput target;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  target.kinematics.position_lla_deg_m.latitude_deg = lat_deg;
  target.kinematics.position_lla_deg_m.longitude_deg = lon_deg;
  target.kinematics.position_lla_deg_m.altitude_m = alt_m;

  oneq::coordinate::LlaPositionDegM target_lla;
  target_lla.latitude_deg = lat_deg;
  target_lla.longitude_deg = lon_deg;
  target_lla.altitude_m = alt_m;

  oneq::coordinate::EnuVelocityMps target_vel_enu;
  target_vel_enu.east_mps = vel_east_mps;
  target_vel_enu.north_mps = vel_north_mps;
  target_vel_enu.up_mps = vel_up_mps;

  oneq::coordinate::TryEnuToEcefVelocity(target_vel_enu, target_lla, &target.kinematics.velocity_mps);

  target.appearance.apparent_temperature_k = temperature_k;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.35f;
  target.appearance.projected_area_m2 = area_m2;
  return target;
}

bool RunMovingTargetsScenario() {
  eos_session::EosSession session = CreateFusedSearchSession();

  // Platform altitude in range 6000-8000m (we use 7000m)
  oneq::coordinate::LlaPositionDegM platform_lla_init;
  platform_lla_init.latitude_deg = 35.0;
  platform_lla_init.longitude_deg = 114.5;
  platform_lla_init.altitude_m = 7000.0;

  oneq::coordinate::EcefPositionM platform_pos;
  if (!oneq::coordinate::TryLlaToEcef(platform_lla_init, &platform_pos)) {
    std::cerr << "Failed to convert platform LLA to ECEF\n";
    return false;
  }

  // Platform moves East at 150 m/s
  oneq::coordinate::EnuVelocityMps platform_vel_enu;
  platform_vel_enu.east_mps = 150.0;
  platform_vel_enu.north_mps = 0.0;
  platform_vel_enu.up_mps = 0.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  if (!oneq::coordinate::TryEnuToEcefVelocity(platform_vel_enu, platform_lla_init, &platform_vel)) {
    std::cerr << "Failed to convert platform ENU velocity to ECEF\n";
    return false;
  }

  eos_session::EosEnvironmentInput environment;
  environment.solar_altitude_deg = 42.0f;
  environment.solar_azimuth_deg = 165.0f;
  environment.solar_irradiance_w_m2 = 850.0f;
  environment.cloud_coverage_ratio = 0.15f;
  environment.background_temperature_k = 288.0f;
  environment.day_night_type = eos_session::DayNightType::kDay;

  const std::uint32_t num_cycles = 50;
  const double dt = 1.0;
  std::uint32_t validation_error_count = 0;
  std::size_t max_detected = 0;
  std::size_t min_detected = 100;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    eos_session::EosExternalPoseInput platform;
    platform.platform_position_ecef_m.x_m = platform_pos.x_m + platform_vel.x_mps * dt * static_cast<double>(i);
    platform.platform_position_ecef_m.y_m = platform_pos.y_m + platform_vel.y_mps * dt * static_cast<double>(i);
    platform.platform_position_ecef_m.z_m = platform_pos.z_m + platform_vel.z_mps * dt * static_cast<double>(i);
    platform.platform_velocity_mps = platform_vel;

    platform.platform_attitude_deg.yaw_deg = 0.0;
    platform.platform_attitude_deg.pitch_deg = 0.0;
    platform.platform_attitude_deg.roll_deg = 0.0;

    // Ground target longitude offset corresponding to ~6303m sensor depression distance
    // (7000 / tan(48 deg)). One degree longitude at 35N is ~91024m.
    // Base longitude of sensor footprint center at start is 114.5 + 0.06925.
    double base_lon = 114.5 + 0.06925;

    std::vector<eos_session::EosExternalTargetInput> target_inputs;

    // 1. Stationary Target 1 (ID = 101): Placed at Lon = base_lon + 0.013 (~114.58225)
    // Make target hotter (480K) and larger (15.0 m2) to achieve detection SNR
    target_inputs.push_back(MakeTargetLlaWithVelocity(
        35.0, base_lon + 0.013, 0.0,
        0.0, 0.0, 0.0,
        480.0f, 15.0f
    ));
    target_inputs.back().target_id = 101;

    // 2. Stationary Target 2 (ID = 102): Placed at Lon = base_lon + 0.050 (~114.61925)
    target_inputs.push_back(MakeTargetLlaWithVelocity(
        35.002, base_lon + 0.050, 0.0,
        0.0, 0.0, 0.0,
        490.0f, 12.0f
    ));
    target_inputs.back().target_id = 102;

    // 3. Moving Target 1 (ID = 201): Starts at Lon = base_lon + 0.025, moves North at 15 m/s
    double t201_lat = 34.998 + (15.0 * dt * static_cast<double>(i)) / 111111.0;
    double t201_lon = base_lon + 0.025;
    target_inputs.push_back(MakeTargetLlaWithVelocity(
        t201_lat, t201_lon, 0.0,
        0.0, 15.0, 0.0,
        500.0f, 14.0f
    ));
    target_inputs.back().target_id = 201;

    // 4. Moving Target 2 (ID = 202): Starts at Lon = base_lon + 0.060, moves West at 25 m/s (oncoming)
    double t202_lat = 35.001;
    double t202_lon = base_lon + 0.060 - (25.0 * dt * static_cast<double>(i)) / (111111.0 * std::cos(35.0 * 3.141592653589793 / 180.0));
    target_inputs.push_back(MakeTargetLlaWithVelocity(
        t202_lat, t202_lon, 0.0,
        -25.0, 0.0, 0.0,
        520.0f, 18.0f
    ));
    target_inputs.back().target_id = 202;

    eos_session::EosCycleInput input;
    eos_session::EosCoordinateStatus status;
    if (!eos_session::EosCycleInputBuilder::Build(platform, target_inputs, static_cast<float>(dt), environment,
                                                  &input, &status)) {
      std::cerr << "eos-moving: cycle " << (i + 1)
                << " build failed (status=" << static_cast<int>(status) << ")\n";
      return false;
    }
    input.cycle_index = i + 1;

    eos_session::EosCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) ++validation_error_count;

    oneq::coordinate::LocalFrameReference output_reference;
    output_reference.origin_lla.latitude_deg = 31.0;
    output_reference.origin_lla.longitude_deg = 121.0;
    output_reference.origin_lla.altitude_m = 0.0;
    eos_session::EosExternalOutputFrame external_output;
    const bool external_output_ok = eos_session::EosCycleOutputBuilder::Build(
        output_reference, input.platform_pose, result.output_frame, &external_output);

    std::size_t ndetected = 0;
    std::string detected_ids = "";
    for (std::size_t j = 0; j < result.output_frame.detections.size(); ++j) {
      const auto& det = result.output_frame.detections[j];
      if (det.detected) {
        ++ndetected;
        detected_ids += std::to_string(det.detection_id) + " ";
      }
      std::cout << "    [Detection ID " << det.detection_id << "] range=" << det.range_m
                << "m, az=" << det.azimuth_deg << "deg, el=" << det.elevation_deg
                << "deg, IR_SNR_lin=" << det.infrared_snr_linear
                << ", Vis_SNR_lin=" << det.visible_snr_linear
                << ", Fused_SNR_db=" << det.fused_snr_db
                << " (detected=" << (det.detected ? "true" : "false") << ")\n";
    }
    if (ndetected > max_detected) max_detected = ndetected;
    if (ndetected < min_detected) min_detected = ndetected;

    std::cout << "eos-moving: cycle=" << result.input_cycle_index
              << " detections=" << result.output_frame.detections.size() << " detected=" << ndetected
              << " (IDs: " << (detected_ids.empty() ? "None" : detected_ids) << ")"
              << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
    std::cout << "  external_output="
              << (external_output_ok ? external_output.detections.size() : 0U) << "\n";
  }

  std::cout << "\n=== EOS Summary ===\n"
            << "cycles=" << num_cycles << " min_detected=" << min_detected
            << " max_detected=" << max_detected << " validation_errors=" << validation_error_count
            << "\n";
  return validation_error_count == 0;
}

}  // namespace

int main() { return RunMovingTargetsScenario() ? 0 : 1; }
