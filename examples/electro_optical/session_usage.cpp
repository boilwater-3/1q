/**
 * @file electro_optical_sensor_session_usage.cpp
 * @brief Demonstrates recommended EOS session configuration and runtime usage.
 */

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "config_loader.h"

namespace eos_config = electro_optical_sensor::config;
namespace eos_env = electro_optical_sensor::environment;
namespace eos_session = electro_optical_sensor::session;

namespace {

eos_session::EosSessionConfig LoadConfigFromFile() {
  eos_session::EosSessionConfig config;
  std::string error;
  if (!examples::LoadEosSessionConfigFromFile("configs/electro_optical.json",
                                               &config, &error)) {
    std::cerr << "Failed to load EOS config: " << error << "\n";
    std::exit(1);
  }
  return config;
}

eos_session::EosSession CreateFusedSearchSession() {
  return eos_session::EosSessionFactory::Create(LoadConfigFromFile());
}

eos_session::EosExternalTargetInput MakeTargetLla(double lat_deg, double lon_deg, double alt_m,
                                                   float temperature_k, float area_m2) {
  eos_session::EosExternalTargetInput target;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  target.kinematics.position_lla_deg_m.latitude_deg = lat_deg;
  target.kinematics.position_lla_deg_m.longitude_deg = lon_deg;
  target.kinematics.position_lla_deg_m.altitude_m = alt_m;
  target.appearance.apparent_temperature_k = temperature_k;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.35f;
  target.appearance.projected_area_m2 = area_m2;
  return target;
}

void PrintResult(const char* label, const eos_session::EosCycleResult& result) {
  std::size_t detected = 0;
  for (std::size_t i = 0; i < result.output_frame.detections.size(); ++i) {
    if (result.output_frame.detections[i].detected) ++detected;
  }
  std::cout << label << ": cycle=" << result.input_cycle_index
            << " detections=" << result.output_frame.detections.size() << " detected=" << detected
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

struct LlaTarget {
  double lat_deg;
  double lon_deg;
  double alt_m;
  float temperature_k;
  float area_m2;
};

bool RunMovingTargetsScenario() {
  eos_session::EosSession session = CreateFusedSearchSession();

  oneq::coordinate::EcefPositionM platform_pos;
  platform_pos.x_m = -2169532.0;
  platform_pos.y_m = 4760604.0;
  platform_pos.z_m = 3638727.0;

  oneq::coordinate::EcefVelocityMps platform_vel;
  platform_vel.x_mps = 120.0;
  platform_vel.y_mps = -80.0;
  platform_vel.z_mps = 30.0;

  // Targets ~2km north of platform at same altitude, well within sensor FOV and range.
  // Platform is ~35.0N, 114.5E at ~1500m MSL.

  eos_session::EosEnvironmentInput environment;
  environment.solar_altitude_deg = 42.0f;
  environment.solar_azimuth_deg = 165.0f;
  environment.solar_irradiance_w_m2 = 850.0f;
  environment.cloud_coverage_ratio = 0.15f;
  environment.background_temperature_k = 288.0f;
  environment.day_night_type = eos_session::DayNightType::kDay;

  // Ground targets east of platform — within forward-looking FOV (~±10°×±4°) and
  // detection range (~1900-2160m). Platform at ~1500m MSL, boresight 48° downward.
  const std::vector<LlaTarget> targets = {
      {35.0, 114.514, 0.0, 345.0f, 5.0f},
      {35.0, 114.515, 0.0, 350.0f, 6.0f},
      {35.0, 114.516, 0.0, 335.0f, 4.0f},
  };

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_detected = 0;
  std::uint32_t min_detected = 100;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    eos_session::EosExternalPoseInput platform;
    platform.platform_position_ecef_m = platform_pos;
    platform.platform_velocity_mps = platform_vel;
    platform.platform_attitude_deg.yaw_deg = 0.0;
    platform.platform_attitude_deg.pitch_deg = 0.0;
    platform.platform_attitude_deg.roll_deg = 0.0;

    std::vector<eos_session::EosExternalTargetInput> target_inputs;
    target_inputs.reserve(targets.size());
    for (std::size_t ti = 0; ti < targets.size(); ++ti) {
      target_inputs.push_back(MakeTargetLla(targets[ti].lat_deg, targets[ti].lon_deg,
                                            targets[ti].alt_m, targets[ti].temperature_k,
                                            targets[ti].area_m2));
    }

    eos_session::EosCycleInput input;
    eos_session::EosCoordinateStatus status;
    if (!eos_session::EosCycleInputBuilder::Build(platform, target_inputs, 1.0f, environment,
                                                  &input, &status)) {
      std::cerr << "eos-moving: cycle " << (i + 1)
                << " build failed (status=" << static_cast<int>(status) << ")\n";
      return false;
    }
    input.cycle_index = i + 1;

    eos_session::EosCycleResult result = session.StepWithResult(input);
    if (result.has_validation_error) ++validation_error_count;

    eos_session::EosCoordinateReference output_reference;
    output_reference.origin_lla.latitude_deg = 31.0;
    output_reference.origin_lla.longitude_deg = 121.0;
    output_reference.origin_lla.altitude_m = 0.0;
    eos_session::EosExternalOutputFrame external_output;
    const bool external_output_ok = eos_session::EosCycleOutputBuilder::Build(
        output_reference, input.platform_pose, result.output_frame, &external_output);

    std::size_t ndetected = 0;
    for (std::size_t j = 0; j < result.output_frame.detections.size(); ++j) {
      if (result.output_frame.detections[j].detected) ++ndetected;
    }
    if (ndetected > max_detected) max_detected = ndetected;
    if (ndetected < min_detected) min_detected = ndetected;

    PrintResult("eos-moving", result);
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
