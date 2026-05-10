/**
 * @file electro_optical_sensor_session_usage.cpp
 * @brief Demonstrates recommended EOS session configuration and runtime usage.
 */

#include <cstddef>
#include <cstdint>
#include <iostream>

#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"

namespace eos_config = electro_optical_sensor::config;
namespace eos_env = electro_optical_sensor::environment;
namespace eos_session = electro_optical_sensor::session;

namespace {

eos_session::EosSessionConfig MakeFusedSearchConfig() {
  eos_session::EosSessionConfig config =
      eos_config::EosSessionConfigBuilder()
          .Mission()
          .WithWorkMode(eos_config::EosWorkMode::kFused)
          .WithScanRateDegPerSec(5.0f)
          .WithFrameRateHz(30.0f)
          .End()
          .Detection()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .End()
          .StrayLight()
          .WithStrayLightProfile(eos_config::EosStrayLightProfile::kStandardHood)
          .End()
          .Environment()
          .WithEnvironmentModelType(eos_env::EosEnvironmentModelType::kSimplified)
          .End()
          .Build();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  return config;
}

eos_session::EosSession CreateFusedSearchSession() {
  return eos_session::EosSessionFactory::Create(MakeFusedSearchConfig());
}

eos_session::EosSceneTarget MakeTarget(std::uint64_t id, float range_m, float azimuth_deg,
                                       float temperature_k, float area_m2) {
  eos_session::EosSceneTarget target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = temperature_k;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.35f;
  target.appearance.projected_area_m2 = area_m2;
  return target;
}

struct MovingEosTarget {
  std::uint64_t id;
  float range_m;
  float azimuth_deg;
  float temperature_k;
  float area_m2;
  float speed_range_mps;
  float speed_az_dps;
};

void PrintResult(const char* label, const eos_session::EosCycleResult& result) {
  std::size_t detected = 0;
  for (std::size_t i = 0; i < result.output_frame.detections.size(); ++i) {
    if (result.output_frame.detections[i].detected) ++detected;
  }
  std::cout << label << ": cycle=" << result.input_cycle_index
            << " detections=" << result.output_frame.detections.size() << " detected=" << detected
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

bool RunMovingTargetsScenario() {
  eos_session::EosSession session = CreateFusedSearchSession();

  std::vector<MovingEosTarget> targets = {
      {1U, 1400.0f, -5.0f, 335.0f, 4.0f, -10.0f, 0.2f},
      {2U, 2100.0f, 4.0f, 315.0f, 6.0f, -15.0f, -0.15f},
      {3U, 3200.0f, 1.5f, 350.0f, 3.0f, -8.0f, 0.1f},
  };

  const std::uint32_t num_cycles = 50;
  std::uint32_t validation_error_count = 0;
  std::uint32_t max_detected = 0;
  std::uint32_t min_detected = 100;

  for (std::uint32_t i = 0; i < num_cycles; ++i) {
    eos_session::EosCycleInput input;
    input.cycle_index = i + 1;
    input.dt_sec = 1.0f;
    input.platform_pose.position_m.z = 1500.0f;
    input.environment.solar_altitude_deg = 42.0f;
    input.environment.solar_azimuth_deg = 165.0f;
    input.environment.solar_irradiance_w_m2 = 850.0f;
    input.environment.cloud_coverage_ratio = 0.15f;
    input.environment.background_temperature_k = 288.0f;
    input.environment.day_night_type = eos_session::DayNightType::kDay;

    for (const auto& mt : targets) {
      input.scene.push_back(
          MakeTarget(mt.id, mt.range_m, mt.azimuth_deg, mt.temperature_k, mt.area_m2));
    }

    const float dt = input.dt_sec;
    for (auto& mt : targets) {
      mt.range_m += mt.speed_range_mps * dt;
      mt.azimuth_deg += mt.speed_az_dps * dt;
      if (mt.range_m < 100.0f) mt.speed_range_mps = -mt.speed_range_mps;
    }

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
