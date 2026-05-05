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

eos_session::EosSessionConfig MakeLongRangeInfraredConfig() {
  eos_session::EosSessionConfig config =
      eos_config::EosSessionConfigBuilder()
          .Mission()
          .WithWorkMode(eos_config::EosWorkMode::kInfraredOnly)
          .WithScanRateDegPerSec(5.0f)
          .WithFrameRateHz(20.0f)
          .End()
          .Detection()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .End()
          .Environment()
          .WithEnvironmentModelType(eos_env::EosEnvironmentModelType::kAdvanced)
          .WithEnvironmentPreset(eos_config::EosEnvironmentPreset::kMaritime)
          .End()
          .Build();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  return config;
}

eos_session::EosSessionConfig MakeGlareResistantConfig() {
  eos_session::EosSessionConfig config =
      eos_config::EosSessionConfigBuilder()
          .Mission()
          .WithWorkMode(eos_config::EosWorkMode::kVisibleOnly)
          .WithScanRateDegPerSec(5.0f)
          .End()
          .Detection()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kConservative)
          .End()
          .StrayLight()
          .WithStrayLightProfile(eos_config::EosStrayLightProfile::kEnhancedHood)
          .End()
          .Environment()
          .WithEnvironmentPreset(eos_config::EosEnvironmentPreset::kDusty)
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

eos_session::EosSession CreateLongRangeInfraredSession() {
  return eos_session::EosSessionFactory::Create(MakeLongRangeInfraredConfig());
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

eos_session::EosCycleInput MakeCycleInput(std::uint32_t cycle_index) {
  eos_session::EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.z = 1500.0f;
  input.environment.solar_altitude_deg = 42.0f;
  input.environment.solar_azimuth_deg = 165.0f;
  input.environment.solar_irradiance_w_m2 = 850.0f;
  input.environment.cloud_coverage_ratio = 0.15f;
  input.environment.background_temperature_k = 288.0f;
  input.environment.day_night_type = eos_session::DayNightType::kDay;
  input.scene.push_back(MakeTarget(1U, 1400.0f, -5.0f, 335.0f, 4.0f));
  input.scene.push_back(MakeTarget(2U, 2100.0f, 4.0f, 315.0f, 6.0f));
  return input;
}

std::size_t CountDetected(const eos_session::EosOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    if (frame.detections[i].detected) {
      ++count;
    }
  }
  return count;
}

void PrintResult(const char* label, const eos_session::EosCycleResult& result) {
  std::cout << label << ": input_cycle=" << result.input_cycle_index
            << " output_cycle=" << result.output_frame.cycle_index
            << " detections=" << result.output_frame.detections.size()
            << " detected=" << CountDetected(result.output_frame)
            << " validation_errors=" << (result.has_validation_error ? "true" : "false") << "\n";
}

bool RunRecommendedScenario() {
  eos_session::EosSession session = CreateFusedSearchSession();
  eos_session::EosCycleInput input = MakeCycleInput(1U);
  const eos_session::ValidationIssueList issues = eos_session::ValidateEosCycleInput(input);
  if (eos_session::HasValidationError(issues)) {
    std::cerr << "EOS input is invalid: " << issues.size() << " issues\n";
    return false;
  }
  PrintResult("fused-search", session.StepWithResult(input));

  const eos_session::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithWorkMode(eos_config::EosWorkMode::kInfraredOnly)
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .WithStrayLightProfile(eos_config::EosStrayLightProfile::kEnhancedHood)
          .Build();
  session.ApplyRuntimeConfig(patch);
  PrintResult("runtime-ir", session.StepWithResult(MakeCycleInput(2U)));

  const eos_session::EosOutputFrame output_only = session.Step(MakeCycleInput(3U));
  std::cout << "output-only detections=" << output_only.detections.size() << "\n";

  eos_session::EosSession ir_session = CreateLongRangeInfraredSession();
  PrintResult("long-range-ir", ir_session.StepWithResult(MakeCycleInput(10U)));

  eos_session::EosSession glare_session =
      eos_session::EosSessionFactory::Create(MakeGlareResistantConfig());
  PrintResult("glare-resistant", glare_session.StepWithResult(MakeCycleInput(20U)));
  return true;
}

}  // namespace

int main() { return RunRecommendedScenario() ? 0 : 1; }
