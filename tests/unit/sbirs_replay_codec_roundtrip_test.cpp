#include <gtest/gtest.h>

#include <string>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

TEST(SbirsReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 9U;
  target.target_name = "boost";
  target.position_ecef_m = Vector(8000000.0, 2.0, 3.0);
  target.temperature_k = 2300.0f;
  target.emissivity = 0.91f;
  target.projected_area_m2 = 42.0f;
  target.active = false;

  sbirs_sensor::session::SbirsEnvironmentInput environment;
  environment.has_environment_override = true;
  environment.environment.weather_type = sbirs_sensor::config::SbirsWeatherType::kFog;
  environment.environment.sea_state = sbirs_sensor::config::SbirsSeaState::kHigh;
  environment.environment.visibility_km = 4.0f;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(3U)
          .WithDeltaTimeSec(0.25f)
          .WithSatellitePosition(Vector(7000000.0, 4.0, 5.0))
          .WithEnvironment(environment)
          .AddTarget(target)
          .Build();

  const std::string bytes = sbirs_sensor::session::EncodeSbirsCycleInput(input);
  ASSERT_FALSE(bytes.empty());
  sbirs_sensor::session::SbirsCycleInput decoded;
  ASSERT_TRUE(sbirs_sensor::session::DecodeSbirsCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 3U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, 0.25f);
  EXPECT_TRUE(decoded.has_satellite_position);
  EXPECT_DOUBLE_EQ(decoded.satellite_position_ecef_m.x, 7000000.0);
  EXPECT_TRUE(decoded.environment.has_environment_override);
  EXPECT_EQ(decoded.environment.environment.weather_type,
            sbirs_sensor::config::SbirsWeatherType::kFog);
  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].target_id, 9U);
  EXPECT_EQ(decoded.scene[0].target_name, "boost");
  EXPECT_DOUBLE_EQ(decoded.scene[0].position_ecef_m.y, 2.0);
  EXPECT_FLOAT_EQ(decoded.scene[0].temperature_k, 2300.0f);
  EXPECT_FALSE(decoded.scene[0].active);
}

TEST(SbirsReplayCodecRoundtripTest, CycleResultPreservesOutputAndAttributionFields) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = 5U;
  result.output_frame.cycle_index = 5U;
  result.output_frame.scan_azimuth_deg = 12.0f;
  result.executed_this_cycle = true;

  sbirs_sensor::output::SbirsDetectionRecord detection;
  detection.detection_id = 33U;
  detection.azimuth_deg = 1.5f;
  detection.elevation_deg = -2.5f;
  detection.infrared_snr_linear = 9.0f;
  detection.observation_stage = sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  result.output_frame.detections.push_back(detection);

  sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 33U;
  attribution.target_id = 44U;
  attribution.target_name = "truth";
  attribution.estimated_range_m = 1234.0f;
  attribution.used_truth_assist = true;
  result.detection_attributions.push_back(attribution);

  sbirs_sensor::session::ValidationIssue issue;
  issue.severity = sbirs_sensor::session::ValidationSeverity::kWarning;
  issue.location.kind = sbirs_sensor::session::ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = 1U;
  issue.message = "warn";
  result.validation_issues.push_back(issue);

  const std::string bytes = sbirs_sensor::session::EncodeSbirsCycleResult(result);
  sbirs_sensor::session::SbirsCycleResult decoded;
  ASSERT_TRUE(sbirs_sensor::session::DecodeSbirsCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, 5U);
  EXPECT_TRUE(decoded.executed_this_cycle);
  ASSERT_EQ(decoded.output_frame.detections.size(), 1U);
  EXPECT_EQ(decoded.output_frame.detections[0].observation_stage,
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack);
  ASSERT_EQ(decoded.detection_attributions.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].estimated_range_m, 1234.0f);
  EXPECT_TRUE(decoded.detection_attributions[0].used_truth_assist);
  ASSERT_EQ(decoded.validation_issues.size(), 1U);
  EXPECT_EQ(decoded.validation_issues[0].location.entity_index, 1U);
}

TEST(SbirsReplayCodecRoundtripTest, SessionConfigAndRuntimePatchPreserveFields) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.optical_aperture_m = 0.9f;
  config.mission.scan_rate_deg_per_sec = 3.0f;
  config.policy.detection.narrow_min_snr_linear = 7.0f;
  config.environment.weather_type = sbirs_sensor::config::SbirsWeatherType::kRain;

  sbirs_sensor::config::SbirsSessionConfig decoded_config;
  ASSERT_TRUE(sbirs_sensor::session::DecodeSbirsSessionConfig(
      sbirs_sensor::session::EncodeSbirsSessionConfig(config), &decoded_config));
  EXPECT_FLOAT_EQ(decoded_config.hardware.optical_aperture_m, 0.9f);
  EXPECT_FLOAT_EQ(decoded_config.mission.scan_rate_deg_per_sec, 3.0f);
  EXPECT_FLOAT_EQ(decoded_config.policy.detection.narrow_min_snr_linear, 7.0f);
  EXPECT_EQ(decoded_config.environment.weather_type, sbirs_sensor::config::SbirsWeatherType::kRain);

  const sbirs_sensor::config::SbirsRuntimeConfigPatch patch =
      sbirs_sensor::config::SbirsRuntimeConfigBuilder()
          .WithWorkMode(sbirs_sensor::config::SbirsWorkMode::kWideSearch)
          .WithScanRateDegPerSec(4.0f)
          .WithSensorEnabled(false)
          .Build();
  sbirs_sensor::config::SbirsRuntimeConfigPatch decoded_patch;
  ASSERT_TRUE(sbirs_sensor::session::DecodeSbirsRuntimeConfigPatch(
      sbirs_sensor::session::EncodeSbirsRuntimeConfigPatch(patch), &decoded_patch));
  EXPECT_TRUE(decoded_patch.has_work_mode);
  EXPECT_EQ(decoded_patch.work_mode, sbirs_sensor::config::SbirsWorkMode::kWideSearch);
  EXPECT_TRUE(decoded_patch.has_scan_rate_deg_per_sec);
  EXPECT_FLOAT_EQ(decoded_patch.scan_rate_deg_per_sec, 4.0f);
  EXPECT_TRUE(decoded_patch.has_sensor_enabled);
  EXPECT_FALSE(decoded_patch.sensor_enabled);
}

TEST(SbirsReplayCodecRoundtripTest, RejectsEmptyPayloads) {
  sbirs_sensor::session::SbirsCycleInput input;
  sbirs_sensor::session::SbirsCycleResult result;
  sbirs_sensor::config::SbirsSessionConfig config;
  sbirs_sensor::config::SbirsRuntimeConfigPatch patch;
  EXPECT_FALSE(sbirs_sensor::session::DecodeSbirsCycleInput("", &input));
  EXPECT_FALSE(sbirs_sensor::session::DecodeSbirsCycleResult("", &result));
  EXPECT_FALSE(sbirs_sensor::session::DecodeSbirsSessionConfig("", &config));
  EXPECT_FALSE(sbirs_sensor::session::DecodeSbirsRuntimeConfigPatch("", &patch));
}

}  // namespace
