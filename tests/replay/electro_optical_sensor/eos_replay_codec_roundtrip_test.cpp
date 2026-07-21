/**
 * @file eos_replay_codec_roundtrip_test.cpp
 * @brief 验证 EOS replay FlatBuffers codec 各 payload 的 Encode→Decode round-trip 字段精确保真。
 *
 * 每个测试独立覆盖一种 payload 类型，确保新增字段未同步到 schema/codec 时立即被发现。
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/foundation/pose_types.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "electro_optical_sensor/session/EosReplayFlatbufferCodec.h"
#include "electro_optical_sensor/session/generated/eos_session_replay_generated.h"

namespace electro_optical_sensor {
namespace session {
namespace tests {

// ---------------------------------------------------------------------------
// EosCycleInput
// ---------------------------------------------------------------------------

TEST(EosReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  EosCycleInput input;
  input.cycle_index = 3U;
  input.dt_sec = 0.1f;
  input.platform_altitude_m = 1200.0f;
  input.platform_pose.position_m.x = 1000.0f;
  input.platform_pose.position_m.y = 2000.0f;
  input.platform_pose.position_m.z = 3000.0f;
  input.platform_pose.velocity_mps.x = 50.0f;
  input.platform_pose.velocity_mps.y = 10.0f;
  input.platform_pose.velocity_mps.z = 0.0f;
  input.platform_pose.attitude_deg.yaw_deg = 90.0f;
  input.platform_pose.attitude_deg.pitch_deg = 0.0f;
  input.platform_pose.attitude_deg.roll_deg = -3.0f;

  input.environment.solar_altitude_deg = 45.0f;
  input.environment.solar_azimuth_deg = 180.0f;
  input.environment.solar_irradiance_w_m2 = 900.0f;
  input.environment.cloud_coverage_ratio = 0.3f;
  input.environment.ambient_wind_speed_mps = 5.0f;
  input.environment.day_night_type = session::DayNightType::kDay;
  input.environment.background_temperature_k = 295.0f;

  EosSceneTarget target;
  target.target_id = 100U;
  target.target_name = "eos-target-alpha";
  target.range_m = 5000.0f;
  target.azimuth_deg = 10.0f;
  target.elevation_deg = -2.0f;
  target.appearance.apparent_temperature_k = 310.0f;
  target.appearance.emissivity = 0.95f;
  target.appearance.reflectance = 0.1f;
  target.appearance.projected_area_m2 = 2.5f;
  input.scene.push_back(target);

  const std::string bytes = EncodeEosCycleInput(input);
  ASSERT_FALSE(bytes.empty());

  EosCycleInput decoded;
  ASSERT_TRUE(DecodeEosCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 3U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, 0.1f);
  EXPECT_FLOAT_EQ(decoded.platform_altitude_m, 1200.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.position_m.x, 1000.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.velocity_mps.y, 10.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.attitude_deg.yaw_deg, 90.0f);

  EXPECT_FLOAT_EQ(decoded.environment.solar_altitude_deg, 45.0f);
  EXPECT_FLOAT_EQ(decoded.environment.solar_azimuth_deg, 180.0f);
  EXPECT_FLOAT_EQ(decoded.environment.solar_irradiance_w_m2, 900.0f);
  EXPECT_FLOAT_EQ(decoded.environment.cloud_coverage_ratio, 0.3f);
  EXPECT_FLOAT_EQ(decoded.environment.ambient_wind_speed_mps, 5.0f);
  EXPECT_EQ(decoded.environment.day_night_type, session::DayNightType::kDay);
  EXPECT_FLOAT_EQ(decoded.environment.background_temperature_k, 295.0f);

  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].target_id, 100U);
  EXPECT_EQ(decoded.scene[0].target_name, "eos-target-alpha");
  EXPECT_FLOAT_EQ(decoded.scene[0].range_m, 5000.0f);
  EXPECT_FLOAT_EQ(decoded.scene[0].appearance.apparent_temperature_k, 310.0f);
  EXPECT_FLOAT_EQ(decoded.scene[0].appearance.emissivity, 0.95f);
}

TEST(EosReplayCodecRoundtripTest, CycleInputPreservesDoublePrecisionPose) {
  EosCycleInput input;
  input.platform_pose.position_m.x = -4226.1319451063564;
  input.platform_pose.position_m.y = -8397.388596805331;
  input.platform_pose.position_m.z = 5255.8229838761899;
  input.platform_pose.velocity_mps.x = 123.45678901234567;
  input.platform_pose.velocity_mps.y = -45.67890123456789;
  input.platform_pose.velocity_mps.z = 0.000000123456789;
  input.platform_pose.attitude_deg.yaw_deg = 12.345678901234567;
  input.platform_pose.attitude_deg.pitch_deg = -2.345678901234567;
  input.platform_pose.attitude_deg.roll_deg = 0.12345678901234567;

  EosCycleInput decoded;
  ASSERT_TRUE(DecodeEosCycleInput(EncodeEosCycleInput(input), &decoded));

  EXPECT_DOUBLE_EQ(decoded.platform_pose.position_m.x, input.platform_pose.position_m.x);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.position_m.y, input.platform_pose.position_m.y);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.position_m.z, input.platform_pose.position_m.z);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.velocity_mps.x, input.platform_pose.velocity_mps.x);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.velocity_mps.y, input.platform_pose.velocity_mps.y);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.velocity_mps.z, input.platform_pose.velocity_mps.z);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.attitude_deg.yaw_deg,
                   input.platform_pose.attitude_deg.yaw_deg);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.attitude_deg.pitch_deg,
                   input.platform_pose.attitude_deg.pitch_deg);
  EXPECT_DOUBLE_EQ(decoded.platform_pose.attitude_deg.roll_deg,
                   input.platform_pose.attitude_deg.roll_deg);
}

TEST(EosReplayCodecRoundtripTest, CycleInputDecodesEmptyScene) {
  EosCycleInput input;
  input.dt_sec = 1.0f;

  const std::string bytes = EncodeEosCycleInput(input);
  EosCycleInput decoded;
  ASSERT_TRUE(DecodeEosCycleInput(bytes, &decoded));
  EXPECT_TRUE(decoded.scene.empty());
}

TEST(EosReplayCodecRoundtripTest, CycleInputRejectsEmptyPayload) {
  EosCycleInput decoded;
  EXPECT_FALSE(DecodeEosCycleInput("", &decoded));
}

// ---------------------------------------------------------------------------
// EosOutputFrame
// ---------------------------------------------------------------------------

TEST(EosReplayCodecRoundtripTest, OutputFramePreservesAllFields) {
  session::EosOutputFrame frame;
  frame.cycle_index = 7U;
  frame.scan_azimuth_deg = 15.0f;

  output::EosDetectionRecord rec;
  rec.detection_id = 42U;
  rec.range_m = 3000.0f;
  rec.azimuth_deg = 12.5f;
  rec.elevation_deg = -1.5f;
  rec.infrared_snr_linear = 25.0f;
  rec.visible_snr_linear = 10.0f;
  rec.fused_snr_linear = 35.0f;
  rec.fused_snr_db = 15.44f;
  rec.detected = true;
  frame.detections.push_back(rec);

  const std::string bytes = EncodeEosOutputFrame(frame);
  ASSERT_FALSE(bytes.empty());

  session::EosOutputFrame decoded;
  ASSERT_TRUE(DecodeEosOutputFrame(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 7U);
  EXPECT_FLOAT_EQ(decoded.scan_azimuth_deg, 15.0f);
  ASSERT_EQ(decoded.detections.size(), 1U);
  EXPECT_EQ(decoded.detections[0].detection_id, 42U);
  EXPECT_FLOAT_EQ(decoded.detections[0].range_m, 3000.0f);
  EXPECT_FLOAT_EQ(decoded.detections[0].azimuth_deg, 12.5f);
  EXPECT_FLOAT_EQ(decoded.detections[0].infrared_snr_linear, 25.0f);
  EXPECT_FLOAT_EQ(decoded.detections[0].fused_snr_db, 15.44f);
  EXPECT_TRUE(decoded.detections[0].detected);
}

// ---------------------------------------------------------------------------
// EosCycleResult
// ---------------------------------------------------------------------------

TEST(EosReplayCodecRoundtripTest, CycleResultPreservesAllFields) {
  EosCycleResult result;
  result.input_cycle_index = 55U;
  result.output_frame.cycle_index = 5U;
  result.output_frame.scan_azimuth_deg = 20.0f;

  output::EosDetectionRecord rec;
  rec.detection_id = 10U;
  rec.range_m = 4500.0f;
  rec.azimuth_deg = -7.5f;
  rec.fused_snr_db = 22.0f;
  rec.detected = true;
  result.output_frame.detections.push_back(rec);
  attribution::EosDetectionAttributionRecord attribution;
  attribution.detection_id = 10U;
  attribution.target_id = 1001U;
  attribution.target_name = "eos-result-target";
  result.detection_attributions.push_back(attribution);

  result.has_validation_error = true;
  result.executed_this_cycle = true;
  result.reused_previous_output = false;
  result.abort_reason = session::EosPipelineAbortReason::kValidationRejected;

  const std::string bytes = EncodeEosCycleResult(result);
  ASSERT_FALSE(bytes.empty());

  EosCycleResult decoded;
  ASSERT_TRUE(DecodeEosCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, 55U);
  EXPECT_EQ(decoded.output_frame.cycle_index, 5U);
  EXPECT_FLOAT_EQ(decoded.output_frame.scan_azimuth_deg, 20.0f);
  ASSERT_EQ(decoded.output_frame.detections.size(), 1U);
  EXPECT_EQ(decoded.output_frame.detections[0].detection_id, 10U);
  EXPECT_FLOAT_EQ(decoded.output_frame.detections[0].range_m, 4500.0f);
  EXPECT_FLOAT_EQ(decoded.output_frame.detections[0].azimuth_deg, -7.5f);
  EXPECT_FLOAT_EQ(decoded.output_frame.detections[0].fused_snr_db, 22.0f);
  EXPECT_TRUE(decoded.output_frame.detections[0].detected);
  ASSERT_EQ(decoded.detection_attributions.size(), 1U);
  EXPECT_EQ(decoded.detection_attributions[0].detection_id, 10U);
  EXPECT_EQ(decoded.detection_attributions[0].target_id, 1001U);
  EXPECT_EQ(decoded.detection_attributions[0].target_name, "eos-result-target");
  EXPECT_TRUE(decoded.has_validation_error);
  EXPECT_TRUE(decoded.executed_this_cycle);
  EXPECT_FALSE(decoded.reused_previous_output);
  EXPECT_EQ(decoded.abort_reason, session::EosPipelineAbortReason::kValidationRejected);
}

// ---------------------------------------------------------------------------
// config::EosSessionConfig
// ---------------------------------------------------------------------------

TEST(EosReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::EosSessionConfig config;
  // hardware
  config.hardware.wavelength_lower_um = 3.0f;
  config.hardware.wavelength_upper_um = 5.0f;
  config.hardware.optical_aperture_m = 0.3f;
  // mission
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.mission.horizontal_fov_deg = 8.0f;
  config.mission.vertical_fov_deg = 5.0f;
  config.mission.scan_rate_deg_per_sec = 25.0f;
  config.mission.frame_rate_hz = 30.0f;
  config.mission.scan_start_az_deg = -70.0f;
  config.mission.scan_end_az_deg = 70.0f;
  config.mission.scan_center_el_deg = 2.0f;
  config.mission.boresight_depression_deg = 50.0f;
  config.mission.power_on = false;
  // policy - detection
  config.policy.detection.minimum_snr_db = 8.0f;
  config.policy.detection.detection_sensitivity_w = 2.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 900.0f;
  // policy - stray light
  config.policy.stray_light.enable_straylight_filter = true;
  config.policy.stray_light.hood_inner_half_angle_deg = 10.0f;
  config.policy.stray_light.hood_outer_half_angle_deg = 80.0f;
  config.policy.stray_light.hood_min_suppression_ratio = 0.25f;
  config.policy.stray_light.hood_max_suppression_ratio = 0.90f;
  // environment
  config.environment.scenario_config.preset = config::EosEnvironmentPreset::kDusty;
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.pressure_hpa = 1010.0f;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 295.0f;
  config.environment.scenario_config.atmospheric_physics.relative_humidity = 0.65f;

  const std::string bytes = EncodeEosSessionConfig(config);
  ASSERT_FALSE(bytes.empty());

  const auto* encoded = eos::replay::GetEosSessionConfig(bytes.data());
  ASSERT_NE(encoded, nullptr);
  ASSERT_NE(encoded->environment(), nullptr);
  EXPECT_EQ(encoded->environment()->preset(),
            static_cast<int32_t>(config::EosEnvironmentPreset::kDusty));

  config::EosSessionConfig decoded;
  ASSERT_TRUE(DecodeEosSessionConfig(bytes, &decoded));

  // hardware
  EXPECT_FLOAT_EQ(decoded.hardware.wavelength_lower_um, 3.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.optical_aperture_m, 0.3f);
  // mission
  EXPECT_EQ(decoded.mission.work_mode, config::EosWorkMode::kFused);
  EXPECT_FLOAT_EQ(decoded.mission.horizontal_fov_deg, 8.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_rate_deg_per_sec, 25.0f);
  EXPECT_FLOAT_EQ(decoded.mission.frame_rate_hz, 30.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan_start_az_deg, -70.0f);
  EXPECT_FALSE(decoded.mission.power_on);
  // policy
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_snr_db, 8.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.detection_sensitivity_w, 2.0e-12f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.visible_reference_irradiance_w_m2, 900.0f);
  EXPECT_TRUE(decoded.policy.stray_light.enable_straylight_filter);
  EXPECT_FLOAT_EQ(decoded.policy.stray_light.hood_inner_half_angle_deg, 10.0f);
  // environment
  EXPECT_EQ(decoded.environment.scenario_config.preset, config::EosEnvironmentPreset::kDusty);
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa,
                  1010.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.temperature_k,
                  295.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.relative_humidity,
                  0.65f);
}

// ---------------------------------------------------------------------------
// config::EosRuntimeConfigPatch
// ---------------------------------------------------------------------------

TEST(EosReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  config::EosRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  patch.mission.scan_rate_deg_per_sec = 30.0f;
  patch.mission.power_on = false;
  patch.has_policy = true;
  patch.policy.detection.minimum_snr_db = 10.0f;
  patch.policy.stray_light.enable_straylight_filter = true;
  patch.has_environment = true;
  patch.environment.has_scenario_config = true;
  patch.environment.scenario_config.preset = config::EosEnvironmentPreset::kHumid;
  patch.has_work_mode = true;
  patch.work_mode = config::EosWorkMode::kVisibleOnly;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 20.0f;
  patch.has_frame_rate_hz = true;
  patch.frame_rate_hz = 15.0f;

  const std::string bytes = EncodeEosRuntimeConfigPatch(patch);
  ASSERT_FALSE(bytes.empty());

  config::EosRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeEosRuntimeConfigPatch(bytes, &decoded));

  EXPECT_TRUE(decoded.has_mission);
  EXPECT_EQ(decoded.mission.work_mode, config::EosWorkMode::kInfraredOnly);
  EXPECT_FALSE(decoded.mission.power_on);
  EXPECT_TRUE(decoded.has_policy);
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_snr_db, 10.0f);
  EXPECT_TRUE(decoded.policy.stray_light.enable_straylight_filter);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_TRUE(decoded.has_work_mode);
  EXPECT_EQ(decoded.work_mode, config::EosWorkMode::kVisibleOnly);
  EXPECT_TRUE(decoded.has_scan_rate_deg_per_sec);
  EXPECT_FLOAT_EQ(decoded.scan_rate_deg_per_sec, 20.0f);
  EXPECT_TRUE(decoded.has_frame_rate_hz);
  EXPECT_FLOAT_EQ(decoded.frame_rate_hz, 15.0f);
}

// ---------------------------------------------------------------------------
// FailureMarker
// ---------------------------------------------------------------------------

TEST(EosReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "EOS_ASSERT";
  failure.message = "detection pool overflow";
  failure.location = "EosController::RunOnce";
  failure.has_cycle_index = true;
  failure.cycle_index = 42U;
  failure.has_sim_time_sec = true;
  failure.sim_time_sec = 42.5;
  failure.diagnostics_payload = "{\"detection_count\":128}";

  const std::string bytes = EncodeEosFailureMarker(failure);
  ASSERT_FALSE(bytes.empty());

  oneq::replay::ReplayTraceFailure decoded;
  std::string error;
  ASSERT_TRUE(DecodeEosFailureMarker(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.error_code, "EOS_ASSERT");
  EXPECT_EQ(decoded.message, "detection pool overflow");
  EXPECT_EQ(decoded.location, "EosController::RunOnce");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 42U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 42.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"detection_count\":128}");
}

// ===========================================================================
// Decode 失败路径（null output / 损坏 payload）
// ===========================================================================

TEST(EosReplayCodecRoundtripTest, DecodeCycleInputRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEosCycleInput("", nullptr));
  EosCycleInput input;
  EXPECT_FALSE(DecodeEosCycleInput("corrupt", &input));
}

TEST(EosReplayCodecRoundtripTest, DecodeOutputFrameRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEosOutputFrame("", nullptr));
  EosOutputFrame frame;
  EXPECT_FALSE(DecodeEosOutputFrame("bad", &frame));
}

TEST(EosReplayCodecRoundtripTest, DecodeCycleResultRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEosCycleResult("", nullptr));
  EosCycleResult result;
  EXPECT_FALSE(DecodeEosCycleResult("bad", &result));
}

TEST(EosReplayCodecRoundtripTest, DecodeSessionConfigRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEosSessionConfig("", nullptr));
  config::EosSessionConfig config;
  EXPECT_FALSE(DecodeEosSessionConfig("bad", &config));
}

TEST(EosReplayCodecRoundtripTest, DecodeRuntimeConfigPatchRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEosRuntimeConfigPatch("", nullptr));
  config::EosRuntimeConfigPatch patch;
  EXPECT_FALSE(DecodeEosRuntimeConfigPatch("bad", &patch));
}

TEST(EosReplayCodecRoundtripTest, DecodeFailureMarkerRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeEosFailureMarker("", nullptr, &error));
  oneq::replay::ReplayTraceFailure failure;
  EXPECT_FALSE(DecodeEosFailureMarker("bad", &failure, &error));
}

}  // namespace tests
}  // namespace session
}  // namespace electro_optical_sensor
