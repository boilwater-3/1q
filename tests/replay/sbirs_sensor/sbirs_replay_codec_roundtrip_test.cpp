#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#include "1q/replay/ReplayTrace.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

namespace sbirs_sensor {
namespace session {
namespace tests {
namespace {

using config::SbirsEnvironmentConfig;
using config::SbirsHardwareConfig;
using config::SbirsMissionConfig;
using config::SbirsPolicyConfig;
using config::SbirsRuntimeConfigBuilder;
using config::SbirsRuntimeConfigPatch;
using config::SbirsSeaState;
using config::SbirsSessionConfig;
using config::SbirsWeatherType;
using config::SbirsWorkMode;
using output::SbirsDetectionRecord;
using output::SbirsObservationStage;

SbirsVector3M Vector(double x, double y, double z) {
  SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

}  // namespace

// --- CycleInput ---

TEST(SbirsReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  SbirsSceneTarget target;
  target.target_id = 9U;
  target.target_name = "boost";
  target.position_ecef_m = Vector(8000000.0, 2.0, 3.0);
  target.temperature_k = 2300.0f;
  target.emissivity = 0.91f;
  target.projected_area_m2 = 42.0f;
  target.velocity_ecef_m_per_s = Vector(1000.0, -500.0, 250.0);
  target.has_velocity_ecef_m_per_s = true;
  target.active = false;

  SbirsEnvironmentInput environment;
  environment.has_environment_override = true;
  environment.environment.weather_type = SbirsWeatherType::kFog;
  environment.environment.sea_state = SbirsSeaState::kHigh;
  environment.environment.visibility_km = 4.0f;

  const SbirsCycleInput input = SbirsCycleInputBuilder()
                                    .WithCycleIndex(3U)
                                    .WithDeltaTimeSec(0.25f)
                                    .WithSatellitePosition(Vector(7000000.0, 4.0, 5.0))
                                    .WithEnvironment(environment)
                                    .AddTarget(target)
                                    .Build();

  const std::string bytes = EncodeSbirsCycleInput(input);
  ASSERT_FALSE(bytes.empty());
  SbirsCycleInput decoded;
  ASSERT_TRUE(DecodeSbirsCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 3U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, 0.25f);
  EXPECT_TRUE(decoded.has_satellite_position);
  EXPECT_DOUBLE_EQ(decoded.satellite_position_ecef_m.x, 7000000.0);
  EXPECT_TRUE(decoded.environment.has_environment_override);
  EXPECT_EQ(decoded.environment.environment.weather_type, SbirsWeatherType::kFog);
  EXPECT_EQ(decoded.environment.environment.sea_state, SbirsSeaState::kHigh);
  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].target_id, 9U);
  EXPECT_EQ(decoded.scene[0].target_name, "boost");
  EXPECT_DOUBLE_EQ(decoded.scene[0].position_ecef_m.y, 2.0);
  EXPECT_FLOAT_EQ(decoded.scene[0].temperature_k, 2300.0f);
  EXPECT_FLOAT_EQ(decoded.scene[0].emissivity, 0.91f);
  EXPECT_FLOAT_EQ(decoded.scene[0].projected_area_m2, 42.0f);
  EXPECT_DOUBLE_EQ(decoded.scene[0].velocity_ecef_m_per_s.x, 1000.0);
  EXPECT_DOUBLE_EQ(decoded.scene[0].velocity_ecef_m_per_s.y, -500.0);
  EXPECT_DOUBLE_EQ(decoded.scene[0].velocity_ecef_m_per_s.z, 250.0);
  EXPECT_TRUE(decoded.scene[0].has_velocity_ecef_m_per_s);
  EXPECT_FALSE(decoded.scene[0].active);
}

TEST(SbirsReplayCodecRoundtripTest, CycleInputDecodesEmptyScene) {
  const SbirsCycleInput input = SbirsCycleInputBuilder().WithCycleIndex(1U).Build();
  const std::string bytes = EncodeSbirsCycleInput(input);
  SbirsCycleInput decoded;
  ASSERT_TRUE(DecodeSbirsCycleInput(bytes, &decoded));
  EXPECT_EQ(decoded.cycle_index, 1U);
  EXPECT_TRUE(decoded.scene.empty());
}

// --- OutputFrame ---

TEST(SbirsReplayCodecRoundtripTest, OutputFramePreservesAllFields) {
  SbirsOutputFrame frame;
  frame.cycle_index = 7U;
  frame.scan_azimuth_deg = -15.5f;

  SbirsDetectionRecord detection;
  detection.detection_id = 33U;
  detection.azimuth_deg = 1.5f;
  detection.elevation_deg = -2.5f;
  detection.infrared_snr_linear = 9.0f;
  detection.observation_stage = SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  frame.detections.push_back(detection);

  const std::string bytes = EncodeSbirsOutputFrame(frame);
  ASSERT_FALSE(bytes.empty());
  SbirsOutputFrame decoded;
  ASSERT_TRUE(DecodeSbirsOutputFrame(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 7U);
  EXPECT_FLOAT_EQ(decoded.scan_azimuth_deg, -15.5f);
  ASSERT_EQ(decoded.detections.size(), 1U);
  EXPECT_EQ(decoded.detections[0].detection_id, 33U);
  EXPECT_FLOAT_EQ(decoded.detections[0].azimuth_deg, 1.5f);
  EXPECT_FLOAT_EQ(decoded.detections[0].elevation_deg, -2.5f);
  EXPECT_FLOAT_EQ(decoded.detections[0].infrared_snr_linear, 9.0f);
  EXPECT_EQ(decoded.detections[0].observation_stage, SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_TRUE(decoded.detections[0].detected);
}

// --- CycleResult ---

TEST(SbirsReplayCodecRoundtripTest, CycleResultPreservesOutputAndAttributionFields) {
  SbirsCycleResult result;
  result.input_cycle_index = 5U;
  result.output_frame.cycle_index = 5U;
  result.output_frame.scan_azimuth_deg = 12.0f;
  result.executed_this_cycle = true;
  result.reused_previous_output = true;
  result.has_validation_error = true;
  result.abort_reason = SbirsPipelineAbortReason::kValidationRejected;

  SbirsDetectionRecord detection;
  detection.detection_id = 33U;
  detection.observation_stage = SbirsObservationStage::kNarrowFieldTrack;
  detection.detected = true;
  result.output_frame.detections.push_back(detection);

  attribution::SbirsDetectionAttributionRecord attribution;
  attribution.detection_id = 33U;
  attribution.target_id = 44U;
  attribution.target_name = "truth";
  attribution.estimated_range_m = 1234.0f;
  attribution.used_truth_assist = true;
  attribution.capture_failure_reason =
      attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed;
  attribution.has_estimation_nis = true;
  attribution.estimation_nis = 6.25f;
  attribution.estimation_nis_gate_exceeded = true;
  attribution.nfov_channel_id = 1;
  attribution.has_nfov_tracking_diagnostics = true;
  attribution.nfov_pointing_error_deg = 0.75f;
  attribution.nfov_geometry_gate_passed = false;
  attribution.nfov_snr_gate_passed = true;
  attribution.nfov_tracking_gate_failure_count = 1U;
  attribution.nfov_tracking_coasting = true;
  result.detection_attributions.push_back(attribution);

  // 带 entity_index 的 scene 级 issue
  ValidationIssue scene_issue;
  scene_issue.severity = ValidationSeverity::kWarning;
  scene_issue.location.kind = ValidationLocationKind::kSceneEntity;
  scene_issue.location.entity_index = 1U;
  scene_issue.message = "warn";
  result.validation_issues.push_back(scene_issue);

  // kGlobal location（entity_index 哨兵值 size_t::max ↔ int64 -1）
  ValidationIssue global_issue;
  global_issue.severity = ValidationSeverity::kError;
  global_issue.location.kind = ValidationLocationKind::kGlobal;
  global_issue.location.entity_index = std::numeric_limits<std::size_t>::max();
  global_issue.message = "global";
  result.validation_issues.push_back(global_issue);

  const std::string bytes = EncodeSbirsCycleResult(result);
  SbirsCycleResult decoded;
  ASSERT_TRUE(DecodeSbirsCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, 5U);
  EXPECT_TRUE(decoded.executed_this_cycle);
  EXPECT_TRUE(decoded.reused_previous_output);
  EXPECT_TRUE(decoded.has_validation_error);
  EXPECT_EQ(decoded.abort_reason, SbirsPipelineAbortReason::kValidationRejected);
  ASSERT_EQ(decoded.output_frame.detections.size(), 1U);
  EXPECT_EQ(decoded.output_frame.detections[0].observation_stage,
            SbirsObservationStage::kNarrowFieldTrack);
  ASSERT_EQ(decoded.detection_attributions.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].estimated_range_m, 1234.0f);
  EXPECT_TRUE(decoded.detection_attributions[0].used_truth_assist);
  EXPECT_EQ(decoded.detection_attributions[0].capture_failure_reason,
            attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
  EXPECT_TRUE(decoded.detection_attributions[0].has_estimation_nis);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].estimation_nis, 6.25f);
  EXPECT_TRUE(decoded.detection_attributions[0].estimation_nis_gate_exceeded);
  EXPECT_EQ(decoded.detection_attributions[0].nfov_channel_id, 1);
  EXPECT_TRUE(decoded.detection_attributions[0].has_nfov_tracking_diagnostics);
  EXPECT_FLOAT_EQ(decoded.detection_attributions[0].nfov_pointing_error_deg, 0.75f);
  EXPECT_FALSE(decoded.detection_attributions[0].nfov_geometry_gate_passed);
  EXPECT_TRUE(decoded.detection_attributions[0].nfov_snr_gate_passed);
  EXPECT_EQ(decoded.detection_attributions[0].nfov_tracking_gate_failure_count, 1U);
  EXPECT_TRUE(decoded.detection_attributions[0].nfov_tracking_coasting);
  ASSERT_EQ(decoded.validation_issues.size(), 2U);
  EXPECT_EQ(decoded.validation_issues[0].location.entity_index, 1U);
  // 哨兵值经 size_t::max → int64(-1) → size_t::max 的往返
  EXPECT_EQ(decoded.validation_issues[1].location.kind, ValidationLocationKind::kGlobal);
  EXPECT_EQ(decoded.validation_issues[1].location.entity_index,
            std::numeric_limits<std::size_t>::max());
}

// --- SessionConfig ---

TEST(SbirsReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  SbirsSessionConfig config;
  config.hardware.wavelength_lower_um = 2.0f;
  config.hardware.wavelength_upper_um = 5.0f;
  config.hardware.optical_aperture_m = 0.9f;
  config.hardware.detector_area_m2 = 2.0e-4f;
  config.hardware.optical_transmission = 0.75f;
  config.hardware.detector_quantum_efficiency = 0.65f;
  config.hardware.integration_time_sec = 0.04f;
  config.hardware.noise_equivalent_power_w = 2.0e-12f;
  config.mission.work_mode = SbirsWorkMode::kWideSearch;
  config.mission.sensor_enabled = true;
  config.mission.scan_rate_deg_per_sec = 3.0f;
  config.mission.narrow_cue_latency_s = 0.05f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 17.5f;
  config.mission.narrow_pointing_settle_tolerance_deg = 0.025f;
  config.policy.detection.wide_min_snr_linear = 3.5f;
  config.policy.detection.narrow_min_snr_linear = 7.0f;
  config.policy.error_model.angular_sigma_deg = 0.08f;
  config.policy.error_model.range_fraction_sigma = 0.002f;
  config.policy.error_model.random_seed = 42U;
  config.policy.scheduler.max_concurrent_nfov_locks = 3;
  config.policy.tracking.enable_estimated_tracking = true;
  config.policy.tracking.process_noise_diff_coeff = 2.5f;
  config.policy.tracking.initial_position_std_m = 1500.0f;
  config.policy.tracking.initial_velocity_std_m_per_s = 80.0f;
  config.policy.tracking.nis_gate_loss_cycles = 2U;
  config.policy.tracking.nfov_tracking_gate_loss_cycles = 4U;
  config.environment.weather_type = SbirsWeatherType::kRain;
  config.environment.sea_state = SbirsSeaState::kMedium;
  config.environment.temperature_c = 25.0f;
  config.environment.base_atmospheric_transmittance = 0.7f;

  SbirsSessionConfig decoded;
  ASSERT_TRUE(DecodeSbirsSessionConfig(EncodeSbirsSessionConfig(config), &decoded));
  EXPECT_FLOAT_EQ(decoded.hardware.optical_aperture_m, 0.9f);
  EXPECT_FLOAT_EQ(decoded.hardware.noise_equivalent_power_w, 2.0e-12f);
  EXPECT_EQ(decoded.mission.work_mode, SbirsWorkMode::kWideSearch);
  EXPECT_TRUE(decoded.mission.sensor_enabled);
  EXPECT_FLOAT_EQ(decoded.mission.scan_rate_deg_per_sec, 3.0f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_max_slew_rate_deg_per_sec, 17.5f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_settle_tolerance_deg, 0.025f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.narrow_min_snr_linear, 7.0f);
  EXPECT_FLOAT_EQ(decoded.policy.error_model.angular_sigma_deg, 0.08f);
  EXPECT_EQ(decoded.policy.error_model.random_seed, 42U);
  EXPECT_EQ(decoded.policy.scheduler.max_concurrent_nfov_locks, 3);
  EXPECT_TRUE(decoded.policy.tracking.enable_estimated_tracking);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.process_noise_diff_coeff, 2.5f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.initial_position_std_m, 1500.0f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.initial_velocity_std_m_per_s, 80.0f);
  EXPECT_EQ(decoded.policy.tracking.nis_gate_loss_cycles, 2U);
  EXPECT_EQ(decoded.policy.tracking.nfov_tracking_gate_loss_cycles, 4U);
  EXPECT_EQ(decoded.environment.weather_type, SbirsWeatherType::kRain);
  EXPECT_EQ(decoded.environment.sea_state, SbirsSeaState::kMedium);
  EXPECT_FLOAT_EQ(decoded.environment.base_atmospheric_transmittance, 0.7f);
}

// --- RuntimeConfigPatch ---

TEST(SbirsReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  SbirsMissionConfig mission;
  mission.work_mode = SbirsWorkMode::kStandby;
  mission.narrow_pointing_max_slew_rate_deg_per_sec = 12.0f;
  mission.narrow_pointing_settle_tolerance_deg = 0.03f;
  SbirsPolicyConfig policy;
  policy.detection.wide_min_snr_linear = 5.0f;
  policy.tracking.nis_gate_loss_cycles = 3U;
  policy.tracking.nfov_tracking_gate_loss_cycles = 5U;
  SbirsEnvironmentConfig environment;
  environment.weather_type = SbirsWeatherType::kCloudy;

  const SbirsRuntimeConfigPatch patch = SbirsRuntimeConfigBuilder()
                                            .WithMission(mission)
                                            .WithPolicy(policy)
                                            .WithEnvironment(environment)
                                            .WithWorkMode(SbirsWorkMode::kWideSearch)
                                            .WithScanRateDegPerSec(4.0f)
                                            .WithSensorEnabled(false)
                                            .Build();
  SbirsRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeSbirsRuntimeConfigPatch(EncodeSbirsRuntimeConfigPatch(patch), &decoded));

  EXPECT_TRUE(decoded.has_mission);
  EXPECT_EQ(decoded.mission.work_mode, SbirsWorkMode::kStandby);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_max_slew_rate_deg_per_sec, 12.0f);
  EXPECT_FLOAT_EQ(decoded.mission.narrow_pointing_settle_tolerance_deg, 0.03f);
  EXPECT_TRUE(decoded.has_policy);
  EXPECT_FLOAT_EQ(decoded.policy.detection.wide_min_snr_linear, 5.0f);
  EXPECT_EQ(decoded.policy.tracking.nis_gate_loss_cycles, 3U);
  EXPECT_EQ(decoded.policy.tracking.nfov_tracking_gate_loss_cycles, 5U);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_EQ(decoded.environment.weather_type, SbirsWeatherType::kCloudy);
  EXPECT_TRUE(decoded.has_work_mode);
  EXPECT_EQ(decoded.work_mode, SbirsWorkMode::kWideSearch);
  EXPECT_TRUE(decoded.has_scan_rate_deg_per_sec);
  EXPECT_FLOAT_EQ(decoded.scan_rate_deg_per_sec, 4.0f);
  EXPECT_TRUE(decoded.has_sensor_enabled);
  EXPECT_FALSE(decoded.sensor_enabled);
}

TEST(SbirsReplayCodecRoundtripTest, EmptyRuntimeConfigPatchKeepsAllFlagsFalse) {
  const SbirsRuntimeConfigPatch patch;  // 默认构造：全部 has_* 为 false
  SbirsRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeSbirsRuntimeConfigPatch(EncodeSbirsRuntimeConfigPatch(patch), &decoded));
  EXPECT_FALSE(decoded.has_mission);
  EXPECT_FALSE(decoded.has_policy);
  EXPECT_FALSE(decoded.has_environment);
  EXPECT_FALSE(decoded.has_work_mode);
  EXPECT_FALSE(decoded.has_scan_rate_deg_per_sec);
  EXPECT_FALSE(decoded.has_sensor_enabled);
}

// --- FailureMarker ---

TEST(SbirsReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "E_TIMEOUT";
  failure.message = "narrow cue exceeded latency budget";
  failure.location = "SbirsSession::ExecuteCycle";
  failure.has_cycle_index = true;
  failure.cycle_index = 88U;
  failure.has_sim_time_sec = true;
  failure.sim_time_sec = 12.5;
  failure.diagnostics_payload = "{\"retry\":3}";

  const std::string bytes = EncodeSbirsFailureMarker(failure);
  ASSERT_FALSE(bytes.empty());

  oneq::replay::ReplayTraceFailure decoded;
  std::string error;
  ASSERT_TRUE(DecodeSbirsFailureMarker(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.error_code, "E_TIMEOUT");
  EXPECT_EQ(decoded.message, "narrow cue exceeded latency budget");
  EXPECT_EQ(decoded.location, "SbirsSession::ExecuteCycle");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 88U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 12.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"retry\":3}");
}

// --- 负路径：null out / 空 payload / 损坏 payload ---

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleInputRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsCycleInput("", nullptr));
  SbirsCycleInput input;
  EXPECT_FALSE(DecodeSbirsCycleInput("corrupt", &input));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeOutputFrameRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsOutputFrame("", nullptr));
  SbirsOutputFrame frame;
  EXPECT_FALSE(DecodeSbirsOutputFrame("bad", &frame));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeCycleResultRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsCycleResult("", nullptr));
  SbirsCycleResult result;
  EXPECT_FALSE(DecodeSbirsCycleResult("bad", &result));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeSessionConfigRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsSessionConfig("", nullptr));
  SbirsSessionConfig config;
  EXPECT_FALSE(DecodeSbirsSessionConfig("bad", &config));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeRuntimeConfigPatchRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeSbirsRuntimeConfigPatch("", nullptr));
  SbirsRuntimeConfigPatch patch;
  EXPECT_FALSE(DecodeSbirsRuntimeConfigPatch("bad", &patch));
}

TEST(SbirsReplayCodecRoundtripTest, DecodeFailureMarkerRejectsNullAndCorrupted) {
  std::string error;
  // null out
  EXPECT_FALSE(DecodeSbirsFailureMarker("", nullptr, &error));
  // 空 payload
  oneq::replay::ReplayTraceFailure failure;
  EXPECT_FALSE(DecodeSbirsFailureMarker("", &failure, &error));
  // 损坏 payload
  EXPECT_FALSE(DecodeSbirsFailureMarker("bad", &failure, &error));
}

TEST(SbirsReplayCodecRoundtripTest, SessionConfigPreservesImmTrackingFields) {
  SbirsSessionConfig config;
  config.policy.tracking.enable_imm_tracking = true;
  config.policy.tracking.imm_model_noise_diff_coeffs = {0.5f, 80.0f, 200.0f};

  const std::string encoded = EncodeSbirsSessionConfig(config);
  SbirsSessionConfig decoded;
  ASSERT_TRUE(DecodeSbirsSessionConfig(encoded, &decoded));

  EXPECT_TRUE(decoded.policy.tracking.enable_imm_tracking);
  ASSERT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs.size(), 3U);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs[0], 0.5f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs[1], 80.0f);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.imm_model_noise_diff_coeffs[2], 200.0f);
}

}  // namespace tests
}  // namespace session
}  // namespace sbirs_sensor
