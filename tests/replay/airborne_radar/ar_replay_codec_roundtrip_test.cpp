/**
 * @file ar_replay_codec_roundtrip_test.cpp
 * @brief 验证 AR replay FlatBuffers codec 各 payload 的 Encode→Decode round-trip 字段精确保真。
 *
 * 每个测试独立覆盖一种 payload 类型，确保新增字段未同步到 schema/codec 时立即被发现。
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/replay/ReplayTrace.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace tests {

// ---------------------------------------------------------------------------
// config::ArSessionConfig（含 environment 域）
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::ArSessionConfig config;
  config.hardware.transmitter.equipment_id = 41U;
  config.hardware.transmitter.maximum_peak_power_w = 2.5e6f;
  config.hardware.transmitter.maximum_duty_cycle = 0.08f;
  config.hardware.transmitter.maximum_pulse_energy_j = 18.0f;
  config.hardware.transmitter.frequency_plan_hz = {8.0e9, 8.2e9, 8.4e9};
  config.hardware.receiver.equipment_id = 42U;
  config.hardware.receiver.polarization =
      oneq::electromagnetics::RfPolarization::kVertical;
  config.hardware.receiver.cross_polarization_isolation_db = 27.0f;
  config.hardware.receiver.minimum_far_field_range_m = 3.0f;
  config.hardware.receiver.has_co_site_isolation = true;
  config.hardware.receiver.co_site_isolation_db = 111.0f;
  config.hardware.receiver.maximum_linear_input_power_w = 2.0e-4f;
  config.hardware.receiver.preselector_bandwidth_hz = 32.0e6f;
  config.hardware.receiver.interference_observation_jn_gate_db = 8.5f;
  config.hardware.receiver.scene_polarization =
      oneq::electromagnetics::RfScenePolarization::kVertical;
  config.hardware.receiver.co_site_paths = {{41U, 42U, 117.0}, {43U, 42U, 104.0}};
  // hardware
  config.hardware.transmitter.peak_power_w = 50000.0f;
  config.hardware.transmitter.frequency_hz = 9.5e9f;
  config.hardware.antenna.antenna_length_m = 1.75f;
  config.hardware.antenna.antenna_width_m = 0.85f;
  // mission
  config.mission.orientation.scan_center_deg.az_deg = 15.0f;
  config.mission.orientation.scan_center_deg.el_deg = -2.0f;
  // policy
  config.policy.lifecycle.confirm_hits = 2U;
  config.policy.detection.minimum_detection_margin_db = -20.0f;
  config.policy.detection.minimum_snr_db = -8.0f;
  config.policy.detection.pfa = 2.0e-6f;
  config.policy.detection.pulse_count = 24;
  config.policy.lifecycle.max_miss_before_lost = 3U;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.tracking.kalman_measurement_noise_std = 5.5f;
  config.policy.decision_control.lpi_hold_cycles_after_request = 2U;
  config.policy.decision_control.eccm_hold_cycles_after_request = 3U;
  config.policy.decision_control.lpi_cooldown_cycles_after_release = 4U;
  config.policy.decision_control.eccm_cooldown_cycles_after_release = 5U;
  // natural environment
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.pressure_hpa = 1010.0f;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 290.0f;
  config.environment.scenario_config.atmospheric_physics.relative_humidity = 0.6f;
  config.environment.scenario_config.vegetation_scatter_physics.enable_physical_model = true;
  config.environment.scenario_config.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kSparseWoodland;
  const std::string bytes = EncodeSessionConfigFlatbuffer(config);
  ASSERT_FALSE(bytes.empty());

  config::ArSessionConfig decoded;
  std::string error;
  ASSERT_TRUE(DecodeSessionConfigFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.hardware.transmitter.equipment_id, 41U);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.maximum_peak_power_w, 2.5e6f);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.maximum_duty_cycle, 0.08f);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.maximum_pulse_energy_j, 18.0f);
  EXPECT_EQ(decoded.hardware.transmitter.frequency_plan_hz,
            config.hardware.transmitter.frequency_plan_hz);
  EXPECT_EQ(decoded.hardware.receiver.equipment_id, 42U);
  EXPECT_EQ(decoded.hardware.receiver.polarization,
            oneq::electromagnetics::RfPolarization::kVertical);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver.maximum_linear_input_power_w, 2.0e-4f);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver.preselector_bandwidth_hz, 32.0e6f);
  EXPECT_EQ(decoded.hardware.receiver.scene_polarization,
            oneq::electromagnetics::RfScenePolarization::kVertical);
  ASSERT_EQ(decoded.hardware.receiver.co_site_paths.size(), 2U);
  EXPECT_EQ(decoded.hardware.receiver.co_site_paths[0].transmitter_equipment_id, 41U);
  EXPECT_EQ(decoded.hardware.receiver.co_site_paths[0].receiver_equipment_id, 42U);
  EXPECT_DOUBLE_EQ(decoded.hardware.receiver.co_site_paths[0].isolation_db, 117.0);

  // hardware
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.peak_power_w, 50000.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.transmitter.frequency_hz, 9.5e9f);
  EXPECT_FLOAT_EQ(decoded.hardware.antenna.antenna_length_m, 1.75f);
  EXPECT_FLOAT_EQ(decoded.hardware.antenna.antenna_width_m, 0.85f);
  // mission
  EXPECT_FLOAT_EQ(decoded.mission.orientation.scan_center_deg.az_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.orientation.scan_center_deg.el_deg, -2.0f);
  // policy
  EXPECT_EQ(decoded.policy.lifecycle.confirm_hits, 2U);
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_detection_margin_db, -20.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_snr_db, -8.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.pfa, 2.0e-6f);
  EXPECT_EQ(decoded.policy.detection.pulse_count, 24);
  EXPECT_EQ(decoded.policy.lifecycle.max_miss_before_lost, 3U);
  EXPECT_TRUE(decoded.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.kalman_measurement_noise_std, 5.5f);
  EXPECT_EQ(decoded.policy.decision_control.lpi_hold_cycles_after_request, 2U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_hold_cycles_after_request, 3U);
  EXPECT_EQ(decoded.policy.decision_control.lpi_cooldown_cycles_after_release, 4U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_cooldown_cycles_after_release, 5U);
  // natural environment
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa, 1010.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.temperature_k, 290.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.relative_humidity, 0.6f);
  EXPECT_TRUE(decoded.environment.scenario_config.vegetation_scatter_physics.enable_physical_model);
  EXPECT_EQ(decoded.environment.scenario_config.vegetation_scatter_physics.cover_profile,
            config::VegetationCoverProfile::kSparseWoodland);
}

// ---------------------------------------------------------------------------
// ArRuntimeConfigPatch
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  config::ArRuntimeConfigPatch patch;
  patch.has_policy = true;
  patch.policy.tracking.enable_kalman_filter = true;
  patch.policy.tracking.kalman_measurement_noise_std = 7.5f;
  patch.policy.lifecycle.confirm_hits = 1U;
  patch.policy.decision_control.lpi_hold_cycles_after_request = 6U;
  patch.policy.decision_control.eccm_hold_cycles_after_request = 7U;
  patch.policy.decision_control.lpi_cooldown_cycles_after_release = 8U;
  patch.policy.decision_control.eccm_cooldown_cycles_after_release = 9U;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = 30.0f;
  patch.scan_center_deg.el_deg = -3.0f;
  patch.has_dwell_center_deg = true;
  patch.dwell_center_deg.az_deg = 31.0f;
  patch.dwell_center_deg.el_deg = -3.5f;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_environment = true;
  patch.environment.has_scenario_config = true;
  patch.environment.scenario_config.atmospheric_physics.relative_humidity = 0.45f;

  const std::string bytes = EncodeRuntimeConfigPatchFlatbuffer(patch);
  ASSERT_FALSE(bytes.empty());

  config::ArRuntimeConfigPatch decoded;
  std::string error;
  ASSERT_TRUE(DecodeRuntimeConfigPatchFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_TRUE(decoded.has_policy);
  EXPECT_TRUE(decoded.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.kalman_measurement_noise_std, 7.5f);
  EXPECT_EQ(decoded.policy.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(decoded.policy.decision_control.lpi_hold_cycles_after_request, 6U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_hold_cycles_after_request, 7U);
  EXPECT_EQ(decoded.policy.decision_control.lpi_cooldown_cycles_after_release, 8U);
  EXPECT_EQ(decoded.policy.decision_control.eccm_cooldown_cycles_after_release, 9U);
  EXPECT_TRUE(decoded.has_scan_center_deg);
  EXPECT_FLOAT_EQ(decoded.scan_center_deg.az_deg, 30.0f);
  EXPECT_FLOAT_EQ(decoded.scan_center_deg.el_deg, -3.0f);
  EXPECT_TRUE(decoded.has_dwell_center_deg);
  EXPECT_FLOAT_EQ(decoded.dwell_center_deg.az_deg, 31.0f);
  EXPECT_TRUE(decoded.has_commanded_beamwidth_enabled);
  EXPECT_TRUE(decoded.commanded_beamwidth_enabled);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_TRUE(decoded.environment.has_scenario_config);
  EXPECT_FLOAT_EQ(
      decoded.environment.scenario_config.atmospheric_physics.relative_humidity,
      0.45f);
}

// ---------------------------------------------------------------------------
// FailureMarker
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "AR_ASSERT";
  failure.message = "track pool overflow";
  failure.location = "ArController::RunOnce";
  failure.has_cycle_index = true;
  failure.cycle_index = 42U;
  failure.has_sim_time_sec = true;
  failure.sim_time_sec = 42.5;
  failure.diagnostics_payload = "{\"track_count\":128}";

  const std::string bytes = EncodeFailureMarkerFlatbuffer(failure, true, 99U);
  ASSERT_FALSE(bytes.empty());

  oneq::replay::ReplayTraceFailure decoded;
  std::string error;
  ASSERT_TRUE(DecodeFailureMarkerFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.error_code, "AR_ASSERT");
  EXPECT_EQ(decoded.message, "track pool overflow");
  EXPECT_EQ(decoded.location, "ArController::RunOnce");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 42U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 42.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"track_count\":128}");
}

TEST(ArReplayCodecRoundtripTest, SingleCycleInputPreservesWorldAndRfFacts) {
  ArCycleInput input;
  input.cycle_index = 71U;
  input.cycle_start_time_s = 12.5;
  input.dt_sec = 0.25;
  input.platform.platform_entity_id = 901U;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  input.platform.platform_velocity_mps.y_mps = 210.0;
  input.platform.platform_attitude_deg.yaw_deg = 17.25;
  input.platform.radar_mount_angles_deg.pitch_deg = -2.5;
  ArTargetInput target;
  target.target_id = 123U;
  target.target_name = "replay-target";
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  target.kinematics.position_lla_deg_m.latitude_deg = 31.2;
  target.kinematics.position_lla_deg_m.longitude_deg = 121.5;
  target.kinematics.position_lla_deg_m.altitude_m = 8000.0;
  target.kinematics.velocity_mps.z_mps = 8.0;
  target.rcs = 3.5f;
  target.swerling_type = 2;
  input.targets.push_back(target);
  input.environment.atmospheric_observation.temperature_k = 284.0f;
  input.interference.world_cycle_index = 71U;
  input.interference.window_start_time_s = 12.5;
  input.interference.window_duration_s = 0.25;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = {700U, 1U, 33U};
  emission.position_ecef_m.x_m = 6388137.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
      12.5, 0.25, 8.0e9, 10.0e9, 1.0e6, 1000.0, 0.1,
      &emission.waveform));
  input.interference.emissions.push_back(emission);

  ArCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(EncodeCycleInputFlatbuffer(input),
                                         &decoded, &error))
      << error;
  EXPECT_EQ(decoded.cycle_index, 71U);
  EXPECT_DOUBLE_EQ(decoded.cycle_start_time_s, 12.5);
  EXPECT_DOUBLE_EQ(decoded.platform.platform_attitude_deg.yaw_deg, 17.25);
  EXPECT_DOUBLE_EQ(decoded.platform.radar_mount_angles_deg.pitch_deg, -2.5);
  ASSERT_EQ(decoded.targets.size(), 1U);
  EXPECT_EQ(decoded.targets.front().target_name, "replay-target");
  EXPECT_EQ(decoded.targets.front().kinematics.position_frame,
            oneq::coordinate::PositionFrame::kLla);
  EXPECT_DOUBLE_EQ(
      decoded.targets.front().kinematics.position_lla_deg_m.longitude_deg,
      121.5);
  ASSERT_EQ(decoded.interference.emissions.size(), 1U);
  EXPECT_EQ(decoded.interference.emissions.front().waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep);
}

TEST(ArReplayCodecRoundtripTest, SingleCycleRecordPreservesResultAndState) {
  ArCycleReplayRecord record;
  record.result.input_cycle_index = 81U;
  record.result.status = ArCycleStatus::kCompleted;
  record.result.track_output_frame.cycle_index = 81U;
  record.result.emission_frame.world_cycle_index = 81U;
  ArInterferenceObservation observation;
  observation.observation_id = 91U;
  observation.jammer_to_noise_db = 14.0;
  record.result.interference_observations.push_back(observation);
  record.result.receiver_impairment = ArReceiverImpairment::kSaturated;
  record.result.submitted_commands.push_back(
      ArCommand(ArCommandType::SET_AGILITY_FREQ, ArCommandSource::ECCM));
  ValidationIssue issue;
  issue.severity = ValidationSeverity::kWarning;
  issue.code = ValidationCode::kNegativeRcs;
  issue.location.kind = ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = 3U;
  issue.field = "rcs";
  issue.message = "negative rcs";
  record.result.validation_issues.push_back(issue);
  record.result.has_control_profile = true;
  record.result.control_profile.version = 4U;
  record.result.association_quality_metrics.matched_count = 5U;
  record.result.association_quality_metrics.match_rate = 0.75f;
  record.result.has_decision_observation = true;
  record.result.decision_observation.input_frame.cycle_index = 81U;
  record.result.decision_observation.input_frame.batch_id = 82U;
  record.result.applied_decision_source = DecisionControlSource::kExternal;
  record.session_state.has_world_chronology = true;
  record.session_state.last_world_window_end_s = 21.5;
  record.session_state.next_emission_id = 34U;
  record.session_state.successful_prepare_count = 7U;
  record.session_state.frequency_hop_index = 2U;
  record.session_state.has_pending_runtime_update = true;
  record.session_state.decision_state.has_pending_external_decision = true;

  ArCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleReplayRecordFlatbuffer(
      EncodeCycleReplayRecordFlatbuffer(record), &decoded, &error))
      << error;
  EXPECT_EQ(decoded.result.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(decoded.result.receiver_impairment,
            ArReceiverImpairment::kSaturated);
  ASSERT_EQ(decoded.result.interference_observations.size(), 1U);
  EXPECT_DOUBLE_EQ(
      decoded.result.interference_observations.front().jammer_to_noise_db,
      14.0);
  ASSERT_EQ(decoded.result.submitted_commands.size(), 1U);
  EXPECT_EQ(decoded.result.submitted_commands.front().type,
            ArCommandType::SET_AGILITY_FREQ);
  ASSERT_EQ(decoded.result.validation_issues.size(), 1U);
  EXPECT_EQ(decoded.result.validation_issues.front().field, "rcs");
  EXPECT_EQ(decoded.result.association_quality_metrics.matched_count, 5U);
  EXPECT_TRUE(decoded.result.has_decision_observation);
  EXPECT_EQ(decoded.result.decision_observation.input_frame.batch_id, 82U);
  EXPECT_EQ(decoded.session_state.next_emission_id, 34U);
  EXPECT_TRUE(decoded.session_state.has_pending_runtime_update);
  EXPECT_TRUE(
      decoded.session_state.decision_state.has_pending_external_decision);
}

TEST(ArReplayCodecRoundtripTest, AttemptsPreserveRejectedRuntimeAndDecisionResults) {
  config::ArRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  config::ArRuntimeConfigPatch decoded_patch;
  bool accepted = true;
  std::string error;
  ASSERT_TRUE(DecodeRuntimeConfigAttemptFlatbuffer(
      EncodeRuntimeConfigAttemptFlatbuffer(patch, false), &decoded_patch, &accepted, &error))
      << error;
  EXPECT_TRUE(decoded_patch.has_sensor_enabled);
  EXPECT_FALSE(decoded_patch.sensor_enabled);
  EXPECT_FALSE(accepted);

  ExternalDecisionResponse response;
  response.source_cycle_index = 44U;
  response.source_batch_id = 55U;
  ExternalDecisionResponse decoded_response;
  ExternalDecisionSubmitStatus decoded_status = ExternalDecisionSubmitStatus::kAccepted;
  ASSERT_TRUE(DecodeExternalDecisionAttemptFlatbuffer(
      EncodeExternalDecisionAttemptFlatbuffer(response,
                                              ExternalDecisionSubmitStatus::kSourceMismatch),
      &decoded_response, &decoded_status, &error))
      << error;
  EXPECT_EQ(decoded_response.source_cycle_index, 44U);
  EXPECT_EQ(decoded_status, ExternalDecisionSubmitStatus::kSourceMismatch);
}

// ===========================================================================
// Decode 失败路径（null output / 空 payload / 损坏 payload）
// ===========================================================================

TEST(ArReplayCodecRoundtripTest, DecodeSessionConfigRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeSessionConfigFlatbuffer("", nullptr, &error));
  config::ArSessionConfig config;
  EXPECT_FALSE(DecodeSessionConfigFlatbuffer("bad", &config, &error));
}

TEST(ArReplayCodecRoundtripTest, DecodeRuntimeConfigPatchRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeRuntimeConfigPatchFlatbuffer("", nullptr, &error));
  config::ArRuntimeConfigPatch patch;
  EXPECT_FALSE(DecodeRuntimeConfigPatchFlatbuffer("bad", &patch, &error));
}

TEST(ArReplayCodecRoundtripTest, DecodeFailureMarkerRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeFailureMarkerFlatbuffer("", nullptr, &error));
  oneq::replay::ReplayTraceFailure failure;
  EXPECT_FALSE(DecodeFailureMarkerFlatbuffer("bad", &failure, &error));
}

}  // namespace tests
}  // namespace session
}  // namespace airborne_radar
