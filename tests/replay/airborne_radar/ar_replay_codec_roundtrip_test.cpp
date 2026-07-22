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
#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/replay/ReplayTrace.h"
#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace tests {

// ---------------------------------------------------------------------------
// ArCycleInput
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  ArCycleInput input;
  input.cycle_index = 7U;
  input.dt_sec = 0.5f;
  input.platform_altitude_m = 1200.0f;
  input.platform_entity_id = 77U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.platform_velocity_ecef_mps.y_mps = 150.0;
  input.platform_pose.position_m.x = 100.0f;
  input.platform_pose.position_m.y = 200.0f;
  input.platform_pose.position_m.z = 300.0f;
  input.platform_pose.velocity_mps.x = 10.0f;
  input.platform_pose.velocity_mps.y = 20.0f;
  input.platform_pose.velocity_mps.z = 5.0f;
  input.platform_pose.attitude_deg.yaw_deg = 45.0f;
  input.platform_pose.attitude_deg.pitch_deg = -5.0f;
  input.platform_pose.attitude_deg.roll_deg = 2.0f;
  input.has_environment = true;
  input.environment.atmospheric_observation.enable_physical_model = true;
  input.environment.atmospheric_observation.temperature_k = 300.0f;

  ArSceneTarget target;
  target.external_target_id = 42U;
  target.velocity_x = 80.0f;
  target.velocity_y = 1.5f;
  target.velocity_z = -0.5f;
  target.rcs = 3.0f;
  target.range_m = 1234.5f;
  target.position_x = 1234.0f;
  target.position_y = 56.0f;
  target.position_z = 78.0f;
  target.target_swerling_type = 2;
  input.scene.push_back(target);

  const std::string bytes = EncodeCycleInputFlatbuffer(input);
  ASSERT_FALSE(bytes.empty());

  ArCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.cycle_index, 7U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, input.dt_sec);
  EXPECT_FLOAT_EQ(decoded.platform_altitude_m, input.platform_altitude_m);
  EXPECT_EQ(decoded.platform_entity_id, 77U);
  EXPECT_TRUE(decoded.has_platform_ecef_kinematics);
  EXPECT_DOUBLE_EQ(decoded.platform_position_ecef_m.x_m, 6378137.0);
  EXPECT_FLOAT_EQ(decoded.platform_pose.position_m.x, 100.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.velocity_mps.y, 20.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.attitude_deg.yaw_deg, 45.0f);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_TRUE(decoded.environment.atmospheric_observation.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.atmospheric_observation.temperature_k, 300.0f);
  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].external_target_id, 42U);
  EXPECT_FLOAT_EQ(decoded.scene[0].rcs, 3.0f);
  EXPECT_FLOAT_EQ(decoded.scene[0].range_m, 1234.5f);
  EXPECT_FLOAT_EQ(decoded.scene[0].position_x, 1234.0f);
  EXPECT_EQ(decoded.scene[0].target_swerling_type, 2);
}

TEST(ArReplayCodecRoundtripTest, CycleInputPreservesEngineeringRfInterference) {
  ArCycleInput input;
  input.dt_sec = 1.0f;
  input.has_environment = true;
  input.environment.interference.mode =
      oneq::electromagnetics::RfInterferenceMode::kEngineering;
  oneq::electromagnetics::RfEmission emission;
  emission.emission_id = 10U;
  emission.entity_id = 20U;
  emission.position_ecef_m.x_m = 6379137.0;
  emission.antenna.peak_gain_dbi = 12.0;
  emission.waveform_kind = oneq::electromagnetics::RfWaveformKind::kSwept;
  oneq::electromagnetics::RfEmissionSegment segment;
  segment.duration_s = 1.0;
  segment.center_frequency_hz = 10.0e9;
  segment.bandwidth_hz = 20.0e6;
  segment.transmit_power_w = 500.0;
  emission.segments.push_back(segment);
  input.environment.interference.engineering_emissions.push_back(emission);

  ArCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(EncodeCycleInputFlatbuffer(input), &decoded, &error))
      << error;
  EXPECT_EQ(decoded.environment.interference.mode,
            oneq::electromagnetics::RfInterferenceMode::kEngineering);
  ASSERT_EQ(decoded.environment.interference.engineering_emissions.size(), 1U);
  EXPECT_EQ(decoded.environment.interference.engineering_emissions.front().emission_id, 10U);
  EXPECT_DOUBLE_EQ(decoded.environment.interference.engineering_emissions.front()
                       .segments.front()
                       .transmit_power_w,
                   500.0);
}

TEST(ArReplayCodecRoundtripTest, CycleInputDecodesEmptyTargetList) {
  ArCycleInput input;
  input.dt_sec = 1.0f;

  const std::string bytes = EncodeCycleInputFlatbuffer(input);
  ArCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(bytes, &decoded, &error)) << error;
  EXPECT_TRUE(decoded.scene.empty());
}

TEST(ArReplayCodecRoundtripTest, CycleInputRejectsEmptyPayload) {
  ArCycleInput decoded;
  std::string error;
  EXPECT_FALSE(DecodeCycleInputFlatbuffer("", &decoded, &error));
  EXPECT_FALSE(error.empty());
}

// ---------------------------------------------------------------------------
// TrackOutputFrame (深度 per-track 字段)
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, TrackOutputFramePreservesAllFields) {
  session::TrackOutputFrame frame;
  frame.cycle_index = 77U;
  frame.batch_id = 5U;

  session::TrackStateSnapshot snap;
  snap.association_key = 999U;
  snap.external_target_id = 42U;
  snap.status = session::TrackStatus::kConfirmed;
  snap.position_x = 100.0f;
  snap.position_y = 200.0f;
  snap.position_z = 50.0f;
  snap.velocity_x = 30.0f;
  snap.velocity_y = 5.0f;
  snap.velocity_z = -2.0f;
  snap.speed = 30.5f;
  snap.acceleration_x = 1.0f;
  snap.acceleration_y = 0.5f;
  snap.acceleration_z = 0.1f;
  snap.acceleration = 1.12f;
  snap.rcs = 2.5f;
  snap.hit_count = 5U;
  snap.miss_count = 1U;
  frame.tracks.push_back(snap);
  session::TrackStateSnapshot lost = snap;
  lost.association_key = 1001U;
  lost.status = session::TrackStatus::kLost;
  frame.tracks.push_back(lost);

  const std::string bytes = EncodeTrackOutputFrameFlatbuffer(frame);
  ASSERT_FALSE(bytes.empty());

  session::TrackOutputFrame decoded;
  std::string error;
  ASSERT_TRUE(DecodeTrackOutputFrameFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.cycle_index, 77U);
  EXPECT_EQ(decoded.batch_id, 5U);
  EXPECT_EQ(decoded.tracks.size(), 2U);
  EXPECT_EQ(session::CountTracksByStatus(decoded, session::TrackStatus::kConfirmed), 1U);
  EXPECT_TRUE(session::CountTracksByStatus(decoded, session::TrackStatus::kLost) > 0U);
  ASSERT_EQ(decoded.tracks.size(), 2U);

  const session::TrackStateSnapshot& ds = decoded.tracks[0];
  EXPECT_EQ(ds.association_key, 999U);
  EXPECT_EQ(ds.external_target_id, 42U);
  EXPECT_EQ(ds.status, session::TrackStatus::kConfirmed);
  EXPECT_FLOAT_EQ(ds.position_x, 100.0f);
  EXPECT_FLOAT_EQ(ds.position_y, 200.0f);
  EXPECT_FLOAT_EQ(ds.position_z, 50.0f);
  EXPECT_FLOAT_EQ(ds.velocity_x, 30.0f);
  EXPECT_FLOAT_EQ(ds.rcs, 2.5f);
  EXPECT_EQ(ds.hit_count, 5U);
  EXPECT_EQ(ds.miss_count, 1U);
}

// ---------------------------------------------------------------------------
// ArCycleResult
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, CycleResultPreservesAllFields) {
  ArReplayCycleRecord record;
  ArCycleResult& result = record.result;
  result.input_cycle_index = 55U;
  result.track_output_frame.cycle_index = 5U;
  result.track_output_frame.batch_id = 3U;

  session::TrackStateSnapshot snap;
  snap.association_key = 1U;
  snap.position_x = 50.0f;
  snap.rcs = 1.5f;
  snap.status = session::TrackStatus::kTentative;
  snap.acceleration_x = 0.4f;
  snap.acceleration_y = 0.5f;
  snap.acceleration_z = 0.6f;
  snap.acceleration = 0.9f;
  snap.target_type = "HIGH_THREAT_FIGHTER";
  snap.target_probability = 0.75f;
  result.track_output_frame.tracks.push_back(snap);

  result.submitted_commands.push_back(
      session::ArCommand(session::ArCommandType::SET_AGILITY_FREQ,
                                       session::ArCommandSource::ECCM));
  ValidationIssue issue;
  issue.severity = ValidationSeverity::kError;
  issue.code = ValidationCode::kInvalidCycleDeltaTime;
  issue.location.kind = ValidationLocationKind::kGlobal;
  issue.field = "dt_sec";
  issue.message = "dt_sec must be positive";
  result.validation_issues.push_back(issue);
  result.has_validation_error = true;
  result.executed_this_cycle = true;
  result.abort_reason = session::SignalCycleAbortReason::kRuntimePreparationFailed;
  result.reused_previous_output = true;
  result.has_control_profile = true;
  result.control_profile.version = 7U;
  result.control_profile.enable_lpi_power_control = true;
  result.control_profile.lpi_power_scale = 0.8f;
  result.control_profile.enable_lpi_beamforming = true;
  result.control_profile.lpi_dwell_scale = 1.2f;
  result.control_profile.enable_agility_frequency = true;
  result.control_profile.agility_frequency_hop_phase = 1U;
  result.control_profile.enable_sidelobe_canceller = true;
  result.control_profile.enable_adaptive_beamforming = true;
  result.control_profile.enable_eccm_rejitter = true;
  result.control_profile.eccm_burnthrough_gain = 1.4f;
  result.association_quality_metrics.prior_track_count = 3U;
  result.association_quality_metrics.detection_count = 4U;
  result.association_quality_metrics.matched_count = 2U;
  result.association_quality_metrics.new_track_count = 1U;
  result.association_quality_metrics.missed_track_count = 1U;
  result.association_quality_metrics.match_rate = 0.5f;
  result.association_quality_metrics.new_track_rate = 0.25f;
  result.association_quality_metrics.missed_track_rate = 0.33f;
  result.association_quality_metrics.mean_match_cost = 1.1f;
  result.association_quality_metrics.p95_match_cost = 2.2f;
  result.association_quality_metrics.dominant_jamming_semantic =
      config::JammingSemantic::kNoiseSuppression;
  result.association_quality_metrics.jamming_severity = 0.6f;
  result.association_quality_metrics.association_stress = 0.7f;
  result.has_decision_observation = true;
  result.decision_observation.input_frame.cycle_index = 54U;
  result.decision_observation.input_frame.batch_id = 12U;
  result.decision_observation.input_frame.environment_jamming_detected = true;
  session::ArInterferenceObservation interference_observation;
  interference_observation.observation_id = 1U;
  interference_observation.estimated_bearing_azimuth_deg = 12.0;
  interference_observation.estimated_bearing_elevation_deg = 3.0;
  interference_observation.estimated_off_boresight_deg = 8.0;
  interference_observation.estimated_center_frequency_hz = 3.1e9;
  interference_observation.estimated_bandwidth_hz = 2.0e6;
  interference_observation.estimated_waveform_kind =
      oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise;
  interference_observation.jammer_to_noise_db = 14.0;
  interference_observation.bearing_standard_deviation_deg = 0.5;
  interference_observation.frequency_standard_deviation_hz = 1000.0;
  interference_observation.bandwidth_standard_deviation_hz = 2000.0;
  result.decision_observation.input_frame.interference_observations.push_back(
      interference_observation);
  result.decision_observation.input_frame.association_quality_info.match_rate = 0.75f;
  result.decision_observation.input_frame.perception_quality_info.input_target_count = 2U;
  result.decision_observation.input_frame.tracks.push_back(snap);
  result.decision_observation.active_control_profile = result.control_profile;
  result.applied_decision_source = session::DecisionControlSource::kExternal;
  result.applied_decision_cycle_index = 54U;
  result.applied_decision_batch_id = 12U;
  record.decision_state.applied_decision_source = session::DecisionControlSource::kExternal;
  record.decision_state.applied_decision_cycle_index = 54U;
  record.decision_state.applied_decision_batch_id = 12U;
  record.decision_state.applied_decision_proposals.push_back(session::TacticalProposal{
      session::ControlDirective(
          session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
          session::ControlDirectiveSource::EMISSION_CONTROL, 0.6f),
      80, "external replay proposal"});
  record.decision_state.has_pending_external_decision = true;
  record.decision_state.pending_external_decision.source_cycle_index = 55U;
  record.decision_state.pending_external_decision.source_batch_id = 13U;
  record.decision_state.has_pending_internal_decision = true;
  record.decision_state.pending_internal_cycle_index = 55U;
  record.decision_state.pending_internal_batch_id = 13U;
  record.decision_state.reducer_state.lpi_hold_cycles_remaining = 2U;
  record.decision_state.reducer_state.eccm_hold_cycles_remaining = 3U;
  record.decision_state.reducer_state.lpi_cooldown_cycles_remaining = 4U;
  record.decision_state.reducer_state.eccm_cooldown_cycles_remaining = 5U;

  const std::string bytes = EncodeReplayCycleRecordFlatbuffer(record);
  ASSERT_FALSE(bytes.empty());

  ArReplayCycleRecord decoded_record;
  std::string error;
  ASSERT_TRUE(DecodeReplayCycleRecordFlatbuffer(bytes, &decoded_record, &error)) << error;
  const ArCycleResult& decoded = decoded_record.result;

  EXPECT_EQ(decoded.input_cycle_index, 55U);
  EXPECT_EQ(decoded.track_output_frame.cycle_index, 5U);
  EXPECT_EQ(decoded.track_output_frame.tracks.size(), 1U);
  EXPECT_EQ(decoded.track_output_frame.batch_id, 3U);
  EXPECT_TRUE(decoded.executed_this_cycle);
  EXPECT_TRUE(decoded.has_validation_error);
  EXPECT_EQ(decoded.abort_reason,
            session::SignalCycleAbortReason::kRuntimePreparationFailed);
  EXPECT_TRUE(decoded.reused_previous_output);
  ASSERT_EQ(decoded.submitted_commands.size(), 1U);
  EXPECT_EQ(decoded.submitted_commands[0].type,
            session::ArCommandType::SET_AGILITY_FREQ);
  EXPECT_EQ(decoded.submitted_commands[0].source, session::ArCommandSource::ECCM);
  ASSERT_EQ(decoded.validation_issues.size(), 1U);
  EXPECT_EQ(decoded.validation_issues[0].severity, ValidationSeverity::kError);
  EXPECT_EQ(decoded.validation_issues[0].code, ValidationCode::kInvalidCycleDeltaTime);
  EXPECT_EQ(decoded.validation_issues[0].location.kind, ValidationLocationKind::kGlobal);
  EXPECT_EQ(decoded.validation_issues[0].field, "dt_sec");
  EXPECT_EQ(decoded.validation_issues[0].message, "dt_sec must be positive");
  EXPECT_TRUE(decoded.has_control_profile);
  EXPECT_EQ(decoded.control_profile.version, 7U);
  EXPECT_TRUE(decoded.control_profile.enable_lpi_power_control);
  EXPECT_FLOAT_EQ(decoded.control_profile.lpi_power_scale, 0.8f);
  EXPECT_TRUE(decoded.control_profile.enable_lpi_beamforming);
  EXPECT_FLOAT_EQ(decoded.control_profile.lpi_dwell_scale, 1.2f);
  EXPECT_TRUE(decoded.control_profile.enable_agility_frequency);
  EXPECT_EQ(decoded.control_profile.agility_frequency_hop_phase, 1U);
  EXPECT_TRUE(decoded.control_profile.enable_sidelobe_canceller);
  EXPECT_TRUE(decoded.control_profile.enable_adaptive_beamforming);
  EXPECT_TRUE(decoded.control_profile.enable_eccm_rejitter);
  EXPECT_FLOAT_EQ(decoded.control_profile.eccm_burnthrough_gain, 1.4f);
  EXPECT_EQ(decoded.association_quality_metrics.prior_track_count, 3U);
  EXPECT_EQ(decoded.association_quality_metrics.detection_count, 4U);
  EXPECT_EQ(decoded.association_quality_metrics.matched_count, 2U);
  EXPECT_EQ(decoded.association_quality_metrics.new_track_count, 1U);
  EXPECT_EQ(decoded.association_quality_metrics.missed_track_count, 1U);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.match_rate, 0.5f);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.new_track_rate, 0.25f);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.missed_track_rate, 0.33f);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.mean_match_cost, 1.1f);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.p95_match_cost, 2.2f);
  EXPECT_EQ(decoded.association_quality_metrics.dominant_jamming_semantic,
            config::JammingSemantic::kNoiseSuppression);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.jamming_severity, 0.6f);
  EXPECT_FLOAT_EQ(decoded.association_quality_metrics.association_stress, 0.7f);
  EXPECT_TRUE(decoded.has_decision_observation);
  EXPECT_EQ(decoded.decision_observation.input_frame.cycle_index, 54U);
  EXPECT_EQ(decoded.decision_observation.input_frame.batch_id, 12U);
  EXPECT_TRUE(decoded.decision_observation.input_frame.environment_jamming_detected);
  ASSERT_EQ(decoded.decision_observation.input_frame.interference_observations.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.decision_observation.input_frame.interference_observations[0]
                       .estimated_off_boresight_deg,
                   8.0);
  EXPECT_EQ(decoded.decision_observation.input_frame.interference_observations[0]
                .estimated_waveform_kind,
            oneq::electromagnetics::RfSceneWaveformKind::kBandLimitedNoise);
  EXPECT_FLOAT_EQ(decoded.decision_observation.input_frame.association_quality_info.match_rate,
                  0.75f);
  EXPECT_EQ(decoded.decision_observation.input_frame.perception_quality_info.input_target_count,
            2U);
  ASSERT_EQ(decoded.decision_observation.input_frame.tracks.size(), 1U);
  EXPECT_EQ(decoded.applied_decision_source, session::DecisionControlSource::kExternal);
  EXPECT_EQ(decoded.applied_decision_cycle_index, 54U);
  EXPECT_EQ(decoded.applied_decision_batch_id, 12U);
  ASSERT_EQ(decoded_record.decision_state.applied_decision_proposals.size(), 1U);
  EXPECT_FLOAT_EQ(decoded_record.decision_state.applied_decision_proposals[0]
                      .directive.requested_value,
                  0.6f);
  EXPECT_EQ(decoded_record.decision_state.applied_decision_proposals[0].rationale,
            "external replay proposal");
  EXPECT_TRUE(decoded_record.decision_state.has_pending_external_decision);
  EXPECT_EQ(decoded_record.decision_state.pending_external_decision.source_cycle_index, 55U);
  EXPECT_EQ(decoded_record.decision_state.pending_external_decision.source_batch_id, 13U);
  EXPECT_TRUE(decoded_record.decision_state.has_pending_internal_decision);
  EXPECT_EQ(decoded_record.decision_state.reducer_state.lpi_hold_cycles_remaining, 2U);
  EXPECT_EQ(decoded_record.decision_state.reducer_state.eccm_hold_cycles_remaining, 3U);
  EXPECT_EQ(decoded_record.decision_state.reducer_state.lpi_cooldown_cycles_remaining, 4U);
  EXPECT_EQ(decoded_record.decision_state.reducer_state.eccm_cooldown_cycles_remaining, 5U);
  ASSERT_EQ(decoded.track_output_frame.tracks.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].position_x, 50.0f);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].rcs, 1.5f);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].acceleration_x, 0.4f);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].acceleration_y, 0.5f);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].acceleration_z, 0.6f);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].acceleration, 0.9f);
  EXPECT_EQ(decoded.track_output_frame.tracks[0].target_type, "HIGH_THREAT_FIGHTER");
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].target_probability, 0.75f);
}

// ---------------------------------------------------------------------------
// config::ArSessionConfig（含 environment 域）
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::ArSessionConfig config;
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
  // jamming_sensitivity_profile
  config.environment.jamming_sensitivity_profile = config::JammingSensitivityProfile::kStrict;
  // environment (previously missing from codec!)
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.pressure_hpa = 1010.0f;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 290.0f;
  config.environment.scenario_config.atmospheric_physics.relative_humidity = 0.6f;
  config.environment.scenario_config.atmospheric_context.has_simulation_unix_seconds = true;
  config.environment.scenario_config.atmospheric_context.simulation_unix_seconds = 1700000000LL;
  config.environment.scenario_config.vegetation_scatter_physics.enable_physical_model = true;
  config.environment.scenario_config.vegetation_scatter_physics.cover_profile =
      config::VegetationCoverProfile::kSparseWoodland;
  config::JammerEmitterState jammer;
  jammer.technique = config::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 25.0f;
  jammer.js_db = 8.0f;
  jammer.position_x = 5000.0f;     // range 10000m * sin(30 deg)
  jammer.position_y = 8660.25f;    // range 10000m * cos(30 deg)
  jammer.position_z = 874.887f;    // range 10000m * tan(5 deg)
  config.environment.scenario_config.jammer_sources.push_back(jammer);

  const std::string bytes = EncodeSessionConfigFlatbuffer(config);
  ASSERT_FALSE(bytes.empty());

  config::ArSessionConfig decoded;
  std::string error;
  ASSERT_TRUE(DecodeSessionConfigFlatbuffer(bytes, &decoded, &error)) << error;

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
  // jamming_sensitivity_profile
  EXPECT_EQ(decoded.environment.jamming_sensitivity_profile, config::JammingSensitivityProfile::kStrict);
  // environment (previously broken!)
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa, 1010.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.temperature_k, 290.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.relative_humidity, 0.6f);
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_context.has_simulation_unix_seconds);
  EXPECT_EQ(decoded.environment.scenario_config.atmospheric_context.simulation_unix_seconds,
            1700000000LL);
  EXPECT_TRUE(decoded.environment.scenario_config.vegetation_scatter_physics.enable_physical_model);
  EXPECT_EQ(decoded.environment.scenario_config.vegetation_scatter_physics.cover_profile,
            config::VegetationCoverProfile::kSparseWoodland);
  ASSERT_EQ(decoded.environment.scenario_config.jammer_sources.size(), 1U);
  EXPECT_EQ(decoded.environment.scenario_config.jammer_sources[0].technique,
            config::JammingTechnique::kNoiseSuppression);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.jammer_sources[0].power_db, 25.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.jammer_sources[0].position_x, 5000.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.jammer_sources[0].position_y, 8660.25f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.jammer_sources[0].position_z, 874.887f);
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
  patch.environment.has_jamming_sensitivity_profile = true;
  patch.environment.jamming_sensitivity_profile =
      config::JammingSensitivityProfile::kStrict;
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
  EXPECT_TRUE(decoded.environment.has_jamming_sensitivity_profile);
  EXPECT_EQ(decoded.environment.jamming_sensitivity_profile,
            config::JammingSensitivityProfile::kStrict);
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

// ===========================================================================
// Decode 失败路径（null output / 空 payload / 损坏 payload）
// ===========================================================================

TEST(ArReplayCodecRoundtripTest, DecodeCycleInputRejectsNullOutput) {
  std::string error;
  EXPECT_FALSE(DecodeCycleInputFlatbuffer("", nullptr, &error));
}

TEST(ArReplayCodecRoundtripTest, DecodeCycleInputRejectsCorruptedPayload) {
  ArCycleInput input;
  std::string error;
  EXPECT_FALSE(DecodeCycleInputFlatbuffer("garbage", &input, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ArReplayCodecRoundtripTest, DecodeTrackOutputFrameRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeTrackOutputFrameFlatbuffer("", nullptr, &error));
  TrackOutputFrame frame;
  EXPECT_FALSE(DecodeTrackOutputFrameFlatbuffer("corrupt", &frame, &error));
}

TEST(ArReplayCodecRoundtripTest, DecodeCycleResultRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeCycleResultFlatbuffer("", nullptr, &error));
  ArCycleResult result;
  EXPECT_FALSE(DecodeCycleResultFlatbuffer("xyz", &result, &error));
}

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
