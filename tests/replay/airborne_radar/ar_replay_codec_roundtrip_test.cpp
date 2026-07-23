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

TEST(ArReplayCodecRoundtripTest, RfV2PreparePayloadPreservesEmissionAndSessionState) {
  ArPrepareCycleInput input;
  input.world_cycle_index = 71U;
  input.window_start_time_s = 12.5;
  input.window_duration_s = 0.25;
  input.platform_id = 901U;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.platform_velocity_ecef_mps.y_mps = 210.0;
  input.radar_frame_attitude_deg.yaw_deg = 17.25;
  input.beam_pointing_deg.az_deg = -4.5f;

  ArPrepareCycleInput decoded_input;
  std::string error;
  ASSERT_TRUE(DecodePrepareCycleInputFlatbuffer(EncodePrepareCycleInputFlatbuffer(input),
                                                &decoded_input, &error))
      << error;
  EXPECT_EQ(decoded_input.world_cycle_index, 71U);
  EXPECT_DOUBLE_EQ(decoded_input.window_start_time_s, 12.5);
  EXPECT_DOUBLE_EQ(decoded_input.radar_frame_attitude_deg.yaw_deg, 17.25);
  EXPECT_FLOAT_EQ(decoded_input.beam_pointing_deg.az_deg, -4.5f);

  ArPrepareReplayRecord record;
  record.result.status = ArPrepareCycleStatus::kPrepared;
  record.result.token = {9U, 71U};
  record.result.has_emission = true;
  record.result.emission.identity = {901U, 11U, 99U};
  record.result.emission.position_ecef_m = input.platform_position_ecef_m;
  record.result.emission.antenna.boresight_ecef.y = 1.0;
  record.result.emission.polarization = oneq::electromagnetics::RfScenePolarization::kVertical;
  record.result.emission.waveform.kind = oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  record.result.emission.waveform.center_frequency_hz = 9.1e9;
  record.result.emission.waveform.timing_seed = 123456U;
  record.result.emission.waveform.timing_epoch = 7U;
  record.result.operating_state.rf_receiver.platform_id = 901U;
  record.result.operating_state.rf_receiver.equipment_id = 12U;
  record.result.operating_state.rf_receiver.co_site_paths = {{11U, 12U, 118.0}};
  record.result.operating_state.matched_filter_bandwidth_hz = 7.5e6;
  record.result.operating_state.adaptive_nulls_ecef.push_back({0.0, 1.0, 0.0});
  record.session_state.has_prepared_cycle = true;
  record.session_state.prepared_token = record.result.token;
  record.session_state.next_token_value = 10U;
  record.session_state.next_emission_id = 100U;
  record.session_state.successful_prepare_count = 8U;
  record.session_state.timing_seed = 123456U;
  record.session_state.frequency_hop_index = 2U;
  record.session_state.has_pending_runtime_update = true;
  record.session_state.decision_state.has_pending_external_decision = true;
  record.session_state.decision_state.pending_external_decision.source_cycle_index = 70U;

  ArPrepareReplayRecord decoded_record;
  ASSERT_TRUE(DecodePrepareReplayRecordFlatbuffer(EncodePrepareReplayRecordFlatbuffer(record),
                                                  &decoded_record, &error))
      << error;
  EXPECT_EQ(decoded_record.result.status, ArPrepareCycleStatus::kPrepared);
  EXPECT_EQ(decoded_record.result.token.value, 9U);
  EXPECT_EQ(decoded_record.result.emission.identity.emission_id, 99U);
  EXPECT_DOUBLE_EQ(decoded_record.result.emission.waveform.center_frequency_hz, 9.1e9);
  ASSERT_EQ(decoded_record.result.operating_state.rf_receiver.co_site_paths.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded_record.result.operating_state.rf_receiver.co_site_paths[0].isolation_db,
                   118.0);
  EXPECT_EQ(decoded_record.session_state.next_emission_id, 100U);
  EXPECT_TRUE(decoded_record.session_state.has_pending_runtime_update);
  EXPECT_TRUE(decoded_record.session_state.decision_state.has_pending_external_decision);
}

TEST(ArReplayCodecRoundtripTest, RfV2CompleteAndAbandonPayloadsPreserveProtocolState) {
  ArCompleteReplayOperationInput operation;
  operation.token = {5U, 81U};
  operation.input.rf_scene.world_cycle_index = 81U;
  operation.input.rf_scene.window_start_time_s = 21.0;
  operation.input.rf_scene.window_duration_s = 0.5;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = {700U, 1U, 33U};
  emission.waveform.kind = oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep;
  emission.waveform.sweep_start_frequency_hz = 8.0e9;
  emission.waveform.sweep_stop_frequency_hz = 10.0e9;
  operation.input.rf_scene.emissions.push_back(emission);
  ArSceneTarget target;
  target.external_target_id = 123U;
  target.target_name = "replay-target";
  operation.input.targets.push_back(target);
  operation.input.atmospheric_observation.temperature_k = 284.0f;

  std::string error;
  ArCompleteReplayOperationInput decoded_operation;
  ASSERT_TRUE(DecodeCompleteReplayOperationInputFlatbuffer(
      EncodeCompleteReplayOperationInputFlatbuffer(operation), &decoded_operation, &error))
      << error;
  EXPECT_EQ(decoded_operation.token.world_cycle_index, 81U);
  ASSERT_EQ(decoded_operation.input.rf_scene.emissions.size(), 1U);
  EXPECT_EQ(decoded_operation.input.rf_scene.emissions[0].waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep);
  ASSERT_EQ(decoded_operation.input.targets.size(), 1U);
  EXPECT_EQ(decoded_operation.input.targets[0].target_name, "replay-target");

  ArCompleteReplayRecord record;
  record.result.status = ArCompleteCycleStatus::kCompleted;
  record.result.world_cycle_index = 81U;
  record.result.track_output_frame.cycle_index = 81U;
  ArInterferenceObservation observation;
  observation.observation_id = 91U;
  observation.jammer_to_noise_db = 14.0;
  record.result.interference_observations.push_back(observation);
  record.result.receiver_impairment = ArReceiverImpairment::kSaturated;
  record.result.has_decision_observation = true;
  record.result.decision_observation.input_frame.cycle_index = 81U;
  record.result.decision_observation.input_frame.batch_id = 82U;
  record.session_state.next_emission_id = 34U;

  ArCompleteReplayRecord decoded_record;
  ASSERT_TRUE(DecodeCompleteReplayRecordFlatbuffer(EncodeCompleteReplayRecordFlatbuffer(record),
                                                   &decoded_record, &error))
      << error;
  EXPECT_EQ(decoded_record.result.status, ArCompleteCycleStatus::kCompleted);
  EXPECT_EQ(decoded_record.result.world_cycle_index, 81U);
  EXPECT_EQ(decoded_record.result.receiver_impairment, ArReceiverImpairment::kSaturated);
  ASSERT_TRUE(decoded_record.result.has_decision_observation);
  EXPECT_EQ(decoded_record.result.decision_observation.input_frame.cycle_index, 81U);
  EXPECT_EQ(decoded_record.result.decision_observation.input_frame.batch_id, 82U);
  ASSERT_EQ(decoded_record.result.interference_observations.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded_record.result.interference_observations[0].jammer_to_noise_db, 14.0);

  ArAbandonReplayOperationInput abandon_input;
  abandon_input.token = operation.token;
  ArAbandonReplayOperationInput decoded_abandon_input;
  ASSERT_TRUE(DecodeAbandonReplayOperationInputFlatbuffer(
      EncodeAbandonReplayOperationInputFlatbuffer(abandon_input), &decoded_abandon_input, &error))
      << error;
  EXPECT_EQ(decoded_abandon_input.token.value, 5U);

  ArAbandonReplayRecord abandon_record;
  abandon_record.status = ArAbandonCycleStatus::kAbandoned;
  abandon_record.session_state.next_token_value = 6U;
  ArAbandonReplayRecord decoded_abandon_record;
  ASSERT_TRUE(DecodeAbandonReplayRecordFlatbuffer(
      EncodeAbandonReplayRecordFlatbuffer(abandon_record), &decoded_abandon_record, &error))
      << error;
  EXPECT_EQ(decoded_abandon_record.status, ArAbandonCycleStatus::kAbandoned);
  EXPECT_EQ(decoded_abandon_record.session_state.next_token_value, 6U);
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
