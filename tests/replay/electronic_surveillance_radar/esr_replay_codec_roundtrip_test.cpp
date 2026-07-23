/**
 * @file esr_replay_codec_roundtrip_test.cpp
 * @brief 验证 ESR replay FlatBuffers codec 各 payload 的 Encode→Decode round-trip 字段精确保真。
 *
 * 每个测试独立覆盖一种 payload 类型，确保新增字段未同步到 schema/codec 时立即被发现。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"

namespace electronic_surveillance_radar {
namespace session {
namespace tests {

// ---------------------------------------------------------------------------
// EsrCycleInput
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  EsrCycleInput input;
  input.cycle_index = 3U;
  input.dt_sec = 0.1f;
  input.platform_altitude_m = 5000.0f;
  input.platform_entity_id = 77U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.platform_velocity_ecef_mps.y_mps = 120.0;
  input.platform_pose.position_m.x = 1000.0f;
  input.platform_pose.position_m.y = 2000.0f;
  input.platform_pose.position_m.z = 3000.0f;
  input.platform_pose.velocity_mps.x = 50.0f;
  input.platform_pose.velocity_mps.y = 10.0f;
  input.platform_pose.velocity_mps.z = 0.0f;
  input.platform_pose.attitude_deg.yaw_deg = 90.0f;
  input.platform_pose.attitude_deg.pitch_deg = 0.0f;
  input.platform_pose.attitude_deg.roll_deg = -3.0f;

  EsrSceneEmitter emitter;
  emitter.emitter_id = 1001U;
  emitter.emitter_name = "esr-emitter-alpha";
  emitter.has_ecef_kinematics = true;
  emitter.position_ecef_m.x_m = 6379137.0;
  emitter.velocity_ecef_mps.y_mps = 100.0;
  emitter.carrier_hz = 9.5e9;
  emitter.bandwidth_hz = 5.0e6;
  emitter.tx_power_w = 1000.0;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-3;
  emitter.is_emitting = true;
  emitter.pose.position_m.x = 5000.0f;
  emitter.pose.velocity_mps.y = 30.0f;
  emitter.beam_state.center_az_deg = 15.0;
  emitter.beam_state.center_el_deg = -2.0;
  emitter.beam_state.az_beamwidth_deg = 10.0;
  emitter.beam_state.el_beamwidth_deg = 10.0;
  emitter.beam_state.beam_state_valid = true;
  input.scene.push_back(emitter);

  input.environment.propagation_profile = session::EsrPropagationEnvironmentProfile::kComplex;
  input.environment.clutter_density = session::EsrClutterDensityLevel::kHigh;
  input.environment.spectrum_occupancy_ratio = 0.45f;
  input.environment.atmospheric_observation.relative_humidity_ratio = 0.6f;
  input.environment.atmospheric_observation.precipitation_rate_mmph = 2.5f;
  input.environment.atmospheric_observation.visibility_km = 8.0f;
  input.environment.interference_mode = oneq::electromagnetics::RfInterferenceMode::kLegacy;

  session::EsrJammerSource jammer;
  jammer.technique = session::EsrJammingTechnique::kNoiseSuppression;
  jammer.active = true;
  jammer.center_hz = 9.5e9;
  jammer.bandwidth_hz = 20.0e6;
  jammer.power_w = 100.0f;
  jammer.deception_risk = 0.1f;
  jammer.confidence = 0.9f;
  input.environment.jammer_sources.push_back(jammer);

  const std::string bytes = EncodeEsrCycleInput(input);
  ASSERT_FALSE(bytes.empty());

  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 3U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, 0.1f);
  EXPECT_FLOAT_EQ(decoded.platform_altitude_m, 5000.0f);
  EXPECT_EQ(decoded.platform_entity_id, 77U);
  EXPECT_TRUE(decoded.has_platform_ecef_kinematics);
  EXPECT_DOUBLE_EQ(decoded.platform_position_ecef_m.x_m, 6378137.0);
  EXPECT_FLOAT_EQ(decoded.platform_pose.position_m.x, 1000.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.velocity_mps.y, 10.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.attitude_deg.yaw_deg, 90.0f);

  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].emitter_id, 1001U);
  EXPECT_EQ(decoded.scene[0].emitter_name, "esr-emitter-alpha");
  EXPECT_TRUE(decoded.scene[0].has_ecef_kinematics);
  EXPECT_DOUBLE_EQ(decoded.scene[0].position_ecef_m.x_m, 6379137.0);
  EXPECT_DOUBLE_EQ(decoded.scene[0].carrier_hz, 9.5e9);
  EXPECT_DOUBLE_EQ(decoded.scene[0].pulse_width_s, 1.0e-6);
  EXPECT_TRUE(decoded.scene[0].is_emitting);
  EXPECT_DOUBLE_EQ(decoded.scene[0].beam_state.center_az_deg, 15.0);
  EXPECT_TRUE(decoded.scene[0].beam_state.beam_state_valid);

  EXPECT_EQ(decoded.environment.propagation_profile,
            session::EsrPropagationEnvironmentProfile::kComplex);
  EXPECT_EQ(decoded.environment.clutter_density, session::EsrClutterDensityLevel::kHigh);
  EXPECT_FLOAT_EQ(decoded.environment.spectrum_occupancy_ratio, 0.45f);
  EXPECT_FLOAT_EQ(decoded.environment.atmospheric_observation.relative_humidity_ratio, 0.6f);

  ASSERT_EQ(decoded.environment.jammer_sources.size(), 1U);
  EXPECT_EQ(decoded.environment.jammer_sources[0].technique,
            session::EsrJammingTechnique::kNoiseSuppression);
  EXPECT_TRUE(decoded.environment.jammer_sources[0].active);
  EXPECT_DOUBLE_EQ(decoded.environment.jammer_sources[0].center_hz, 9.5e9);
  EXPECT_EQ(decoded.environment.interference_mode,
            oneq::electromagnetics::RfInterferenceMode::kLegacy);
}

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesEngineeringRfSegments) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 10U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.environment.interference_mode =
      oneq::electromagnetics::RfInterferenceMode::kEngineering;
  oneq::electromagnetics::RfEmission emission;
  emission.emission_id = 501U;
  emission.entity_id = 50U;
  emission.position_ecef_m.x_m = 6388137.0;
  emission.antenna.peak_gain_dbi = 12.0;
  emission.polarization = oneq::electromagnetics::RfPolarization::kVertical;
  emission.waveform_kind = oneq::electromagnetics::RfWaveformKind::kSwept;
  oneq::electromagnetics::RfEmissionSegment segment;
  segment.start_time_s = 0.2;
  segment.duration_s = 0.4;
  segment.center_frequency_hz = 9.5e9;
  segment.bandwidth_hz = 20.0e6;
  segment.transmit_power_w = 800.0;
  emission.segments.push_back(segment);
  input.environment.engineering_emissions.push_back(emission);

  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(EncodeEsrCycleInput(input), &decoded));
  EXPECT_EQ(decoded.environment.interference_mode,
            oneq::electromagnetics::RfInterferenceMode::kEngineering);
  ASSERT_EQ(decoded.environment.engineering_emissions.size(), 1U);
  const oneq::electromagnetics::RfEmission& decoded_emission =
      decoded.environment.engineering_emissions.front();
  EXPECT_EQ(decoded_emission.emission_id, 501U);
  EXPECT_DOUBLE_EQ(decoded_emission.antenna.peak_gain_dbi, 12.0);
  EXPECT_EQ(decoded_emission.waveform_kind, oneq::electromagnetics::RfWaveformKind::kSwept);
  ASSERT_EQ(decoded_emission.segments.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded_emission.segments.front().start_time_s, 0.2);
  EXPECT_DOUBLE_EQ(decoded_emission.segments.front().transmit_power_w, 800.0);
}

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesRfV2EmissionFrame) {
  EsrCycleInput input;
  input.cycle_index = 17U;
  input.cycle_start_time_s = 42.5;
  input.dt_sec = 0.25f;
  input.has_rf_emission_frame = true;
  input.rf_emission_frame.world_cycle_index = input.cycle_index;
  input.rf_emission_frame.window_start_time_s = input.cycle_start_time_s;
  input.rf_emission_frame.window_duration_s = input.dt_sec;

  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 70U;
  emission.identity.equipment_id = 71U;
  emission.identity.emission_id = 72U;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.velocity_ecef_mps.y_mps = 120.0;
  emission.antenna.peak_gain_dbi = 15.0;
  emission.antenna.boresight_ecef.y = 1.0;
  emission.polarization = oneq::electromagnetics::RfScenePolarization::kVertical;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      input.cycle_start_time_s, 9.5e9, 2.0e6, 400.0, 5.0e-6, 1.0e-3, 20U, 0.05, 123U, 9U,
      &emission.waveform));
  input.rf_emission_frame.emissions.push_back(emission);

  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(EncodeEsrCycleInput(input), &decoded));
  EXPECT_EQ(decoded.cycle_index, input.cycle_index);
  EXPECT_DOUBLE_EQ(decoded.cycle_start_time_s, input.cycle_start_time_s);
  EXPECT_TRUE(decoded.has_rf_emission_frame);
  EXPECT_EQ(decoded.rf_emission_frame.world_cycle_index, input.cycle_index);
  EXPECT_DOUBLE_EQ(decoded.rf_emission_frame.window_start_time_s, input.cycle_start_time_s);
  EXPECT_DOUBLE_EQ(decoded.rf_emission_frame.window_duration_s, input.dt_sec);
  ASSERT_EQ(decoded.rf_emission_frame.emissions.size(), 1U);
  const oneq::electromagnetics::RfSceneEmission& decoded_emission =
      decoded.rf_emission_frame.emissions.front();
  EXPECT_EQ(decoded_emission.identity.platform_id, 70U);
  EXPECT_EQ(decoded_emission.identity.equipment_id, 71U);
  EXPECT_EQ(decoded_emission.identity.emission_id, 72U);
  EXPECT_EQ(decoded_emission.waveform.kind,
            oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain);
  EXPECT_EQ(decoded_emission.waveform.timing_seed, 123U);
  EXPECT_EQ(decoded_emission.waveform.timing_epoch, 9U);
  EXPECT_EQ(decoded_emission.waveform.pulse_count, 20U);
}

TEST(EsrReplayCodecRoundtripTest, CycleInputPreservesDoublePrecisionPose) {
  EsrCycleInput input;
  input.platform_pose.position_m.x = -4226.1319451063564;
  input.platform_pose.position_m.y = -8397.388596805331;
  input.platform_pose.position_m.z = 5255.8229838761899;
  input.platform_pose.velocity_mps.x = 123.45678901234567;
  input.platform_pose.velocity_mps.y = -45.67890123456789;
  input.platform_pose.velocity_mps.z = 0.000000123456789;
  input.platform_pose.attitude_deg.yaw_deg = 12.345678901234567;
  input.platform_pose.attitude_deg.pitch_deg = -2.345678901234567;
  input.platform_pose.attitude_deg.roll_deg = 0.12345678901234567;

  EsrSceneEmitter emitter;
  emitter.emitter_id = 2001U;
  emitter.pose = input.platform_pose;
  input.scene.push_back(emitter);

  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(EncodeEsrCycleInput(input), &decoded));

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
  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.scene[0].pose.position_m.x, emitter.pose.position_m.x);
  EXPECT_DOUBLE_EQ(decoded.scene[0].pose.attitude_deg.pitch_deg,
                   emitter.pose.attitude_deg.pitch_deg);
}

TEST(EsrReplayCodecRoundtripTest, CycleInputDecodesEmptyEmitterList) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;

  const std::string bytes = EncodeEsrCycleInput(input);
  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(bytes, &decoded));
  EXPECT_TRUE(decoded.scene.empty());
}

TEST(EsrReplayCodecRoundtripTest, CycleInputRejectsEmptyPayload) {
  EsrCycleInput decoded;
  EXPECT_FALSE(DecodeEsrCycleInput("", &decoded));
}

// ---------------------------------------------------------------------------
// EsrOutputFrame
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, OutputFramePreservesAllFields) {
  session::EsrOutputFrame frame;
  frame.cycle_index = 7U;
  frame.batch_id = 42U;
  frame.observation_output.receiver_center_frequency_hz = 9.4e9;
  frame.observation_output.receiver_bandwidth_hz = 20.0e6;
  frame.observation_output.receiver_saturated = true;

  session::EmitterObservation obs;
  obs.observation_id = 100U;
  obs.timestamp_s = 12.5;
  obs.aoa_az_deg = 45.0;
  obs.aoa_el_deg = -3.0;
  obs.rf_hz = 9.4e9;
  obs.bandwidth_hz = 3.0e6;
  obs.pri_s = 1.0e-3;
  obs.pulse_width_s = 2.0e-6;
  obs.rf_std_hz = 1000.0;
  obs.bandwidth_std_hz = 2000.0;
  obs.pri_std_s = 1.0e-6;
  obs.pulse_width_std_s = 1.0e-8;
  obs.amplitude_db = -80.0;
  obs.snr_db = 25.0;
  obs.quality = session::EsrObservationQuality::kHigh;
  frame.observation_output.observations.push_back(obs);

  session::EmitterHypothesis hyp;
  hyp.hypothesis_id = 10U;
  hyp.candidate_classes.push_back("FireControl");
  hyp.candidate_classes.push_back("Surveillance");
  hyp.mode = session::EsrEmitterMode::kTracking;
  hyp.threat_level = session::EsrThreatLevel::kHigh;
  hyp.bearing_az_deg = 44.5f;
  hyp.bearing_el_deg = -2.8f;
  hyp.bearing_std_deg = 0.5f;
  hyp.estimated_center_frequency_hz = 9.4e9;
  hyp.estimated_bandwidth_hz = 3.0e6;
  hyp.estimated_pri_s = 1.0e-3;
  hyp.estimated_pulse_width_s = 2.0e-6;
  hyp.center_frequency_std_hz = 1000.0;
  hyp.bandwidth_std_hz = 2000.0;
  hyp.pri_std_s = 1.0e-6;
  hyp.pulse_width_std_s = 1.0e-8;
  hyp.confidence = 0.85f;
  hyp.last_seen_cycle = 7U;
  frame.emitter_output.hypotheses.push_back(hyp);

  session::TruthAssociationRecord truth;
  truth.observation_id = 100U;
  truth.truth_emitter_id = 2001U;
  truth.matched = true;
  truth.confidence = 0.95f;
  frame.truth_evaluation_output.associations.push_back(truth);

  const std::string bytes = EncodeEsrOutputFrame(frame);
  ASSERT_FALSE(bytes.empty());

  session::EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 7U);
  EXPECT_EQ(decoded.batch_id, 42U);
  EXPECT_DOUBLE_EQ(decoded.observation_output.receiver_center_frequency_hz, 9.4e9);
  EXPECT_TRUE(decoded.observation_output.receiver_saturated);

  ASSERT_EQ(decoded.observation_output.observations.size(), 1U);
  EXPECT_EQ(decoded.observation_output.observations[0].observation_id, 100U);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].timestamp_s, 12.5);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].aoa_az_deg, 45.0);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].rf_hz, 9.4e9);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].bandwidth_hz, 3.0e6);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].pri_s, 1.0e-3);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].snr_db, 25.0);
  EXPECT_EQ(decoded.observation_output.observations[0].quality,
            session::EsrObservationQuality::kHigh);

  ASSERT_EQ(decoded.emitter_output.hypotheses.size(), 1U);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].hypothesis_id, 10U);
  ASSERT_GE(decoded.emitter_output.hypotheses[0].candidate_classes.size(), 1U);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].candidate_classes[0], "FireControl");
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].mode, session::EsrEmitterMode::kTracking);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].threat_level, session::EsrThreatLevel::kHigh);
  EXPECT_FLOAT_EQ(decoded.emitter_output.hypotheses[0].bearing_az_deg, 44.5f);
  EXPECT_DOUBLE_EQ(decoded.emitter_output.hypotheses[0].estimated_bandwidth_hz, 3.0e6);
  EXPECT_DOUBLE_EQ(decoded.emitter_output.hypotheses[0].estimated_pri_s, 1.0e-3);
  EXPECT_FLOAT_EQ(decoded.emitter_output.hypotheses[0].confidence, 0.85f);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].last_seen_cycle, 7U);

  ASSERT_EQ(decoded.truth_evaluation_output.associations.size(), 1U);
  EXPECT_EQ(decoded.truth_evaluation_output.associations[0].observation_id, 100U);
  EXPECT_EQ(decoded.truth_evaluation_output.associations[0].truth_emitter_id, 2001U);
  EXPECT_TRUE(decoded.truth_evaluation_output.associations[0].matched);
  EXPECT_FLOAT_EQ(decoded.truth_evaluation_output.associations[0].confidence, 0.95f);
}

// ---------------------------------------------------------------------------
// EsrCycleResult
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, CycleResultPreservesAllFields) {
  EsrCycleResult result;
  result.input_cycle_index = 55U;
  result.output_frame.cycle_index = 5U;
  result.output_frame.batch_id = 3U;

  session::EmitterObservation obs;
  obs.observation_id = 1U;
  obs.snr_db = 20.0;
  result.output_frame.observation_output.observations.push_back(obs);

  result.has_validation_error = true;
  result.executed_this_cycle = true;
  result.reused_previous_output = false;
  result.abort_reason = session::EsrPipelineAbortReason::kValidationRejected;

  ValidationIssue issue;
  issue.severity = ValidationSeverity::kError;
  issue.code = ValidationCode::kInvalidEmitterFrequency;
  issue.field = "carrier_hz";
  issue.message = "emitter frequency must be positive";
  result.validation_issues.push_back(issue);

  const std::string bytes = EncodeEsrCycleResult(result);
  ASSERT_FALSE(bytes.empty());

  EsrCycleResult decoded;
  ASSERT_TRUE(DecodeEsrCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, 55U);
  EXPECT_EQ(decoded.output_frame.cycle_index, 5U);
  EXPECT_EQ(decoded.output_frame.batch_id, 3U);
  ASSERT_EQ(decoded.output_frame.observation_output.observations.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.output_frame.observation_output.observations[0].snr_db, 20.0);

  EXPECT_TRUE(decoded.has_validation_error);
  EXPECT_TRUE(decoded.executed_this_cycle);
  EXPECT_FALSE(decoded.reused_previous_output);
  EXPECT_EQ(decoded.abort_reason, session::EsrPipelineAbortReason::kValidationRejected);

  ASSERT_EQ(decoded.validation_issues.size(), 1U);
  EXPECT_EQ(decoded.validation_issues[0].severity, ValidationSeverity::kError);
  EXPECT_EQ(decoded.validation_issues[0].code, ValidationCode::kInvalidEmitterFrequency);
  EXPECT_EQ(decoded.validation_issues[0].field, "carrier_hz");
  EXPECT_EQ(decoded.validation_issues[0].message, "emitter frequency must be positive");
}

TEST(EsrReplayCodecRoundtripTest, CycleResultPreservesBatchIdAboveUint32Max) {
  EsrCycleResult result;
  result.input_cycle_index = 56U;
  result.output_frame.cycle_index = 6U;
  result.output_frame.batch_id =
      static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max()) + 1U;
  result.executed_this_cycle = true;

  const std::string bytes = EncodeEsrCycleResult(result);
  ASSERT_FALSE(bytes.empty());

  EsrCycleResult decoded;
  ASSERT_TRUE(DecodeEsrCycleResult(bytes, &decoded));
  EXPECT_EQ(decoded.output_frame.batch_id, result.output_frame.batch_id);
}

// ---------------------------------------------------------------------------
// config::EsrSessionConfig
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::EsrSessionConfig config;
  // hardware
  config.hardware.receiver_equipment_id = 43U;
  config.hardware.receiver_band_lower_hz = 0.5e9;
  config.hardware.receiver_band_upper_hz = 18.0e9;
  config.hardware.receiver_sensitivity_w = 1.0e-12f;
  config.hardware.receiver_noise_figure_db = 6.5f;
  config.hardware.receiver_reference_temperature_k = 295.0f;
  config.hardware.integrated_receive_loss_db = 3.0f;
  config.hardware.beam_az_width_deg = 5.0f;
  config.hardware.antenna_mount_az_deg = 0.0f;
  config.hardware.antenna_peak_gain_dbi = 8.0f;
  config.hardware.polarization = oneq::electromagnetics::RfPolarization::kHorizontal;
  config.hardware.has_co_site_isolation = true;
  config.hardware.co_site_isolation_db = 70.0f;
  config::EsrCoSiteIsolationPath co_site_path;
  co_site_path.transmitter_equipment_id = 44U;
  co_site_path.isolation_db = 73.0;
  config.hardware.co_site_paths.push_back(co_site_path);
  config.hardware.maximum_linear_input_power_w = 2.0e-3f;
  config::EsrTuningWindow tuning_window;
  tuning_window.center_frequency_hz = 9.5e9;
  tuning_window.bandwidth_hz = 20.0e6;
  tuning_window.dwell_cycles = 3U;
  config.hardware.tuning_plan.push_back(tuning_window);
  // mission + scan
  config.mission.power_on = true;
  config.mission.work_mode = config::EsrWorkMode::kHgesm;
  config.mission.scan.scan_center_az_deg = 15.0f;
  config.mission.scan.scan_center_el_deg = -2.0f;
  config.mission.scan.scan_rate_hz = 2.0f;
  config.mission.scan.scan_start_position = config::EsrScanStartPosition::kRightTop;
  config.mission.scan.scan_sequence = config::EsrScanSequence::kElevationFirst;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = -30.0f;
  config.mission.scan.scan_end_az_deg = 30.0f;
  // policy
  config.policy.detection.minimum_snr_db = 4.0f;
  config.policy.detection.pfa = 1.0e-5f;
  config.policy.detection.pulse_count = 16U;
  config.policy.detection.threshold_scale = 0.8f;
  config.policy.detection.enable_statistical_detection = true;
  // environment
  config.environment.scenario_config.preset = config::EsrEnvironmentPreset::kDenseClutter;
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.pressure_hpa = 1010.0f;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 290.0f;
  config.environment.scenario_config.atmospheric_physics.relative_humidity = 0.6f;
  config.environment.scenario_config.atmospheric_context.has_k_factor = true;
  config.environment.scenario_config.atmospheric_context.k_factor = 1.2f;
  config.environment.scenario_config.atmospheric_context.has_day_of_year = true;
  config.environment.scenario_config.atmospheric_context.day_of_year = 180;
  config.environment.scenario_config.atmospheric_context.solar_flux_f107a = 155.0f;
  config.environment.scenario_config.atmospheric_context.solar_flux_f107 = 160.0f;
  config.environment.scenario_config.atmospheric_context.geomagnetic_ap = 8.0f;

  const std::string bytes = EncodeEsrSessionConfig(config);
  ASSERT_FALSE(bytes.empty());

  config::EsrSessionConfig decoded;
  ASSERT_TRUE(DecodeEsrSessionConfig(bytes, &decoded));

  // hardware
  EXPECT_EQ(decoded.hardware.receiver_equipment_id, 43U);
  EXPECT_DOUBLE_EQ(decoded.hardware.receiver_band_lower_hz, 0.5e9);
  EXPECT_DOUBLE_EQ(decoded.hardware.receiver_band_upper_hz, 18.0e9);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver_sensitivity_w, 1.0e-12f);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver_noise_figure_db, 6.5f);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver_reference_temperature_k, 295.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.integrated_receive_loss_db, 3.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.antenna_peak_gain_dbi, 8.0f);
  EXPECT_EQ(decoded.hardware.polarization,
            oneq::electromagnetics::RfPolarization::kHorizontal);
  EXPECT_TRUE(decoded.hardware.has_co_site_isolation);
  EXPECT_FLOAT_EQ(decoded.hardware.co_site_isolation_db, 70.0f);
  ASSERT_EQ(decoded.hardware.co_site_paths.size(), 1U);
  EXPECT_EQ(decoded.hardware.co_site_paths.front().transmitter_equipment_id, 44U);
  EXPECT_DOUBLE_EQ(decoded.hardware.co_site_paths.front().isolation_db, 73.0);
  ASSERT_EQ(decoded.hardware.tuning_plan.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.hardware.tuning_plan.front().center_frequency_hz, 9.5e9);
  // mission
  EXPECT_TRUE(decoded.mission.power_on);
  EXPECT_EQ(decoded.mission.work_mode, config::EsrWorkMode::kHgesm);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_center_az_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_rate_hz, 2.0f);
  EXPECT_EQ(decoded.mission.scan.scan_start_position, config::EsrScanStartPosition::kRightTop);
  EXPECT_EQ(decoded.mission.scan.scan_sequence, config::EsrScanSequence::kElevationFirst);
  EXPECT_TRUE(decoded.mission.scan.use_explicit_scan_bounds);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_start_az_deg, -30.0f);
  // policy
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_snr_db, 4.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.pfa, 1.0e-5f);
  EXPECT_EQ(decoded.policy.detection.pulse_count, 16U);
  EXPECT_FLOAT_EQ(decoded.policy.detection.threshold_scale, 0.8f);
  EXPECT_TRUE(decoded.policy.detection.enable_statistical_detection);
  // environment
  EXPECT_EQ(decoded.environment.scenario_config.preset,
            config::EsrEnvironmentPreset::kDenseClutter);
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa, 1010.0f);
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_context.k_factor, 1.2f);
  EXPECT_EQ(decoded.environment.scenario_config.atmospheric_context.day_of_year, 180);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_context.solar_flux_f107, 160.0f);
}

// ---------------------------------------------------------------------------
// config::EsrRuntimeConfigPatch
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  config::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = false;
  patch.has_work_mode = true;
  patch.work_mode = config::EsrWorkMode::kRwr;
  patch.has_scan_rate_hz = true;
  patch.scan_rate_hz = 3.0f;
  patch.has_scan_start_position = true;
  patch.scan_start_position = config::EsrScanStartPosition::kRightTop;
  patch.has_scan_sequence = true;
  patch.scan_sequence = config::EsrScanSequence::kElevationFirst;
  patch.has_scan_center_az_deg = true;
  patch.scan_center_az_deg = 10.0f;
  patch.has_scan_center_el_deg = true;
  patch.scan_center_el_deg = -1.0f;
  patch.has_explicit_scan_bounds = true;
  patch.explicit_scan_bounds.enabled = true;
  patch.explicit_scan_bounds.scan_start_az_deg = -40.0f;
  patch.explicit_scan_bounds.scan_end_az_deg = 40.0f;
  patch.explicit_scan_bounds.scan_start_el_deg = -15.0f;
  patch.explicit_scan_bounds.scan_end_el_deg = 15.0f;
  patch.has_mission = true;
  patch.mission.power_on = false;
  patch.mission.work_mode = config::EsrWorkMode::kEsm;
  patch.mission.scan.scan_center_az_deg = 15.0f;
  patch.mission.scan.scan_rate_hz = 2.0f;
  patch.has_policy = true;
  patch.policy.detection.minimum_snr_db = 8.0f;
  patch.has_environment = true;
  patch.environment.has_atmospheric_physics = true;
  patch.environment.atmospheric_physics.relative_humidity = 0.45f;
  patch.environment.has_atmospheric_context = true;
  patch.environment.atmospheric_context.has_k_factor = true;
  patch.environment.atmospheric_context.k_factor = 1.5f;

  const std::string bytes = EncodeEsrRuntimeConfigPatch(patch);
  ASSERT_FALSE(bytes.empty());

  config::EsrRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeEsrRuntimeConfigPatch(bytes, &decoded));

  EXPECT_TRUE(decoded.has_sensor_enabled);
  EXPECT_FALSE(decoded.sensor_enabled);
  EXPECT_TRUE(decoded.has_work_mode);
  EXPECT_EQ(decoded.work_mode, config::EsrWorkMode::kRwr);
  EXPECT_TRUE(decoded.has_scan_rate_hz);
  EXPECT_FLOAT_EQ(decoded.scan_rate_hz, 3.0f);
  EXPECT_TRUE(decoded.has_scan_start_position);
  EXPECT_EQ(decoded.scan_start_position, config::EsrScanStartPosition::kRightTop);
  EXPECT_TRUE(decoded.has_scan_sequence);
  EXPECT_EQ(decoded.scan_sequence, config::EsrScanSequence::kElevationFirst);
  EXPECT_TRUE(decoded.has_scan_center_az_deg);
  EXPECT_FLOAT_EQ(decoded.scan_center_az_deg, 10.0f);
  EXPECT_TRUE(decoded.has_scan_center_el_deg);
  EXPECT_FLOAT_EQ(decoded.scan_center_el_deg, -1.0f);
  EXPECT_TRUE(decoded.has_explicit_scan_bounds);
  EXPECT_TRUE(decoded.explicit_scan_bounds.enabled);
  EXPECT_FLOAT_EQ(decoded.explicit_scan_bounds.scan_start_az_deg, -40.0f);
  EXPECT_FLOAT_EQ(decoded.explicit_scan_bounds.scan_end_az_deg, 40.0f);
  EXPECT_FLOAT_EQ(decoded.explicit_scan_bounds.scan_start_el_deg, -15.0f);
  EXPECT_FLOAT_EQ(decoded.explicit_scan_bounds.scan_end_el_deg, 15.0f);
  EXPECT_TRUE(decoded.has_mission);
  EXPECT_FALSE(decoded.mission.power_on);
  EXPECT_EQ(decoded.mission.work_mode, config::EsrWorkMode::kEsm);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_center_az_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_rate_hz, 2.0f);
  EXPECT_TRUE(decoded.has_policy);
  EXPECT_FLOAT_EQ(decoded.policy.detection.minimum_snr_db, 8.0f);
  EXPECT_TRUE(decoded.has_environment);
  EXPECT_TRUE(decoded.environment.has_atmospheric_physics);
  EXPECT_FLOAT_EQ(decoded.environment.atmospheric_physics.relative_humidity, 0.45f);
  EXPECT_TRUE(decoded.environment.has_atmospheric_context);
  EXPECT_TRUE(decoded.environment.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(decoded.environment.atmospheric_context.k_factor, 1.5f);
}

// ---------------------------------------------------------------------------
// FailureMarker
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "ESR_ASSERT";
  failure.message = "emitter pool overflow";
  failure.location = "EsrController::RunOnce";
  failure.has_cycle_index = true;
  failure.cycle_index = 42U;
  failure.has_sim_time_sec = true;
  failure.sim_time_sec = 42.5;
  failure.diagnostics_payload = "{\"hypothesis_count\":128}";

  const std::string bytes = EncodeEsrFailureMarker(failure);
  ASSERT_FALSE(bytes.empty());

  oneq::replay::ReplayTraceFailure decoded;
  std::string error;
  ASSERT_TRUE(DecodeEsrFailureMarker(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.error_code, "ESR_ASSERT");
  EXPECT_EQ(decoded.message, "emitter pool overflow");
  EXPECT_EQ(decoded.location, "EsrController::RunOnce");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 42U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 42.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"hypothesis_count\":128}");
}

// ===========================================================================
// Decode 失败路径（null output / 损坏 payload）
// ===========================================================================

TEST(EsrReplayCodecRoundtripTest, DecodeCycleInputRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEsrCycleInput("", nullptr));
  EsrCycleInput input;
  EXPECT_FALSE(DecodeEsrCycleInput("corrupt", &input));
}

TEST(EsrReplayCodecRoundtripTest, DecodeOutputFrameRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEsrOutputFrame("", nullptr));
  EsrOutputFrame frame;
  EXPECT_FALSE(DecodeEsrOutputFrame("bad", &frame));
}

TEST(EsrReplayCodecRoundtripTest, DecodeCycleResultRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEsrCycleResult("", nullptr));
  EsrCycleResult result;
  EXPECT_FALSE(DecodeEsrCycleResult("bad", &result));
}

TEST(EsrReplayCodecRoundtripTest, DecodeSessionConfigRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEsrSessionConfig("", nullptr));
  config::EsrSessionConfig config;
  EXPECT_FALSE(DecodeEsrSessionConfig("bad", &config));
}

TEST(EsrReplayCodecRoundtripTest, DecodeRuntimeConfigPatchRejectsNullAndCorrupted) {
  EXPECT_FALSE(DecodeEsrRuntimeConfigPatch("", nullptr));
  config::EsrRuntimeConfigPatch patch;
  EXPECT_FALSE(DecodeEsrRuntimeConfigPatch("bad", &patch));
}

TEST(EsrReplayCodecRoundtripTest, DecodeFailureMarkerRejectsNullAndCorrupted) {
  std::string error;
  EXPECT_FALSE(DecodeEsrFailureMarker("", nullptr, &error));
  oneq::replay::ReplayTraceFailure failure;
  EXPECT_FALSE(DecodeEsrFailureMarker("bad", &failure, &error));
}

}  // namespace tests
}  // namespace session
}  // namespace electronic_surveillance_radar
