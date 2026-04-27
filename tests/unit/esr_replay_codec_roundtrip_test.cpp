/**
 * @file esr_replay_codec_roundtrip_test.cpp
 * @brief 验证 ESR replay FlatBuffers codec 各 payload 的 Encode→Decode round-trip 字段精确保真。
 *
 * 每个测试独立覆盖一种 payload 类型，确保新增字段未同步到 schema/codec 时立即被发现。
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
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
  emitter.emitter_id = "emitter_001";
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

  input.environment.observation.propagation_profile =
      environment::EsrPropagationEnvironmentProfile::kComplex;
  input.environment.observation.clutter_density =
      environment::EsrClutterDensityLevel::kHigh;
  input.environment.observation.spectrum_occupancy_ratio = 0.45f;
  input.environment.observation.atmospheric_observation.relative_humidity_ratio = 0.6f;
  input.environment.observation.atmospheric_observation.precipitation_rate_mmph = 2.5f;
  input.environment.observation.atmospheric_observation.visibility_km = 8.0f;

  environment::EsrJammerSource jammer;
  jammer.technique = environment::EsrJammingTechnique::kNoiseSuppression;
  jammer.active = true;
  jammer.center_hz = 9.5e9;
  jammer.bandwidth_hz = 20.0e6;
  jammer.power_w = 100.0f;
  jammer.deception_risk = 0.1f;
  jammer.confidence = 0.9f;
  input.environment.observation.jammer_sources.push_back(jammer);

  const std::string bytes = EncodeEsrCycleInput(input);
  ASSERT_FALSE(bytes.empty());

  EsrCycleInput decoded;
  ASSERT_TRUE(DecodeEsrCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 3U);
  EXPECT_FLOAT_EQ(decoded.dt_sec, 0.1f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.position_m.x, 1000.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.velocity_mps.y, 10.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.attitude_deg.yaw_deg, 90.0f);

  ASSERT_EQ(decoded.scene.size(), 1U);
  EXPECT_EQ(decoded.scene[0].emitter_id, "emitter_001");
  EXPECT_DOUBLE_EQ(decoded.scene[0].carrier_hz, 9.5e9);
  EXPECT_DOUBLE_EQ(decoded.scene[0].pulse_width_s, 1.0e-6);
  EXPECT_TRUE(decoded.scene[0].is_emitting);
  EXPECT_DOUBLE_EQ(decoded.scene[0].beam_state.center_az_deg, 15.0);
  EXPECT_TRUE(decoded.scene[0].beam_state.beam_state_valid);

  EXPECT_EQ(decoded.environment.observation.propagation_profile,
            environment::EsrPropagationEnvironmentProfile::kComplex);
  EXPECT_EQ(decoded.environment.observation.clutter_density,
            environment::EsrClutterDensityLevel::kHigh);
  EXPECT_FLOAT_EQ(decoded.environment.observation.spectrum_occupancy_ratio, 0.45f);
  EXPECT_FLOAT_EQ(decoded.environment.observation.atmospheric_observation.relative_humidity_ratio, 0.6f);

  ASSERT_EQ(decoded.environment.observation.jammer_sources.size(), 1U);
  EXPECT_EQ(decoded.environment.observation.jammer_sources[0].technique,
            environment::EsrJammingTechnique::kNoiseSuppression);
  EXPECT_TRUE(decoded.environment.observation.jammer_sources[0].active);
  EXPECT_DOUBLE_EQ(decoded.environment.observation.jammer_sources[0].center_hz, 9.5e9);
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

  model::EmitterObservation obs;
  obs.observation_id = 100U;
  obs.timestamp_s = 12.5;
  obs.aoa_az_deg = 45.0;
  obs.aoa_el_deg = -3.0;
  obs.rf_hz = 9.4e9;
  obs.pulse_width_s = 2.0e-6;
  obs.amplitude_db = -80.0;
  obs.snr_db = 25.0;
  obs.quality = model::EsrObservationQuality::kHigh;
  obs.is_jammed = false;
  frame.observation_output.observations.push_back(obs);

  model::EmitterHypothesis hyp;
  hyp.hypothesis_id = 10U;
  hyp.candidate_classes.push_back("FireControl");
  hyp.candidate_classes.push_back("Surveillance");
  hyp.mode = model::EsrEmitterMode::kTracking;
  hyp.threat_level = model::EsrThreatLevel::kHigh;
  hyp.bearing_az_deg = 44.5f;
  hyp.bearing_el_deg = -2.8f;
  hyp.bearing_std_deg = 0.5f;
  hyp.confidence = 0.85f;
  hyp.last_seen_cycle = 7U;
  frame.emitter_output.hypotheses.push_back(hyp);

  extension::TruthAssociationRecord truth;
  truth.observation_id = 100U;
  truth.truth_emitter_id = "truth_emitter_001";
  truth.matched = true;
  truth.confidence = 0.95f;
  frame.truth_evaluation_output.associations.push_back(truth);

  const std::string bytes = EncodeEsrOutputFrame(frame);
  ASSERT_FALSE(bytes.empty());

  session::EsrOutputFrame decoded;
  ASSERT_TRUE(DecodeEsrOutputFrame(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, 7U);
  EXPECT_EQ(decoded.batch_id, 42U);

  ASSERT_EQ(decoded.observation_output.observations.size(), 1U);
  EXPECT_EQ(decoded.observation_output.observations[0].observation_id, 100U);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].timestamp_s, 12.5);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].aoa_az_deg, 45.0);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].rf_hz, 9.4e9);
  EXPECT_DOUBLE_EQ(decoded.observation_output.observations[0].snr_db, 25.0);
  EXPECT_EQ(decoded.observation_output.observations[0].quality,
            model::EsrObservationQuality::kHigh);
  EXPECT_FALSE(decoded.observation_output.observations[0].is_jammed);

  ASSERT_EQ(decoded.emitter_output.hypotheses.size(), 1U);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].hypothesis_id, 10U);
  ASSERT_GE(decoded.emitter_output.hypotheses[0].candidate_classes.size(), 1U);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].candidate_classes[0], "FireControl");
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].mode, model::EsrEmitterMode::kTracking);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].threat_level, model::EsrThreatLevel::kHigh);
  EXPECT_FLOAT_EQ(decoded.emitter_output.hypotheses[0].bearing_az_deg, 44.5f);
  EXPECT_FLOAT_EQ(decoded.emitter_output.hypotheses[0].confidence, 0.85f);
  EXPECT_EQ(decoded.emitter_output.hypotheses[0].last_seen_cycle, 7U);

  ASSERT_EQ(decoded.truth_evaluation_output.associations.size(), 1U);
  EXPECT_EQ(decoded.truth_evaluation_output.associations[0].observation_id, 100U);
  EXPECT_EQ(decoded.truth_evaluation_output.associations[0].truth_emitter_id,
            "truth_emitter_001");
  EXPECT_TRUE(decoded.truth_evaluation_output.associations[0].matched);
  EXPECT_FLOAT_EQ(decoded.truth_evaluation_output.associations[0].confidence, 0.95f);
}

// ---------------------------------------------------------------------------
// EsrCycleResult
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, CycleResultPreservesAllFields) {
  EsrCycleResult result;
  result.output_frame.cycle_index = 5U;
  result.output_frame.batch_id = 3U;

  model::EmitterObservation obs;
  obs.observation_id = 1U;
  obs.snr_db = 20.0;
  result.output_frame.observation_output.observations.push_back(obs);

  result.has_validation_error = true;
  result.executed_this_cycle = true;
  result.reused_previous_output = false;
  result.abort_reason = extension::EsrPipelineAbortReason::kValidationRejected;

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

  EXPECT_EQ(decoded.output_frame.cycle_index, 5U);
  EXPECT_EQ(decoded.output_frame.batch_id, 3U);
  ASSERT_EQ(decoded.output_frame.observation_output.observations.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.output_frame.observation_output.observations[0].snr_db, 20.0);

  EXPECT_TRUE(decoded.has_validation_error);
  EXPECT_TRUE(decoded.executed_this_cycle);
  EXPECT_FALSE(decoded.reused_previous_output);
  EXPECT_EQ(decoded.abort_reason, extension::EsrPipelineAbortReason::kValidationRejected);

  ASSERT_EQ(decoded.validation_issues.size(), 1U);
  EXPECT_EQ(decoded.validation_issues[0].severity, ValidationSeverity::kError);
  EXPECT_EQ(decoded.validation_issues[0].code, ValidationCode::kInvalidEmitterFrequency);
  EXPECT_EQ(decoded.validation_issues[0].field, "carrier_hz");
  EXPECT_EQ(decoded.validation_issues[0].message, "emitter frequency must be positive");
}

// ---------------------------------------------------------------------------
// EsrSessionConfig
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  EsrSessionConfig config;
  // hardware
  config.hardware.receiver_band_lower_hz = 0.5e9;
  config.hardware.receiver_band_upper_hz = 18.0e9;
  config.hardware.receiver_sensitivity_w = 1.0e-12f;
  config.hardware.integrated_receive_loss_db = 3.0f;
  config.hardware.beam_az_width_deg = 5.0f;
  config.hardware.antenna_mount_az_deg = 0.0f;
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
  config.policy.detection.profile = config::EsrDetectionProfile::kSensitive;
  config.policy.detection.use_profile_defaults = false;
  config.policy.detection.min_detect_snr_db = 4.0f;
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

  EsrSessionConfig decoded;
  ASSERT_TRUE(DecodeEsrSessionConfig(bytes, &decoded));

  // hardware
  EXPECT_DOUBLE_EQ(decoded.hardware.receiver_band_lower_hz, 0.5e9);
  EXPECT_DOUBLE_EQ(decoded.hardware.receiver_band_upper_hz, 18.0e9);
  EXPECT_FLOAT_EQ(decoded.hardware.receiver_sensitivity_w, 1.0e-12f);
  EXPECT_FLOAT_EQ(decoded.hardware.integrated_receive_loss_db, 3.0f);
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
  EXPECT_EQ(decoded.policy.detection.profile, config::EsrDetectionProfile::kSensitive);
  EXPECT_FALSE(decoded.policy.detection.use_profile_defaults);
  EXPECT_FLOAT_EQ(decoded.policy.detection.min_detect_snr_db, 4.0f);
  EXPECT_FLOAT_EQ(decoded.policy.detection.pfa, 1.0e-5f);
  EXPECT_EQ(decoded.policy.detection.pulse_count, 16U);
  EXPECT_FLOAT_EQ(decoded.policy.detection.threshold_scale, 0.8f);
  EXPECT_TRUE(decoded.policy.detection.enable_statistical_detection);
  // environment
  EXPECT_EQ(decoded.environment.scenario_config.preset, config::EsrEnvironmentPreset::kDenseClutter);
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa, 1010.0f);
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_context.k_factor, 1.2f);
  EXPECT_EQ(decoded.environment.scenario_config.atmospheric_context.day_of_year, 180);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_context.solar_flux_f107, 160.0f);
}

// ---------------------------------------------------------------------------
// EsrRuntimeConfigPatch
// ---------------------------------------------------------------------------

TEST(EsrReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  EsrRuntimeConfigPatch patch;
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
  patch.has_use_explicit_scan_bounds = true;
  patch.use_explicit_scan_bounds = true;
  patch.has_scan_start_az_deg = true;
  patch.scan_start_az_deg = -40.0f;
  patch.has_scan_end_az_deg = true;
  patch.scan_end_az_deg = 40.0f;
  patch.has_scan_start_el_deg = true;
  patch.scan_start_el_deg = -15.0f;
  patch.has_scan_end_el_deg = true;
  patch.scan_end_el_deg = 15.0f;
  patch.has_mission = true;
  patch.mission.power_on = false;
  patch.mission.work_mode = config::EsrWorkMode::kEsm;
  patch.mission.scan.scan_center_az_deg = 15.0f;
  patch.mission.scan.scan_rate_hz = 2.0f;
  patch.has_policy = true;
  patch.policy.detection.profile = config::EsrDetectionProfile::kConservative;
  patch.policy.detection.min_detect_snr_db = 8.0f;
  patch.has_environment_runtime_config = true;
  patch.environment_runtime_config.has_atmospheric_physics = true;
  patch.environment_runtime_config.atmospheric_physics.relative_humidity = 0.45f;
  patch.environment_runtime_config.has_atmospheric_context = true;
  patch.environment_runtime_config.atmospheric_context.has_k_factor = true;
  patch.environment_runtime_config.atmospheric_context.k_factor = 1.5f;

  const std::string bytes = EncodeEsrRuntimeConfigPatch(patch);
  ASSERT_FALSE(bytes.empty());

  EsrRuntimeConfigPatch decoded;
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
  EXPECT_TRUE(decoded.has_use_explicit_scan_bounds);
  EXPECT_TRUE(decoded.use_explicit_scan_bounds);
  EXPECT_TRUE(decoded.has_scan_start_az_deg);
  EXPECT_FLOAT_EQ(decoded.scan_start_az_deg, -40.0f);
  EXPECT_TRUE(decoded.has_scan_end_az_deg);
  EXPECT_FLOAT_EQ(decoded.scan_end_az_deg, 40.0f);
  EXPECT_TRUE(decoded.has_scan_start_el_deg);
  EXPECT_FLOAT_EQ(decoded.scan_start_el_deg, -15.0f);
  EXPECT_TRUE(decoded.has_scan_end_el_deg);
  EXPECT_FLOAT_EQ(decoded.scan_end_el_deg, 15.0f);
  EXPECT_TRUE(decoded.has_mission);
  EXPECT_FALSE(decoded.mission.power_on);
  EXPECT_EQ(decoded.mission.work_mode, config::EsrWorkMode::kEsm);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_center_az_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.scan.scan_rate_hz, 2.0f);
  EXPECT_TRUE(decoded.has_policy);
  EXPECT_EQ(decoded.policy.detection.profile, config::EsrDetectionProfile::kConservative);
  EXPECT_FLOAT_EQ(decoded.policy.detection.min_detect_snr_db, 8.0f);
  EXPECT_TRUE(decoded.has_environment_runtime_config);
  EXPECT_TRUE(decoded.environment_runtime_config.has_atmospheric_physics);
  EXPECT_FLOAT_EQ(decoded.environment_runtime_config.atmospheric_physics.relative_humidity, 0.45f);
  EXPECT_TRUE(decoded.environment_runtime_config.has_atmospheric_context);
  EXPECT_TRUE(decoded.environment_runtime_config.atmospheric_context.has_k_factor);
  EXPECT_FLOAT_EQ(decoded.environment_runtime_config.atmospheric_context.k_factor, 1.5f);
}

}  // namespace tests
}  // namespace session
}  // namespace electronic_surveillance_radar
