/**
 * @file ar_replay_codec_roundtrip_test.cpp
 * @brief 验证 AR replay FlatBuffers codec 各 payload 的 Encode→Decode round-trip 字段精确保真。
 *
 * 每个测试独立覆盖一种 payload 类型，确保新增字段未同步到 schema/codec 时立即被发现。
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/replay/ReplayTrace.h"
#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

namespace airborne_radar {
namespace session {
namespace tests {

// ---------------------------------------------------------------------------
// RadarCycleInput
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, CycleInputPreservesAllFields) {
  RadarCycleInput input;
  input.dt_sec = 0.5f;
  input.platform_pose.position_m.x = 100.0f;
  input.platform_pose.position_m.y = 200.0f;
  input.platform_pose.position_m.z = 300.0f;
  input.platform_pose.velocity_mps.x = 10.0f;
  input.platform_pose.velocity_mps.y = 20.0f;
  input.platform_pose.velocity_mps.z = 5.0f;
  input.platform_pose.attitude_deg.yaw_deg = 45.0f;
  input.platform_pose.attitude_deg.pitch_deg = -5.0f;
  input.platform_pose.attitude_deg.roll_deg = 2.0f;

  model::TargetFeature target;
  target.external_target_id = 42U;
  target.current_track_velocity_x = 80.0f;
  target.current_track_velocity_y = 1.5f;
  target.current_track_velocity_z = -0.5f;
  target.current_track_speed = 80.02f;
  target.current_track_rcs = 3.0f;
  target.range_m = 1234.5f;
  target.has_cartesian_position = true;
  target.position_x = 1234.0f;
  target.position_y = 56.0f;
  target.position_z = 78.0f;
  target.target_swerling_type = 2;
  input.target_features.push_back(target);

  const std::string bytes = EncodeCycleInputFlatbuffer(input);
  ASSERT_FALSE(bytes.empty());

  RadarCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_FLOAT_EQ(decoded.dt_sec, input.dt_sec);
  EXPECT_FLOAT_EQ(decoded.platform_pose.position_m.x, 100.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.velocity_mps.y, 20.0f);
  EXPECT_FLOAT_EQ(decoded.platform_pose.attitude_deg.yaw_deg, 45.0f);
  ASSERT_EQ(decoded.target_features.size(), 1U);
  EXPECT_EQ(decoded.target_features[0].external_target_id, 42U);
  EXPECT_FLOAT_EQ(decoded.target_features[0].current_track_rcs, 3.0f);
  EXPECT_FLOAT_EQ(decoded.target_features[0].range_m, 1234.5f);
  EXPECT_TRUE(decoded.target_features[0].has_cartesian_position);
  EXPECT_FLOAT_EQ(decoded.target_features[0].position_x, 1234.0f);
  EXPECT_EQ(decoded.target_features[0].target_swerling_type, 2);
}

TEST(ArReplayCodecRoundtripTest, CycleInputDecodesEmptyTargetList) {
  RadarCycleInput input;
  input.dt_sec = 1.0f;

  const std::string bytes = EncodeCycleInputFlatbuffer(input);
  RadarCycleInput decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleInputFlatbuffer(bytes, &decoded, &error)) << error;
  EXPECT_TRUE(decoded.target_features.empty());
}

TEST(ArReplayCodecRoundtripTest, CycleInputRejectsEmptyPayload) {
  RadarCycleInput decoded;
  std::string error;
  EXPECT_FALSE(DecodeCycleInputFlatbuffer("", &decoded, &error));
  EXPECT_FALSE(error.empty());
}

// ---------------------------------------------------------------------------
// TrackOutputFrame (深度 per-track 字段)
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, TrackOutputFramePreservesAllFields) {
  output::TrackOutputFrame frame;
  frame.cycle_index = 77U;
  frame.batch_id = 5U;
  frame.published_track_count = 1U;
  frame.confirmed_track_count = 1U;
  frame.contains_lost_tracks = true;

  model::DecisionTrackSnapshot snap;
  snap.state.association_key = 999U;
  snap.state.external_target_id = 42U;
  snap.state.status = model::DecisionTrackStatus::kConfirmed;
  snap.state.position_x = 100.0f;
  snap.state.position_y = 200.0f;
  snap.state.position_z = 50.0f;
  snap.state.velocity_x = 30.0f;
  snap.state.velocity_y = 5.0f;
  snap.state.velocity_z = -2.0f;
  snap.state.speed = 30.5f;
  snap.state.acceleration_x = 1.0f;
  snap.state.acceleration_y = 0.5f;
  snap.state.acceleration_z = 0.1f;
  snap.state.acceleration = 1.12f;
  snap.state.rcs = 2.5f;
  snap.state.jamming_detected = true;
  snap.state.hit_count = 5U;
  snap.state.miss_count = 1U;
  snap.evidence.has_measurement_evidence = true;
  snap.evidence.updated_this_cycle = true;
  snap.evidence.detection_margin_db = 12.0f;
  frame.tracks.push_back(snap);

  const std::string bytes = EncodeTrackOutputFrameFlatbuffer(frame);
  ASSERT_FALSE(bytes.empty());

  output::TrackOutputFrame decoded;
  std::string error;
  ASSERT_TRUE(DecodeTrackOutputFrameFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.cycle_index, 77U);
  EXPECT_EQ(decoded.batch_id, 5U);
  EXPECT_EQ(decoded.published_track_count, 1U);
  EXPECT_EQ(decoded.confirmed_track_count, 1U);
  EXPECT_TRUE(decoded.contains_lost_tracks);
  ASSERT_EQ(decoded.tracks.size(), 1U);

  const model::DecisionTrackStateSnapshot& ds = decoded.tracks[0].state;
  EXPECT_EQ(ds.association_key, 999U);
  EXPECT_EQ(ds.external_target_id, 42U);
  EXPECT_EQ(ds.status, model::DecisionTrackStatus::kConfirmed);
  EXPECT_FLOAT_EQ(ds.position_x, 100.0f);
  EXPECT_FLOAT_EQ(ds.position_y, 200.0f);
  EXPECT_FLOAT_EQ(ds.position_z, 50.0f);
  EXPECT_FLOAT_EQ(ds.velocity_x, 30.0f);
  EXPECT_FLOAT_EQ(ds.rcs, 2.5f);
  EXPECT_TRUE(ds.jamming_detected);
  EXPECT_EQ(ds.hit_count, 5U);
  EXPECT_EQ(ds.miss_count, 1U);

  const model::DecisionMeasurementEvidence& ev = decoded.tracks[0].evidence;
  EXPECT_TRUE(ev.has_measurement_evidence);
  EXPECT_TRUE(ev.updated_this_cycle);
  EXPECT_FLOAT_EQ(ev.detection_margin_db, 12.0f);
}

// ---------------------------------------------------------------------------
// RadarCycleResult
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, CycleResultPreservesAllFields) {
  RadarCycleResult result;
  result.track_output_frame.cycle_index = 5U;
  result.track_output_frame.published_track_count = 2U;
  result.track_output_frame.batch_id = 3U;
  result.track_output_frame.confirmed_track_count = 2U;
  result.track_output_frame.contains_lost_tracks = false;

  model::DecisionTrackSnapshot snap;
  snap.state.association_key = 1U;
  snap.state.position_x = 50.0f;
  snap.state.rcs = 1.5f;
  snap.state.status = model::DecisionTrackStatus::kTentative;
  result.track_output_frame.tracks.push_back(snap);

  result.validation_issues.resize(0U);
  result.executed_this_cycle = true;

  const std::string bytes = EncodeCycleResultFlatbuffer(result);
  ASSERT_FALSE(bytes.empty());

  RadarCycleResult decoded;
  std::string error;
  ASSERT_TRUE(DecodeCycleResultFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_EQ(decoded.track_output_frame.cycle_index, 5U);
  EXPECT_EQ(decoded.track_output_frame.published_track_count, 2U);
  EXPECT_EQ(decoded.track_output_frame.batch_id, 3U);
  EXPECT_TRUE(decoded.executed_this_cycle);
  ASSERT_EQ(decoded.track_output_frame.tracks.size(), 1U);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].state.position_x, 50.0f);
  EXPECT_FLOAT_EQ(decoded.track_output_frame.tracks[0].state.rcs, 1.5f);
}

// ---------------------------------------------------------------------------
// RadarSessionConfig（含 environment 域）
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  RadarSessionConfig config;
  // hardware
  config.hardware.detection.transmitter.peak_power_w = 50000.0f;
  config.hardware.detection.transmitter.frequency_hz = 9.5e9f;
  config.hardware.detection.min_detection_margin_db = -20.0f;
  // mission
  config.mission.orientation.scan_center_deg.az_deg = 15.0f;
  config.mission.orientation.scan_center_deg.el_deg = -2.0f;
  // policy
  config.policy.lifecycle.confirm_hits = 2U;
  config.policy.lifecycle.max_miss_before_lost = 3U;
  config.policy.tracking.enable_kalman_filter = true;
  config.policy.tracking.kalman_measurement_noise_std = 5.5f;
  // jamming_sensitivity_profile
  config.jamming_sensitivity_profile = environment::JammingSensitivityProfile::kStrict;
  // environment (previously missing from codec!)
  config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;
  config.environment.scenario_config.atmospheric_physics.pressure_hpa = 1010.0f;
  config.environment.scenario_config.atmospheric_physics.temperature_k = 290.0f;
  config.environment.scenario_config.atmospheric_physics.relative_humidity = 0.6f;
  config.environment.scenario_config.atmospheric_context.has_simulation_unix_seconds = true;
  config.environment.scenario_config.atmospheric_context.simulation_unix_seconds = 1700000000LL;
  config.environment.scenario_config.vegetation_scatter_physics.enable_physical_model = true;
  config.environment.scenario_config.vegetation_scatter_physics.cover_profile =
      environment::VegetationCoverProfile::kSparseWoodland;
  environment::JammerEmitterState jammer;
  jammer.technique = environment::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 25.0f;
  jammer.js_db = 8.0f;
  jammer.has_direction_deg = true;
  jammer.azimuth_deg = 30.0f;
  jammer.elevation_deg = 5.0f;
  config.environment.scenario_config.jammer_sources.push_back(jammer);

  const std::string bytes = EncodeSessionConfigFlatbuffer(config);
  ASSERT_FALSE(bytes.empty());

  RadarSessionConfig decoded;
  std::string error;
  ASSERT_TRUE(DecodeSessionConfigFlatbuffer(bytes, &decoded, &error)) << error;

  // hardware
  EXPECT_FLOAT_EQ(decoded.hardware.detection.transmitter.peak_power_w, 50000.0f);
  EXPECT_FLOAT_EQ(decoded.hardware.detection.transmitter.frequency_hz, 9.5e9f);
  EXPECT_FLOAT_EQ(decoded.hardware.detection.min_detection_margin_db, -20.0f);
  // mission
  EXPECT_FLOAT_EQ(decoded.mission.orientation.scan_center_deg.az_deg, 15.0f);
  EXPECT_FLOAT_EQ(decoded.mission.orientation.scan_center_deg.el_deg, -2.0f);
  // policy
  EXPECT_EQ(decoded.policy.lifecycle.confirm_hits, 2U);
  EXPECT_EQ(decoded.policy.lifecycle.max_miss_before_lost, 3U);
  EXPECT_TRUE(decoded.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.kalman_measurement_noise_std, 5.5f);
  // jamming_sensitivity_profile
  EXPECT_EQ(decoded.jamming_sensitivity_profile, environment::JammingSensitivityProfile::kStrict);
  // environment (previously broken!)
  EXPECT_TRUE(decoded.environment.scenario_config.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.pressure_hpa, 1010.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.temperature_k, 290.0f);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.atmospheric_physics.relative_humidity, 0.6f);
  EXPECT_TRUE(
      decoded.environment.scenario_config.atmospheric_context.has_simulation_unix_seconds);
  EXPECT_EQ(decoded.environment.scenario_config.atmospheric_context.simulation_unix_seconds,
            1700000000LL);
  EXPECT_TRUE(decoded.environment.scenario_config.vegetation_scatter_physics.enable_physical_model);
  EXPECT_EQ(decoded.environment.scenario_config.vegetation_scatter_physics.cover_profile,
            environment::VegetationCoverProfile::kSparseWoodland);
  ASSERT_EQ(decoded.environment.scenario_config.jammer_sources.size(), 1U);
  EXPECT_EQ(decoded.environment.scenario_config.jammer_sources[0].technique,
            environment::JammingTechnique::kNoiseSuppression);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.jammer_sources[0].power_db, 25.0f);
  EXPECT_TRUE(decoded.environment.scenario_config.jammer_sources[0].has_direction_deg);
  EXPECT_FLOAT_EQ(decoded.environment.scenario_config.jammer_sources[0].azimuth_deg, 30.0f);
}

// ---------------------------------------------------------------------------
// RadarRuntimeConfigPatch
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, RuntimeConfigPatchPreservesAllFields) {
  config::RadarRuntimeConfigPatch patch;
  patch.has_policy = true;
  patch.policy.tracking.enable_kalman_filter = true;
  patch.policy.tracking.kalman_measurement_noise_std = 7.5f;
  patch.policy.lifecycle.confirm_hits = 1U;
  patch.has_scan_center_deg = true;
  patch.scan_center_deg.az_deg = 30.0f;
  patch.scan_center_deg.el_deg = -3.0f;
  patch.has_dwell_center_deg = true;
  patch.dwell_center_deg.az_deg = 31.0f;
  patch.dwell_center_deg.el_deg = -3.5f;
  patch.has_commanded_beamwidth_enabled = true;
  patch.commanded_beamwidth_enabled = true;
  patch.has_environment_runtime_config = true;
  patch.environment_runtime_config.has_jamming_sensitivity_profile = true;
  patch.environment_runtime_config.jamming_sensitivity_profile =
      environment::JammingSensitivityProfile::kStrict;
  patch.environment_runtime_config.has_scenario_config = true;
  patch.environment_runtime_config.scenario_config.atmospheric_physics.relative_humidity = 0.45f;

  const std::string bytes = EncodeRuntimeConfigPatchFlatbuffer(patch);
  ASSERT_FALSE(bytes.empty());

  config::RadarRuntimeConfigPatch decoded;
  std::string error;
  ASSERT_TRUE(DecodeRuntimeConfigPatchFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_TRUE(decoded.has_policy);
  EXPECT_TRUE(decoded.policy.tracking.enable_kalman_filter);
  EXPECT_FLOAT_EQ(decoded.policy.tracking.kalman_measurement_noise_std, 7.5f);
  EXPECT_EQ(decoded.policy.lifecycle.confirm_hits, 1U);
  EXPECT_TRUE(decoded.has_scan_center_deg);
  EXPECT_FLOAT_EQ(decoded.scan_center_deg.az_deg, 30.0f);
  EXPECT_FLOAT_EQ(decoded.scan_center_deg.el_deg, -3.0f);
  EXPECT_TRUE(decoded.has_dwell_center_deg);
  EXPECT_FLOAT_EQ(decoded.dwell_center_deg.az_deg, 31.0f);
  EXPECT_TRUE(decoded.has_commanded_beamwidth_enabled);
  EXPECT_TRUE(decoded.commanded_beamwidth_enabled);
  EXPECT_TRUE(decoded.has_environment_runtime_config);
  EXPECT_TRUE(decoded.environment_runtime_config.has_jamming_sensitivity_profile);
  EXPECT_EQ(decoded.environment_runtime_config.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
  EXPECT_TRUE(decoded.environment_runtime_config.has_scenario_config);
  EXPECT_FLOAT_EQ(
      decoded.environment_runtime_config.scenario_config.atmospheric_physics.relative_humidity,
      0.45f);
}

// ---------------------------------------------------------------------------
// EnvironmentSceneState
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, SceneStatePreservesAllFields) {
  environment::EnvironmentSceneState scene;
  scene.atmospheric_physics.enable_physical_model = true;
  scene.atmospheric_physics.pressure_hpa = 1005.0f;
  scene.atmospheric_physics.temperature_k = 285.0f;
  scene.atmospheric_physics.relative_humidity = 0.7f;
  scene.atmospheric_context.has_simulation_unix_seconds = true;
  scene.atmospheric_context.simulation_unix_seconds = 1600000000LL;
  scene.atmospheric_context.solar_flux_f107a = 155.0f;
  scene.vegetation_scatter_physics.enable_physical_model = true;
  scene.vegetation_scatter_physics.cover_profile =
      environment::VegetationCoverProfile::kDeciduousForest;

  environment::JammerEmitterState jammer;
  jammer.technique = environment::JammingTechnique::kNoiseSuppression;
  jammer.power_db = 20.0f;
  jammer.js_db = 6.0f;
  jammer.has_direction_deg = false;
  jammer.confidence = 0.9f;
  scene.jammer_emitters.push_back(jammer);

  const std::string bytes = EncodeSceneStateFlatbuffer(scene);
  ASSERT_FALSE(bytes.empty());

  environment::EnvironmentSceneState decoded;
  std::string error;
  ASSERT_TRUE(DecodeSceneStateFlatbuffer(bytes, &decoded, &error)) << error;

  EXPECT_TRUE(decoded.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(decoded.atmospheric_physics.pressure_hpa, 1005.0f);
  EXPECT_FLOAT_EQ(decoded.atmospheric_physics.relative_humidity, 0.7f);
  EXPECT_TRUE(decoded.atmospheric_context.has_simulation_unix_seconds);
  EXPECT_EQ(decoded.atmospheric_context.simulation_unix_seconds, 1600000000LL);
  EXPECT_FLOAT_EQ(decoded.atmospheric_context.solar_flux_f107a, 155.0f);
  EXPECT_TRUE(decoded.vegetation_scatter_physics.enable_physical_model);
  EXPECT_EQ(decoded.vegetation_scatter_physics.cover_profile,
            environment::VegetationCoverProfile::kDeciduousForest);
  ASSERT_EQ(decoded.jammer_emitters.size(), 1U);
  EXPECT_EQ(decoded.jammer_emitters[0].technique, environment::JammingTechnique::kNoiseSuppression);
  EXPECT_FLOAT_EQ(decoded.jammer_emitters[0].power_db, 20.0f);
  EXPECT_FLOAT_EQ(decoded.jammer_emitters[0].confidence, 0.9f);
  EXPECT_FALSE(decoded.jammer_emitters[0].has_direction_deg);
}

// ---------------------------------------------------------------------------
// FailureMarker
// ---------------------------------------------------------------------------

TEST(ArReplayCodecRoundtripTest, FailureMarkerPreservesAllFields) {
  oneq::replay::ReplayTraceFailure failure;
  failure.error_code = "AR_ASSERT";
  failure.message = "track pool overflow";
  failure.location = "RadarController::RunOnce";
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
  EXPECT_EQ(decoded.location, "RadarController::RunOnce");
  EXPECT_TRUE(decoded.has_cycle_index);
  EXPECT_EQ(decoded.cycle_index, 42U);
  EXPECT_TRUE(decoded.has_sim_time_sec);
  EXPECT_DOUBLE_EQ(decoded.sim_time_sec, 42.5);
  EXPECT_EQ(decoded.diagnostics_payload, "{\"track_count\":128}");
}

}  // namespace tests
}  // namespace session
}  // namespace airborne_radar
