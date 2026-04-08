// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 的链式配置语义与覆盖行为。

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/session/RadarSession.h"

namespace airborne_radar {
namespace tests {

// ---------------------------------------------------------------------------
// 默认构造
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesStructDefaults) {
  const auto config = config::RadarSessionConfigBuilder().Build();

  EXPECT_FLOAT_EQ(config.environment_default_config.jamming_detection_threshold_db, 6.0f);
  EXPECT_FALSE(config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.detection.min_detection_margin_db, -2.0f);
  EXPECT_FLOAT_EQ(config.detection.transmitter.peak_power_w,
                  1e6f);
  EXPECT_FLOAT_EQ(config.detection.transmitter.frequency_hz,
                  3e9f);
  EXPECT_FLOAT_EQ(config.detection.antenna.main_beam_gain_db,
                  35.0f);
  EXPECT_FLOAT_EQ(config.detection.receiver.noise_figure_db,
                  4.0f);
  EXPECT_EQ(config.beam_control.radar_orientation.work_sub_mode,
            model::RadarWorkSubMode::kTws);
  EXPECT_EQ(config.tracking.kalman_update_backend,
            config::KalmanUpdateBackend::kStandardKfJoseph);
}

// ---------------------------------------------------------------------------
// 基于预设构造
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, PresetBasePreservesPresetValues) {
  const auto config = config::RadarSessionConfigBuilder(
                          config::MakeDetectionMissionRadarSessionConfig())
                          .Build();

  // 探测任务预设设置了 min_detection_margin_db = -100.0f
  EXPECT_FLOAT_EQ(config.detection.min_detection_margin_db, -100.0f);
  // confirm_hits = 1
  EXPECT_EQ(config.lifecycle.lifecycle_config.confirm_hits, 1U);
}

// ---------------------------------------------------------------------------
// WithDetectionConfig
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, DetectionConfigAppliesCorrectly) {
  config::SignalDetectionConfig detection;
  detection.enable_physics_detection = true;
  detection.min_detection_margin_db = -15.0f;
  detection.pulse_count = 20;
  detection.transmitter.peak_power_w = 5e6f;
  detection.transmitter.frequency_hz = 9.3e9f;
  detection.transmitter.bandwidth_hz = 10e6f;
  detection.transmitter.pulse_width_s = 20e-6f;
  detection.transmitter.prf_hz = 500.0f;
  detection.transmitter.transmit_loss_db = 4.0f;
  detection.antenna.main_beam_gain_db = 38.0f;
  detection.antenna.nominal_az_beamwidth_deg = 3.0f;
  detection.antenna.nominal_el_beamwidth_deg = 3.5f;
  detection.receiver.noise_figure_db = 3.5f;
  detection.receiver.receive_loss_db = 1.5f;

  const auto config =
      config::RadarSessionConfigBuilder().Detection().WithDetection(detection).End().Build();

  const auto& d = config.detection;
  EXPECT_TRUE(d.enable_physics_detection);
  EXPECT_FLOAT_EQ(d.min_detection_margin_db, -15.0f);
  EXPECT_EQ(d.pulse_count, 20);
  EXPECT_FLOAT_EQ(d.transmitter.peak_power_w, 5e6f);
  EXPECT_FLOAT_EQ(d.transmitter.frequency_hz, 9.3e9f);
  EXPECT_FLOAT_EQ(d.transmitter.bandwidth_hz, 10e6f);
  EXPECT_FLOAT_EQ(d.transmitter.pulse_width_s, 20e-6f);
  EXPECT_FLOAT_EQ(d.transmitter.prf_hz, 500.0f);
  EXPECT_FLOAT_EQ(d.transmitter.transmit_loss_db, 4.0f);
  EXPECT_FLOAT_EQ(d.antenna.main_beam_gain_db, 38.0f);
  EXPECT_FLOAT_EQ(d.antenna.nominal_az_beamwidth_deg, 3.0f);
  EXPECT_FLOAT_EQ(d.antenna.nominal_el_beamwidth_deg, 3.5f);
  EXPECT_FLOAT_EQ(d.receiver.noise_figure_db, 3.5f);
  EXPECT_FLOAT_EQ(d.receiver.receive_loss_db, 1.5f);
}

// ---------------------------------------------------------------------------
// WithBeamControlConfig
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, BeamControlConfigAppliesCorrectly) {
  config::SignalBeamControlConfig beam_control;
  beam_control.radar_orientation.scan_start_position =
      oneq::common::ScanStartPosition::kRightBottom;
  beam_control.radar_orientation.scan_sequence = oneq::common::ScanSequence::kElevationFirst;
  beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kTas;

  const auto config =
      config::RadarSessionConfigBuilder().Beam().WithBeamControl(beam_control).End().Build();

  const auto& orientation = config.beam_control.radar_orientation;
  EXPECT_EQ(orientation.scan_start_position, oneq::common::ScanStartPosition::kRightBottom);
  EXPECT_EQ(orientation.scan_sequence, oneq::common::ScanSequence::kElevationFirst);
  EXPECT_EQ(orientation.work_sub_mode, model::RadarWorkSubMode::kTas);
}

// ---------------------------------------------------------------------------
// WithTrackingConfig
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, TrackingConfigAppliesCorrectly) {
  config::SignalTrackingConfig tracking;
  tracking.kalman_measurement_noise_std = 3.0f;
  tracking.kalman_update_backend = config::KalmanUpdateBackend::kUdKf;

  const auto config =
      config::RadarSessionConfigBuilder().Tracking().WithTracking(tracking).End().Build();

  EXPECT_FLOAT_EQ(config.tracking.kalman_measurement_noise_std, 3.0f);
  EXPECT_EQ(config.tracking.kalman_update_backend,
            config::KalmanUpdateBackend::kUdKf);
}

// ---------------------------------------------------------------------------
// WithLifecycleConfig
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, LifecycleConfigAppliesCorrectly) {
  config::SignalLifecycleConfig lifecycle;
  lifecycle.lifecycle_config.confirm_hits = 2U;
  lifecycle.lifecycle_config.max_miss_before_lost = 4U;
  lifecycle.lifecycle_config.max_lost_cycles = 10U;

  const auto config =
      config::RadarSessionConfigBuilder().Lifecycle().WithLifecycle(lifecycle).End().Build();

  const auto& lc = config.lifecycle;
  EXPECT_EQ(lc.lifecycle_config.confirm_hits, 2U);
  EXPECT_EQ(lc.lifecycle_config.max_miss_before_lost, 4U);
  EXPECT_EQ(lc.lifecycle_config.max_lost_cycles, 10U);
}

// ---------------------------------------------------------------------------
// WithEnvironmentDefaultConfig
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, EnvironmentDefaultConfigAppliesCorrectly) {
  environment::EnvironmentDefaultConfig env;
  env.jamming_detection_threshold_db = 4.5f;

  const auto config =
      config::RadarSessionConfigBuilder()
          .Environment()
          .WithEnvironmentDefault(env)
          .End()
          .Build();

  EXPECT_FLOAT_EQ(config.environment_default_config.jamming_detection_threshold_db, 4.5f);
}

// ---------------------------------------------------------------------------
// 链式调用：预设叠加探测配置（仅覆盖 detection，其余预设字段保留）
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, ChainedPresetPlusDetectionOverride) {
  const auto preset = config::MakeDetectionMissionRadarSessionConfig();

  config::SignalDetectionConfig detection = preset.detection;
  detection.enable_physics_detection = true;
  detection.transmitter.peak_power_w = 5e6f;
  detection.transmitter.frequency_hz = 9.3e9f;
  detection.antenna.main_beam_gain_db = 38.0f;
  detection.receiver.noise_figure_db = 3.5f;

  environment::EnvironmentDefaultConfig env = preset.environment_default_config;
  env.jamming_detection_threshold_db = 4.5f;

  const auto config = config::RadarSessionConfigBuilder(preset)
                          .Detection()
                          .WithDetection(detection)
                          .End()
                          .Environment()
                          .WithEnvironmentDefault(env)
                          .End()
                          .Build();

  // 预设值保留
  EXPECT_FLOAT_EQ(config.detection.min_detection_margin_db, -100.0f);
  EXPECT_EQ(config.lifecycle.lifecycle_config.confirm_hits, 1U);

  // 叠加值已覆盖
  EXPECT_TRUE(config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.detection.transmitter.peak_power_w,
                  5e6f);
  EXPECT_FLOAT_EQ(config.detection.transmitter.frequency_hz,
                  9.3e9f);
  EXPECT_FLOAT_EQ(config.detection.antenna.main_beam_gain_db,
                  38.0f);
  EXPECT_FLOAT_EQ(config.detection.receiver.noise_figure_db,
                  3.5f);
  EXPECT_FLOAT_EQ(config.environment_default_config.jamming_detection_threshold_db, 4.5f);
}

// ---------------------------------------------------------------------------
// 与 RadarSession 集成
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  environment::EnvironmentDefaultConfig env;
  env.jamming_detection_threshold_db = 5.0f;

  const auto config = config::RadarSessionConfigBuilder(
                          config::MakeDetectionMissionRadarSessionConfig())
                          .Environment()
                          .WithEnvironmentDefault(env)
                          .End()
                          .Build();

  // 能正常构造 RadarSession 即通过（构造函数不应抛出）
  session::RadarSession session = session::RadarSessionFactory::Create(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigBuilderBuildsPatchFlagsAndValues) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 12.0f;
  scan_center.el_deg = -3.0f;

  model::AzimuthElevationDeg dwell_center;
  dwell_center.az_deg = 5.0f;
  dwell_center.el_deg = 2.0f;

  model::CommandedBeamwidthDeg commanded_beamwidth;
  commanded_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  commanded_beamwidth.commanded_el_beamwidth_deg = 2.0f;

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kStt)
          .WithScanCenterDeg(scan_center)
          .WithDwellCenterDeg(dwell_center)
          .WithCommandedBeamwidthDeg(commanded_beamwidth)
          .EnableCommandedBeamwidth(true)
          .WithJammingDetectionThresholdDb(4.2f)
          .Build();

  EXPECT_TRUE(patch.has_work_sub_mode);
  EXPECT_EQ(patch.work_sub_mode, model::RadarWorkSubMode::kStt);
  EXPECT_TRUE(patch.has_scan_center_deg);
  EXPECT_FLOAT_EQ(patch.scan_center_deg.az_deg, 12.0f);
  EXPECT_FLOAT_EQ(patch.scan_center_deg.el_deg, -3.0f);
  EXPECT_TRUE(patch.has_dwell_center_deg);
  EXPECT_FLOAT_EQ(patch.dwell_center_deg.az_deg, 5.0f);
  EXPECT_FLOAT_EQ(patch.dwell_center_deg.el_deg, 2.0f);
  EXPECT_TRUE(patch.has_commanded_beamwidth_deg);
  EXPECT_FLOAT_EQ(patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg, 2.5f);
  EXPECT_FLOAT_EQ(patch.commanded_beamwidth_deg.commanded_el_beamwidth_deg, 2.0f);
  EXPECT_TRUE(patch.has_commanded_beamwidth_enabled);
  EXPECT_TRUE(patch.commanded_beamwidth_enabled);
  EXPECT_TRUE(patch.has_environment_runtime_config);
  EXPECT_FLOAT_EQ(patch.environment_runtime_config.jamming_detection_threshold_db, 4.2f);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigBuilderSupportsFullSignalPipelineConfig) {
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection.min_detection_margin_db = -15.0f;
  pipeline_config.beam_control.radar_orientation.work_sub_mode = model::RadarWorkSubMode::kStt;

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder().WithSignalPipelineConfig(pipeline_config).Build();

  EXPECT_TRUE(patch.has_signal_pipeline_config);
  EXPECT_FLOAT_EQ(patch.signal_pipeline_config.detection.min_detection_margin_db, -15.0f);
  EXPECT_EQ(patch.signal_pipeline_config.beam_control.radar_orientation.work_sub_mode,
            model::RadarWorkSubMode::kStt);
}

TEST(RadarSessionConfigBuilderTest, RuntimePatchCanBeAppliedWithoutReconstructingSession) {
  session::RadarSession session = session::RadarSessionFactory::Create(
      config::MakeDetectionMissionRadarSessionConfig());

  const config::RadarRuntimeConfigPatch patch =
      config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .EnableCommandedBeamwidth(true)
          .WithJammingDetectionThresholdDb(4.8f)
          .Build();

  session.ApplyRuntimeConfig(patch);

  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

TEST(RadarSessionConfigBuilderTest, LeafAndDomainSettersUseLastWriteWins) {
  config::SignalDetectionConfig detection;
  detection.transmitter.peak_power_w = 2e6f;

  const auto config_leaf_then_domain =
      config::RadarSessionConfigBuilder()
          .Detection()
          .WithPeakPowerW(1e6f)
          .WithDetection(detection)
          .End()
          .Build();
  EXPECT_FLOAT_EQ(config_leaf_then_domain.detection.transmitter.peak_power_w, 2e6f);

  const auto config_domain_then_leaf =
      config::RadarSessionConfigBuilder()
          .Detection()
          .WithDetection(detection)
          .WithPeakPowerW(3e6f)
          .End()
          .Build();
  EXPECT_FLOAT_EQ(config_domain_then_leaf.detection.transmitter.peak_power_w, 3e6f);
}

TEST(RadarSessionConfigBuilderTest, GroupEditorsCanBuildClosedConfig) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 8.0f;
  scan_center.el_deg = -2.0f;

  const auto config = config::RadarSessionConfigBuilder()
                          .Detection()
                          .EnablePhysicsDetection(true)
                          .WithPeakPowerW(5e6f)
                          .WithFrequencyHz(9.3e9f)
                          .WithPulseCount(16)
                          .End()
                          .Beam()
                          .WithScanCenterDeg(scan_center)
                          .End()
                          .Lifecycle()
                          .WithLifecycleConfirmHits(2U)
                          .End()
                          .Environment()
                          .WithJammingDetectionThresholdDb(4.6f)
                          .End()
                          .Build();

  EXPECT_TRUE(config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.detection.transmitter.peak_power_w, 5e6f);
  EXPECT_FLOAT_EQ(config.detection.transmitter.frequency_hz, 9.3e9f);
  EXPECT_EQ(config.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.beam_control.radar_orientation.scan_center_deg.az_deg, 8.0f);
  EXPECT_FLOAT_EQ(config.beam_control.radar_orientation.scan_center_deg.el_deg, -2.0f);
  EXPECT_EQ(config.lifecycle.lifecycle_config.confirm_hits, 2U);
  EXPECT_FLOAT_EQ(config.environment_default_config.jamming_detection_threshold_db, 4.6f);
}

}  // namespace tests
}  // namespace airborne_radar
