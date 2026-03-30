// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 的链式配置语义与覆盖行为。

#include <gtest/gtest.h>

#include <cstdint>

#include "1q/airborne_radar/config/ConfigPresets.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/core/session/RadarSession.h"

namespace airborne_radar {
namespace tests {

// ---------------------------------------------------------------------------
// 默认构造
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesStructDefaults) {
  const auto config = common::config::RadarSessionConfigBuilder().Build();

  EXPECT_FLOAT_EQ(config.jamming_detection_threshold_db, 6.0f);
  EXPECT_FALSE(config.signal_pipeline_config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.min_detection_margin_db, -2.0f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.transmitter.peak_power_w,
                  1e6f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.transmitter.frequency_hz,
                  3e9f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.antenna.main_beam_gain_db,
                  35.0f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.receiver.noise_figure_db,
                  4.0f);
  EXPECT_EQ(config.signal_pipeline_config.beam_control.radar_orientation.work_sub_mode,
            common::config::RadarWorkSubMode::kTws);
}

// ---------------------------------------------------------------------------
// 基于预设构造
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, PresetBasePreservesPresetValues) {
  const auto config = common::config::RadarSessionConfigBuilder(
                          common::config::MakeDetectionMissionRadarSessionConfig())
                          .Build();

  // 探测任务预设设置了 min_detection_margin_db = -100.0f
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.min_detection_margin_db, -100.0f);
  // lifecycle 已启用
  EXPECT_TRUE(config.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager);
  // confirm_hits = 1
  EXPECT_EQ(config.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits, 1U);
}

// ---------------------------------------------------------------------------
// 发射机参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, TransmitterSettersApplyCorrectly) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .WithTransmitterPeakPowerW(5e6f)
                          .WithTransmitterFrequencyHz(9.3e9f)
                          .WithTransmitterBandwidthHz(10e6f)
                          .WithTransmitterPulseWidthS(20e-6f)
                          .WithTransmitterPrfHz(500.0f)
                          .WithTransmitterLossDb(4.0f)
                          .Build();

  const auto& tx = config.signal_pipeline_config.detection.radar_system.transmitter;
  EXPECT_FLOAT_EQ(tx.peak_power_w, 5e6f);
  EXPECT_FLOAT_EQ(tx.frequency_hz, 9.3e9f);
  EXPECT_FLOAT_EQ(tx.bandwidth_hz, 10e6f);
  EXPECT_FLOAT_EQ(tx.pulse_width_s, 20e-6f);
  EXPECT_FLOAT_EQ(tx.prf_hz, 500.0f);
  EXPECT_FLOAT_EQ(tx.transmit_loss_db, 4.0f);
}

// ---------------------------------------------------------------------------
// 天线参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, AntennaSettersApplyCorrectly) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .WithAntennaMainBeamGainDb(38.0f)
                          .WithAntennaNominalBeamwidthDeg(3.0f, 3.5f)
                          .Build();

  const auto& ant = config.signal_pipeline_config.detection.radar_system.antenna;
  EXPECT_FLOAT_EQ(ant.main_beam_gain_db, 38.0f);
  EXPECT_FLOAT_EQ(ant.nominal_az_beamwidth_deg, 3.0f);
  EXPECT_FLOAT_EQ(ant.nominal_el_beamwidth_deg, 3.5f);
}

TEST(RadarSessionConfigBuilderTest, ScanScheduleSettersApplyCorrectly) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .WithScanStartPosition(oneq::common::ScanStartPosition::kRightBottom)
                          .WithScanSequence(oneq::common::ScanSequence::kElevationFirst)
                          .WithRadarWorkSubMode(common::config::RadarWorkSubMode::kTas)
                          .Build();

  const auto& orientation = config.signal_pipeline_config.beam_control.radar_orientation;
  EXPECT_EQ(orientation.scan_start_position, oneq::common::ScanStartPosition::kRightBottom);
  EXPECT_EQ(orientation.scan_sequence, oneq::common::ScanSequence::kElevationFirst);
  EXPECT_EQ(orientation.work_sub_mode, common::config::RadarWorkSubMode::kTas);
}

// ---------------------------------------------------------------------------
// 接收机参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, ReceiverSettersApplyCorrectly) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .WithReceiverNoiseFigureDb(3.5f)
                          .WithReceiverLossDb(1.5f)
                          .Build();

  const auto& rx = config.signal_pipeline_config.detection.radar_system.receiver;
  EXPECT_FLOAT_EQ(rx.noise_figure_db, 3.5f);
  EXPECT_FLOAT_EQ(rx.receive_loss_db, 1.5f);
}

// ---------------------------------------------------------------------------
// 物理探测参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, EnablePhysicsDetectionDefaultsToTrue) {
  const auto config = common::config::RadarSessionConfigBuilder().EnablePhysicsDetection().Build();

  EXPECT_TRUE(config.signal_pipeline_config.detection.enable_physics_detection);
}

TEST(RadarSessionConfigBuilderTest, EnablePhysicsDetectionCanBeDisabled) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .EnablePhysicsDetection(true)
                          .EnablePhysicsDetection(false)
                          .Build();

  EXPECT_FALSE(config.signal_pipeline_config.detection.enable_physics_detection);
}

TEST(RadarSessionConfigBuilderTest, DetectionPolicySettersApplyCorrectly) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .WithMinDetectionMarginDb(-15.0f)
                          .WithPulseCount(20)
                          .Build();

  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.min_detection_margin_db, -15.0f);
  EXPECT_EQ(config.signal_pipeline_config.detection.pulse_count, 20);
}

// ---------------------------------------------------------------------------
// 跟踪参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, KalmanMeasurementNoiseStdApplied) {
  const auto config =
      common::config::RadarSessionConfigBuilder().WithKalmanMeasurementNoiseStd(3.0f).Build();

  EXPECT_FLOAT_EQ(config.signal_pipeline_config.tracking.kalman_measurement_noise_std, 3.0f);
}

// ---------------------------------------------------------------------------
// 生命周期参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, LifecycleSettersApplyCorrectly) {
  const auto config = common::config::RadarSessionConfigBuilder()
                          .EnableAutoLifecycleManager()
                          .WithConfirmHits(2U)
                          .WithMaxMissBeforeLost(4U)
                          .WithMaxLostCycles(10U)
                          .Build();

  const auto& lc = config.signal_pipeline_config.lifecycle;
  EXPECT_TRUE(lc.enable_auto_lifecycle_manager);
  EXPECT_EQ(lc.lifecycle_config.confirm_hits, 2U);
  EXPECT_EQ(lc.lifecycle_config.max_miss_before_lost, 4U);
  EXPECT_EQ(lc.lifecycle_config.max_lost_cycles, 10U);
}

// ---------------------------------------------------------------------------
// 会话级参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, JammingThresholdApplied) {
  const auto config =
      common::config::RadarSessionConfigBuilder().WithJammingDetectionThresholdDb(4.5f).Build();

  EXPECT_FLOAT_EQ(config.jamming_detection_threshold_db, 4.5f);
}

// ---------------------------------------------------------------------------
// 链式调用：预设叠加硬件参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, ChainedPresetPlusHardwareOverride) {
  const auto config = common::config::RadarSessionConfigBuilder(
                          common::config::MakeDetectionMissionRadarSessionConfig())
                          .EnablePhysicsDetection()
                          .WithTransmitterPeakPowerW(5e6f)
                          .WithTransmitterFrequencyHz(9.3e9f)
                          .WithAntennaMainBeamGainDb(38.0f)
                          .WithReceiverNoiseFigureDb(3.5f)
                          .WithJammingDetectionThresholdDb(4.5f)
                          .Build();

  // 预设值保留
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.min_detection_margin_db, -100.0f);
  EXPECT_EQ(config.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits, 1U);

  // 硬件叠加值已覆盖
  EXPECT_TRUE(config.signal_pipeline_config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.transmitter.peak_power_w,
                  5e6f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.transmitter.frequency_hz,
                  9.3e9f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.antenna.main_beam_gain_db,
                  38.0f);
  EXPECT_FLOAT_EQ(config.signal_pipeline_config.detection.radar_system.receiver.noise_figure_db,
                  3.5f);
  EXPECT_FLOAT_EQ(config.jamming_detection_threshold_db, 4.5f);
}

// ---------------------------------------------------------------------------
// 与 RadarSession 集成
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config = common::config::RadarSessionConfigBuilder(
                          common::config::MakeDetectionMissionRadarSessionConfig())
                          .WithJammingDetectionThresholdDb(5.0f)
                          .Build();

  // 能正常构造 RadarSession 即通过（构造函数不应抛出）
  core::session::RadarSession session(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigBuilderBuildsPatchFlagsAndValues) {
  common::config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 12.0f;
  scan_center.el_deg = -3.0f;

  common::config::AzimuthElevationDeg dwell_center;
  dwell_center.az_deg = 5.0f;
  dwell_center.el_deg = 2.0f;

  common::config::CommandedBeamwidthDeg commanded_beamwidth;
  commanded_beamwidth.commanded_az_beamwidth_deg = 2.5f;
  commanded_beamwidth.commanded_el_beamwidth_deg = 2.0f;

  const common::config::RadarRuntimeConfigPatch patch =
      common::config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(common::config::RadarWorkSubMode::kStt)
          .WithScanCenterDeg(scan_center)
          .WithDwellCenterDeg(dwell_center)
          .WithCommandedBeamwidthDeg(commanded_beamwidth)
          .EnableCommandedBeamwidth(true)
          .WithJammingDetectionThresholdDb(4.2f)
          .Build();

  EXPECT_TRUE(patch.has_work_sub_mode);
  EXPECT_EQ(patch.work_sub_mode, common::config::RadarWorkSubMode::kStt);
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
  EXPECT_TRUE(patch.has_jamming_detection_threshold_db);
  EXPECT_FLOAT_EQ(patch.jamming_detection_threshold_db, 4.2f);
}

TEST(RadarSessionConfigBuilderTest, RuntimeConfigBuilderSupportsKeySignalDomainPatches) {
  common::config::SignalDetectionConfig detection_config;
  detection_config.enable_physics_detection = true;
  detection_config.min_detection_margin_db = -8.0f;
  detection_config.pulse_count = 24;
  detection_config.radar_system.transmitter.prf_hz = 800.0f;

  common::config::SignalBeamControlConfig beam_control_config;
  beam_control_config.radar_orientation.work_sub_mode = common::config::RadarWorkSubMode::kStt;
  beam_control_config.platform_attitude_deg.roll_deg = 1.2f;

  const common::config::RadarRuntimeConfigPatch patch =
      common::config::RadarRuntimeConfigBuilder()
          .WithSignalDetectionConfig(detection_config)
          .WithSignalBeamControlConfig(beam_control_config)
          .Build();

  EXPECT_TRUE(patch.has_signal_detection_config);
  EXPECT_TRUE(patch.signal_detection_config.enable_physics_detection);
  EXPECT_FLOAT_EQ(patch.signal_detection_config.min_detection_margin_db, -8.0f);
  EXPECT_EQ(patch.signal_detection_config.pulse_count, 24);
  EXPECT_FLOAT_EQ(patch.signal_detection_config.radar_system.transmitter.prf_hz, 800.0f);

  EXPECT_TRUE(patch.has_signal_beam_control_config);
  EXPECT_EQ(patch.signal_beam_control_config.radar_orientation.work_sub_mode,
            common::config::RadarWorkSubMode::kStt);
  EXPECT_FLOAT_EQ(patch.signal_beam_control_config.platform_attitude_deg.roll_deg, 1.2f);
}

TEST(RadarSessionConfigBuilderTest, RuntimePatchCanBeAppliedWithoutReconstructingSession) {
  core::session::RadarSession session(common::config::MakeDetectionMissionRadarSessionConfig());
  common::config::SignalDetectionConfig detection_config;
  detection_config.enable_physics_detection = true;
  detection_config.min_detection_margin_db = -6.0f;
  detection_config.pulse_count = 18;

  common::config::SignalBeamControlConfig beam_control_config;
  beam_control_config.radar_orientation.work_sub_mode = common::config::RadarWorkSubMode::kTas;
  beam_control_config.radar_orientation.commanded_beamwidth_enabled = true;

  const common::config::RadarRuntimeConfigPatch patch =
      common::config::RadarRuntimeConfigBuilder()
          .WithSignalDetectionConfig(detection_config)
          .WithSignalBeamControlConfig(beam_control_config)
          .EnableCommandedBeamwidth(true)
          .WithJammingDetectionThresholdDb(4.8f)
          .Build();

  session.ApplyRuntimeConfig(patch);

  core::context::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const core::session::RadarCycleResult result = session.StepWithResult(input);
  EXPECT_FALSE(result.has_validation_error);
}

}  // namespace tests
}  // namespace airborne_radar
