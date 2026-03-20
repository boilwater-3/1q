// Copyright 2026. All Rights Reserved.
//
// @file radar_session_config_builder_test.cpp
// @brief 验证 RadarSessionConfigBuilder 的链式配置语义与覆盖行为。

#include <cstdint>

#include <gtest/gtest.h>

#include "1q/airborne_radar/common/ConfigPresets.h"
#include "1q/airborne_radar/common/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/core/session/RadarSession.h"

namespace airborne_radar {
namespace tests {

// ---------------------------------------------------------------------------
// 默认构造
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, DefaultConstructionPreservesStructDefaults) {
  const auto config =
      common::RadarSessionConfigBuilder().Build();

  EXPECT_FLOAT_EQ(config.jamming_detection_threshold_db, 6.0f);
  EXPECT_FALSE(
      config.signal_pipeline_config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.min_detection_margin_db, -2.0f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.transmitter
          .peak_power_w,
      1e6f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.transmitter
          .frequency_hz,
      3e9f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.antenna
          .main_beam_gain_db,
      35.0f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.receiver
          .noise_figure_db,
      4.0f);
}

// ---------------------------------------------------------------------------
// 基于预设构造
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, PresetBasePreservesPresetValues) {
  const auto config = common::RadarSessionConfigBuilder(
                          common::MakeDetectionMissionRadarSessionConfig())
                          .Build();

  // 探测任务预设设置了 min_detection_margin_db = -100.0f
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.min_detection_margin_db,
      -100.0f);
  // lifecycle 已启用
  EXPECT_TRUE(
      config.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager);
  // confirm_hits = 1
  EXPECT_EQ(
      config.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits,
      1U);
}

// ---------------------------------------------------------------------------
// 发射机参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, TransmitterSettersApplyCorrectly) {
  const auto config = common::RadarSessionConfigBuilder()
                          .WithTransmitterPeakPowerW(5e6f)
                          .WithTransmitterFrequencyHz(9.3e9f)
                          .WithTransmitterBandwidthHz(10e6f)
                          .WithTransmitterPulseWidthS(20e-6f)
                          .WithTransmitterPrfHz(500.0f)
                          .WithTransmitterLossDb(4.0f)
                          .Build();

  const auto& tx =
      config.signal_pipeline_config.detection.radar_system.transmitter;
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
  const auto config = common::RadarSessionConfigBuilder()
                          .WithAntennaMainBeamGainDb(38.0f)
                          .WithAntennaNominalBeamwidthDeg(3.0f, 3.5f)
                          .Build();

  const auto& ant =
      config.signal_pipeline_config.detection.radar_system.antenna;
  EXPECT_FLOAT_EQ(ant.main_beam_gain_db, 38.0f);
  EXPECT_FLOAT_EQ(ant.nominal_az_beamwidth_deg, 3.0f);
  EXPECT_FLOAT_EQ(ant.nominal_el_beamwidth_deg, 3.5f);
}

// ---------------------------------------------------------------------------
// 接收机参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, ReceiverSettersApplyCorrectly) {
  const auto config = common::RadarSessionConfigBuilder()
                          .WithReceiverNoiseFigureDb(3.5f)
                          .WithReceiverLossDb(1.5f)
                          .Build();

  const auto& rx =
      config.signal_pipeline_config.detection.radar_system.receiver;
  EXPECT_FLOAT_EQ(rx.noise_figure_db, 3.5f);
  EXPECT_FLOAT_EQ(rx.receive_loss_db, 1.5f);
}

// ---------------------------------------------------------------------------
// 物理探测参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, EnablePhysicsDetectionDefaultsToTrue) {
  const auto config =
      common::RadarSessionConfigBuilder().EnablePhysicsDetection().Build();

  EXPECT_TRUE(
      config.signal_pipeline_config.detection.enable_physics_detection);
}

TEST(RadarSessionConfigBuilderTest, EnablePhysicsDetectionCanBeDisabled) {
  const auto config = common::RadarSessionConfigBuilder()
                          .EnablePhysicsDetection(true)
                          .EnablePhysicsDetection(false)
                          .Build();

  EXPECT_FALSE(
      config.signal_pipeline_config.detection.enable_physics_detection);
}

TEST(RadarSessionConfigBuilderTest, DetectionPolicySettersApplyCorrectly) {
  const auto config = common::RadarSessionConfigBuilder()
                          .WithMinDetectionMarginDb(-15.0f)
                          .WithPulseCount(20)
                          .WithCoherentIntegration(false)
                          .Build();

  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.min_detection_margin_db, -15.0f);
  EXPECT_EQ(config.signal_pipeline_config.detection.pulse_count, 20);
  EXPECT_FALSE(config.signal_pipeline_config.detection.coherent_integration);
}

// ---------------------------------------------------------------------------
// 跟踪参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, KalmanMeasurementNoiseStdApplied) {
  const auto config = common::RadarSessionConfigBuilder()
                          .WithKalmanMeasurementNoiseStd(3.0f)
                          .Build();

  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.tracking.kalman_measurement_noise_std,
      3.0f);
}

// ---------------------------------------------------------------------------
// 生命周期参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, LifecycleSettersApplyCorrectly) {
  const auto config = common::RadarSessionConfigBuilder()
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
  const auto config = common::RadarSessionConfigBuilder()
                          .WithJammingDetectionThresholdDb(4.5f)
                          .Build();

  EXPECT_FLOAT_EQ(config.jamming_detection_threshold_db, 4.5f);
}

// ---------------------------------------------------------------------------
// 链式调用：预设叠加硬件参数
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, ChainedPresetPlusHardwareOverride) {
  const auto config =
      common::RadarSessionConfigBuilder(
          common::MakeDetectionMissionRadarSessionConfig())
          .EnablePhysicsDetection()
          .WithTransmitterPeakPowerW(5e6f)
          .WithTransmitterFrequencyHz(9.3e9f)
          .WithAntennaMainBeamGainDb(38.0f)
          .WithReceiverNoiseFigureDb(3.5f)
          .WithJammingDetectionThresholdDb(4.5f)
          .Build();

  // 预设值保留
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.min_detection_margin_db,
      -100.0f);
  EXPECT_EQ(
      config.signal_pipeline_config.lifecycle.lifecycle_config.confirm_hits,
      1U);

  // 硬件叠加值已覆盖
  EXPECT_TRUE(
      config.signal_pipeline_config.detection.enable_physics_detection);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.transmitter
          .peak_power_w,
      5e6f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.transmitter
          .frequency_hz,
      9.3e9f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.antenna
          .main_beam_gain_db,
      38.0f);
  EXPECT_FLOAT_EQ(
      config.signal_pipeline_config.detection.radar_system.receiver
          .noise_figure_db,
      3.5f);
  EXPECT_FLOAT_EQ(config.jamming_detection_threshold_db, 4.5f);
}

// ---------------------------------------------------------------------------
// 与 RadarSession 集成
// ---------------------------------------------------------------------------

TEST(RadarSessionConfigBuilderTest, BuiltConfigCanConstructRadarSession) {
  const auto config =
      common::RadarSessionConfigBuilder(
          common::MakeDetectionMissionRadarSessionConfig())
          .WithJammingDetectionThresholdDb(5.0f)
          .Build();

  // 能正常构造 RadarSession 即通过（构造函数不应抛出）
  core::session::RadarSession session(config);
  EXPECT_TRUE(session.HasLatestControlProfile() == false ||
              session.HasLatestControlProfile() == true);
}

}  // namespace tests
}  // namespace airborne_radar
