// Copyright 2026. All Rights Reserved.
//
// @file antenna_pattern_utils_test.cpp
// @brief 验证机载雷达天线方向图工程近似工具函数。

#include <gtest/gtest.h>

#include "airborne_radar/signal/detection/AntennaPatternRuntime.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

TEST(AntennaPatternUtilsTest, MainLobeGainDropsWithOffset) {
  config::engineering::AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  config::AzimuthElevationDeg beam_pointing_deg;
  AntennaLookOffsetDeg boresight_offset_deg;
  AntennaLookOffsetDeg off_axis_offset_deg;
  off_axis_offset_deg.delta_az_deg = 1.0f;
  const AntennaPatternSample boresight = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, boresight_offset_deg, beam_pointing_deg);
  const AntennaPatternSample off_axis = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, off_axis_offset_deg, beam_pointing_deg);

  EXPECT_TRUE(boresight.inside_main_lobe);
  EXPECT_TRUE(off_axis.inside_main_lobe);
  EXPECT_GT(boresight.gain_dbi, off_axis.gain_dbi);
}

TEST(AntennaPatternUtilsTest, WiderBeamwidthReducesSameOffsetAttenuation) {
  config::engineering::AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config::AzimuthElevationDeg beam_pointing_deg;
  AntennaLookOffsetDeg offset_deg;
  offset_deg.delta_az_deg = 1.0f;

  AntennaPatternBeamwidthDeg narrow_beamwidth;
  narrow_beamwidth.az_beamwidth_deg = 2.0f;
  narrow_beamwidth.el_beamwidth_deg = 2.0f;

  AntennaPatternBeamwidthDeg wide_beamwidth;
  wide_beamwidth.az_beamwidth_deg = 8.0f;
  wide_beamwidth.el_beamwidth_deg = 8.0f;

  const AntennaPatternSample narrow =
      EvaluateAntennaPattern(peak_gain_dbi, config, narrow_beamwidth, offset_deg, beam_pointing_deg);
  const AntennaPatternSample wide =
      EvaluateAntennaPattern(peak_gain_dbi, config, wide_beamwidth, offset_deg, beam_pointing_deg);

  EXPECT_LT(narrow.gain_dbi, wide.gain_dbi);
  EXPECT_GT(narrow.main_lobe_attenuation_db, wide.main_lobe_attenuation_db);
}

TEST(AntennaPatternUtilsTest, ScanLossIncreasesAwayFromBoresight) {
  config::engineering::AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config.scan_loss_coeff_db_per_deg2 = 0.02f;
  config.max_scan_loss_db = 6.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  config::AzimuthElevationDeg centered_beam_pointing_deg;
  config::AzimuthElevationDeg scanned_beam_pointing_deg;
  scanned_beam_pointing_deg.az_deg = 10.0f;
  AntennaLookOffsetDeg boresight_offset_deg;
  const AntennaPatternSample centered = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, boresight_offset_deg, centered_beam_pointing_deg);
  const AntennaPatternSample scanned = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, boresight_offset_deg, scanned_beam_pointing_deg);

  EXPECT_LT(scanned.gain_dbi, centered.gain_dbi);
  EXPECT_GT(scanned.scan_loss_db, centered.scan_loss_db);
}

TEST(AntennaPatternUtilsTest, ScanLossChangesWithBeamPointingWhileLookOffsetIsFixed) {
  config::engineering::AntennaPatternConfig config;
  config.scan_loss_coeff_db_per_deg2 = 0.01f;
  config.max_scan_loss_db = 20.0f;
  config.boresight_offset_deg.az_deg = 2.0f;
  config.boresight_offset_deg.el_deg = -1.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 6.0f;
  beamwidth.el_beamwidth_deg = 6.0f;

  AntennaLookOffsetDeg offset_deg;
  offset_deg.delta_az_deg = 0.5f;
  offset_deg.delta_el_deg = 0.25f;

  config::AzimuthElevationDeg beam_pointing_a;
  beam_pointing_a.az_deg = 2.0f;
  beam_pointing_a.el_deg = -1.0f;
  config::AzimuthElevationDeg beam_pointing_b;
  beam_pointing_b.az_deg = 12.0f;
  beam_pointing_b.el_deg = -1.0f;

  const AntennaPatternSample sample_a =
      EvaluateAntennaPattern(30.0f, config, beamwidth, offset_deg, beam_pointing_a);
  const AntennaPatternSample sample_b =
      EvaluateAntennaPattern(30.0f, config, beamwidth, offset_deg, beam_pointing_b);

  EXPECT_FLOAT_EQ(sample_a.scan_loss_db, 0.0f);
  EXPECT_GT(sample_b.scan_loss_db, sample_a.scan_loss_db);
  EXPECT_GT(sample_a.gain_dbi, sample_b.gain_dbi);
}

TEST(AntennaPatternUtilsTest, OutsideMainLobeUsesSidelobeFloor) {
  config::engineering::AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config.max_sidelobe_level_db = -18.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  // 近副瓣区（评审 2026-08-26 条14 sinc² 包络延拓）：u_az = 3/2 = 1.5，包络衰减
  // ≈7.6dB < 18dB → 钳制在最大旁瓣电平（峰值 − 18 = 12 dBi）。
  AntennaLookOffsetDeg sidelobe_offset_deg;
  sidelobe_offset_deg.delta_az_deg = 3.0f;
  config::AzimuthElevationDeg beam_pointing_deg;
  const AntennaPatternSample sample = EvaluateAntennaPattern(peak_gain_dbi, config, beamwidth,
                                                             sidelobe_offset_deg, beam_pointing_deg);

  EXPECT_FALSE(sample.inside_main_lobe);
  EXPECT_FALSE(sample.inside_back_lobe);
  EXPECT_FLOAT_EQ(sample.gain_dbi, 12.0f);

  // 零陷区：u_az = 10/2 = 5，sinc² 衰减 > 18dB → 严格低于钳制电平。
  AntennaLookOffsetDeg null_offset_deg;
  null_offset_deg.delta_az_deg = 10.0f;
  const AntennaPatternSample null_sample =
      EvaluateAntennaPattern(peak_gain_dbi, config, beamwidth, null_offset_deg, beam_pointing_deg);
  EXPECT_FALSE(null_sample.inside_main_lobe);
  EXPECT_LT(null_sample.gain_dbi, peak_gain_dbi + config.max_sidelobe_level_db - 1.0f);
}

TEST(AntennaPatternUtilsTest, BackLobeUsesConfiguredBacklobeLevel) {
  config::engineering::AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config.backlobe_level_db = -40.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AntennaLookOffsetDeg backlobe_offset_deg;
  backlobe_offset_deg.delta_az_deg = 120.0f;
  config::AzimuthElevationDeg beam_pointing_deg;
  const AntennaPatternSample sample = EvaluateAntennaPattern(peak_gain_dbi, config, beamwidth,
                                                             backlobe_offset_deg, beam_pointing_deg);

  EXPECT_TRUE(sample.inside_back_lobe);
  EXPECT_FLOAT_EQ(sample.gain_dbi, -10.0f);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
