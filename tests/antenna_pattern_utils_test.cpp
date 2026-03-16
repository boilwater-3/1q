// Copyright 2026. All Rights Reserved.
//
// Description: 验证机载雷达天线方向图工程近似工具函数。

#include "1q/airborne_radar/common/AntennaPatternUtils.h"

#include <gtest/gtest.h>

namespace airborne_radar {
namespace common {
namespace {

TEST(AntennaPatternUtilsTest, MainLobeGainDropsWithOffset) {
  AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AzimuthElevationDeg scan_center_deg;
  AntennaLookOffsetDeg boresight_offset_deg;
  AntennaLookOffsetDeg off_axis_offset_deg;
  off_axis_offset_deg.delta_az_deg = 1.0f;
  const AntennaPatternSample boresight = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, boresight_offset_deg,
      scan_center_deg);
  const AntennaPatternSample off_axis = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, off_axis_offset_deg, scan_center_deg);

  EXPECT_TRUE(boresight.inside_main_lobe);
  EXPECT_TRUE(off_axis.inside_main_lobe);
  EXPECT_GT(boresight.gain_dbi, off_axis.gain_dbi);
}

TEST(AntennaPatternUtilsTest, WiderBeamwidthReducesSameOffsetAttenuation) {
  AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  AzimuthElevationDeg scan_center_deg;
  AntennaLookOffsetDeg offset_deg;
  offset_deg.delta_az_deg = 1.0f;

  AntennaPatternBeamwidthDeg narrow_beamwidth;
  narrow_beamwidth.az_beamwidth_deg = 2.0f;
  narrow_beamwidth.el_beamwidth_deg = 2.0f;

  AntennaPatternBeamwidthDeg wide_beamwidth;
  wide_beamwidth.az_beamwidth_deg = 8.0f;
  wide_beamwidth.el_beamwidth_deg = 8.0f;

  const AntennaPatternSample narrow = EvaluateAntennaPattern(
      peak_gain_dbi, config, narrow_beamwidth, offset_deg, scan_center_deg);
  const AntennaPatternSample wide = EvaluateAntennaPattern(
      peak_gain_dbi, config, wide_beamwidth, offset_deg, scan_center_deg);

  EXPECT_LT(narrow.gain_dbi, wide.gain_dbi);
  EXPECT_GT(narrow.main_lobe_attenuation_db, wide.main_lobe_attenuation_db);
}

TEST(AntennaPatternUtilsTest, ScanLossIncreasesAwayFromBoresight) {
  AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config.scan_loss_coeff_db_per_deg2 = 0.02f;
  config.max_scan_loss_db = 6.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AzimuthElevationDeg centered_scan_deg;
  AzimuthElevationDeg scanned_scan_deg;
  scanned_scan_deg.az_deg = 10.0f;
  AntennaLookOffsetDeg boresight_offset_deg;
  const AntennaPatternSample centered = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, boresight_offset_deg,
      centered_scan_deg);
  const AntennaPatternSample scanned = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, boresight_offset_deg,
      scanned_scan_deg);

  EXPECT_LT(scanned.gain_dbi, centered.gain_dbi);
  EXPECT_GT(scanned.scan_loss_db, centered.scan_loss_db);
}

TEST(AntennaPatternUtilsTest, OutsideMainLobeUsesSidelobeFloor) {
  AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config.max_sidelobe_level_db = -18.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AntennaLookOffsetDeg sidelobe_offset_deg;
  sidelobe_offset_deg.delta_az_deg = 10.0f;
  AzimuthElevationDeg scan_center_deg;
  const AntennaPatternSample sample = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, sidelobe_offset_deg, scan_center_deg);

  EXPECT_FALSE(sample.inside_main_lobe);
  EXPECT_FALSE(sample.inside_back_lobe);
  EXPECT_FLOAT_EQ(sample.gain_dbi, 12.0f);
}

TEST(AntennaPatternUtilsTest, BackLobeUsesConfiguredBacklobeLevel) {
  AntennaPatternConfig config;
  const float peak_gain_dbi = 30.0f;
  config.backlobe_level_db = -40.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AntennaLookOffsetDeg backlobe_offset_deg;
  backlobe_offset_deg.delta_az_deg = 120.0f;
  AzimuthElevationDeg scan_center_deg;
  const AntennaPatternSample sample = EvaluateAntennaPattern(
      peak_gain_dbi, config, beamwidth, backlobe_offset_deg, scan_center_deg);

  EXPECT_TRUE(sample.inside_back_lobe);
  EXPECT_FLOAT_EQ(sample.gain_dbi, -10.0f);
}

}  // namespace
}  // namespace common
}  // namespace airborne_radar
