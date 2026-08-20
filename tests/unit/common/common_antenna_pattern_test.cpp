// Copyright 2026. All Rights Reserved.
//
// @file common_antenna_pattern_test.cpp
// @brief 验证 common 天线方向图基础行为。

#include <gtest/gtest.h>

#include "common/radar/AntennaPatternRuntime.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

TEST(CommonAntennaPatternTest, IsInsideMainLobeUsesHalfBeamwidth) {
  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AntennaLookOffsetDeg inside;
  inside.delta_az_deg = 1.0f;
  inside.delta_el_deg = 1.0f;
  EXPECT_TRUE(IsInsideMainLobe(beamwidth, inside));

  AntennaLookOffsetDeg outside;
  outside.delta_az_deg = 3.0f;
  outside.delta_el_deg = 0.0f;
  EXPECT_FALSE(IsInsideMainLobe(beamwidth, outside));
}

TEST(CommonAntennaPatternTest, EvaluateAntennaPatternReturnsSample) {
  AntennaPatternConfig config;
  config.model_type = AntennaPatternModelType::kGaussianMainLobe;
  config.max_sidelobe_level_db = -20.0f;
  config.backlobe_level_db = -35.0f;

  AntennaPatternBeamwidthDeg beamwidth;
  beamwidth.az_beamwidth_deg = 4.0f;
  beamwidth.el_beamwidth_deg = 4.0f;

  AntennaLookOffsetDeg offset;
  offset.delta_az_deg = 0.0f;
  offset.delta_el_deg = 0.0f;

  AzimuthElevationDeg pointing;
  const AntennaPatternSample sample =
      EvaluateAntennaPattern(38.0f, config, beamwidth, offset, pointing);
  EXPECT_TRUE(sample.inside_main_lobe);
  EXPECT_NEAR(sample.gain_dbi, 38.0f, 1e-5f);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
