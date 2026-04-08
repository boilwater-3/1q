// Copyright 2026. All Rights Reserved.
//
// @file beamwidth_resolution_test.cpp
// @brief 验证名义波束宽度与指令态波束宽度的解析规则。

#include <gtest/gtest.h>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"

namespace airborne_radar {
namespace tests {

using model::RadarOrientationConfig;
using config::AntennaConfig;
using signal::detection::EffectiveBeamwidthDeg;
using signal::detection::ResolveEffectiveBeamwidth;

TEST(BeamwidthResolutionTest, FallsBackToNominalWhenCommandedDisabled) {
  AntennaConfig antenna_config;
  antenna_config.nominal_az_beamwidth_deg = 3.5f;
  antenna_config.nominal_el_beamwidth_deg = 5.5f;

  RadarOrientationConfig orientation_config;
  orientation_config.commanded_beamwidth_enabled = false;
  orientation_config.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 8.0f;
  orientation_config.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 9.0f;

  const EffectiveBeamwidthDeg effective_beamwidth =
      ResolveEffectiveBeamwidth(antenna_config, orientation_config);

  EXPECT_FLOAT_EQ(effective_beamwidth.az_beamwidth_deg, 3.5f);
  EXPECT_FLOAT_EQ(effective_beamwidth.el_beamwidth_deg, 5.5f);
}

TEST(BeamwidthResolutionTest, UsesCommandedBeamwidthWhenEnabled) {
  AntennaConfig antenna_config;
  antenna_config.nominal_az_beamwidth_deg = 3.5f;
  antenna_config.nominal_el_beamwidth_deg = 5.5f;

  RadarOrientationConfig orientation_config;
  orientation_config.commanded_beamwidth_enabled = true;
  orientation_config.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 8.0f;
  orientation_config.commanded_beamwidth_deg.commanded_el_beamwidth_deg = 9.0f;

  const EffectiveBeamwidthDeg effective_beamwidth =
      ResolveEffectiveBeamwidth(antenna_config, orientation_config);

  EXPECT_FLOAT_EQ(effective_beamwidth.az_beamwidth_deg, 8.0f);
  EXPECT_FLOAT_EQ(effective_beamwidth.el_beamwidth_deg, 9.0f);
}

}  // namespace tests
}  // namespace airborne_radar
