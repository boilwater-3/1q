// Copyright 2026. All Rights Reserved.
//
// @file rir_effective_rcs_test.cpp
// @brief 验证 RIR 有效 RCS 混合逻辑。

#include <gtest/gtest.h>

#include "remote_identification_radar/dwell/RirEffectiveRcs.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::hardware::RirRcsPhysicsConfig;
using dwell::ComputeEffectiveTargetRcsM2;
using dwell::RirTargetLookAngles;
using session::RirSceneTarget;

TEST(RirEffectiveRcsTest, ReturnsInputRcsWhenPhysicalModelDisabled) {
  RirSceneTarget target;
  target.rcs = 2.5f;
  RirRcsPhysicsConfig rcs_config;
  rcs_config.enable_physical_rcs = false;
  rcs_config.physics_mix_ratio = 1.0f;
  EXPECT_FLOAT_EQ(ComputeEffectiveTargetRcsM2(target, RirTargetLookAngles{}, rcs_config, 3.0e9f),
                  2.5f);
}

TEST(RirEffectiveRcsTest, MixRatioZeroPreservesInputRcs) {
  RirSceneTarget target;
  target.rcs = 4.0f;
  RirRcsPhysicsConfig rcs_config;
  rcs_config.enable_physical_rcs = true;
  rcs_config.physics_mix_ratio = 0.0f;
  EXPECT_FLOAT_EQ(ComputeEffectiveTargetRcsM2(target, RirTargetLookAngles{}, rcs_config, 3.0e9f),
                  4.0f);
}

TEST(RirEffectiveRcsTest, FullPhysicalMixChangesRcsWithLookAngles) {
  RirSceneTarget target;
  target.rcs = 1.0f;
  RirRcsPhysicsConfig rcs_config;
  rcs_config.enable_physical_rcs = true;
  rcs_config.physics_mix_ratio = 1.0f;
  rcs_config.cylinder_weight = 0.5f;

  RirTargetLookAngles look{};
  look.has_look_angles = true;
  look.look_az_deg = 10.0f;
  look.look_el_deg = 5.0f;

  const float effective =
      ComputeEffectiveTargetRcsM2(target, look, rcs_config, 3.0e9f);
  EXPECT_GT(effective, 0.0f);
  EXPECT_NE(effective, target.rcs);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
