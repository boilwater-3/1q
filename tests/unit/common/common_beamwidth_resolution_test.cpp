/**
 * @file common_beamwidth_resolution_test.cpp
 * @brief 验证 common 波束宽度解析的 nominal、物理推导与 commanded 覆盖语义。
 */

#include <gtest/gtest.h>

#include <cmath>

#include "common/radar/BeamwidthResolution.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

TEST(CommonBeamwidthResolutionTest, NominalBeamwidthWinsWithoutWavelength) {
  const EffectiveBeamwidthDeg beamwidth = ResolveEffectiveBeamwidth(2.0f, 4.0f, 0.5f, 0.25f, 0.0f);
  EXPECT_FLOAT_EQ(beamwidth.az_beamwidth_deg, 2.0f);
  EXPECT_FLOAT_EQ(beamwidth.el_beamwidth_deg, 4.0f);
}

TEST(CommonBeamwidthResolutionTest, ZeroNominalFallsBackToApertureDerivation) {
  const EffectiveBeamwidthDeg beamwidth = ResolveEffectiveBeamwidth(0.0f, 0.0f, 0.5f, 0.25f, 0.1f);
  const float expected_az_deg =
      DeriveBeamwidthFromApertureRad(0.5f, 0.1f) * (180.0f / 3.14159265358979f);
  const float expected_el_deg =
      DeriveBeamwidthFromApertureRad(0.25f, 0.1f) * (180.0f / 3.14159265358979f);
  EXPECT_FLOAT_EQ(beamwidth.az_beamwidth_deg, expected_az_deg);
  EXPECT_FLOAT_EQ(beamwidth.el_beamwidth_deg, expected_el_deg);
}

TEST(CommonBeamwidthResolutionTest, CommandedOverrideWinsOverNominalAndAperture) {
  CommandedBeamwidthOverride override;
  override.enabled = true;
  override.az_beamwidth_deg = 8.0f;
  override.el_beamwidth_deg = 10.0f;
  const EffectiveBeamwidthDeg beamwidth =
      ResolveEffectiveBeamwidth(2.0f, 3.0f, 0.5f, 0.25f, 0.1f, override);
  EXPECT_FLOAT_EQ(beamwidth.az_beamwidth_deg, 8.0f);
  EXPECT_FLOAT_EQ(beamwidth.el_beamwidth_deg, 10.0f);
}

TEST(CommonBeamwidthResolutionTest, InvalidApertureOrWavelengthDoesNotDerive) {
  const EffectiveBeamwidthDeg beamwidth = ResolveEffectiveBeamwidth(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(beamwidth.az_beamwidth_deg, 0.0f);
  EXPECT_FLOAT_EQ(beamwidth.el_beamwidth_deg, 0.0f);
}

TEST(CommonBeamwidthResolutionTest, DeriveBeamwidthRejectsInvalidInputs) {
  EXPECT_FLOAT_EQ(DeriveBeamwidthFromApertureRad(0.0f, 0.1f), 0.0f);
  EXPECT_FLOAT_EQ(DeriveBeamwidthFromApertureRad(0.5f, 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(DeriveBeamwidthFromApertureRad(-0.5f, 0.1f), 0.0f);
  EXPECT_NEAR(DeriveBeamwidthFromApertureRad(0.5f, 0.1f), 0.2f, 1.0e-6f);
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
