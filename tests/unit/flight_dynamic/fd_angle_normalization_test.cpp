// Copyright 2026. All Rights Reserved.
//
// @file fd_angle_normalization_test.cpp
// @brief 验证角度归一化单一源（NormalizeRad → [-π,π]、RadToDeg360 → [0,360)）。
//
// 这些 helper 原先在 Autopilot 与 Maneuver 逐字重复。本测试锁定其 while-loop 边界
// 语义（逐字复刻既有实现，不改 fmod），任一处被改动立即报红。

#include <gtest/gtest.h>

#include <cmath>

#include "flight_dynamic/AngleNormalization.h"

namespace oneq {
namespace flight_dynamic {
namespace {

constexpr double kPi = M_PI;

// ---------------------------------------------------------------------------
// NormalizeRad → [-π, π]
// ---------------------------------------------------------------------------

TEST(AngleNormalizationTest, ZeroIsUnchanged) {
  EXPECT_DOUBLE_EQ(NormalizeRad(0.0), 0.0);
}

TEST(AngleNormalizationTest, PiAndMinusPiAreFixedPoints) {
  // 既有实现：>π 才减，<-π 才加；故 +π 本身不动，-π 本身也不动。
  EXPECT_DOUBLE_EQ(NormalizeRad(kPi), kPi);
  EXPECT_DOUBLE_EQ(NormalizeRad(-kPi), -kPi);
}

TEST(AngleNormalizationTest, JustOverPiWrapsToJustUnderMinusPi) {
  // π + ε  →  -π + ε
  const double eps = 1.0e-6;
  EXPECT_NEAR(NormalizeRad(kPi + eps), -kPi + eps, 1.0e-9);
}

TEST(AngleNormalizationTest, LargePositiveWrapsByFullTurns) {
  // 3π → 3π - 2π = π（π 本身是 fixed point）
  EXPECT_NEAR(NormalizeRad(3.0 * kPi), kPi, 1.0e-9);
  // 5π → π
  EXPECT_NEAR(NormalizeRad(5.0 * kPi), kPi, 1.0e-9);
}

TEST(AngleNormalizationTest, LargeNegativeWrapsByFullTurns) {
  EXPECT_NEAR(NormalizeRad(-3.0 * kPi), -kPi, 1.0e-9);
  EXPECT_NEAR(NormalizeRad(-5.0 * kPi), -kPi, 1.0e-9);
}

TEST(AngleNormalizationTest, MidRangeValueUnchanged) {
  EXPECT_NEAR(NormalizeRad(1.23), 1.23, 1.0e-9);
  EXPECT_NEAR(NormalizeRad(-2.0), -2.0, 1.0e-9);
}

// ---------------------------------------------------------------------------
// RadToDeg360 → [0, 360)
// ---------------------------------------------------------------------------

TEST(AngleNormalizationTest, RadToDeg360ZeroIsZero) {
  EXPECT_DOUBLE_EQ(RadToDeg360(0.0), 0.0);
}

TEST(AngleNormalizationTest, RadToDeg360PositiveFullTurnIsZero) {
  // 2π rad = 360° → while deg>=360 减 360 → 0
  EXPECT_NEAR(RadToDeg360(2.0 * kPi), 0.0, 1.0e-9);
}

TEST(AngleNormalizationTest, RadToDeg360QuarterTurnIs90) {
  EXPECT_NEAR(RadToDeg360(0.5 * kPi), 90.0, 1.0e-9);
}

TEST(AngleNormalizationTest, RadToDeg360NegativeWrapsPositive) {
  // -π/2 = -90° → +360 → 270°
  EXPECT_NEAR(RadToDeg360(-0.5 * kPi), 270.0, 1.0e-9);
}

TEST(AngleNormalizationTest, RadToDeg360StaysBelow360) {
  // 任何输出必须 < 360（while >=360 才减）。
  // 用一个略小于 2π 的值 → 略小于 360°，不应再被减。
  const double rad = 2.0 * kPi - 1.0e-6;
  const double deg = RadToDeg360(rad);
  EXPECT_GE(deg, 0.0);
  EXPECT_LT(deg, 360.0);
}

TEST(AngleNormalizationTest, RadToDeg360LargeNegativeWraps) {
  // -3π/2 = -270° → +360 → 90°
  EXPECT_NEAR(RadToDeg360(-1.5 * kPi), 90.0, 1.0e-9);
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
