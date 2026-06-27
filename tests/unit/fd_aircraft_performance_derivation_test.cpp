// Copyright 2026. All Rights Reserved.
//
// @file fd_aircraft_performance_derivation_test.cpp
// @brief 验证飞机性能推导单一源（CLmax planform 分类 + V_stall 公式）。
//
// 直接锁定 SelectClMax / DeriveStallAndWingLoading 的全部分支。这些逻辑原先在
// Autopilot 与 EngineManager（GetRotationSpeedKts / GetDefaultApproachSpeedMps）三处
// 重复，本测试作为单一源的独立护栏：任一处常量或分支被改都会立即报红。

#include <gtest/gtest.h>

#include <cmath>

#include "flight_dynamic/AircraftPerformanceDerivation.h"

namespace oneq {
namespace flight_dynamic {
namespace {

// 构造一个几何有效的常规机翼输入（非涡桨、非 delta），便于在各用例里只改关心字段。
PerformanceDerivationInputs MakeBaseInputs() {
  PerformanceDerivationInputs inputs;
  inputs.weight_lbs = 10000.0;
  inputs.wing_area_ft2 = 200.0;
  // 翼展 40ft → AR = 1600/200 = 8.0（远大于 2.5，非 delta）。
  inputs.wingspan_ft = 40.0;
  inputs.is_turboprop = false;
  inputs.has_cl_max_override = false;
  inputs.cl_max_override = 0.0;
  return inputs;
}

// ---------------------------------------------------------------------------
// SelectClMax
// ---------------------------------------------------------------------------

TEST(AircraftPerformanceDerivationTest, DefaultPlanformSelectsCleanWingClmax) {
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDefault);  // 1.6
}

TEST(AircraftPerformanceDerivationTest, TurbopropSelectsHighLiftClmax) {
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.is_turboprop = true;
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffTurboprop);  // 2.0
}

TEST(AircraftPerformanceDerivationTest, DeltaWingOverridesTurboprop) {
  // 涡桨 + 低展弦比 → delta 优先（与 EngineManager 既有顺序一致）。
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.is_turboprop = true;
  // 翼展 10ft, area 200 → AR = 100/200 = 0.5 < 2.5。
  inputs.wingspan_ft = 10.0;
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDeltaWing);  // 2.5
}

TEST(AircraftPerformanceDerivationTest, DeltaWingDetectionUsesArThreshold) {
  // AR 恰好等于阈值 2.5 时不判为 delta（严格 <）。
  // area=100, span²=250 → AR=2.5：需 span=sqrt(250)≈15.811。
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.wing_area_ft2 = 100.0;
  inputs.wingspan_ft = std::sqrt(250.0);  // AR == 2.5 exactly
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDefault);

  // AR 略低于阈值 → delta。
  inputs.wingspan_ft = std::sqrt(249.0);  // AR == 2.49 < 2.5
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDeltaWing);
}

TEST(AircraftPerformanceDerivationTest, DeltaDetectionRequiresValidGeometry) {
  // 翼展/面积无效（<=1.0）时跳过 delta 检测，回落到 default。
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.wingspan_ft = 0.0;  // invalid
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDefault);

  inputs.wingspan_ft = 40.0;
  inputs.wing_area_ft2 = 0.0;  // invalid
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDefault);
}

TEST(AircraftPerformanceDerivationTest, XmlOverrideTakesPrecedenceWhenPhysicallyReasonable) {
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.is_turboprop = true;  // would normally pick 2.0
  inputs.has_cl_max_override = true;
  inputs.cl_max_override = 1.8;
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), 1.8);
}

TEST(AircraftPerformanceDerivationTest, XmlOverrideIgnoredWhenNotPhysicallyReasonable) {
  // override <= 0.5 被视为不合理，回退到 planform 检测。
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.is_turboprop = true;
  inputs.has_cl_max_override = true;
  inputs.cl_max_override = 0.4;  // <= 0.5
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffTurboprop);  // 2.0
}

TEST(AircraftPerformanceDerivationTest, NoOverrideFlagFallsBackToDetection) {
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  inputs.has_cl_max_override = false;
  inputs.cl_max_override = 1.8;  // present but flag says no override
  EXPECT_DOUBLE_EQ(SelectClMax(inputs), kClMaxTakeoffDefault);
}

// ---------------------------------------------------------------------------
// DeriveStallAndWingLoading
// ---------------------------------------------------------------------------

TEST(AircraftPerformanceDerivationTest, VstallFollowsSqrtFormula) {
  // V_stall = sqrt(2W / (ρ·S·CLmax))。用 default CLmax=1.6 验证数值。
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  const double rho = 0.002377;
  const PerformanceDerivationResult result = DeriveStallAndWingLoading(inputs, rho);
  EXPECT_DOUBLE_EQ(result.cl_max, kClMaxTakeoffDefault);
  const double expected =
      std::sqrt((2.0 * inputs.weight_lbs) / (rho * inputs.wing_area_ft2 * kClMaxTakeoffDefault));
  EXPECT_NEAR(result.v_stall_ftps, expected, 1e-9);
}

TEST(AircraftPerformanceDerivationTest, VstallScalesWithRho) {
  // 同输入、ρ 翻倍 → V_stall 缩小到 1/sqrt(2)。
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  const double rho1 = 0.002377;
  const double rho2 = rho1 * 2.0;
  const PerformanceDerivationResult r1 = DeriveStallAndWingLoading(inputs, rho1);
  const PerformanceDerivationResult r2 = DeriveStallAndWingLoading(inputs, rho2);
  EXPECT_DOUBLE_EQ(r1.cl_max, r2.cl_max);
  EXPECT_NEAR(r2.v_stall_ftps, r1.v_stall_ftps / std::sqrt(2.0), 1e-9);
}

TEST(AircraftPerformanceDerivationTest, VstallZeroForInvalidInputs) {
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  // weight 非正
  inputs.weight_lbs = 0.0;
  PerformanceDerivationResult result = DeriveStallAndWingLoading(inputs, 0.002377);
  EXPECT_DOUBLE_EQ(result.v_stall_ftps, 0.0);
  // CLmax 仍正常推导（v_stall 才是受非法输入影响的量）
  EXPECT_DOUBLE_EQ(result.cl_max, kClMaxTakeoffDefault);
}

TEST(AircraftPerformanceDerivationTest, VstallZeroForNonPositiveRho) {
  PerformanceDerivationInputs inputs = MakeBaseInputs();
  PerformanceDerivationResult result = DeriveStallAndWingLoading(inputs, 0.0);
  EXPECT_DOUBLE_EQ(result.v_stall_ftps, 0.0);
}

// 锁定常量值（任一被改动立即报红，对应三处历史重复中的漂移风险）。
TEST(AircraftPerformanceDerivationTest, PlanformConstantsHoldApprovedValues) {
  EXPECT_DOUBLE_EQ(kClMaxTakeoffDefault, 1.6);
  EXPECT_DOUBLE_EQ(kClMaxTakeoffTurboprop, 2.0);
  EXPECT_DOUBLE_EQ(kClMaxTakeoffDeltaWing, 2.5);
  EXPECT_DOUBLE_EQ(kDeltaWingArThreshold, 2.5);
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
