/**
 * @file esr_resolution_rules_test.cpp
 * @brief ESR 配置解析规则单元测试。
 *
 * 直接覆盖 EsrResolutionRules 的三个 helper 的所有分支，作为该单一规则源的独立
 * 护栏：未来若常量阈值或分支被改（哪怕只改一处），此处会直接报红，而无需依赖两个
 * resolver 测试间接捕获。数值断言对应 EsrResolutionRules.cpp 的当前实现：
 *   - kActiveScanPulseMultiplier = 4, kMaxPulseCount = 4096
 *   - kMinimumThresholdScale = 0.1, kHgesmThresholdScale = 0.85, kRwrThresholdScale = 1.25
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/session/EsrResolutionRules.h"

namespace electronic_surveillance_radar {
namespace session {
namespace resolution_rules {
namespace {
namespace test_config = ::electronic_surveillance_radar::config;

constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// ---------------------------------------------------------------------------
// NormalizeScanBounds
// ---------------------------------------------------------------------------

TEST(EsrResolutionRulesTest, NormalizeScanBoundsSwapsWhenStartGreaterThanEnd) {
  float start = 30.0f;
  float end = -30.0f;
  NormalizeScanBounds(&start, &end);
  EXPECT_FLOAT_EQ(start, -30.0f);
  EXPECT_FLOAT_EQ(end, 30.0f);
}

TEST(EsrResolutionRulesTest, NormalizeScanBoundsKeepsOrderWhenAlreadySorted) {
  float start = -20.0f;
  float end = 20.0f;
  NormalizeScanBounds(&start, &end);
  EXPECT_FLOAT_EQ(start, -20.0f);
  EXPECT_FLOAT_EQ(end, 20.0f);
}

TEST(EsrResolutionRulesTest, NormalizeScanBoundsIsSafeOnNullptr) {
  NormalizeScanBounds(nullptr, nullptr);  // must not crash
  SUCCEED();
}

// ---------------------------------------------------------------------------
// ApplyWorkModeAdjustment
// ---------------------------------------------------------------------------

TEST(EsrResolutionRulesTest, WorkModeHgesmMultipliesPulsesAndScalesThreshold) {
  DetectionConfig detection;
  detection.pulse_count = 100U;
  detection.threshold_scale = 1.0f;

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, &detection);

  EXPECT_EQ(detection.pulse_count, 400U);              // 100 * 4
  EXPECT_FLOAT_EQ(detection.threshold_scale, 0.85f);   // 1.0 * 0.85
}

TEST(EsrResolutionRulesTest, WorkModeHgesmClampsPulsesToMax) {
  DetectionConfig detection;
  detection.pulse_count = 2000U;  // 2000 * 4 = 8000 > 4096
  detection.threshold_scale = 1.0f;

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, &detection);

  EXPECT_EQ(detection.pulse_count, 4096U);
}

TEST(EsrResolutionRulesTest, WorkModeHgesmFloorsThresholdAtMinimum) {
  DetectionConfig detection;
  detection.pulse_count = 1U;
  detection.threshold_scale = 0.05f;  // 0.05 * 0.85 = 0.0425 < 0.1

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, &detection);

  EXPECT_FLOAT_EQ(detection.threshold_scale, 0.1f);
}

TEST(EsrResolutionRulesTest, WorkModeRwrHalvesPulsesAndScalesThreshold) {
  DetectionConfig detection;
  detection.pulse_count = 100U;
  detection.threshold_scale = 1.0f;

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kRwr, &detection);

  EXPECT_EQ(detection.pulse_count, 50U);               // 100 / 2
  EXPECT_FLOAT_EQ(detection.threshold_scale, 1.25f);   // 1.0 * 1.25
}

TEST(EsrResolutionRulesTest, WorkModeRwrFloorsPulsesAtOne) {
  DetectionConfig detection;
  detection.pulse_count = 1U;  // 1 / 2 = 0 -> floored to 1
  detection.threshold_scale = 0.05f;  // 0.05 * 1.25 = 0.0625 < 0.1

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kRwr, &detection);

  EXPECT_EQ(detection.pulse_count, 1U);
  EXPECT_FLOAT_EQ(detection.threshold_scale, 0.1f);
}

TEST(EsrResolutionRulesTest, WorkModeEsmLeavesDetectionUntouched) {
  DetectionConfig detection;
  detection.pulse_count = 64U;
  detection.threshold_scale = 2.0f;

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kEsm, &detection);

  EXPECT_EQ(detection.pulse_count, 64U);
  EXPECT_FLOAT_EQ(detection.threshold_scale, 2.0f);
}

TEST(EsrResolutionRulesTest, WorkModePreClampsInvalidPulseCountToOne) {
  DetectionConfig detection;
  detection.pulse_count = 0U;  // clamped to 1 before mode adjustment
  detection.threshold_scale = 1.0f;

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, &detection);

  EXPECT_EQ(detection.pulse_count, 4U);  // (max(1, 0)) * 4
}

TEST(EsrResolutionRulesTest, WorkModeFallsBackThresholdToOneForNonPositive) {
  DetectionConfig detection;
  detection.pulse_count = 10U;
  detection.threshold_scale = -3.0f;  // non-positive -> fallback to 1.0

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, &detection);

  EXPECT_FLOAT_EQ(detection.threshold_scale, 0.85f);  // 1.0 * 0.85
}

TEST(EsrResolutionRulesTest, WorkModeFallsBackThresholdToOneForNonFinite) {
  DetectionConfig detection;
  detection.pulse_count = 10U;
  detection.threshold_scale = kNaN;  // non-finite -> fallback to 1.0

  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, &detection);

  EXPECT_FALSE(std::isnan(detection.threshold_scale));
  EXPECT_FLOAT_EQ(detection.threshold_scale, 0.85f);
}

TEST(EsrResolutionRulesTest, WorkModeIsSafeOnNullptr) {
  ApplyWorkModeAdjustment(test_config::EsrWorkMode::kHgesm, nullptr);
  SUCCEED();
}

// ---------------------------------------------------------------------------
// ApplyScanPolicy
// ---------------------------------------------------------------------------

TEST(EsrResolutionRulesTest, ScanPolicyExplicitBoundsSubtractMountAndNormalize) {
  test_config::EsrHardwareConfig hardware;
  hardware.antenna_mount_az_deg = 5.0f;
  hardware.antenna_mount_el_deg = 2.0f;
  test_config::EsrScanPolicyConfig scan_policy;
  scan_policy.use_explicit_scan_bounds = true;
  // Reversed order to exercise NormalizeScanBounds swap.
  scan_policy.scan_start_az_deg = 60.0f;
  scan_policy.scan_end_az_deg = -60.0f;
  scan_policy.scan_start_el_deg = 10.0f;
  scan_policy.scan_end_el_deg = -10.0f;

  extension::InterceptScanConfig scan_config;

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  // Bounds are mount-relative and normalized (start <= end).
  EXPECT_FLOAT_EQ(scan_config.scan_start_az_deg, -65.0f);  // -60 - 5
  EXPECT_FLOAT_EQ(scan_config.scan_end_az_deg, 55.0f);     // 60 - 5
  EXPECT_FLOAT_EQ(scan_config.scan_start_el_deg, -12.0f);  // -10 - 2
  EXPECT_FLOAT_EQ(scan_config.scan_end_el_deg, 8.0f);      // 10 - 2
}

TEST(EsrResolutionRulesTest, ScanPolicyCenterAzUsesHardwareScanRangeWhenPresent) {
  test_config::EsrHardwareConfig hardware;
  hardware.az_scan_range_deg = 120.0f;  // half-span = 60
  test_config::EsrScanPolicyConfig scan_policy;
  scan_policy.use_explicit_scan_bounds = false;
  scan_policy.scan_center_az_deg = 10.0f;

  extension::InterceptScanConfig scan_config;

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  EXPECT_FLOAT_EQ(scan_config.scan_start_az_deg, 10.0f - 60.0f);  // -50
  EXPECT_FLOAT_EQ(scan_config.scan_end_az_deg, 10.0f + 60.0f);    // 70
}

TEST(EsrResolutionRulesTest, ScanPolicyCenterAzFallsBackToCurrentSpanWithoutRange) {
  test_config::EsrHardwareConfig hardware;  // no az_scan_range_deg
  test_config::EsrScanPolicyConfig scan_policy;
  scan_policy.use_explicit_scan_bounds = false;
  scan_policy.scan_center_az_deg = 0.0f;

  extension::InterceptScanConfig scan_config;  // default az span [-60, 60] => half=60

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  EXPECT_FLOAT_EQ(scan_config.scan_start_az_deg, -60.0f);
  EXPECT_FLOAT_EQ(scan_config.scan_end_az_deg, 60.0f);
}

TEST(EsrResolutionRulesTest, ScanPolicyBeamWidthsOverrideStepsWhenFinite) {
  test_config::EsrHardwareConfig hardware;
  hardware.beam_az_width_deg = 8.0f;
  hardware.beam_el_width_deg = 6.0f;
  test_config::EsrScanPolicyConfig scan_policy;

  extension::InterceptScanConfig scan_config;

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  EXPECT_FLOAT_EQ(scan_config.az_step_deg, 8.0f);
  EXPECT_FLOAT_EQ(scan_config.el_step_deg, 6.0f);
}

TEST(EsrResolutionRulesTest, ScanPolicyStartPosAndSequenceAreMappedToInt) {
  test_config::EsrHardwareConfig hardware;
  test_config::EsrScanPolicyConfig scan_policy;
  scan_policy.scan_start_position = test_config::EsrScanStartPosition::kRightBottom;
  scan_policy.scan_sequence = test_config::EsrScanSequence::kElevationFirst;

  extension::InterceptScanConfig scan_config;

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  EXPECT_EQ(scan_config.scan_start_pos, static_cast<int>(test_config::EsrScanStartPosition::kRightBottom));
  EXPECT_EQ(scan_config.scan_sequence, static_cast<int>(test_config::EsrScanSequence::kElevationFirst));
}

TEST(EsrResolutionRulesTest, ScanPolicyWithoutExplicitOrCenterPassesThroughDefaults) {
  test_config::EsrHardwareConfig hardware;
  test_config::EsrScanPolicyConfig scan_policy;  // use_explicit=false, no centers

  extension::InterceptScanConfig scan_config;  // defaults: az [-60,60], el [-10,10]

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  EXPECT_FLOAT_EQ(scan_config.scan_start_az_deg, -60.0f);
  EXPECT_FLOAT_EQ(scan_config.scan_end_az_deg, 60.0f);
  EXPECT_FLOAT_EQ(scan_config.scan_start_el_deg, -10.0f);
  EXPECT_FLOAT_EQ(scan_config.scan_end_el_deg, 10.0f);
}

TEST(EsrResolutionRulesTest, ScanPolicyIsSafeOnNullptr) {
  test_config::EsrHardwareConfig hardware;
  test_config::EsrScanPolicyConfig scan_policy;
  ApplyScanPolicy(hardware, scan_policy, nullptr);  // must not crash
  SUCCEED();
}

TEST(EsrResolutionRulesTest, ScanPolicyExplicitBoundsDominatesCenterWhenBothSet) {
  test_config::EsrHardwareConfig hardware;
  test_config::EsrScanPolicyConfig scan_policy;
  // Both explicit and center provided: explicit must win.
  scan_policy.use_explicit_scan_bounds = true;
  scan_policy.scan_start_az_deg = -40.0f;
  scan_policy.scan_end_az_deg = 40.0f;
  scan_policy.scan_start_el_deg = -5.0f;
  scan_policy.scan_end_el_deg = 5.0f;
  scan_policy.scan_center_az_deg = 999.0f;  // should be ignored

  extension::InterceptScanConfig scan_config;

  ApplyScanPolicy(hardware, scan_policy, &scan_config);

  EXPECT_FLOAT_EQ(scan_config.scan_start_az_deg, -40.0f);
  EXPECT_FLOAT_EQ(scan_config.scan_end_az_deg, 40.0f);
  EXPECT_NE(scan_config.scan_start_az_deg, 999.0f);
}

}  // namespace
}  // namespace resolution_rules
}  // namespace session
}  // namespace electronic_surveillance_radar
