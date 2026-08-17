/**
 * @file common_scan_schedule_test.cpp
 * @brief 验证 common 单源二维扫描调度内核（ScanScheduleRuntime.h）。
 *
 * 覆盖：轴步长解析（hint 派生与回退）、波位序列构建（起点象限控制、
 * 蛇形往返、端点补齐）、非法输入回退空序列。AR/RIR 共用本内核，
 * 模块侧语义测试见各模块扫描/驻留测试。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/foundation/scan_schedule_types.h"
#include "common/radar/ScanScheduleRuntime.h"

namespace oneq {
namespace common {
namespace radar {
namespace {

using oneq::common::radar::AzimuthElevationDeg;

TEST(CommonScanScheduleTest, AxisStepUsesHintWhenValid) {
  // 跨度 20°、5 个步进点 → 步长 5°。
  EXPECT_FLOAT_EQ(ResolveAxisStepDeg(-10.0f, 10.0f, 4.0f, 5U), 5.0f);
  // hint <= 1 → 默认步长。
  EXPECT_FLOAT_EQ(ResolveAxisStepDeg(-10.0f, 10.0f, 4.0f, 0U), 4.0f);
  EXPECT_FLOAT_EQ(ResolveAxisStepDeg(-10.0f, 10.0f, 4.0f, 1U), 4.0f);
  // 非法跨度（min > max）→ hint 派生为 0 → 回退默认步长。
  EXPECT_FLOAT_EQ(ResolveAxisStepDeg(10.0f, -10.0f, 4.0f, 5U), 4.0f);
}

TEST(CommonScanScheduleTest, StartPositionControlsFirstWavePosition) {
  const float kAzMin = -10.0f;
  const float kAzMax = 10.0f;
  const float kElMin = -5.0f;
  const float kElMax = 5.0f;

  const std::vector<AzimuthElevationDeg> left_top = BuildScanPattern(
      kAzMin, kAzMax, kElMin, kElMax, 10.0f, 5.0f,
      oneq::foundation::ScanStartPosition::kLeftTop, oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<AzimuthElevationDeg> right_top = BuildScanPattern(
      kAzMin, kAzMax, kElMin, kElMax, 10.0f, 5.0f,
      oneq::foundation::ScanStartPosition::kRightTop, oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<AzimuthElevationDeg> right_bottom = BuildScanPattern(
      kAzMin, kAzMax, kElMin, kElMax, 10.0f, 5.0f,
      oneq::foundation::ScanStartPosition::kRightBottom,
      oneq::foundation::ScanSequence::kAzimuthFirst);
  const std::vector<AzimuthElevationDeg> left_bottom = BuildScanPattern(
      kAzMin, kAzMax, kElMin, kElMax, 10.0f, 5.0f,
      oneq::foundation::ScanStartPosition::kLeftBottom,
      oneq::foundation::ScanSequence::kAzimuthFirst);

  ASSERT_FALSE(left_top.empty());
  ASSERT_FALSE(right_top.empty());
  ASSERT_FALSE(right_bottom.empty());
  ASSERT_FALSE(left_bottom.empty());

  EXPECT_FLOAT_EQ(left_top.front().az_deg, -10.0f);
  EXPECT_FLOAT_EQ(left_top.front().el_deg, 5.0f);
  EXPECT_FLOAT_EQ(right_top.front().az_deg, 10.0f);
  EXPECT_FLOAT_EQ(right_top.front().el_deg, 5.0f);
  EXPECT_FLOAT_EQ(right_bottom.front().az_deg, 10.0f);
  EXPECT_FLOAT_EQ(right_bottom.front().el_deg, -5.0f);
  EXPECT_FLOAT_EQ(left_bottom.front().az_deg, -10.0f);
  EXPECT_FLOAT_EQ(left_bottom.front().el_deg, -5.0f);
}

TEST(CommonScanScheduleTest, AzimuthFirstSnakeAlternatesRows) {
  const std::vector<AzimuthElevationDeg> pattern =
      BuildScanPattern(-10.0f, 10.0f, -5.0f, 5.0f, 10.0f, 5.0f,
                       oneq::foundation::ScanStartPosition::kLeftTop,
                       oneq::foundation::ScanSequence::kAzimuthFirst);
  ASSERT_EQ(pattern.size(), 3U * 3U);
  // 第一行：az 从左到右（-10 → 10），el 固定 +5。
  EXPECT_FLOAT_EQ(pattern[0].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(pattern[0].el_deg, 5.0f);
  EXPECT_FLOAT_EQ(pattern[1].az_deg, 0.0f);
  EXPECT_FLOAT_EQ(pattern[1].el_deg, 5.0f);
  EXPECT_FLOAT_EQ(pattern[2].az_deg, 10.0f);
  EXPECT_FLOAT_EQ(pattern[2].el_deg, 5.0f);
  // 第二行：蛇形往返，az 从右到左，el 固定 0。
  EXPECT_FLOAT_EQ(pattern[3].az_deg, 10.0f);
  EXPECT_FLOAT_EQ(pattern[3].el_deg, 0.0f);
  EXPECT_FLOAT_EQ(pattern[4].az_deg, 0.0f);
  EXPECT_FLOAT_EQ(pattern[4].el_deg, 0.0f);
  EXPECT_FLOAT_EQ(pattern[5].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(pattern[5].el_deg, 0.0f);
}

TEST(CommonScanScheduleTest, ElevationFirstBuildsColumns) {
  const std::vector<AzimuthElevationDeg> pattern =
      BuildScanPattern(-10.0f, 10.0f, -5.0f, 5.0f, 10.0f, 5.0f,
                       oneq::foundation::ScanStartPosition::kLeftTop,
                       oneq::foundation::ScanSequence::kElevationFirst);
  ASSERT_EQ(pattern.size(), 3U * 3U);
  // 第一列：el 从上到下（+5 → -5），az 固定 -10。
  EXPECT_FLOAT_EQ(pattern[0].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(pattern[0].el_deg, 5.0f);
  EXPECT_FLOAT_EQ(pattern[1].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(pattern[1].el_deg, 0.0f);
  EXPECT_FLOAT_EQ(pattern[2].az_deg, -10.0f);
  EXPECT_FLOAT_EQ(pattern[2].el_deg, -5.0f);
}

TEST(CommonScanScheduleTest, InvalidInputsReturnEmptyPattern) {
  // 非法范围（min > max）。
  EXPECT_TRUE(BuildScanPattern(10.0f, -10.0f, -5.0f, 5.0f, 10.0f, 5.0f,
                               oneq::foundation::ScanStartPosition::kLeftTop,
                               oneq::foundation::ScanSequence::kAzimuthFirst)
                  .empty());
  // 非有限限位。
  EXPECT_TRUE(BuildScanPattern(-10.0f, 10.0f, -5.0f, std::numeric_limits<float>::quiet_NaN(),
                               10.0f, 5.0f, oneq::foundation::ScanStartPosition::kLeftTop,
                               oneq::foundation::ScanSequence::kAzimuthFirst)
                  .empty());
  // 非正步长。
  EXPECT_TRUE(BuildScanPattern(-10.0f, 10.0f, -5.0f, 5.0f, 0.0f, 5.0f,
                               oneq::foundation::ScanStartPosition::kLeftTop,
                               oneq::foundation::ScanSequence::kAzimuthFirst)
                  .empty());
}

}  // namespace
}  // namespace radar
}  // namespace common
}  // namespace oneq
