// 规则 13b 门内归因分类器单测：ClassifySnrExclusionCause 五类输出与数值边界。
// 参考状态（全部损失为 0）：1 km 距离、主瓣中心增益、1 m² RCS、热噪声底、零传播损耗。

#include "airborne_radar/signal/pipeline/DetectionExecution.h"

#include <gtest/gtest.h>

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace {

using session::ArIssueCause;

TEST(ArSnrExclusionCauseTest, DistanceDominates) {
  // 50 km 距离：距离损失 4·10·log10(50) ≈ 67.9 dB，其余项为零或负。
  EXPECT_EQ(ClassifySnrExclusionCause(50000.0f, 5.0f, 40.0f, 40.0f, 0.0f, 1.0f, 1.0f),
            ArIssueCause::kDistanceLimited);
}

TEST(ArSnrExclusionCauseTest, BeamOffAxisDominates) {
  // 5 km（距离损失 ≈ 28 dB）+ 单程偏轴损失 20 dB（波束损失 2×20 = 40 dB）。
  EXPECT_EQ(ClassifySnrExclusionCause(5000.0f, 5.0f, 20.0f, 40.0f, 0.0f, 1.0f, 1.0f),
            ArIssueCause::kBeamLimited);
}

TEST(ArSnrExclusionCauseTest, NoiseFloorDominates) {
  // 2 km（距离损失 ≈ 12 dB）+ 噪声底为热噪声 100 倍（噪声损失 20 dB）。
  EXPECT_EQ(ClassifySnrExclusionCause(2000.0f, 5.0f, 40.0f, 40.0f, 0.0f, 100.0f, 1.0f),
            ArIssueCause::kNoiseLimited);
}

TEST(ArSnrExclusionCauseTest, RcsDominates) {
  // 极小 RCS 1e-9 m²：RCS 损失 90 dB，远超 5 km 距离损失 ≈ 28 dB。
  EXPECT_EQ(ClassifySnrExclusionCause(5000.0f, 1.0e-9f, 40.0f, 40.0f, 0.0f, 1.0f, 1.0f),
            ArIssueCause::kRcsLimited);
}

TEST(ArSnrExclusionCauseTest, NoLossYieldsUnknown) {
  // 全部因素处于参考状态（损失 <= 0）→ 无法判定主因。
  EXPECT_EQ(ClassifySnrExclusionCause(1000.0f, 1.0f, 40.0f, 40.0f, 0.0f, 1.0f, 1.0f),
            ArIssueCause::kUnknown);
}

TEST(ArSnrExclusionCauseTest, PropagationLossJoinsDistance) {
  // 传播损耗并入距离项：近距（1 km，距离损失 0）+ 传播损耗 30 dB → 距离主因。
  EXPECT_EQ(ClassifySnrExclusionCause(1000.0f, 5.0f, 40.0f, 40.0f, 30.0f, 1.0f, 1.0f),
            ArIssueCause::kDistanceLimited);
}

TEST(ArSnrExclusionCauseTest, DegenerateInputsAreClamped) {
  // 退化输入（零/负距离、零 RCS、零噪声）不产生 NaN/Inf：距离损失按 1 m 下限、
  // RCS 按下限 1e-12 计算；噪声比值在 total<=0 时仍有限。
  EXPECT_EQ(ClassifySnrExclusionCause(0.0f, 0.0f, 40.0f, 40.0f, -5.0f, 0.0f, 0.0f),
            ArIssueCause::kRcsLimited);
  EXPECT_EQ(ClassifySnrExclusionCause(-100.0f, -1.0f, 40.0f, 40.0f, -3.0f, -1.0f, 0.0f),
            ArIssueCause::kRcsLimited);
}

TEST(ArSnrExclusionCauseTest, TiePrefersEarlierCandidate) {
  // 平局取先：距离损失 == 波束损失 == 20 dB → 按候选序取 kDistanceLimited。
  // 距离损失 20 dB ⟺ range = 1000·10^0.5；波束损失 20 dB ⟺ 单程偏轴损失 10 dB。
  const float range_tie = 1000.0f * std::pow(10.0f, 0.5f);
  EXPECT_EQ(ClassifySnrExclusionCause(range_tie, 5.0f, 30.0f, 40.0f, 0.0f, 1.0f, 1.0f),
            ArIssueCause::kDistanceLimited);
}

}  // namespace
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
