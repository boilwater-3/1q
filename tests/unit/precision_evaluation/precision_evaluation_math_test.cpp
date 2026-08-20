#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "1q/precision_evaluation/AhpEvaluator.h"
#include "1q/precision_evaluation/DualLosFix.h"
#include "1q/precision_evaluation/PrecisionEvaluationMetrics.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"

namespace {

namespace pe = precision_evaluation;
using pe::kPrecisionMetricCount;

oneq::coordinate::Vector3d Vec(double x, double y, double z) {
  return oneq::coordinate::Vector3d(x, y, z);
}

oneq::coordinate::EcefPositionM Pos(double x, double y, double z) {
  return oneq::coordinate::EcefPositionM(x, y, z);
}

TEST(PrecisionDualLosFixTest, IntersectingLinesResolveAtCrossingPoint) {
  // A 沿 x 轴，B 过 (5,0,-5) 沿 z 轴：交点 (5,0,0)，残差 0。
  oneq::coordinate::EcefPositionM fix;
  double residual_m = -1.0;
  ASSERT_TRUE(pe::TryComputeDualLosFixM(Pos(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0),
                                       Pos(5.0, 0.0, -5.0), Vec(0.0, 0.0, 1.0), &fix,
                                       &residual_m));
  EXPECT_NEAR(fix.x_m, 5.0, 1.0e-9);
  EXPECT_NEAR(fix.y_m, 0.0, 1.0e-9);
  EXPECT_NEAR(fix.z_m, 0.0, 1.0e-9);
  EXPECT_NEAR(residual_m, 0.0, 1.0e-9);
}

TEST(PrecisionDualLosFixTest, SkewLinesTakeMidpointWithResidual) {
  // B 平移 1 m：最近点 A(5,0,0)、B(5,1,0) → 中点 (5,0.5,0)，残差 1 m。
  oneq::coordinate::EcefPositionM fix;
  double residual_m = -1.0;
  ASSERT_TRUE(pe::TryComputeDualLosFixM(Pos(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0),
                                       Pos(5.0, 1.0, -5.0), Vec(0.0, 0.0, 1.0), &fix,
                                       &residual_m));
  EXPECT_NEAR(fix.x_m, 5.0, 1.0e-9);
  EXPECT_NEAR(fix.y_m, 0.5, 1.0e-9);
  EXPECT_NEAR(fix.z_m, 0.0, 1.0e-9);
  EXPECT_NEAR(residual_m, 1.0, 1.0e-9);
}

TEST(PrecisionDualLosFixTest, ParallelLinesAreUnresolvable) {
  oneq::coordinate::EcefPositionM fix;
  double residual_m = -1.0;
  EXPECT_FALSE(pe::TryComputeDualLosFixM(Pos(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0),
                                         Pos(0.0, 1.0, 0.0), Vec(2.0, 0.0, 0.0), &fix,
                                         &residual_m));
}

TEST(PrecisionDualLosFixTest, NonUnitDirectionsGiveSameFix) {
  oneq::coordinate::EcefPositionM fix_unit;
  oneq::coordinate::EcefPositionM fix_scaled;
  double residual_unit = -1.0;
  double residual_scaled = -1.0;
  ASSERT_TRUE(pe::TryComputeDualLosFixM(Pos(0.0, 0.0, 0.0), Vec(0.37, 0.0, 0.0),
                                       Pos(5.0, 1.0, -5.0), Vec(0.0, 0.0, 2.5), &fix_scaled,
                                       &residual_scaled));
  ASSERT_TRUE(pe::TryComputeDualLosFixM(Pos(0.0, 0.0, 0.0), Vec(1.0, 0.0, 0.0),
                                       Pos(5.0, 1.0, -5.0), Vec(0.0, 0.0, 1.0), &fix_unit,
                                       &residual_unit));
  EXPECT_NEAR(fix_scaled.x_m, fix_unit.x_m, 1.0e-6);
  EXPECT_NEAR(fix_scaled.y_m, fix_unit.y_m, 1.0e-6);
  EXPECT_NEAR(fix_scaled.z_m, fix_unit.z_m, 1.0e-6);
  EXPECT_NEAR(residual_scaled, residual_unit, 1.0e-6);
}

TEST(PrecisionMetricsTest, SummarizesKnownSeries) {
  const std::vector<double> errors = {3.0, 4.0};
  const pe::ErrorMetricSummary summary = pe::SummarizeErrorSeries(errors);
  EXPECT_EQ(summary.count, 2U);
  EXPECT_NEAR(summary.mean, 3.5, 1.0e-12);
  EXPECT_NEAR(summary.rmse, std::sqrt(12.5), 1.0e-12);
  EXPECT_NEAR(summary.p95, 4.0, 1.0e-12);
  EXPECT_NEAR(summary.max, 4.0, 1.0e-12);
}

TEST(PrecisionMetricsTest, P95UsesNearestRankOnTwentySamples) {
  // 1..20：ceil(0.95·20)=19 → 第 19 个样本 = 19。
  std::vector<double> errors;
  for (int value = 1; value <= 20; ++value) {
    errors.push_back(static_cast<double>(value));
  }
  const pe::ErrorMetricSummary summary = pe::SummarizeErrorSeries(errors);
  EXPECT_EQ(summary.count, 20U);
  EXPECT_NEAR(summary.mean, 10.5, 1.0e-12);
  // Σk²(k=1..20) = 20·21·41/6 = 2870。
  EXPECT_NEAR(summary.rmse, std::sqrt(2870.0 / 20.0), 1.0e-12);
  EXPECT_NEAR(summary.p95, 19.0, 1.0e-12);
  EXPECT_NEAR(summary.max, 20.0, 1.0e-12);
}

TEST(PrecisionMetricsTest, EmptySeriesReturnsZeroCount) {
  const pe::ErrorMetricSummary summary = pe::SummarizeErrorSeries({});
  EXPECT_EQ(summary.count, 0U);
  EXPECT_DOUBLE_EQ(summary.mean, 0.0);
  EXPECT_DOUBLE_EQ(summary.rmse, 0.0);
}

TEST(PrecisionMetricsTest, NormalizeScoreIsHalfAtReference) {
  EXPECT_NEAR(pe::NormalizeErrorScore(100.0, 100.0), 0.5, 1.0e-12);
  EXPECT_NEAR(pe::NormalizeErrorScore(0.0, 100.0), 1.0, 1.0e-12);
  EXPECT_NEAR(pe::NormalizeErrorScore(300.0, 100.0), 0.25, 1.0e-12);
  EXPECT_DOUBLE_EQ(pe::NormalizeErrorScore(100.0, 0.0), 0.0);  // 非法参考
}

TEST(PrecisionAhpTest, UniformMatrixYieldsEqualWeightsAndZeroCr) {
  pe::AhpJudgmentMatrix matrix;  // 默认全 1
  pe::AhpEvaluation evaluation;
  ASSERT_TRUE(pe::TryEvaluateAhp(matrix, &evaluation));
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    EXPECT_NEAR(evaluation.weights[index], 0.2, 1.0e-9);
  }
  EXPECT_NEAR(evaluation.lambda_max, 5.0, 1.0e-9);
  EXPECT_NEAR(evaluation.consistency_index, 0.0, 1.0e-9);
  EXPECT_NEAR(evaluation.consistency_ratio, 0.0, 1.0e-9);
  EXPECT_TRUE(evaluation.is_consistent);
}

TEST(PrecisionAhpTest, PowerIterationRecoversSynthesizedWeights) {
  // 完全一致矩阵 a_ij = w_i/w_j（w=(0.5,0.25,0.125,0.0625,0.0625)，比值 ≤9 合法）：
  // 幂迭代应恢复权重且 CR≈0。
  const double w[kPrecisionMetricCount] = {0.5, 0.25, 0.125, 0.0625, 0.0625};
  pe::AhpJudgmentMatrix matrix;
  for (std::size_t row = 0U; row < kPrecisionMetricCount; ++row) {
    for (std::size_t column = 0U; column < kPrecisionMetricCount; ++column) {
      matrix.values[row][column] = w[row] / w[column];
    }
  }
  pe::AhpEvaluation evaluation;
  ASSERT_TRUE(pe::TryEvaluateAhp(matrix, &evaluation));
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    EXPECT_NEAR(evaluation.weights[index], w[index], 1.0e-9);
  }
  EXPECT_TRUE(evaluation.is_consistent);
}

TEST(PrecisionAhpTest, InconsistentMatrixIsFlaggedBeyondCrThreshold) {
  // 极端环序矛盾（A≫B、B≫C、C≫A，各取 9 倍标度）嵌入 5×5：单 3×3 的 CR≈0.32，
  // 稀释到 5 维后仍显著超过 0.1（Collatz-Wielandt 下界 λmax≥6.4）。
  pe::AhpJudgmentMatrix matrix;  // 默认全 1 后覆写
  matrix.values[0][1] = 9.0;
  matrix.values[1][0] = 1.0 / 9.0;
  matrix.values[1][2] = 9.0;
  matrix.values[2][1] = 1.0 / 9.0;
  matrix.values[0][2] = 1.0 / 9.0;
  matrix.values[2][0] = 9.0;
  pe::AhpEvaluation evaluation;
  ASSERT_TRUE(pe::TryEvaluateAhp(matrix, &evaluation));
  EXPECT_GT(evaluation.consistency_ratio, 0.1);
  EXPECT_FALSE(evaluation.is_consistent);
}

TEST(PrecisionAhpTest, InvalidMatricesAreRejected) {
  pe::AhpEvaluation evaluation;
  pe::AhpJudgmentMatrix broken_diagonal;  // 对角非 1
  broken_diagonal.values[2][2] = 2.0;
  EXPECT_FALSE(pe::TryEvaluateAhp(broken_diagonal, &evaluation));

  pe::AhpJudgmentMatrix broken_reciprocal;  // 互反对称破坏
  broken_reciprocal.values[0][1] = 3.0;
  broken_reciprocal.values[1][0] = 2.0 / 3.0;
  EXPECT_FALSE(pe::TryEvaluateAhp(broken_reciprocal, &evaluation));

  pe::AhpJudgmentMatrix out_of_scale;  // 超出 Saaty 1-9 标度
  out_of_scale.values[0][3] = 10.0;
  out_of_scale.values[3][0] = 0.1;
  EXPECT_FALSE(pe::TryEvaluateAhp(out_of_scale, &evaluation));

  pe::AhpJudgmentMatrix valid;
  EXPECT_FALSE(pe::TryEvaluateAhp(valid, nullptr));
}

TEST(PrecisionScoreTest, ComposeYieldsHalfAtReferenceRmse) {
  const double weights[kPrecisionMetricCount] = {0.2, 0.2, 0.2, 0.2, 0.2};
  const double rmse[kPrecisionMetricCount] = {1.0, 1.0, 1.0, 1.0, 1.0};
  const double references[kPrecisionMetricCount] = {1.0, 1.0, 1.0, 1.0, 1.0};
  pe::PrecisionEvaluationReport report;
  pe::ComposePrecisionScore(weights, rmse, references, &report);
  EXPECT_NEAR(report.composite_score, 0.5, 1.0e-12);
  double contribution_sum = 0.0;
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    EXPECT_NEAR(report.metric_scores[index], 0.5, 1.0e-12);
    contribution_sum += report.metric_contributions[index];
  }
  EXPECT_NEAR(contribution_sum, report.composite_score, 1.0e-12);
}

TEST(PrecisionScoreTest, ZeroRmseScoresOne) {
  const double weights[kPrecisionMetricCount] = {0.1, 0.2, 0.3, 0.2, 0.2};
  const double rmse[kPrecisionMetricCount] = {0.0, 0.0, 0.0, 0.0, 0.0};
  const double references[kPrecisionMetricCount] = {1.0, 2.0, 3.0, 4.0, 5.0};
  pe::PrecisionEvaluationReport report;
  pe::ComposePrecisionScore(weights, rmse, references, &report);
  EXPECT_NEAR(report.composite_score, 1.0, 1.0e-12);
}

}  // namespace
