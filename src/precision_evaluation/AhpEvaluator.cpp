#include "1q/precision_evaluation/AhpEvaluator.h"

#include <algorithm>
#include <cmath>

#include "1q/precision_evaluation/PrecisionEvaluationMetrics.h"

namespace precision_evaluation {
namespace {

// Saaty 随机一致性指标 RI（n=1..5 固化；本模块判断矩阵维度恒为 5）。
const double kRandomIndexTable[kPrecisionMetricCount] = {0.0, 0.0, 0.58, 0.90, 1.12};
const double kMatrixTolerance = 1.0e-6;

bool IsValidJudgmentMatrix(const AhpJudgmentMatrix& matrix) {
  for (std::size_t row = 0U; row < kPrecisionMetricCount; ++row) {
    for (std::size_t column = 0U; column < kPrecisionMetricCount; ++column) {
      const double value = matrix.values[row][column];
      if (!std::isfinite(value) || value < 1.0 / 9.0 - kMatrixTolerance ||
          value > 9.0 + kMatrixTolerance) {
        return false;
      }
    }
  }
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    if (std::fabs(matrix.values[index][index] - 1.0) > kMatrixTolerance) {
      return false;  // 对角须为 1
    }
    for (std::size_t other = index + 1U; other < kPrecisionMetricCount; ++other) {
      // 互反对称：a_ij · a_ji = 1。
      const double product = matrix.values[index][other] * matrix.values[other][index];
      if (std::fabs(product - 1.0) > kMatrixTolerance) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

AhpJudgmentMatrix::AhpJudgmentMatrix() {
  for (std::size_t row = 0U; row < kPrecisionMetricCount; ++row) {
    for (std::size_t column = 0U; column < kPrecisionMetricCount; ++column) {
      values[row][column] = 1.0;  // 全 1：五指标等权、完全一致（CR=0）
    }
  }
}

bool TryEvaluateAhp(const AhpJudgmentMatrix& matrix, AhpEvaluation* out) {
  if (out == nullptr || !IsValidJudgmentMatrix(matrix)) {
    return false;
  }
  // 幂迭代求 Perron 主特征向量：正互反矩阵保证主特征值为正、对应特征向量分量非负。
  double weights[kPrecisionMetricCount];
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    weights[index] = 1.0;
  }
  bool converged = false;
  for (int iteration = 0; iteration < 1000 && !converged; ++iteration) {
    double next[kPrecisionMetricCount];
    double next_norm = 0.0;
    for (std::size_t row = 0U; row < kPrecisionMetricCount; ++row) {
      next[row] = 0.0;
      for (std::size_t column = 0U; column < kPrecisionMetricCount; ++column) {
        next[row] += matrix.values[row][column] * weights[column];
      }
      next_norm += next[row];
    }
    if (next_norm <= 0.0 || !std::isfinite(next_norm)) {
      return false;
    }
    double max_delta = 0.0;
    for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
      next[index] /= next_norm;
      max_delta = std::max(max_delta, std::fabs(next[index] - weights[index]));
      weights[index] = next[index];
    }
    converged = max_delta <= 1.0e-12;
  }
  if (!converged) {
    return false;
  }
  // λmax 用 Rayleigh 商：λ = (Aw)·w / (w·w)（w 为 L1 归一化，分母 = Σw² ≠ 1）。
  double aw[kPrecisionMetricCount];
  for (std::size_t row = 0U; row < kPrecisionMetricCount; ++row) {
    aw[row] = 0.0;
    for (std::size_t column = 0U; column < kPrecisionMetricCount; ++column) {
      aw[row] += matrix.values[row][column] * weights[column];
    }
  }
  double rayleigh_numerator = 0.0;
  double rayleigh_denominator = 0.0;
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    rayleigh_numerator += aw[index] * weights[index];
    rayleigh_denominator += weights[index] * weights[index];
  }
  const double lambda_max = rayleigh_denominator > 0.0
                                ? rayleigh_numerator / rayleigh_denominator
                                : 0.0;
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    out->weights[index] = weights[index];
  }
  out->lambda_max = lambda_max;
  out->consistency_index =
      (lambda_max - static_cast<double>(kPrecisionMetricCount)) /
      static_cast<double>(kPrecisionMetricCount - 1U);
  const double random_index = kRandomIndexTable[kPrecisionMetricCount - 1U];
  out->consistency_ratio =
      random_index > 0.0 ? out->consistency_index / random_index : 0.0;
  out->is_consistent = out->consistency_ratio <= 0.1;
  return true;
}

void ComposePrecisionScore(const double (&weights)[kPrecisionMetricCount],
                           const double (&rmse)[kPrecisionMetricCount],
                           const double (&reference_errors)[kPrecisionMetricCount],
                           PrecisionEvaluationReport* report) {
  if (report == nullptr) {
    return;
  }
  double composite = 0.0;
  for (std::size_t index = 0U; index < kPrecisionMetricCount; ++index) {
    const double score = NormalizeErrorScore(rmse[index], reference_errors[index]);
    report->reference_errors[index] = reference_errors[index];
    report->metric_scores[index] = score;
    report->metric_contributions[index] = weights[index] * score;
    composite += weights[index] * score;
  }
  report->composite_score = composite;
}

}  // namespace precision_evaluation
