/**
 * @file UnscentedTransform.h
 * @brief 定义无迹变换（Unscented Transform）的 sigma 点生成与加权重建辅助（维度模板化）。
 *
 * 参考 Stone Soup UnscentedKalmanPredictor/Updater 的 sigma point 框架与
 * Julier & Uhlmann 无迹变换原始公式。本文件只提供数值辅助，不承载滤波器接口
 * （滤波器见 UnscentedPredictor.h / UnscentedUpdater.h）。
 *
 * 全部以结构体模板静态方法承载（不做函数模板推导）：MSVC v141 无法从 Eigen 的
 * 依赖默认模板参数推导维度，显式模板实参调用是跨编译器安全的形态。
 *
 * 权重公式（n = kDim，λ = α²(n+κ) − n）：
 * - W0^m = λ / (n+λ)，W0^c = W0^m + (1 − α² + β)
 * - Wi^m = Wi^c = 1 / (2(n+λ))，i = 1..2n
 * - sigma 点：X0 = x̄；Xi = x̄ + L_i，Xn+i = x̄ − L_i（L 为 (n+λ)·P 的 Cholesky 下三角列）
 */

#ifndef COMMON_ESTIMATION_UNSCENTED_TRANSFORM_H_
#define COMMON_ESTIMATION_UNSCENTED_TRANSFORM_H_

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <algorithm>
#include <array>

#include "common/numerics/NumericGuard.h"

namespace oneq {
namespace common {
namespace estimation {

using oneq::common::numerics::kCovarianceFloor;
using oneq::common::numerics::kNumericFloor;

/**
 * @brief 无迹变换参数。
 */
struct UnscentedTransformConfig {
  float alpha{1.0f};  /**< sigma 点散布因子，(0,1]，典型取 1e-3~1；必须为正。 */
  float beta{2.0f};   /**< 分布先验参数，高斯分布取 2，仅进入中心点协方差权重。 */
  float kappa{0.0f};  /**< 二次缩放参数，常取 0 或 3−n。 */
};

/**
 * @brief sigma 点集合（2n+1 个点及其均值/协方差权重）。
 * @tparam kDim 维度。
 */
template <int kDim>
struct UnscentedPointSet {
  static constexpr int kPointCount = 2 * kDim + 1; /**< sigma 点总数（2n+1）。 */
  using PointVector = Eigen::Matrix<float, kDim, 1>;

  std::array<PointVector, static_cast<std::size_t>(kPointCount)> points{}; /**< sigma 点。 */
  std::array<float, static_cast<std::size_t>(kPointCount)> mean_weights{}; /**< 均值权重 W^m。 */
  std::array<float, static_cast<std::size_t>(kPointCount)> covariance_weights{}; /**< 协方差权重 W^c。 */
};

/**
 * @brief 无迹变换数值辅助（同维点集的生成、加权均值与协方差稳定化）。
 * @tparam kDim 向量维度。
 */
template <int kDim>
struct UnscentedTransform {
  using PointVector = Eigen::Matrix<float, kDim, 1>;
  using CovarianceMatrix = Eigen::Matrix<float, kDim, kDim>;
  using PointSet = UnscentedPointSet<kDim>;
  static constexpr std::size_t kCount = static_cast<std::size_t>(PointSet::kPointCount);
  using PointArray = std::array<PointVector, kCount>;

  /**
   * @brief 由均值与协方差生成 sigma 点集合并计算权重。
   * @param[in] mean 均值向量。
   * @param[in] covariance 协方差矩阵。
   * @param[in] config 无迹变换参数（alpha 必须为正，退化配置由调用方负责）。
   * @param[out] out 输出 sigma 点集合。
   * @return 成功返回 true；协方差 LLT 分解失败返回 false（调用方按 fail-safe 处理）。
   */
  static bool GenerateSigmaPoints(const PointVector& mean, const CovarianceMatrix& covariance,
                                  const UnscentedTransformConfig& config, PointSet* out) {
    if (out == nullptr) {
      return false;
    }

    const float n = static_cast<float>(kDim);
    /* n + λ = α²(n+κ)；分母下限按数值下限语义桶取通用防护下限。 */
    const float n_plus_lambda =
        std::max(config.alpha * config.alpha * (n + config.kappa),
                 static_cast<float>(kNumericFloor));
    const float lambda = n_plus_lambda - n;

    CovarianceMatrix scaled = (covariance + covariance.transpose()) * 0.5f;
    for (int i = 0; i < kDim; ++i) {
      scaled(i, i) = std::max(scaled(i, i), kCovarianceFloor);
    }
    scaled *= n_plus_lambda;

    const Eigen::LLT<CovarianceMatrix> llt(scaled);
    if (llt.info() != Eigen::Success) {
      return false;
    }
    const CovarianceMatrix lower = llt.matrixL();

    out->points[0U] = mean;
    out->mean_weights[0U] = lambda / n_plus_lambda;
    out->covariance_weights[0U] =
        out->mean_weights[0U] + (1.0f - config.alpha * config.alpha + config.beta);
    const float side_weight = 0.5f / n_plus_lambda;
    for (int i = 0; i < kDim; ++i) {
      const PointVector column = lower.col(i);
      const std::size_t plus = static_cast<std::size_t>(i) + 1U;
      const std::size_t minus = static_cast<std::size_t>(i) + 1U + static_cast<std::size_t>(kDim);
      out->points[plus] = mean + column;
      out->points[minus] = mean - column;
      out->mean_weights[plus] = side_weight;
      out->mean_weights[minus] = side_weight;
      out->covariance_weights[plus] = side_weight;
      out->covariance_weights[minus] = side_weight;
    }
    return true;
  }

  /**
   * @brief 对称化协方差并对对角线施加正定下限。
   * @param[in] covariance 待稳定化的协方差。
   * @return 稳定化后的协方差。
   */
  static CovarianceMatrix SymmetrizeAndFloor(const CovarianceMatrix& covariance) {
    CovarianceMatrix stabilized = (covariance + covariance.transpose()) * 0.5f;
    for (int i = 0; i < kDim; ++i) {
      stabilized(i, i) = std::max(stabilized(i, i), kCovarianceFloor);
    }
    return stabilized;
  }
};

/**
 * @brief 点集的加权均值（维度与点数解耦：量测维点集复用状态维点数）。
 * @tparam kDim 向量维度。
 * @tparam kCount 点数（2n+1，n 为状态维）。
 */
template <int kDim, std::size_t kCount>
struct UnscentedWeightedMean {
  using Vector = Eigen::Matrix<float, kDim, 1>;

  /**
   * @brief 计算点集的加权均值。
   * @param[in] points 点集。
   * @param[in] weights 权重。
   * @return 加权均值向量。
   */
  static Vector Compute(const std::array<Vector, kCount>& points,
                        const std::array<float, kCount>& weights) {
    Vector mean = Vector::Zero();
    for (std::size_t i = 0U; i < kCount; ++i) {
      mean.noalias() += weights[i] * points[i];
    }
    return mean;
  }
};

/**
 * @brief 两个点集的加权互协方差（kRows×kCols；A=B 时为自协方差）。
 * @tparam kRows A 侧维度。
 * @tparam kCols B 侧维度。
 * @tparam kCount 点数（2n+1）。
 */
template <int kRows, int kCols, std::size_t kCount>
struct UnscentedCrossCovariance {
  using RowVector = Eigen::Matrix<float, kRows, 1>;
  using ColVector = Eigen::Matrix<float, kCols, 1>;
  using CrossMatrix = Eigen::Matrix<float, kRows, kCols>;

  /**
   * @brief 计算加权互协方差 Σ W^c_i·(a_i−ā)(b_i−b̄)ᵀ。
   * @param[in] points_a A 侧点集。
   * @param[in] mean_a A 侧均值。
   * @param[in] points_b B 侧点集。
   * @param[in] mean_b B 侧均值。
   * @param[in] weights 协方差权重 W^c。
   * @return 加权互协方差矩阵。
   */
  static CrossMatrix Compute(const std::array<RowVector, kCount>& points_a,
                             const RowVector& mean_a, const std::array<ColVector, kCount>& points_b,
                             const ColVector& mean_b, const std::array<float, kCount>& weights) {
    CrossMatrix cross = CrossMatrix::Zero();
    for (std::size_t i = 0U; i < kCount; ++i) {
      cross.noalias() += weights[i] * (points_a[i] - mean_a) * (points_b[i] - mean_b).transpose();
    }
    return cross;
  }
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_UNSCENTED_TRANSFORM_H_
