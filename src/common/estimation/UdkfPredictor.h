/**
 * @file UdkfPredictor.h
 * @brief 定义 UD 分解稳定化 Kalman 预测器（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_UDKF_PREDICTOR_H_
#define COMMON_ESTIMATION_UDKF_PREDICTOR_H_

#include <Eigen/Cholesky>
#include <algorithm>

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/numerics/NumericGuard.h"

namespace oneq {
namespace common {
namespace estimation {

using oneq::common::numerics::kCovarianceFloor;

/**
 * @brief UD 分解稳定化预测器。
 * @details 参考 REOS `estimation_lib/udkf.f90` 的 time-update 思路：
 *          对时间推进后的协方差执行 UD 结构化重建，抑制数值退化。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class UdkfPredictor final : public IKalmanPredictor<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using StateVector = typename GaussianStateT::StateVector;
  using TransitionMatrix = typename GaussianStateT::TransitionMatrix;
  using ProcessNoiseCovariance = typename GaussianStateT::ProcessNoiseCovariance;

  /** @brief 构造函数。 */
  explicit UdkfPredictor(KalmanPredictorConfig config = {}) : config_(config) {}

  GaussianStateT Predict(const GaussianStateT& prior, float dt) const override {
    const TransitionMatrix F = KalmanPredictor<kStateDim, kMeasurementDim>::BuildTransitionMatrix(dt);
    const ProcessNoiseCovariance Q =
        KalmanPredictor<kStateDim, kMeasurementDim>::BuildProcessNoise(dt, config_.noise_diff_coeff);

    GaussianStateT predicted;
    predicted.mean = F * prior.mean;

    const StateCovariance propagated_covariance = F * prior.covariance * F.transpose() + Q;
    predicted.covariance = StabilizeCovariance(propagated_covariance);
    return predicted;
  }
  void UpdateConfig(KalmanPredictorConfig config) override { config_ = config; }

 private:
  /**
   * @brief 将协方差分解为 UD（上三角 U + 对角 D）结构。
   * @param[in] covariance 待分解的协方差矩阵。
   * @param[out] upper_u 上三角矩阵 U（输出参数）。
   * @param[out] diagonal_d 对角向量 D（输出参数）。
   * @return 分解成功返回 true，否则返回 false。
   */
  static bool Cov2Ud(const StateCovariance& covariance, StateCovariance* upper_u,
                     StateVector* diagonal_d) {
    if (upper_u == nullptr || diagonal_d == nullptr) {
      return false;
    }
    const StateCovariance symmetric_covariance = (covariance + covariance.transpose()) * 0.5f;
    const Eigen::LDLT<StateCovariance> ldlt(symmetric_covariance);
    if (ldlt.info() != Eigen::Success) {
      return false;
    }

    StateCovariance lower_l = StateCovariance::Identity();
    for (int row = 0; row < kStateDim; ++row) {
      for (int col = 0; col < row; ++col) {
        lower_l(row, col) = ldlt.matrixL()(row, col);
      }
    }
    const Eigen::PermutationMatrix<kStateDim, kStateDim, Eigen::Index> permutation(
        ldlt.transpositionsP());
    *upper_u = lower_l.transpose() * permutation;
    *diagonal_d = ldlt.vectorD().cwiseMax(kCovarianceFloor);
    return true;
  }
  /**
   * @brief 由 UD 结构重建协方差矩阵。
   * @param[in] upper_u 上三角矩阵 U。
   * @param[in] diagonal_d 对角向量 D。
   * @return 重建后的协方差矩阵。
   */
  static StateCovariance Ud2Cov(const StateCovariance& upper_u, const StateVector& diagonal_d) {
    return upper_u.transpose() * diagonal_d.asDiagonal() * upper_u;
  }
  /**
   * @brief 通过 UD 分解稳定化协方差矩阵。
   * @param[in] covariance 待稳定化的协方差矩阵。
   * @return 稳定化后的协方差矩阵。
   */
  static StateCovariance StabilizeCovariance(const StateCovariance& covariance) {
    StateCovariance upper_u = StateCovariance::Identity();
    StateVector diagonal_d = StateVector::Ones();
    if (!Cov2Ud(covariance, &upper_u, &diagonal_d)) {
      StateCovariance fallback = (covariance + covariance.transpose()) * 0.5f;
      for (int i = 0; i < kStateDim; ++i) {
        fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
      }
      return fallback;
    }
    return Ud2Cov(upper_u, diagonal_d);
  }

  KalmanPredictorConfig config_{};
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_UDKF_PREDICTOR_H_
