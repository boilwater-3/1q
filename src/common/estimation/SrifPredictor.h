/**
 * @file SrifPredictor.h
 * @brief 定义 SRIF（Square-Root Information Filter）预测器（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_SRIF_PREDICTOR_H_
#define COMMON_ESTIMATION_SRIF_PREDICTOR_H_

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
 * @brief SRIF（Square-Root Information Filter）预测器。
 * @details 参考 REOS `estimation_lib/srif.f90` 的 time-update 思路：
 *          对预测协方差构造信息矩阵并以平方根信息形式稳定回写。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class SrifPredictor final : public IKalmanPredictor<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using TransitionMatrix = typename GaussianStateT::TransitionMatrix;
  using ProcessNoiseCovariance = typename GaussianStateT::ProcessNoiseCovariance;

  /** @brief 构造函数。 */
  explicit SrifPredictor(KalmanPredictorConfig config = {}) : config_(config) {}

  GaussianStateT Predict(const GaussianStateT& prior, float dt) const override {
    const TransitionMatrix F = KalmanPredictor<kStateDim, kMeasurementDim>::BuildTransitionMatrix(dt);
    const ProcessNoiseCovariance Q =
        KalmanPredictor<kStateDim, kMeasurementDim>::BuildProcessNoise(dt, config_.noise_diff_coeff);

    GaussianStateT predicted;
    predicted.mean = F * prior.mean;
    const StateCovariance propagated_covariance = F * prior.covariance * F.transpose() + Q;
    predicted.covariance = StabilizeWithInformationForm(propagated_covariance);
    return predicted;
  }
  void UpdateConfig(KalmanPredictorConfig config) override { config_ = config; }

 private:
  /**
   * @brief 以信息矩阵平方根形式稳定化协方差。
   * @param[in] covariance 待稳定化的协方差矩阵。
   * @return 稳定化后的协方差矩阵。
   */
  static StateCovariance StabilizeWithInformationForm(const StateCovariance& covariance) {
    const StateCovariance symmetric_covariance = (covariance + covariance.transpose()) * 0.5f;
    const Eigen::LLT<StateCovariance> llt(symmetric_covariance);
    if (llt.info() != Eigen::Success) {
      StateCovariance fallback = symmetric_covariance;
      for (int i = 0; i < kStateDim; ++i) {
        fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
      }
      return fallback;
    }

    const StateCovariance information = llt.solve(StateCovariance::Identity());
    const Eigen::LDLT<StateCovariance> information_ldlt(information);
    if (information_ldlt.info() != Eigen::Success) {
      StateCovariance fallback = symmetric_covariance;
      for (int i = 0; i < kStateDim; ++i) {
        fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
      }
      return fallback;
    }

    StateCovariance stabilized = information_ldlt.solve(StateCovariance::Identity());
    stabilized = (stabilized + stabilized.transpose()) * 0.5f;
    for (int i = 0; i < kStateDim; ++i) {
      stabilized(i, i) = std::max(stabilized(i, i), kCovarianceFloor);
    }
    return stabilized;
  }

  KalmanPredictorConfig config_{};
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_SRIF_PREDICTOR_H_
