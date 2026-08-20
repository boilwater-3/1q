/**
 * @file UdkfUpdater.h
 * @brief 定义 UD 分解稳定化 Kalman 量测更新器（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_UDKF_UPDATER_H_
#define COMMON_ESTIMATION_UDKF_UPDATER_H_

#include <Eigen/Cholesky>
#include <algorithm>

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/NumericGuard.h"

namespace oneq {
namespace common {
namespace estimation {

using oneq::common::numerics::kCovarianceFloor;

/**
 * @brief UD 分解稳定化更新器。
 * @details 标准 Kalman 更新后对 Joseph 形式协方差再做 UD 结构化重建，抑制数值退化。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class UdkfUpdater final : public IKalmanUpdater<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using MeasurementMatrix = typename GaussianStateT::MeasurementMatrix;
  using KalmanGainMatrix = typename GaussianStateT::KalmanGainMatrix;
  using Result = KalmanUpdateResult<kStateDim, kMeasurementDim>;

  /** @brief 构造函数。 */
  explicit UdkfUpdater(KalmanUpdaterConfig config = {})
      : config_(config),
        H_(IKalmanUpdater<kStateDim, kMeasurementDim>::BuildPositionMeasurementMatrix()),
        R_(IKalmanUpdater<kStateDim, kMeasurementDim>::BuildDefaultMeasurementNoise(
            config.measurement_noise_std)) {}

  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement) const override {
    return Update(predicted, measurement, R_);
  }
  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement,
                const MeasurementCovariance& dynamic_R) const override {
    Result result;
    result.innovation = measurement - H_ * predicted.mean;
    result.innovation_covariance = H_ * predicted.covariance * H_.transpose() + dynamic_R;

    const Eigen::LLT<MeasurementCovariance> llt(result.innovation_covariance);
    if (llt.info() != Eigen::Success) {
      // 中译：创新协方差 LLT 分解失败。
      // 标识：数值保护——协方差非正定时跳过更新（后验=预测），
      //       防止数值发散。
      PROJECT_LOG_ERROR("[UdkfUpdater] Innovation covariance LLT decomposition failed.");
      result.posterior = predicted;
      return result;
    }

    const KalmanGainMatrix K = llt.solve(H_ * predicted.covariance).transpose();
    result.posterior.mean = predicted.mean + K * result.innovation;

    const StateCovariance I_KH = StateCovariance::Identity() - K * H_;
    const StateCovariance joseph_covariance =
        I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();
    result.posterior.covariance = StabilizeCovarianceWithUd(joseph_covariance);
    return result;
  }
  void UpdateConfig(KalmanUpdaterConfig config) override {
    config_ = config;
    R_ = IKalmanUpdater<kStateDim, kMeasurementDim>::BuildDefaultMeasurementNoise(
        config.measurement_noise_std);
  }

 private:
  /**
   * @brief 通过 UD 分解稳定化协方差矩阵。
   * @param[in] covariance 待稳定化的协方差矩阵。
   * @return 稳定化后的协方差矩阵。
   */
  static StateCovariance StabilizeCovarianceWithUd(const StateCovariance& covariance) {
    const StateCovariance symmetric_covariance = (covariance + covariance.transpose()) * 0.5f;
    Eigen::LDLT<StateCovariance> ldlt(symmetric_covariance);
    if (ldlt.info() != Eigen::Success) {
      StateCovariance fallback = symmetric_covariance;
      for (int i = 0; i < kStateDim; ++i) {
        fallback(i, i) = std::max(fallback(i, i), kCovarianceFloor);
      }
      return fallback;
    }

    const Eigen::Matrix<float, kStateDim, 1> diagonal = ldlt.vectorD().cwiseMax(kCovarianceFloor);
    StateCovariance lower = StateCovariance::Identity();
    for (int row = 0; row < kStateDim; ++row) {
      for (int col = 0; col < row; ++col) {
        lower(row, col) = ldlt.matrixL()(row, col);
      }
    }
    return lower * diagonal.asDiagonal() * lower.transpose();
  }

  KalmanUpdaterConfig config_{};
  MeasurementMatrix H_;
  MeasurementCovariance R_;
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_UDKF_UPDATER_H_
