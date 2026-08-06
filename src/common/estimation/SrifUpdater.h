/**
 * @file SrifUpdater.h
 * @brief 定义 SRIF（Square-Root Information Filter）量测更新器（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_SRIF_UPDATER_H_
#define COMMON_ESTIMATION_SRIF_UPDATER_H_

#include <Eigen/Cholesky>
#include <Eigen/QR>
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
 * @brief SRIF（Square-Root Information Filter）更新器。
 * @details 信息形式更新：将预测协方差与量测噪声协方差都转换为信息矩阵后相加，再以 LDLT 回写。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class SrifUpdater final : public IKalmanUpdater<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using StateVector = typename GaussianStateT::StateVector;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using MeasurementMatrix = typename GaussianStateT::MeasurementMatrix;
  using Result = KalmanUpdateResult<kStateDim, kMeasurementDim>;

  /** @brief 构造函数。 */
  explicit SrifUpdater(KalmanUpdaterConfig config = {})
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

    const Eigen::LLT<StateCovariance> pred_llt(predicted.covariance);
    const Eigen::LLT<MeasurementCovariance> meas_llt(dynamic_R);
    if (pred_llt.info() != Eigen::Success || meas_llt.info() != Eigen::Success) {
      // 中译：预测协方差或量测噪声 R 的 LLT 分解失败。
      // 标识：数值保护——分解失败时跳过更新（后验=预测），
      //       防止数值发散。
      PROJECT_LOG_ERROR("[SrifUpdater] LLT decomposition failed for predicted covariance or R.");
      result.posterior = predicted;
      return result;
    }

    const StateCovariance predicted_information = pred_llt.solve(StateCovariance::Identity());
    const MeasurementCovariance measurement_information =
        meas_llt.solve(MeasurementCovariance::Identity());

    const StateCovariance information =
        predicted_information + H_.transpose() * measurement_information * H_;
    const Eigen::LDLT<StateCovariance> information_ldlt(information);
    if (information_ldlt.info() != Eigen::Success) {
      // 中译：信息矩阵 LDLT 分解失败。
      // 标识：数值保护——信息矩阵非正定时跳过更新（后验=预测），
      //       防止数值发散。
      PROJECT_LOG_ERROR("[SrifUpdater] Information matrix LDLT decomposition failed.");
      result.posterior = predicted;
      return result;
    }

    const StateVector information_vector =
        predicted_information * predicted.mean + H_.transpose() * measurement_information * measurement;
    result.posterior.mean = information_ldlt.solve(information_vector);
    StateCovariance posterior_covariance = information_ldlt.solve(StateCovariance::Identity());
    posterior_covariance = (posterior_covariance + posterior_covariance.transpose()) * 0.5f;
    for (int i = 0; i < kStateDim; ++i) {
      posterior_covariance(i, i) = std::max(posterior_covariance(i, i), kCovarianceFloor);
    }
    result.posterior.covariance = posterior_covariance;
    return result;
  }
  void UpdateConfig(KalmanUpdaterConfig config) override {
    config_ = config;
    R_ = IKalmanUpdater<kStateDim, kMeasurementDim>::BuildDefaultMeasurementNoise(
        config.measurement_noise_std);
  }

 private:
  KalmanUpdaterConfig config_{};
  MeasurementMatrix H_;
  MeasurementCovariance R_;
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_SRIF_UPDATER_H_
