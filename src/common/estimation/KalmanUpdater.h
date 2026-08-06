/**
 * @file KalmanUpdater.h
 * @brief 定义标准线性 Kalman 量测更新器（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_KALMAN_UPDATER_H_
#define COMMON_ESTIMATION_KALMAN_UPDATER_H_

#include <Eigen/Cholesky>

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/logging/ProjectLog.h"

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief 标准线性 Kalman 更新器。
 * @details 参考 Stone Soup KalmanUpdater.update()。
 *          量测模型 H 为线性位置提取矩阵（6 维 CV 状态下：
 *          | 1 0 0 0 0 0 |
 *          | 0 0 1 0 0 0 |
 *          | 0 0 0 0 1 0 |）。
 *
 *          核心公式：
 *          - 新息：y = z - H·x̂
 *          - 新息协方差：S = H·P̂·Hᵀ + R
 *          - Kalman 增益：K = P̂·Hᵀ·S⁻¹
 *          - 后验均值：x = x̂ + K·y
 *          - 后验协方差（Joseph 形式）：P = (I - K·H)·P̂·(I - K·H)ᵀ + K·R·Kᵀ
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class KalmanUpdater final : public IKalmanUpdater<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using MeasurementMatrix = typename GaussianStateT::MeasurementMatrix;
  using KalmanGainMatrix = typename GaussianStateT::KalmanGainMatrix;
  using Result = KalmanUpdateResult<kStateDim, kMeasurementDim>;

  /**
   * @brief 构造函数。
   * @param[in] config 更新器配置。
   */
  explicit KalmanUpdater(KalmanUpdaterConfig config = {})
      : config_(config),
        H_(IKalmanUpdater<kStateDim, kMeasurementDim>::BuildPositionMeasurementMatrix()),
        R_(IKalmanUpdater<kStateDim, kMeasurementDim>::BuildDefaultMeasurementNoise(
            config.measurement_noise_std)) {}

  Result Update(const GaussianStateT& predicted,
                const MeasurementVector& measurement) const override {
    return Update(predicted, measurement, R_);
  }
  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement,
                const MeasurementCovariance& dynamic_R) const override {
    Result result;

    /* Innovation (新息) */
    result.innovation = measurement - H_ * predicted.mean;

    /* Innovation covariance: S = H · P̂ · Hᵀ + R */
    result.innovation_covariance = H_ * predicted.covariance * H_.transpose() + dynamic_R;

    /* Kalman gain: K = (S⁻¹ · H · P̂)ᵀ，避免计算显式逆矩阵，数值更稳定 */
    const Eigen::LLT<MeasurementCovariance> llt(result.innovation_covariance);
    if (llt.info() != Eigen::Success) {
      // 中译：创新协方差 LLT 分解失败，本次更新被跳过。
      // 标识：数值保护——协方差非正定时跳过更新（后验=预测），
      //       防止数值发散。
      PROJECT_LOG_ERROR(
          "[KalmanUpdater] Innovation covariance LLT decomposition failed; update is skipped.");
      result.posterior = predicted;
      return result;
    }
    const KalmanGainMatrix K = llt.solve(H_ * predicted.covariance).transpose();

    /* Posterior mean: x = x̂ + K · y */
    result.posterior.mean = predicted.mean + K * result.innovation;

    /* Posterior covariance (Joseph 形式): P = (I - K·H) · P̂ · (I - K·H)ᵀ + K · R · Kᵀ */
    const StateCovariance I_KH = StateCovariance::Identity() - K * H_;
    result.posterior.covariance =
        I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();

    return result;
  }
  void UpdateConfig(KalmanUpdaterConfig config) override {
    config_ = config;
    R_ = IKalmanUpdater<kStateDim, kMeasurementDim>::BuildDefaultMeasurementNoise(
        config.measurement_noise_std);
  }

 private:
  KalmanUpdaterConfig config_{}; /**< 当前配置。 */
  MeasurementMatrix H_;          /**< 量测矩阵 H（编译期可确定，只构建一次）。 */
  MeasurementCovariance R_;      /**< 量测噪声协方差 R。 */
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_KALMAN_UPDATER_H_
