/**
 * @file UnscentedUpdater.h
 * @brief 定义无迹（Unscented）Kalman 更新器（维度模板化）。
 *
 * 参考 Stone Soup UnscentedKalmanUpdater：状态 sigma 点经非线性量测函数 h(x) 传播得到
 * 量测点集，重建量测先验 (z̄, S)、互协方差 P_xz 后按标准增益形式更新。与 EkfUpdater
 * 共享 IMeasurementModel 注入形态，但只消费 Function，不解 Jacobian，无 H_ 成员。
 */

#ifndef COMMON_ESTIMATION_UNSCENTED_UPDATER_H_
#define COMMON_ESTIMATION_UNSCENTED_UPDATER_H_

#include <Eigen/Cholesky>

#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/estimation/UnscentedTransform.h"
#include "common/logging/ProjectLog.h"

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief 无迹更新器配置。
 */
struct UnscentedUpdaterConfig {
  float measurement_noise_std{10.0f}; /**< 量测噪声标准差（各轴相同）。 */
  UnscentedTransformConfig transform{}; /**< 无迹变换参数。 */
};

/**
 * @brief 无迹 Kalman 更新器。
 * @details 量测更新：
 *          - Z_i = h(X_i)；z̄ = Σ W^m_i·Z_i
 *          - S = Σ W^c_i·(Z_i−z̄)(Z_i−z̄)ᵀ + R；P_xz = Σ W^c_i·(X_i−x̂)(Z_i−z̄)ᵀ
 *          - K = P_xz·S⁻¹；x_post = x̂ + K(z−z̄)；P_post = P̂ − K·S·Kᵀ
 *          线性极限下与 KalmanUpdater 数值一致。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class UnscentedUpdater final : public IKalmanUpdater<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using MeasurementModel = IMeasurementModel<kStateDim, kMeasurementDim>;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using KalmanGainMatrix = typename GaussianStateT::KalmanGainMatrix;
  using Result = KalmanUpdateResult<kStateDim, kMeasurementDim>;
  using Transform = UnscentedTransform<kStateDim>;
  using MeasurementTransform = UnscentedTransform<kMeasurementDim>;
  using PointSet = typename Transform::PointSet;

  /**
   * @brief 构造函数。
   * @param[in] model 量测模型（非拥有指针）。
   * @param[in] config 更新器配置。
   */
  UnscentedUpdater(const MeasurementModel* model, UnscentedUpdaterConfig config = {})
      : model_(model),
        config_(config),
        R_(IKalmanUpdater<kStateDim, kMeasurementDim>::BuildDefaultMeasurementNoise(
            config.measurement_noise_std)) {}

  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement) const override {
    return Update(predicted, measurement, R_);
  }
  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement,
                const MeasurementCovariance& dynamic_R) const override {
    Result result;
    if (model_ == nullptr) {
      // 中译：量测模型未注入，本次更新被跳过（后验=预测）。
      // 标识：注入校验——空模型指针属装配缺陷；fail-safe 跳过更新，
      //       不解引用空指针。
      PROJECT_LOG_ERROR(
          "[UnscentedUpdater] Measurement model is null; update is skipped (posterior = predicted).");
      result.posterior = predicted;
      return result;
    }

    PointSet point_set;
    if (!Transform::GenerateSigmaPoints(predicted.mean, predicted.covariance, config_.transform,
                                        &point_set)) {
      // 中译：sigma 点生成失败（协方差 LLT 分解失败），本次更新被跳过。
      // 标识：数值保护——协方差非正定时 fail-safe 跳过更新（后验=预测），
      //       防止数值发散。
      PROJECT_LOG_ERROR(
          "[UnscentedUpdater] Sigma point generation failed; update is skipped (posterior = predicted).");
      result.posterior = predicted;
      return result;
    }

    /* 状态 sigma 点经非线性量测函数传播 */
    std::array<MeasurementVector, Transform::kCount> measurement_points{};
    for (std::size_t i = 0U; i < Transform::kCount; ++i) {
      measurement_points[i] = model_->Function(point_set.points[i]);
    }

    /* 量测先验与互协方差 */
    const MeasurementVector predicted_measurement =
        UnscentedWeightedMean<kMeasurementDim, Transform::kCount>::Compute(
            measurement_points, point_set.mean_weights);
    const MeasurementCovariance innovation_covariance =
        UnscentedCrossCovariance<kMeasurementDim, kMeasurementDim, Transform::kCount>::Compute(
            measurement_points, predicted_measurement, measurement_points, predicted_measurement,
            point_set.covariance_weights) +
        dynamic_R;
    const KalmanGainMatrix cross_covariance =
        UnscentedCrossCovariance<kStateDim, kMeasurementDim, Transform::kCount>::Compute(
            point_set.points, predicted.mean, measurement_points, predicted_measurement,
            point_set.covariance_weights);

    /* Innovation */
    result.innovation = measurement - predicted_measurement;
    result.innovation_covariance = innovation_covariance;

    /* Kalman gain：K = (S⁻¹·P_xzᵀ)ᵀ，避免计算显式逆 */
    const Eigen::LLT<MeasurementCovariance> llt(innovation_covariance);
    if (llt.info() != Eigen::Success) {
      // 中译：新息协方差 LLT 分解失败，本次更新被跳过。
      // 标识：数值保护——协方差非正定时跳过更新（后验=预测），防止数值发散。
      PROJECT_LOG_ERROR(
          "[UnscentedUpdater] Innovation covariance LLT decomposition failed; update is skipped.");
      result.posterior = predicted;
      return result;
    }
    const KalmanGainMatrix K = llt.solve(cross_covariance.transpose()).transpose();

    /* Posterior */
    result.posterior.mean = predicted.mean + K * result.innovation;
    result.posterior.covariance = Transform::SymmetrizeAndFloor(
        predicted.covariance - K * innovation_covariance * K.transpose());
    return result;
  }

 private:
  const MeasurementModel* model_{nullptr}; /**< 量测模型 */
  UnscentedUpdaterConfig config_{};        /**< 配置参数 */
  MeasurementCovariance R_;                /**< 静态量测噪声协方差 */
};

extern template class UnscentedUpdater<6, 3>;
extern template class UnscentedUpdater<6, 2>;

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_UNSCENTED_UPDATER_H_
