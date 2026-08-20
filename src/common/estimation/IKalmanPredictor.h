/**
 * @file IKalmanPredictor.h
 * @brief 定义基于 Kalman 滤波的状态预测器抽象接口（维度模板化）。
 */

#ifndef COMMON_ESTIMATION_I_KALMAN_PREDICTOR_H_
#define COMMON_ESTIMATION_I_KALMAN_PREDICTOR_H_

#include "common/estimation/GaussianState.h"

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief Kalman 预测器配置。
 */
struct KalmanPredictorConfig {
  /**
   * @brief 过程噪声扩散系数 q（单位：m/s²），建模加速度白噪声强度。
   * @details 对应 Stone Soup ConstantVelocity 的 noise_diff_coeff。
   *          值越大表示目标运动越不确定，协方差增长越快。
   */
  float noise_diff_coeff{1.0f};
};

/**
 * @brief Kalman 预测器抽象接口。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度（预测器不直接使用，保留以与更新器/状态对齐）。
 */
template <int kStateDim, int kMeasurementDim>
class IKalmanPredictor {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;

  virtual ~IKalmanPredictor() = default;
  /**
   * @brief 对先验状态执行时间外推预测。
   * @param[in] prior 先验高斯状态。
   * @param[in] dt 预测时间步长（秒）。
   * @return 预测后的高斯状态。
   */
  virtual GaussianStateT Predict(const GaussianStateT& prior, float dt) const = 0;
  /**
   * @brief 更新预测器配置。
   * @details 默认实现为空操作。使用 KalmanPredictorConfig 的子类应重写本方法。
   *          使用自定义配置类型的子类（如 EkfPredictor）保留其自身的 UpdateConfig 重载。
   * @param[in] config 预测器配置。
   */
  virtual void UpdateConfig(KalmanPredictorConfig /*config*/) {}
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_I_KALMAN_PREDICTOR_H_
