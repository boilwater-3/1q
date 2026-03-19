/**
 * @file IKalmanUpdater.h
 * @brief 定义基于 Kalman 滤波的量测更新器抽象接口。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_UPDATER_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief Kalman 更新器配置。
 */
struct KalmanUpdaterConfig {
  float measurement_noise_std{10.0f};  /**< 位置量测噪声标准差（米），各轴相同。对应 Stone Soup MeasurementModel 中 noise_covar 的对角元素平方根。 */
};
/**
 * @brief Kalman 更新结果，包含后验状态和增益等诊断信息。
 */
struct KalmanUpdateResult {
  GaussianTrackState posterior;  /**< 后验高斯状态。 */
  MeasurementVector innovation{MeasurementVector::Zero()};  /**< 新息向量 y = z - H·x̂。 */
  MeasurementCovariance innovation_covariance{MeasurementCovariance::Identity()};  /**< 新息协方差 S = H·P̂·Hᵀ + R。 */
};
/**
 * @brief Kalman 更新器抽象接口。
 */
class IKalmanUpdater {
 public:
  virtual ~IKalmanUpdater() = default;
/**
 * @brief 对预测状态执行量测更新。
 * @param predicted 预测后的高斯状态。
 * @param measurement 量测向量 [x, y, z]。
 * @return 更新结果，包含后验状态和诊断信息。
 */
  virtual KalmanUpdateResult Update(const GaussianTrackState &predicted,
                                    const MeasurementVector &measurement) const = 0;
/**
 * @brief 对预测状态执行带有动态误差协方差矩阵的量测更新。
 * @param predicted 预测后的高斯状态。
 * @param measurement 量测向量 [x, y, z]。
 * @param dynamic_R 动态计算的笛卡尔系量测噪声协方差矩阵 R。
 * @return 更新结果，包含后验状态和诊断信息。
 */
  virtual KalmanUpdateResult Update(const GaussianTrackState &predicted,
                                    const MeasurementVector &measurement,
                                    const MeasurementCovariance &dynamic_R) const = 0;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_UPDATER_H_
