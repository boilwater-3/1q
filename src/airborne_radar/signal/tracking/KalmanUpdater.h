/**
 * @file KalmanUpdater.h
 * @brief 定义基于 Kalman 滤波的量测更新器接口与标准 Kalman 更新实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_UPDATER_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief 标准线性 Kalman 更新器。
 * @details 参考 Stone Soup KalmanUpdater.update()。
 *          量测模型 H 为线性位置提取矩阵：
 *          | 1 0 0 0 0 0 |
 *          | 0 0 1 0 0 0 |
 *          | 0 0 0 0 1 0 |
 *
 *          核心公式：
 *          - 新息：y = z - H·x̂
 *          - 新息协方差：S = H·P̂·Hᵀ + R
 *          - Kalman 增益：K = P̂·Hᵀ·S⁻¹
 *          - 后验均值：x = x̂ + K·y
 *          - 后验协方差（Joseph 形式）：P = (I - K·H)·P̂·(I - K·H)ᵀ + K·R·Kᵀ
 */
class KalmanUpdater final : public IKalmanUpdater {
 public:
  /**
   * @brief 构造函数。
   * @param config 更新器配置。
   */
  explicit KalmanUpdater(KalmanUpdaterConfig config = {});
  /**
   * @brief 对预测状态执行标准 Kalman 量测更新。
   * @param predicted 预测后的高斯状态。
   * @param measurement 量测向量 [x, y, z]。
   * @return 更新结果。
   */
  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement) const override;
  /**
   * @brief 对预测状态执行标准 Kalman 量测更新（使用动态雷达观测协方差）。
   * @param predicted 预测后的高斯状态。
   * @param measurement 量测向量 [x, y, z]。
   * @param dynamic_R 笛卡尔坐标系下动态解算的量测噪声协方差。
   * @return 更新结果。
   */
  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement,
                            const MeasurementCovariance& dynamic_R) const override;
  /**
   * @brief 更新配置。
   * @param config 新的更新器配置。
   */
  void UpdateConfig(KalmanUpdaterConfig config) override;

 private:
  /**
   * @brief 构建量测矩阵 H（3×6 位置提取矩阵）。
   * @return 量测矩阵。
   */
  static MeasurementMatrix BuildMeasurementMatrix();
  /**
   * @brief 构建量测噪声协方差矩阵 R（3×3 对角矩阵）。
   * @param std_dev 量测噪声标准差。
   * @return 量测噪声协方差矩阵。
   */
  static MeasurementCovariance BuildMeasurementNoise(float std_dev);
  KalmanUpdaterConfig config_{}; /**< 当前配置。 */
  MeasurementMatrix H_;          /**< 量测矩阵 H（编译期可确定，只构建一次）。 */
  MeasurementCovariance R_;      /**< 量测噪声协方差 R。 */
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_UPDATER_H_
