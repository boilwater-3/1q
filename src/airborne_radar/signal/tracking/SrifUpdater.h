/**
 * @file SrifUpdater.h
 * @brief 定义 SRIF 量测更新器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_

#include "airborne_radar/signal/tracking/IKalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief SRIF（Square-Root Information Filter）更新器。
 */
class SrifUpdater final : public IKalmanUpdater {
 public:
  /**
   * @brief 构造函数。
   * @param config 更新器配置。
   */
  explicit SrifUpdater(KalmanUpdaterConfig config = {});

  /**
   * @brief 对预测状态执行 SRIF 量测更新。
   * @param predicted 预测后的高斯状态。
   * @param measurement 量测向量 [x, y, z]。
   * @return 更新结果。
   */
  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement) const override;
  /**
   * @brief 对预测状态执行 SRIF 量测更新（使用动态观测协方差）。
   * @param predicted 预测后的高斯状态。
   * @param measurement 量测向量 [x, y, z]。
   * @param dynamic_R 动态计算的笛卡尔系量测噪声协方差矩阵 R。
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
  KalmanUpdaterConfig config_{};
  MeasurementMatrix H_;
  MeasurementCovariance R_;
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_
