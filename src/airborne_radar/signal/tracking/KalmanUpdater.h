// Copyright 2026. All Rights Reserved.
//
// 文件说明：定义基于 Kalman 滤波的量测更新器接口与标准 Kalman 更新实现。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_UPDATER_H_

#include "1q/airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief Kalman 更新器配置。
struct KalmanUpdaterConfig {
  /// @brief 位置量测噪声标准差（米），各轴相同。
  /// @details 对应 Stone Soup MeasurementModel 中 noise_covar 的对角元素平方根。
  float measurement_noise_std{10.0f};
};

/// @brief Kalman 更新结果，包含后验状态和增益等诊断信息。
struct KalmanUpdateResult {
  /// @brief 后验高斯状态。
  GaussianTrackState posterior;

  /// @brief 新息向量 y = z - H·x̂。
  MeasurementVector innovation{MeasurementVector::Zero()};

  /// @brief 新息协方差 S = H·P̂·Hᵀ + R。
  MeasurementCovariance innovation_covariance{MeasurementCovariance::Identity()};
};

/// @brief Kalman 更新器抽象接口。
class IKalmanUpdater {
 public:
  /// @brief 析构函数。
  virtual ~IKalmanUpdater() = default;

  /// @brief 对预测状态执行量测更新。
  /// @param predicted 预测后的高斯状态。
  /// @param measurement 量测向量 [x, y, z]。
  /// @return 更新结果，包含后验状态和诊断信息。
  virtual KalmanUpdateResult Update(const GaussianTrackState &predicted,
                                    const MeasurementVector &measurement) const = 0;
};

/// @brief 标准线性 Kalman 更新器。
/// @details 参考 Stone Soup KalmanUpdater.update()。
///          量测模型 H 为线性位置提取矩阵：
///          | 1 0 0 0 0 0 |
///          | 0 0 1 0 0 0 |
///          | 0 0 0 0 1 0 |
///
///          核心公式：
///          - 新息：y = z - H·x̂
///          - 新息协方差：S = H·P̂·Hᵀ + R
///          - Kalman 增益：K = P̂·Hᵀ·S⁻¹
///          - 后验均值：x = x̂ + K·y
///          - 后验协方差（Joseph 形式）：P = (I - K·H)·P̂·(I - K·H)ᵀ + K·R·Kᵀ
class KalmanUpdater final : public IKalmanUpdater {
 public:
  /// @brief 构造函数。
  /// @param config 更新器配置。
  explicit KalmanUpdater(KalmanUpdaterConfig config = {});

  /// @brief 对预测状态执行标准 Kalman 量测更新。
  /// @param predicted 预测后的高斯状态。
  /// @param measurement 量测向量 [x, y, z]。
  /// @return 更新结果。
  KalmanUpdateResult Update(const GaussianTrackState &predicted,
                            const MeasurementVector &measurement) const override;

  /// @brief 更新配置。
  /// @param config 新的更新器配置。
  void UpdateConfig(KalmanUpdaterConfig config);

 private:
  /// @brief 构建量测矩阵 H（3×6 位置提取矩阵）。
  /// @return 量测矩阵。
  static MeasurementMatrix BuildMeasurementMatrix();

  /// @brief 构建量测噪声协方差矩阵 R（3×3 对角矩阵）。
  /// @param std_dev 量测噪声标准差。
  /// @return 量测噪声协方差矩阵。
  static MeasurementCovariance BuildMeasurementNoise(float std_dev);

  /// @brief 当前配置。
  KalmanUpdaterConfig config_{};

  /// @brief 量测矩阵 H（编译期可确定，只构建一次）。
  MeasurementMatrix H_;

  /// @brief 量测噪声协方差 R。
  MeasurementCovariance R_;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_UPDATER_H_
