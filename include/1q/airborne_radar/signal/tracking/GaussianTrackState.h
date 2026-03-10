// Copyright 2026. All Rights Reserved.
//
// 文件说明：定义高斯状态表示，用于 Kalman 滤波系列的状态预测与更新。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_GAUSSIAN_TRACK_STATE_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_GAUSSIAN_TRACK_STATE_H_

#include <Eigen/Core>

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief 状态空间维度常量。
/// @details 采用 3D 恒速模型：[x, vx, y, vy, z, vz]。
static constexpr int kStateDim = 6;

/// @brief 量测空间维度常量。
/// @details 直接量测三维位置：[x, y, z]。
static constexpr int kMeasurementDim = 3;

/// @brief 状态向量类型（固定维度，列向量）。
using StateVector = Eigen::Matrix<float, kStateDim, 1>;

/// @brief 状态协方差矩阵类型。
using StateCovariance = Eigen::Matrix<float, kStateDim, kStateDim>;

/// @brief 量测向量类型。
using MeasurementVector = Eigen::Matrix<float, kMeasurementDim, 1>;

/// @brief 量测协方差矩阵类型。
using MeasurementCovariance = Eigen::Matrix<float, kMeasurementDim, kMeasurementDim>;

/// @brief 状态转移矩阵类型。
using TransitionMatrix = Eigen::Matrix<float, kStateDim, kStateDim>;

/// @brief 过程噪声协方差矩阵类型。
using ProcessNoiseCovariance = Eigen::Matrix<float, kStateDim, kStateDim>;

/// @brief 量测矩阵类型（H: kMeasurementDim × kStateDim）。
using MeasurementMatrix = Eigen::Matrix<float, kMeasurementDim, kStateDim>;

/// @brief Kalman 增益矩阵类型。
using KalmanGainMatrix = Eigen::Matrix<float, kStateDim, kMeasurementDim>;

/// @brief 高斯状态表示，作为 Kalman 滤波器的核心数据载体。
/// @details 封装状态均值向量和协方差矩阵，对应 Stone Soup 中的 GaussianState。
///          状态向量布局为 [x, vx, y, vy, z, vz]，位置和速度交替排列。
struct GaussianTrackState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// @brief 状态均值向量。
  StateVector mean{StateVector::Zero()};

  /// @brief 状态协方差矩阵。
  StateCovariance covariance{StateCovariance::Identity()};

  /// @brief 默认构造函数。
  GaussianTrackState() = default;

  /// @brief 从均值和协方差构造。
  /// @param m 状态均值向量。
  /// @param p 状态协方差矩阵。
  GaussianTrackState(const StateVector &m, const StateCovariance &p)
      : mean(m), covariance(p) {}
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_GAUSSIAN_TRACK_STATE_H_
