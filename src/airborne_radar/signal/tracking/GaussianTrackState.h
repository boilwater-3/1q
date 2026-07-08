/**
 * @file GaussianTrackState.h
 * @brief 向后兼容外观：将 common/estimation 的模板化高斯状态重导出为 airborne_radar 命名空间下的
 *        6/3 实例化旧名。
 *
 * 滤波原语已迁移至 `common/estimation/GaussianState.h`（模板化）。本头在
 * `airborne_radar::signal::tracking` 命名空间下重导出机载雷达 6 维状态 / 3 维量测实例化，
 * 以及原代码依赖的简短类型别名与维度常量，使现有航迹管理与 pipeline 代码无需逐处改名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_GAUSSIAN_TRACK_STATE_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_GAUSSIAN_TRACK_STATE_H_

#include "common/estimation/GaussianState.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/** @brief 机载雷达 3D 恒速高斯状态（[x, vx, y, vy, z, vz] / [x, y, z]）别名。 */
using GaussianTrackState = ::oneq::common::estimation::GaussianState<6, 3>;

/** @brief 状态空间维度常量（向后兼容旧引用）。 */
static constexpr int kStateDim = GaussianTrackState::state_dim;
/** @brief 量测空间维度常量（向后兼容旧引用）。 */
static constexpr int kMeasurementDim = GaussianTrackState::measurement_dim;

using StateVector = GaussianTrackState::StateVector;
using StateCovariance = GaussianTrackState::StateCovariance;
using MeasurementVector = GaussianTrackState::MeasurementVector;
using MeasurementCovariance = GaussianTrackState::MeasurementCovariance;
using TransitionMatrix = GaussianTrackState::TransitionMatrix;
using ProcessNoiseCovariance = GaussianTrackState::ProcessNoiseCovariance;
using MeasurementMatrix = GaussianTrackState::MeasurementMatrix;
using KalmanGainMatrix = GaussianTrackState::KalmanGainMatrix;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_GAUSSIAN_TRACK_STATE_H_
