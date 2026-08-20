/**
 * @file KalmanPredictor.h
 * @brief 向后兼容外观：将 common/estimation 模板化恒速 Kalman 预测器重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_PREDICTOR_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "common/estimation/KalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief 3D 恒速 Kalman 预测器（6 维状态 / 3 维量测实例化）。
 * @details 实现已迁移至 `common/estimation/KalmanPredictor.h`。静态工具方法
 *          BuildTransitionMatrix / BuildProcessNoise 仍可经此别名访问。
 */
using KalmanPredictor = ::oneq::common::estimation::KalmanPredictor<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_KALMAN_PREDICTOR_H_
