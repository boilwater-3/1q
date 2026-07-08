/**
 * @file IKalmanUpdater.h
 * @brief 向后兼容外观：将 common/estimation 模板化 Kalman 更新器接口重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_UPDATER_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "common/estimation/IKalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

using ::oneq::common::estimation::KalmanUpdaterConfig;
/** @brief Kalman 更新器抽象接口（6 维状态 / 3 维量测实例化）。 */
using IKalmanUpdater = ::oneq::common::estimation::IKalmanUpdater<6, 3>;
/** @brief Kalman 更新结果（6/3 实例化）。 */
using KalmanUpdateResult = ::oneq::common::estimation::KalmanUpdateResult<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_I_KALMAN_UPDATER_H_
