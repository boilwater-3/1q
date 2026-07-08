/**
 * @file SrifUpdater.h
 * @brief 向后兼容外观：将 common/estimation 模板化 SRIF 更新器重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_

#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "common/estimation/SrifUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/** @brief SRIF 更新器（6 维状态 / 3 维量测实例化）。 */
using SrifUpdater = ::oneq::common::estimation::SrifUpdater<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_
