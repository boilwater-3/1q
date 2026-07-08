/**
 * @file UdkfUpdater.h
 * @brief 向后兼容外观：将 common/estimation 模板化 UD 分解更新器重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_UPDATER_H_

#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "common/estimation/UdkfUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/** @brief UD 分解稳定化更新器（6 维状态 / 3 维量测实例化）。 */
using UdkfUpdater = ::oneq::common::estimation::UdkfUpdater<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_UPDATER_H_
