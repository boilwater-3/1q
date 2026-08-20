/**
 * @file ImmFilter.h
 * @brief 向后兼容外观：将 common/estimation 模板化 IMM 滤波器重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_IMM_FILTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_IMM_FILTER_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "common/estimation/ImmFilter.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/** @brief IMM 模型分支状态（6/3 实例化）。 */
using ImmModelState = ::oneq::common::estimation::ImmModelState<6, 3>;

using ::oneq::common::estimation::ImmConfig;

/** @brief 交互多模型（IMM）滤波器（6 维状态 / 3 维量测实例化）。 */
using ImmFilter = ::oneq::common::estimation::ImmFilter<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_IMM_FILTER_H_
