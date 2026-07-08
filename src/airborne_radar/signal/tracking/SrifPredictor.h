/**
 * @file SrifPredictor.h
 * @brief 向后兼容外观：将 common/estimation 模板化 SRIF 预测器重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_PREDICTOR_H_

#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "common/estimation/SrifPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/** @brief SRIF 预测器（6 维状态 / 3 维量测实例化）。 */
using SrifPredictor = ::oneq::common::estimation::SrifPredictor<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_PREDICTOR_H_
