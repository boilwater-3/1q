/**
 * @file UdkfPredictor.h
 * @brief 向后兼容外观：将 common/estimation 模板化 UD 分解预测器重导出为 6/3 实例化旧名。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_PREDICTOR_H_

#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "common/estimation/UdkfPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/** @brief UD 分解稳定化预测器（6 维状态 / 3 维量测实例化）。 */
using UdkfPredictor = ::oneq::common::estimation::UdkfPredictor<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_PREDICTOR_H_
