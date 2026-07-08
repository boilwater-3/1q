/**
 * @file EkfFilter.h
 * @brief 向后兼容外观：将 common/estimation 模板化 EKF 重导出为 6/3 实例化旧名。
 *
 * 重导出 ITransitionModel / IMeasurementModel 抽象接口、LinearCv / LinearPosition 默认实现，
 * 以及 EkfPredictor / EkfUpdater 和配置结构。非线性量测模型子类化点由 IMeasurementModel 提供。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_EKF_FILTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_EKF_FILTER_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "common/estimation/EkfFilter.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

using ::oneq::common::estimation::EkfPredictorConfig;
using ::oneq::common::estimation::EkfUpdaterConfig;

/** @brief 6 维状态非线性转移模型抽象接口（对应旧 ITransitionModel）。 */
using ITransitionModel = ::oneq::common::estimation::ITransitionModel<6>;

/** @brief 非线性量测模型抽象接口（6 维状态 / 3 维量测实例化）。 */
using IMeasurementModel = ::oneq::common::estimation::IMeasurementModel<6, 3>;

/** @brief 线性恒速转移模型（6 维状态，默认实现）。 */
using LinearCvTransitionModel = ::oneq::common::estimation::LinearCvTransitionModel<6>;

/** @brief 线性位置提取量测模型（6/3 默认实现）。 */
using LinearPositionMeasurementModel = ::oneq::common::estimation::LinearPositionMeasurementModel<6, 3>;

using ::oneq::common::estimation::EkfPredictorConfig;
/** @brief 扩展 Kalman 预测器（6/3 实例化）。 */
using EkfPredictor = ::oneq::common::estimation::EkfPredictor<6, 3>;
/** @brief 扩展 Kalman 更新器（6/3 实例化）。 */
using EkfUpdater = ::oneq::common::estimation::EkfUpdater<6, 3>;

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_EKF_FILTER_H_
