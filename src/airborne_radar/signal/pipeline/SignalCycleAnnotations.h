/**
 * @file SignalCycleAnnotations.h
 * @brief 定义信号流水线单周期的显式接收端 annotation 输入。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_CYCLE_ANNOTATIONS_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_CYCLE_ANNOTATIONS_H_

#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "airborne_radar/signal/detection/ArDeceptionMeasurementCandidate.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief Complete 阶段冻结并随一次 RunCycle 显式传入的接收端 annotation。
 *
 * 该结构是周期输入，不属于 pipeline 累积状态；pipeline 不缓存、不快照，也不跨周期复用。
 */
struct SignalCycleAnnotations {
  session::ArInterferenceObservationList interference_observations{};
  detection::ArDeceptionMeasurementCandidateList deception_measurement_candidates{};
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_CYCLE_ANNOTATIONS_H_
