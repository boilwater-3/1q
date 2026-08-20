/**
 * @file SignalCycleInput.h
 * @brief 定义信号流水线单周期的显式输入结构体，取代分散的 setter 旁路。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_CYCLE_INPUT_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_CYCLE_INPUT_H_

#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/detection/ArDeceptionMeasurementCandidate.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief Complete 阶段冻结并随一次 RunCycle 显式传入的周期输入。
 *
 * 该结构是周期输入，不属于 pipeline 累积状态；pipeline 不缓存、不快照，也不跨周期复用。
 * 它捆绑了原先分散在 ArController::SetPreparedInterferenceObservations、
 * SignalPipeline::SetNextRfV2DetectionContext 和 RunCycle scene_targets 参数中的全部
 * 周期输入。
 */
struct SignalCycleInput {
  session::ArSceneTargetList scene_targets{};
  const RfV2DetectionContext* rf_v2_detection_context{nullptr};
  session::ArInterferenceObservationList interference_observations{};
  detection::ArDeceptionMeasurementCandidateList deception_measurement_candidates{};
  SignalCycleInput() = default;
  explicit SignalCycleInput(session::ArSceneTargetList scene_targets_)
      : scene_targets(std::move(scene_targets_)) {}
  SignalCycleInput(session::ArSceneTargetList scene_targets_,
                   const RfV2DetectionContext* rf_v2_detection_context_,
                   session::ArInterferenceObservationList interference_observations_,
                   detection::ArDeceptionMeasurementCandidateList deception_measurement_candidates_)
      : scene_targets(std::move(scene_targets_)),
        rf_v2_detection_context(rf_v2_detection_context_),
        interference_observations(std::move(interference_observations_)),
        deception_measurement_candidates(std::move(deception_measurement_candidates_)) {}
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_CYCLE_INPUT_H_
