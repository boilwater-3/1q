/**
 * @file DeceptionMeasurementGenerator.h
 * @brief 从欺骗候选量测合成假目标量测的内部 pass。
 *
 * 本 pass 消费 ArDeceptionMeasurementCandidateList（由 resolver 在 Complete 阶段逐 member
 * 生成，携带物理 provenance），加上关联引擎分配的 association_key，合成 TrackMeasurement
 * 注入到 track_measurements。
 *
 * 量测的 classified_as_false_target=true 已贯通到 PromoteState 抑制起批。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECEPTION_MEASUREMENT_GENERATOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECEPTION_MEASUREMENT_GENERATOR_H_

#include "airborne_radar/signal/pipeline/CycleExecutor.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 从欺骗候选量测合成假目标量测并追加到 scratch.track_measurements。
 *
 * 读取 context.cycle_input.deception_measurement_candidates（带物理 provenance 的候选
 * 量测）与 scratch.deception_candidate_keys（关联引擎分配的稳定键），为每个带有非零 key
 * 的 candidate 生成一个 TrackMeasurement。候选的 position/velocity/covariance 由 resolver
 * 在生成时填入，本 pass 直接复制量测字段。
 *
 * candidate 的 position 为零向量或 key=0 的被跳过不生成量测。
 *
 * @param context 周期输入上下文（读取 cycle_input.deception_measurement_candidates）。
 * @param scratch 周期暂存区（读取 deception_candidate_keys，追加到 track_measurements）。
 */
void InjectDeceptionMeasurementsPass(const CycleExecutionContext& context,
                                     CycleExecutionScratch& scratch);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECEPTION_MEASUREMENT_GENERATOR_H_
