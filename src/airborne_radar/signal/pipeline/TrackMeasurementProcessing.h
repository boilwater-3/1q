/**
 * @file TrackMeasurementProcessing.h
 * @brief 定义 SignalPipeline 量测构建与滤波写回的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/signal/pipeline/CycleExecutor.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 量测构建阶段：遍历所有成功检测目标，构建跟踪量测并写入 scratch。
 *
 * 读取 scratch.detection_succeeded / association_result / association_keys /
 * detection_margin_db / target_geometry / measurement_covariances /
 * 写入 scratch.measurement_slots / track_measurements。
 *
 * @param input          本周期输入目标列表。
 * @param scratch        周期暂存区（读取探测/关联结果，写入量测）。
 */
void BuildTrackMeasurementsPass(const session::ArSceneTargetList& input,
                                CycleExecutionScratch& scratch);

/**
 * @brief 滤波写回阶段：对 scratch.output_state 的每个目标执行轨迹滤波，
 *        并将滤波特征回写到对应的 track_measurements 槽位。
 *
 * 读取 scratch.detection_succeeded / detection_margin_db /
 * measurement_slots / track_measurements；
 * 写入 scratch.output_state / scratch.track_measurements（filtered_feature）。
 *
 * @param input          本周期输入目标列表。
 * @param track_filter   轨迹滤波器引用。
 * @param scratch        周期暂存区。
 */
void ApplyTrackFilterPass(const session::ArSceneTargetList& input,
                          tracking::TrackFilter& track_filter, CycleExecutionScratch& scratch);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_
