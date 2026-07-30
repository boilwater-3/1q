/**
 * @file DeceptionMeasurementGenerator.h
 * @brief 从欺骗干扰观测合成假目标量测的内部 pass。
 *
 * 此前「假目标鉴别」实际处理的是真实场景目标：所有航迹量测仅来自 ArSceneTargetList，
 * 仅按方位把真实目标量测标成假目标并阻止 tentative 起批。本 pass 在量测构建阶段从
 * kLikelyFalseTarget 干扰观测合成假距离/多普勒量测注入 track_measurements，使鉴别真正
 * 作用于假目标而非真实目标。
 *
 * 合成量测绕过关联引擎（关联在量测构建前已执行），lifecycle 据其 association_key 作
 * 新航迹处理——这正是「假航迹」的来源。量测的 classified_as_false_target=true 已贯通到
 * PromoteState 抑制起批。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECEPTION_MEASUREMENT_GENERATOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECEPTION_MEASUREMENT_GENERATOR_H_

#include "airborne_radar/signal/pipeline/CycleExecutor.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 从 kLikelyFalseTarget 干扰观测合成假目标量测并追加到 scratch.track_measurements。
 *
 * 合成独立于反制开关：假目标攻击现象始终被注入。反制开关只在下游 PromoteState 控制
 * tentative→confirmed 的抑制策略。resolver 为每个疑似假目标簇生成一条内部簇元数据，
 * 本 pass 按该元数据的 emission_count 合成若干个假距离/多普勒量测：
 * - 位置：由局部系方位 + estimated_slant_range_m 合成笛卡尔局部坐标；
 * - 速度：estimated_range_rate_mps 沿视线方向投影；
 * - classified_as_false_target=true，source_index 取 sentinel（超出场景列表，由下游边界
 *   检查自然跳过），association_key 用 resolver 基于源设备集合生成的稳定种子派生。
 *
 * 合成量测不索引 per-target scratch 数组（target_geometry/measurement_covariances 等
 * 恰为 input.size()），位置与协方差内联进 raw_measurement。
 *
 * @param context 周期输入上下文（读取 interference_observations 与 deception_clusters）。
 * @param scratch 周期暂存区（追加合成量测到 track_measurements）。
 */
void InjectDeceptionMeasurementsPass(const CycleExecutionContext& context,
                                     CycleExecutionScratch& scratch);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECEPTION_MEASUREMENT_GENERATOR_H_
