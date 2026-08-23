/**
 * @file SignalCycleResult.h
 * @brief 定义信号流水线单周期稳定输出（内部 DTO，随 DecisionInputFrame 下沉私有）。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_CYCLE_RESULT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_CYCLE_RESULT_H_

#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "airborne_radar/decision/DecisionInputFrame.h"

namespace airborne_radar {
namespace session {

/**
 * @brief SignalCycleResult 描述信号流水线单周期的稳定输出。
 * @note 当 `executed_this_cycle=false` 时，输出与指标字段保持默认值，不代表真实零值；
 *       统计消费方必须先检查 `executed_this_cycle`。
 * @note 规则 15f：本结构内嵌决策输入帧，属内部管线产物，不出 public 头。
 */
struct SignalCycleResult {
  bool executed_this_cycle{false}; /**< 当前调用是否真正完成了 signal pipeline 主链路 */
  SignalCycleAbortReason abort_reason{
      SignalCycleAbortReason::kNone};           /**< 若当前调用未执行成功，给出结构化 abort 原因 */
  ArSceneTargetList updated_scene_targets{};    /**< 当前周期更新后的场景目标列表 */
  DecisionInputFrame decision_frame{}; /**< 当前周期决策输入帧 */
  AssociationQualityMetrics association_quality_metrics{}; /**< 当前周期关联质量观测指标 */
  ArIssueList issues{}; /**< 统一问题列表（规则 14：正常周期按目标排除的 kInfo 诊断，
                             phase=kExecution；abort 路径诊断由 RecordAbort 写入）。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_CYCLE_RESULT_H_
