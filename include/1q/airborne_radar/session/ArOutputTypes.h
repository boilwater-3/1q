/**
 * @file ArOutputTypes.h
 * @brief 机载雷达输出辅助类型集合。
 *
 * 输出辅助类型（SignalCycleResult、关联质量指标等）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_OUTPUT_TYPES_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_OUTPUT_TYPES_H_

#include <cstddef>

#include "1q/airborne_radar/config/JammingSemantics.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief SignalCycleAbortReason 描述信号流水线单周期终止原因。
 */
enum class ONEQ_API SignalCycleAbortReason {
  kNone = 0,
  kLifecycleUnavailable = 1,
  kInvalidEnvironmentCycle = 2,
  kRuntimePreparationFailed = 3,
  kValidationRejected = 4,
};

/**
 * @brief AssociationQualityMetrics 表示关联质量观测指标（对外公开版本）。
 * @note 字段按量纲分为 counts、rates、costs 与 normalized summaries。聚合统计时，
 *       counts 适合求和，rates/costs/summaries 通常应按有效周期求均值或分位数。
 */
struct ONEQ_API AssociationQualityMetrics {
  std::size_t prior_track_count{0};  /**< count: 进入关联阶段的历史先验轨迹数 */
  std::size_t detection_count{0};    /**< count: 本周期探测成功并参与关联的量测数 */
  std::size_t matched_count{0};      /**< count: 命中已有轨迹的关联数 */
  std::size_t new_track_count{0};    /**< count: 触发新建轨迹键的量测数 */
  std::size_t missed_track_count{0}; /**< count: 未命中任何量测的历史轨迹数 */
  float match_rate{0.0f};            /**< rate [0,1]: matched_count / detection_count */
  float new_track_rate{0.0f};        /**< rate [0,1]: new_track_count / detection_count */
  float missed_track_rate{0.0f};     /**< rate [0,1]: missed_track_count / prior_track_count */
  float mean_match_cost{0.0f};       /**< cost: 命中关联代价均值（仅统计 matches） */
  float p95_match_cost{0.0f};        /**< cost: 命中关联代价 P95（仅统计 matches） */
  config::JammingSemantic dominant_jamming_semantic{
      config::JammingSemantic::kNone}; /**< 当前周期关联质量对应的主导干扰摘要类型 */
  float jamming_severity{0.0f};        /**< summary [0,1]: 当前周期关联质量对应的残余干扰强度 */
  float association_stress{0.0f};      /**< summary [0,1]: 当前周期的归一化关联压力 */
};

/**
 * @brief SignalCycleResult 描述信号流水线单周期的稳定输出。
 * @note 当 `executed_this_cycle=false` 时，输出与指标字段保持默认值，不代表真实零值；
 *       统计消费方必须先检查 `executed_this_cycle`。
 */
struct ONEQ_API SignalCycleResult {
  bool executed_this_cycle{false}; /**< 当前调用是否真正完成了 signal pipeline 主链路 */
  SignalCycleAbortReason abort_reason{
      SignalCycleAbortReason::kNone};           /**< 若当前调用未执行成功，给出结构化 abort 原因 */
  ArSceneTargetList updated_scene_targets{};    /**< 当前周期更新后的场景目标列表 */
  session::DecisionInputFrame decision_frame{}; /**< 当前周期决策输入帧 */
  AssociationQualityMetrics association_quality_metrics{}; /**< 当前周期关联质量观测指标 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_OUTPUT_TYPES_H_
