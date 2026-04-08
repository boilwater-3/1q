/**
 * @file SignalPipelineResultTypes.h
 * @brief 定义信号流水线对外输出结果类型。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RESULT_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RESULT_TYPES_H_

#include <cstddef>

#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/model/JammingSemantics.h"

namespace airborne_radar {
namespace extension {

/**
 * @brief 关联质量观测指标（Pipeline 对外公开版本）。
 */
struct AssociationQualityMetrics {
  std::size_t prior_track_count{0};  /**< 进入关联阶段的历史先验轨迹数 */
  std::size_t detection_count{0};    /**< 本周期探测成功并参与关联的量测数 */
  std::size_t matched_count{0};      /**< 命中已有轨迹的关联数 */
  std::size_t new_track_count{0};    /**< 触发新建轨迹键的量测数 */
  std::size_t missed_track_count{0}; /**< 未命中任何量测的历史轨迹数 */
  float match_rate{0.0f};            /**< 命中率（matched_count / detection_count） */
  float new_track_rate{0.0f};        /**< 新生率（new_track_count / detection_count） */
  float missed_track_rate{0.0f};     /**< 漏失率（missed_track_count / prior_track_count） */
  float mean_match_cost{0.0f};       /**< 命中关联代价均值（仅统计 matches） */
  float p95_match_cost{0.0f};        /**< 命中关联代价 P95（仅统计 matches） */
  model::JammingSemantic dominant_jamming_semantic{
      model::JammingSemantic::kNone}; /**< 当前周期关联质量对应的主导干扰摘要类型 */
  float jamming_severity{0.0f};   /**< 当前周期关联质量对应的残余干扰强度摘要，范围 [0, 1] */
  float association_stress{0.0f}; /**< 当前周期的归一化关联压力，范围 [0, 1] */
};

/**
 * @brief SignalCycleResult 描述信号流水线单周期的稳定输出。
 */
struct SignalCycleResult {
  model::TargetFeatureList updated_features{};     /**< 当前周期更新后的目标特征列表 */
  model::DecisionInputFrame decision_frame{};      /**< 当前周期决策输入帧 */
  AssociationQualityMetrics association_quality_metrics{}; /**< 当前周期关联质量观测指标 */
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_RESULT_TYPES_H_
