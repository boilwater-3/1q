/**
 * @file DecisionInputFrame.h
 * @brief 定义供决策引擎消费的单周期输入帧。
 */

#ifndef AIRBORNE_RADAR_COMMON_DECISION_INPUT_FRAME_H_
#define AIRBORNE_RADAR_COMMON_DECISION_INPUT_FRAME_H_

#include <cstddef>
#include <cstdint>

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/JammingSemantics.h"

namespace airborne_radar {
namespace common {

/**
 * @brief AssociationQualityInfo 表示供决策层消费的关联质量摘要。
 */
struct AssociationQualityInfo {
  float match_rate{0.0f};        /**< 当前周期命中率 */
  float new_track_rate{0.0f};    /**< 当前周期新生率 */
  float missed_track_rate{0.0f}; /**< 当前周期漏失率 */
  float mean_match_cost{0.0f};   /**< 当前周期命中关联代价均值 */
  float p95_match_cost{0.0f};    /**< 当前周期命中关联代价 P95 */
  JammingSemantic dominant_jamming_semantic{
      JammingSemantic::kNone};    /**< 当前周期关联压力对应的主导干扰摘要类型 */
  float jamming_severity{0.0f};   /**< 当前周期关联压力对应的残余干扰强度摘要，范围 [0, 1] */
  float association_stress{0.0f}; /**< 当前周期关联压力摘要，范围 [0, 1] */
};

/**
 * @brief PerceptionQualityInfo 表示供决策层消费的探测质量摘要。
 */
struct PerceptionQualityInfo {
  std::size_t input_target_count{0}; /**< 当前周期输入目标数 */
  std::size_t detection_count{0};    /**< 当前周期进入关联阶段的探测量测数 */
  float detection_rate{0.0f};        /**< 当前周期探测成功率，范围 [0, 1] */
  float detection_stress{
      0.0f}; /**< 当前周期探测压力摘要，范围 [0, 1]，值越大表示探测阶段丢失越明显 */
};

/**
 * @brief DecisionInputFrame 表示单周期决策输入帧。
 */
struct DecisionInputFrame {
  std::uint32_t cycle_index{0};                      /**< 当前周期号 */
  std::uint64_t batch_id{0};                         /**< 当前批号 */
  bool environment_jamming_detected{false};          /**< 环境是否检测到干扰 */
  EccmSourceInfo eccm_source_info{};                 /**< 供 ECCM 消费的干扰事实摘要 */
  AssociationQualityInfo association_quality_info{}; /**< 供决策层消费的关联质量摘要 */
  PerceptionQualityInfo perception_quality_info{};   /**< 供决策层消费的探测质量摘要 */
  DecisionTrackSnapshotList tracks{};                /**< 当前周期可见的轨迹快照 */

  DecisionInputFrame() = default; /**< 默认构造 */

  /** @brief 使用轨迹集合构造输入帧。
   * @param[in] track_snapshots 轨迹快照列表。
   */
  explicit DecisionInputFrame(const DecisionTrackSnapshotList& track_snapshots)
      : tracks(track_snapshots) {}
};

}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_DECISION_INPUT_FRAME_H_
