/**
 * @file ThreatAssessmentEvaluator.h
 * @brief 定义威胁评估 evaluator 类。
 */

#ifndef AIRBORNE_RADAR_DECISION_THREAT_ASSESSMENT_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_THREAT_ASSESSMENT_EVALUATOR_H_

#include <cmath>
#include <string>

#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "airborne_radar/decision/LpiSourceInfo.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/TargetCategory.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/environment/IFeatureRepository.h"

namespace airborne_radar {
namespace decision {

/**
 * @brief 实现目标分类与威胁记忆更新，输出供 LPI 等模块消费的威胁来源信息。
 */
class ThreatAssessmentEvaluator final {
 public:
  /**
   * @brief 构造函数，可选注入特征仓储。
   * @param feature_repository 用于目标特征匹配的仓储接口；可为空。
   */
  explicit ThreatAssessmentEvaluator(
      const environment::IFeatureRepository* feature_repository = nullptr);

  /**
   * @brief 单周期评估结果。
   */
  struct Result {
    session::TargetCategoryList target_classification_result; /**< 目标分类结果 */
    model::LpiSourceInfo lpi_source_info;                   /**< 供 LPI 模块消费的来源信息 */
  };

  /**
   * @brief 评估单周期输入并更新分类与威胁状态。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期决策状态存储。
   * @return 本周期威胁评估结果。
   */
  Result Evaluate(const session::DecisionInputFrame& input_frame,
                  session::TacticalStateStore& state_store) const;

 private:
  /**
   * @brief 识别目标类型，并填充匹配概率。
   * @param track_snapshot 单条轨迹快照。
   * @return 含类型标签及归一化概率的 TargetCategory；启发式路径时 probability 为 0。
   */
  session::TargetCategory IdentifyTarget(
      const session::TrackStateSnapshot& track_snapshot) const;

  /**
   * @brief 计算威胁评分。
   * @param track_snapshot 单条轨迹快照。
   * @return 轨迹的威胁评分。
   */
  float ComputeThreatScore(const session::TrackStateSnapshot& track_snapshot) const;

  /**
   * @brief 更新供 LPI 使用的来源信息。
   * @param[in,out] source_info 待更新的 LPI 来源信息。
   * @param track_snapshot 当前轨迹快照。
   * @param classification 当前识别出的分类标签。
   */
  void UpdateLpiSourceInfo(model::LpiSourceInfo* source_info,
                           const session::TrackStateSnapshot& track_snapshot,
                           const std::string& classification) const;

  /**
   * @brief 判断仓储匹配结果是否足够可靠。
   * @param match_result 仓储匹配结果。
   * @return 结果足够可靠时返回 `true`。
   */
  bool ShouldAcceptRepositoryMatch(const environment::MatchResult& match_result) const;

  /**
   * @brief 更新跨周期置信度。
   * @param track_snapshot 当前轨迹快照。
   * @param previous_confidence 上一周期保留的置信度。
   * @return 更新后的置信度。
   */
  float UpdateConfidence(const session::TrackStateSnapshot& track_snapshot,
                         float previous_confidence) const;

  /**
   * @brief 判断分类标签是否属于高威胁类型。
   * @param category 分类标签。
   * @return 属于高威胁类型时返回 `true`。
   */
  bool IsHighThreatCategory(const std::string& category) const;

  const environment::IFeatureRepository* feature_repository_;
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_THREAT_ASSESSMENT_EVALUATOR_H_
