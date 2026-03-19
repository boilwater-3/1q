// Copyright 2026. All Rights Reserved.
//
// Description: 定义威胁评估 evaluator，与旧分类节点共享核心逻辑。

#ifndef AIRBORNE_RADAR_DECISION_CLASSIFIER_THREAT_ASSESSMENT_EVALUATOR_H_
#define AIRBORNE_RADAR_DECISION_CLASSIFIER_THREAT_ASSESSMENT_EVALUATOR_H_

#include <string>

#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/TargetCategory.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/database/IFeatureRepository.h"

namespace airborne_radar {
namespace decision {
namespace classifier {

/// @brief ThreatAssessmentEvaluator 实现目标分类与威胁记忆更新。
class ThreatAssessmentEvaluator final : public pipeline::ITacticalEvaluator {
 public:
  /// @brief 构造函数，可选注入特征仓储。
  explicit ThreatAssessmentEvaluator(
      const environment::database::IFeatureRepository* feature_repository =
          nullptr);

  /// @brief 评估单周期输入并更新分类与威胁状态。
  void Evaluate(const common::DecisionInputFrame& input_frame,
                pipeline::TacticalStateStore& state_store,
                pipeline::TacticalEvaluationState& evaluation_state) const override;

 private:
  /// @brief 识别目标类型。
  std::string IdentifyTarget(
      const common::DecisionTrackSnapshot& track_snapshot) const;

  /// @brief 计算威胁评分。
  float ComputeThreatScore(
      const common::DecisionTrackSnapshot& track_snapshot) const;

  /// @brief 更新供 LPI 使用的来源信息。
  void UpdateLpiSourceInfo(common::LpiSourceInfo* source_info,
                           const std::string& classification) const;

  /// @brief 判断仓储匹配结果是否足够可靠。
  bool ShouldAcceptRepositoryMatch(
      const environment::database::MatchResult& match_result) const;

  /// @brief 更新跨周期置信度。
  float UpdateConfidence(const common::DecisionTrackSnapshot& track_snapshot,
                         float previous_confidence) const;

  /// @brief 判断分类标签是否属于高威胁类型。
  bool IsHighThreatCategory(const std::string& category) const;

  const environment::database::IFeatureRepository* feature_repository_;
};

} // namespace classifier
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_CLASSIFIER_THREAT_ASSESSMENT_EVALUATOR_H_
