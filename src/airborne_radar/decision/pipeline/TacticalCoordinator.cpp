// Copyright 2026. All Rights Reserved.
//
// @file TacticalCoordinator.cpp
// @brief 实现 TacticalCoordinator 的默认决策协调逻辑。

#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"

#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace pipeline {

namespace {

/// @brief 判断关联压力语义是否指向 ECCM 驱动型干扰。
/// @param semantic 当前周期主导干扰语义。
/// @return 属于 deception/repeater/mixed 时返回 true。
bool IsAssociationDrivenJammingSemantic(common::JammingSemantic semantic) {
  return semantic == common::JammingSemantic::kDeception ||
         semantic == common::JammingSemantic::kRepeater ||
         semantic == common::JammingSemantic::kMixed;
}

/// @brief 判断关联质量摘要是否足以反向触发 ECCM。
/// @param association_quality_info 当前周期关联质量摘要。
/// @return 关联压力足够显著时返回 true。
bool HasAssociationDrivenEccmEvidence(
    const common::AssociationQualityInfo& association_quality_info) {
  if (!IsAssociationDrivenJammingSemantic(
          association_quality_info.dominant_jamming_semantic) ||
      association_quality_info.jamming_severity < 0.35f ||
      association_quality_info.association_stress < 0.18f) {
    return false;
  }
  return true;
}

/// @brief 判断探测质量是否存在可观测压力。
/// @param perception_quality_info 当前周期探测质量摘要。
/// @return 探测压力达到阈值时返回 true。
bool HasMeaningfulDetectionPressure(
    const common::PerceptionQualityInfo& perception_quality_info) {
  return perception_quality_info.input_target_count > 0U &&
         perception_quality_info.detection_stress >= 0.35f;
}

/// @brief 构造单周期决策摘要字符串。
/// @param input_frame 当前周期决策输入帧。
/// @param evaluation_state 本周期 evaluator 聚合状态。
/// @return 用于跨周期记忆与日志的决策摘要。
std::string BuildDecisionSummary(const common::DecisionInputFrame& input_frame,
                                 const TacticalEvaluationState& evaluation_state) {
  std::vector<std::string> causes;
  if (input_frame.environment_jamming_detected ||
      input_frame.eccm_source_info.has_jamming_signal) {
    causes.push_back("environment-jamming");
  }
  if (HasAssociationDrivenEccmEvidence(input_frame.association_quality_info)) {
    causes.push_back("association-pressure");
  }
  if (HasMeaningfulDetectionPressure(input_frame.perception_quality_info)) {
    causes.push_back("detection-pressure");
  }

  std::string summary;
  if (evaluation_state.should_enable_eccm) {
    summary = "protected-emission";
  } else if (evaluation_state.should_reduce_power) {
    summary = "threat-response";
  } else {
    summary = "baseline";
  }

  if (causes.empty()) {
    return summary;
  }

  summary += "(";
  for (std::size_t i = 0; i < causes.size(); ++i) {
    if (i != 0U) {
      summary += "+";
    }
    summary += causes[i];
  }
  summary += ")";
  return summary;
}

/// @brief 用关联质量压力为 ECCM 输入补齐隐式触发信号。
/// @param association_quality_info 当前周期关联质量摘要。
/// @param eccm_source_info 待回填的 ECCM 输入摘要。
void BackfillAssociationDrivenEccmTrigger(
    const common::AssociationQualityInfo& association_quality_info,
    common::EccmSourceInfo* eccm_source_info) {
  if (eccm_source_info == nullptr ||
      !HasAssociationDrivenEccmEvidence(association_quality_info)) {
    return;
  }

  eccm_source_info->has_jamming_signal = true;
}

}  // namespace

TacticalCoordinator::TacticalCoordinator(
    const environment::database::IFeatureRepository* feature_repository)
    : threat_assessment_evaluator_(feature_repository),
      emission_control_evaluator_(),
      survivability_evaluator_() {}

TacticalDecisionResult TacticalCoordinator::Evaluate(
    const common::DecisionInputFrame& input_frame,
    TacticalStateStore& state_store) {
  TacticalEvaluationState evaluation_state;
  evaluation_state.eccm_source_info = input_frame.eccm_source_info;
  if (!evaluation_state.eccm_source_info.has_jamming_signal) {
    evaluation_state.eccm_source_info.has_jamming_signal =
        input_frame.environment_jamming_detected;
  }
  BackfillAssociationDrivenEccmTrigger(input_frame.association_quality_info,
                                       &evaluation_state.eccm_source_info);

  threat_assessment_evaluator_.Evaluate(input_frame, state_store,
                                        evaluation_state);
  emission_control_evaluator_.Evaluate(input_frame, state_store,
                                       evaluation_state);
  survivability_evaluator_.Evaluate(input_frame, state_store,
                                    evaluation_state);

  TacticalDecisionResult result;
  result.target_classification_result.reserve(
      evaluation_state.target_classification_result.size());
  for (std::size_t i = 0;
       i < evaluation_state.target_classification_result.size(); ++i) {
    result.target_classification_result.push_back(common::TargetCategory(
        evaluation_state.target_classification_result[i].target_type));
  }
  result.proposals = evaluation_state.proposals;
  if (evaluation_state.should_enable_eccm) {
    result.selected_mode = TacticalMode::kProtectedEmission;
  } else if (evaluation_state.should_reduce_power) {
    result.selected_mode = TacticalMode::kThreatResponse;
  } else {
    result.selected_mode = TacticalMode::kBaseline;
  }

  state_store.current_mode = result.selected_mode;
  state_store.last_classification_labels.clear();
  state_store.last_classification_labels.reserve(
      result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary =
      BuildDecisionSummary(input_frame, evaluation_state);

  spdlog::debug(
      "[TacticalCoordinator] cycle_index={} tracks={} mode={} proposals={} assoc_stress={:.3f}",
      input_frame.cycle_index, input_frame.tracks.size(),
      static_cast<int>(result.selected_mode), result.proposals.size(),
      input_frame.association_quality_info.association_stress);
  return result;
}

} // namespace pipeline
} // namespace decision
} // namespace airborne_radar
