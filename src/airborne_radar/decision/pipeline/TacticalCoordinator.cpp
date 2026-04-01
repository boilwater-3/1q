#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

namespace {

// ---------- ECCM 反向触发阈值 ----------
constexpr float kAssociationDrivenEccmMinJammingSeverity = 0.35f;   /**< 触发 ECCM 的最低干扰强度 */
constexpr float kAssociationDrivenEccmMinAssociationStress = 0.18f; /**< 触发 ECCM 的最低关联压力 */

// ---------- 探测压力判定阈值 ----------
constexpr float kMeaningfulDetectionPressureThreshold = 0.35f; /**< 可观测探测压力最低值 */

/**
 * @brief 判断关联压力语义是否指向 ECCM 驱动型干扰。
 * @param semantic 当前周期主导干扰语义。
 * @return 属于 deception/repeater/mixed 时返回 true。
 */
bool IsAssociationDrivenJammingSemantic(common::utils::JammingSemantic semantic) {
  return semantic == common::utils::JammingSemantic::kDeception ||
         semantic == common::utils::JammingSemantic::kRepeater ||
         semantic == common::utils::JammingSemantic::kMixed;
}
/**
 * @brief 判断关联质量摘要是否足以反向触发 ECCM。
 * @param association_quality_info 当前周期关联质量摘要。
 * @return 关联压力足够显著时返回 true。
 */
bool HasAssociationDrivenEccmEvidence(
    const common::model::AssociationQualityInfo& association_quality_info) {
  if (!IsAssociationDrivenJammingSemantic(association_quality_info.dominant_jamming_semantic) ||
      association_quality_info.jamming_severity < kAssociationDrivenEccmMinJammingSeverity ||
      association_quality_info.association_stress < kAssociationDrivenEccmMinAssociationStress) {
    return false;
  }
  return true;
}
/**
 * @brief 判断探测质量是否存在可观测压力。
 * @param perception_quality_info 当前周期探测质量摘要。
 * @return 探测压力达到阈值时返回 true。
 */
bool HasMeaningfulDetectionPressure(
    const common::model::PerceptionQualityInfo& perception_quality_info) {
  return perception_quality_info.input_target_count > 0U &&
         perception_quality_info.detection_stress >= kMeaningfulDetectionPressureThreshold;
}
/**
 * @brief 构造单周期决策摘要字符串。
 * @param input_frame 当前周期决策输入帧。
 * @param evaluation_state 本周期 evaluator 聚合状态。
 * @return 用于跨周期记忆与日志的决策摘要。
 */
std::string BuildDecisionSummary(const common::model::DecisionInputFrame& input_frame,
                                 const TacticalEvaluationState& evaluation_state) {
  std::vector<std::string> causes;
  if (input_frame.environment_jamming_detected || input_frame.eccm_source_info.has_jamming_signal) {
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
/**
 * @brief 用关联质量压力为 ECCM 输入补齐隐式触发信号。
 * @param association_quality_info 当前周期关联质量摘要。
 * @param eccm_source_info 待回填的 ECCM 输入摘要。
 */
void BackfillAssociationDrivenEccmTrigger(
    const common::model::AssociationQualityInfo& association_quality_info,
    common::model::EccmSourceInfo* eccm_source_info) {
  if (eccm_source_info == nullptr || !HasAssociationDrivenEccmEvidence(association_quality_info)) {
    return;
  }

  eccm_source_info->has_jamming_signal = true;
}

void PruneInactiveTrackState(const common::model::DecisionTrackSnapshotList& tracks,
                             TacticalStateStore* state_store) {
  if (state_store == nullptr) {
    return;
  }

  std::unordered_set<std::uint64_t> active_keys;
  active_keys.reserve(tracks.size());
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    active_keys.insert(tracks[i].state.association_key);
  }

  constexpr std::size_t kMaxStateStoreEntries = 2048U;
  if (state_store->confidence_memory.size() > kMaxStateStoreEntries) {
    PROJECT_LOG_WARN(
        "[TacticalCoordinator] TacticalStateStore confidence_memory size ({}) exceeds limit ({}). "
        "This may indicate track IDs are not being properly recycled.",
        state_store->confidence_memory.size(), kMaxStateStoreEntries);
  }

  for (std::unordered_map<std::uint64_t, float>::iterator it =
           state_store->confidence_memory.begin();
       it != state_store->confidence_memory.end();) {
    if (active_keys.count(it->first) == 0U) {
      it = state_store->confidence_memory.erase(it);
      continue;
    }
    ++it;
  }

  for (std::unordered_map<std::uint64_t, float>::iterator it = state_store->threat_memory.begin();
       it != state_store->threat_memory.end();) {
    if (active_keys.count(it->first) == 0U) {
      it = state_store->threat_memory.erase(it);
      continue;
    }
    ++it;
  }
}

}  // namespace

TacticalCoordinator::TacticalCoordinator(
    const environment::database::IFeatureRepository* feature_repository)
    : threat_assessment_evaluator_(feature_repository),
      emission_control_evaluator_(),
      survivability_evaluator_() {}

TacticalDecisionResult TacticalCoordinator::Evaluate(
    const common::model::DecisionInputFrame& input_frame, TacticalStateStore& state_store) {
  TacticalEvaluationState evaluation_state;
  evaluation_state.eccm_source_info = input_frame.eccm_source_info;
  if (!evaluation_state.eccm_source_info.has_jamming_signal) {
    evaluation_state.eccm_source_info.has_jamming_signal = input_frame.environment_jamming_detected;
  }
  BackfillAssociationDrivenEccmTrigger(input_frame.association_quality_info,
                                       &evaluation_state.eccm_source_info);

  // Evaluator 执行顺序具有数据依赖，不可随意调整：
  //   [1] ThreatAssessmentEvaluator：写入 target_classification_result /
  //       lpi_source_info / eccm_source_info.has_jamming_signal（基于 track）。
  //   [2] EmissionControlEvaluator：读取 lpi_source_info（[1] 写入）。
  //   [3] SurvivabilityEvaluator：读取 eccm_source_info（Pre-fill + [1] 写入）。
  // 详见 TacticalEvaluationState 注释中的字段所有权说明。
  threat_assessment_evaluator_.Evaluate(input_frame, state_store, evaluation_state);
  emission_control_evaluator_.Evaluate(input_frame, state_store, evaluation_state);
  survivability_evaluator_.Evaluate(input_frame, state_store, evaluation_state);
  PruneInactiveTrackState(input_frame.tracks, &state_store);

  TacticalDecisionResult result;
  result.target_classification_result.reserve(evaluation_state.target_classification_result.size());
  for (std::size_t i = 0; i < evaluation_state.target_classification_result.size(); ++i) {
    result.target_classification_result.push_back(common::model::TargetCategory(
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
  state_store.last_classification_labels.reserve(result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary = BuildDecisionSummary(input_frame, evaluation_state);

  PROJECT_LOG_DEBUG(
      "[TacticalCoordinator] cycle_index={} tracks={} mode={} proposals={} assoc_stress={:.3f}",
      input_frame.cycle_index, input_frame.tracks.size(), static_cast<int>(result.selected_mode),
      result.proposals.size(), input_frame.association_quality_info.association_stress);
  return result;
}

}  // namespace pipeline
}  // namespace decision
}  // namespace airborne_radar
