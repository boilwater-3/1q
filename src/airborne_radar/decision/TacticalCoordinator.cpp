#include "airborne_radar/decision/TacticalCoordinator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {

namespace {

// ===== ECCM 反向触发阈值 =====
constexpr float kAssociationDrivenEccmMinJammingSeverity = 0.35f;
constexpr float kAssociationDrivenEccmMinAssociationStress = 0.18f;

// ===== 关联偏置阈值 =====
constexpr float kMinimumAssociationSeverity = 0.30f;
constexpr float kMinimumAssociationStress = 0.18f;

// ===== 探测压力判定阈值 =====
constexpr float kMeaningfulDetectionPressureThreshold = 0.35f;

// ===== 状态存储容量预警阈值 =====
constexpr std::size_t kMaxStateStoreEntries = 2048U;

/**
 * @brief 判断关联压力语义是否指向 ECCM 驱动型干扰。
 */
bool IsAssociationDrivenJammingSemantic(model::JammingSemantic semantic) {
  return semantic == model::JammingSemantic::kDeception ||
         semantic == model::JammingSemantic::kRepeater ||
         semantic == model::JammingSemantic::kMixed;
}

/**
 * @brief 判断探测质量是否存在可观测压力。
 */
bool HasMeaningfulDetectionPressure(
    const model::PerceptionQualityInfo& perception_quality_info) {
  return perception_quality_info.input_target_count > 0U &&
         perception_quality_info.detection_stress >= kMeaningfulDetectionPressureThreshold;
}

/**
 * @brief 将浮点值裁剪到 [0, 1]。
 */
float ClampUnit(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

/**
 * @brief 描述关联语义。
 */
std::string DescribeAssociationSemantic(model::JammingSemantic semantic) {
  switch (semantic) {
    case model::JammingSemantic::kDeception:
      return "deception-like association stress";
    case model::JammingSemantic::kRepeater:
      return "repeater-like association stress";
    case model::JammingSemantic::kMixed:
      return "mixed association stress";
    case model::JammingSemantic::kNoiseSuppression:
      return "noise-like association stress";
    case model::JammingSemantic::kNone:
    default:
      return "association stress";
  }
}

}  // namespace

// ===== 构造函数 =====

TacticalCoordinator::TacticalCoordinator(
    const environment::IFeatureRepository* feature_repository,
    extension::IOverrideControlStrategy* override_strategy)
    : threat_assessment_evaluator_(feature_repository),
      override_strategy_(override_strategy) {}

// ===== 私有静态方法 =====

bool TacticalCoordinator::ShouldBackfillEccmTrigger(
    const model::AssociationQualityInfo& association_quality_info) {
  if (!IsAssociationDrivenJammingSemantic(
           association_quality_info.dominant_jamming_semantic) ||
      association_quality_info.jamming_severity < kAssociationDrivenEccmMinJammingSeverity ||
      association_quality_info.association_stress < kAssociationDrivenEccmMinAssociationStress) {
    return false;
  }
  return true;
}

void TacticalCoordinator::ApplyAssociationDrivenPriorityBias(
    const model::AssociationQualityInfo& association_quality_info,
    std::vector<extension::TacticalProposal>* proposals) {
  if (proposals == nullptr || proposals->empty()) {
    return;
  }

  if (association_quality_info.association_stress < kMinimumAssociationStress ||
      association_quality_info.jamming_severity < kMinimumAssociationSeverity) {
    return;
  }

  const float severity = ClampUnit(association_quality_info.jamming_severity);
  const float stress = ClampUnit(association_quality_info.association_stress);
  const float cost_pressure = ClampUnit(
      std::max(association_quality_info.mean_match_cost / 3.0f,
               association_quality_info.p95_match_cost / 4.0f));
  const float combined_weight = 0.45f * severity + 0.40f * stress + 0.15f * cost_pressure;

  // 按语义类型计算各措施优先级增加值
  int agility_boost = 0;
  int rejitter_boost = 0;
  int beam_boost = 0;
  std::string semantic_desc;

  switch (association_quality_info.dominant_jamming_semantic) {
    case model::JammingSemantic::kDeception:
      agility_boost = static_cast<int>(3.5f * combined_weight * 10.0f);
      rejitter_boost = static_cast<int>(3.8f * combined_weight * 10.0f);
      beam_boost = static_cast<int>(0.8f * combined_weight * 10.0f);
      semantic_desc = DescribeAssociationSemantic(model::JammingSemantic::kDeception);
      break;
    case model::JammingSemantic::kRepeater:
      agility_boost = static_cast<int>(1.8f * combined_weight * 10.0f);
      rejitter_boost = static_cast<int>(3.6f * combined_weight * 10.0f);
      beam_boost = static_cast<int>(0.9f * combined_weight * 10.0f);
      semantic_desc = DescribeAssociationSemantic(model::JammingSemantic::kRepeater);
      break;
    case model::JammingSemantic::kMixed:
      agility_boost = static_cast<int>(2.8f * combined_weight * 10.0f);
      rejitter_boost = static_cast<int>(3.0f * combined_weight * 10.0f);
      beam_boost = static_cast<int>(0.8f * combined_weight * 10.0f);
      semantic_desc = DescribeAssociationSemantic(model::JammingSemantic::kMixed);
      break;
    case model::JammingSemantic::kNoiseSuppression:
    case model::JammingSemantic::kNone:
    default:
      return;
  }

  for (std::size_t i = 0; i < proposals->size(); ++i) {
    extension::TacticalProposal& p = (*proposals)[i];
    bool boosted = false;
    switch (p.directive.type) {
      case extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
        if (agility_boost > 0) {
          p.priority += agility_boost;
          boosted = true;
        }
        break;
      case extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER:
        if (rejitter_boost > 0) {
          p.priority += rejitter_boost;
          boosted = true;
        }
        break;
      case extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
        if (beam_boost > 0) {
          p.priority += beam_boost;
          boosted = true;
        }
        break;
      default:
        break;
    }
    if (boosted && !p.rationale.empty()) {
      p.rationale += "; ";
      p.rationale += semantic_desc;
      p.rationale += " raises priority";
    }
  }
}

void TacticalCoordinator::PruneInactiveTrackState(
    const model::TrackStateSnapshotList& tracks,
    extension::TacticalStateStore* state_store) {
  if (state_store == nullptr) {
    return;
  }

  std::unordered_set<std::uint64_t> active_keys;
  active_keys.reserve(tracks.size());
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    active_keys.insert(tracks[i].association_key);
  }

  if (state_store->confidence_memory.size() > kMaxStateStoreEntries) {
    PROJECT_LOG_WARN(
        "[TacticalCoordinator] TacticalStateStore confidence_memory size ({}) exceeds limit ({}). "
        "This may indicate track IDs are not being properly recycled.",
        state_store->confidence_memory.size(), kMaxStateStoreEntries);
  }

  for (auto it = state_store->confidence_memory.begin();
       it != state_store->confidence_memory.end();) {
    if (active_keys.count(it->first) == 0U) {
      it = state_store->confidence_memory.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = state_store->threat_memory.begin();
       it != state_store->threat_memory.end();) {
    if (active_keys.count(it->first) == 0U) {
      it = state_store->threat_memory.erase(it);
    } else {
      ++it;
    }
  }
}

std::string TacticalCoordinator::BuildDecisionSummary(
    const model::DecisionInputFrame& input_frame,
    const ThreatAssessmentEvaluator::Result& threat_result,
    const LpiEvaluator::Result& lpi_result,
    const EccmEvaluator::Result& eccm_result) {
  (void)threat_result;
  std::vector<std::string> causes;
  if (input_frame.environment_jamming_detected ||
      input_frame.eccm_source_info.has_jamming_signal) {
    causes.push_back("environment-jamming");
  }
  if (ShouldBackfillEccmTrigger(input_frame.association_quality_info)) {
    causes.push_back("association-pressure");
  }
  if (HasMeaningfulDetectionPressure(input_frame.perception_quality_info)) {
    causes.push_back("detection-pressure");
  }

  std::string summary;
  if (eccm_result.eccm_activated) {
    summary = "protected-emission";
  } else if (lpi_result.requests_power_reduction) {
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

// ===== Evaluate（主入口）=====

extension::TacticalDecisionResult TacticalCoordinator::Evaluate(
    const model::DecisionInputFrame& input_frame,
    extension::TacticalStateStore& state_store) {
  extension::TacticalDecisionResult result;

  // ==== [1] 确定 ECCM 触发信号 ====
  // 来源 A：环境干扰事实
  bool eccm_has_jamming = input_frame.eccm_source_info.has_jamming_signal;
  // 来源 B：关联质量安全网（环境未报干扰但关联质量异常时补填）
  if (!eccm_has_jamming && ShouldBackfillEccmTrigger(input_frame.association_quality_info)) {
    eccm_has_jamming = true;
  }

  // ==== [2] 威胁评估 ====
  const ThreatAssessmentEvaluator::Result threat_result =
      threat_assessment_evaluator_.Evaluate(input_frame, state_store);

  // ==== [3] LPI 发射控制 ====
  std::vector<extension::TacticalProposal> all_proposals;
  LpiEvaluator::Result lpi_result;
  if (override_strategy_ != nullptr &&
      override_strategy_->OverrideLpi(threat_result.lpi_source_info, &all_proposals)) {
    PROJECT_LOG_INFO("[TacticalCoordinator] LPI decision overridden by external strategy.");
  } else {
    lpi_result = lpi_evaluator_.Evaluate(threat_result.lpi_source_info, &all_proposals);
  }

  // ==== [4] ECCM 抗干扰 ====
  bool should_enable_eccm = eccm_has_jamming;
  EccmEvaluator::Result eccm_result;
  if (should_enable_eccm) {
    // 构建含安全网标记的 ECCM 输入
    model::EccmSourceInfo eccm_input = input_frame.eccm_source_info;
    if (!eccm_input.has_jamming_signal) {
      eccm_input.has_jamming_signal = true;
    }
    // 关联质量安全网回填：无实际干扰源时，根据关联语义创建合成干扰源，
    // 使 ECCM 评估器能生成类型特化的对抗提案（而非仅有自适应波束回退）。
    if (eccm_input.jammer_sources.empty() &&
        ShouldBackfillEccmTrigger(input_frame.association_quality_info)) {
      model::EccmJammerSourceInfo synthetic;
      switch (input_frame.association_quality_info.dominant_jamming_semantic) {
        case model::JammingSemantic::kDeception:
          synthetic.technique = model::JammingTechnique::kDeception;
          break;
        case model::JammingSemantic::kRepeater:
          synthetic.technique = model::JammingTechnique::kRepeater;
          break;
        case model::JammingSemantic::kMixed:
          synthetic.technique = model::JammingTechnique::kDeception;
          break;
        default:
          synthetic.technique = model::JammingTechnique::kUnknown;
      }
      synthetic.jammer_power_db = 8.0f;
      synthetic.jammer_to_signal_db = 6.0f;
      synthetic.frequency_overlap_ratio =
          input_frame.association_quality_info.jamming_severity;
      synthetic.prf_lock_risk =
          input_frame.association_quality_info.jamming_severity;
      synthetic.confidence =
          input_frame.association_quality_info.jamming_severity;
      synthetic.jammer_in_sidelobe = false;
      eccm_input.jammer_sources.push_back(synthetic);
    }
    if (override_strategy_ != nullptr &&
        override_strategy_->OverrideEccm(eccm_input, &all_proposals)) {
      PROJECT_LOG_INFO("[TacticalCoordinator] ECCM decision overridden by external strategy.");
      eccm_result.eccm_activated = true;
    } else {
      eccm_result = eccm_evaluator_.Evaluate(eccm_input, false, &all_proposals);
    }
    PROJECT_LOG_INFO(
        "[TacticalCoordinator] Active jamming detected. Appending ECCM proposals.");
  } else {
    PROJECT_LOG_INFO(
        "[TacticalCoordinator] Environment is clear. Continuing nominal operation.");
  }

  // ==== [5] 关联质量优先级偏置后处理 ====
  if (!all_proposals.empty()) {
    ApplyAssociationDrivenPriorityBias(input_frame.association_quality_info, &all_proposals);
  }

  // ==== [6] 状态清理 ====
  PruneInactiveTrackState(input_frame.tracks, &state_store);

  // ==== [7] 组装结果 ====
  result.target_classification_result.reserve(threat_result.target_classification_result.size());
  for (std::size_t i = 0; i < threat_result.target_classification_result.size(); ++i) {
    result.target_classification_result.push_back(
        model::TargetCategory(threat_result.target_classification_result[i].target_type));
  }
  result.proposals = all_proposals;

  if (eccm_result.eccm_activated) {
    result.selected_mode = extension::TacticalMode::kProtectedEmission;
  } else if (lpi_result.requests_power_reduction) {
    result.selected_mode = extension::TacticalMode::kThreatResponse;
  } else {
    result.selected_mode = extension::TacticalMode::kBaseline;
  }

  state_store.current_mode = result.selected_mode;
  state_store.last_classification_labels.clear();
  state_store.last_classification_labels.reserve(result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary =
      BuildDecisionSummary(input_frame, threat_result, lpi_result, eccm_result);

  PROJECT_LOG_DEBUG(
      "[TacticalCoordinator] cycle_index={} tracks={} mode={} proposals={} assoc_stress={:.3f}",
      input_frame.cycle_index, input_frame.tracks.size(), static_cast<int>(result.selected_mode),
      result.proposals.size(), input_frame.association_quality_info.association_stress);

  return result;
}

}  // namespace decision
}  // namespace airborne_radar
