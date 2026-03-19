// Copyright 2026. All Rights Reserved.
//
// @file SurvivabilityEvaluator.cpp
// @brief 实现 SurvivabilityEvaluator 的 ECCM 生存性评估逻辑。

#include "airborne_radar/decision/eccm/SurvivabilityEvaluator.h"

#include <algorithm>
#include <string>

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace eccm {

namespace {

const std::uint32_t kEccmHoldCycles = 2;
const float kHighFrequencyOverlapRatio = 0.5f;
const float kHighPrfLockRisk = 0.5f;
const float kHighJammerPowerDb = 8.0f;
const float kHighJammerToSignalDb = 6.0f;
const float kMinimumCredibleConfidence = 0.35f;
const float kMinimumAssociationSeverity = 0.30f;
const float kMinimumAssociationStress = 0.18f;

const int kBasePrioritySidelobeCanceller = 86;
const int kBasePriorityAdaptiveBeamforming = 80;
const int kBasePriorityAgilityFrequency = 84;
const int kBasePriorityEccmRejitter = 83;
const int kBasePriorityBurnthroughGain = 82;

const float kThresholdSidelobeCanceller = 1.5f;
const float kThresholdAdaptiveBeamforming = 0.8f;
const float kThresholdAgilityFrequency = 1.5f;
const float kThresholdEccmRejitter = 1.5f;
const float kThresholdBurnthroughGain = 1.5f;

/// @brief 构造一条生存性域控制意图。
/// @param type 控制意图类型。
/// @return 来源固定为 SURVIVABILITY 的控制意图。
common::ControlDirective BuildDirective(common::ControlDirectiveType type) {
  return common::ControlDirective(type,
                                  common::ControlDirectiveSource::SURVIVABILITY);
}

/// @brief 判断 ECCM 输入是否携带可用的细粒度干扰事实。
/// @param source_info 当前周期 ECCM 输入摘要。
/// @return 至少存在多源事实或兼容字段时返回 true。
bool HasDetailedEccmFacts(const common::EccmSourceInfo& source_info) {
  if (!source_info.jammer_sources.empty()) {
    return true;
  }
  return source_info.jammer_power_db > 0.0f ||
         source_info.frequency_overlap_ratio > 0.0f ||
         source_info.prf_lock_risk > 0.0f || source_info.jammer_in_sidelobe;
}

struct EccmProposalSelection {
  float sidelobe_canceller_score{0.0f};
  float adaptive_beamforming_score{0.0f};
  float agility_frequency_score{0.0f};
  float eccm_rejitter_score{0.0f};
  float burnthrough_gain_score{0.0f};
  bool has_credible_multisource_evidence{false};
  bool association_supports_beam_adaptation{false};
  bool association_supports_frequency_agility{false};
  bool association_supports_rejitter{false};
  common::JammingSemantic association_semantic{common::JammingSemantic::kNone};
};

/// @brief 根据评分增益调整控制意图优先级。
/// @param base_priority 基础优先级。
/// @param score 当前评分。
/// @return 叠加评分后的优先级。
int ResolvePriorityFromScore(int base_priority, float score) {
  return base_priority + static_cast<int>(score * 10.0f);
}

/// @brief 将浮点值裁剪到 [0, 1] 区间。
/// @param value 输入值。
/// @return 裁剪后的结果。
float ClampUnit(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

bool HasMeaningfulAssociationPressure(
    const common::AssociationQualityInfo& association_quality_info) {
  return association_quality_info.association_stress >=
             kMinimumAssociationStress &&
         association_quality_info.jamming_severity >=
             kMinimumAssociationSeverity;
}

/// @brief 为缺少细粒度事实的场景添加保守自适应波束形成偏置。
/// @param selection 待累加的提案选择状态。
void AccumulateCautiousFallback(EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  selection->adaptive_beamforming_score =
      std::max(selection->adaptive_beamforming_score, 1.0f);
}

/// @brief 将兼容旧版平铺 ECCM 字段累加为提案评分。
/// @param source_info 当前周期 ECCM 输入摘要。
/// @param selection 待累加的提案选择状态。
void AccumulateLegacyEccmFacts(const common::EccmSourceInfo& source_info,
                               EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  const bool has_detailed_facts = HasDetailedEccmFacts(source_info);
  if (!has_detailed_facts || source_info.jammer_in_sidelobe) {
    selection->sidelobe_canceller_score =
        std::max(selection->sidelobe_canceller_score, 2.0f);
  }
  selection->adaptive_beamforming_score =
      std::max(selection->adaptive_beamforming_score, 1.5f);
  if (!has_detailed_facts ||
      source_info.frequency_overlap_ratio >= kHighFrequencyOverlapRatio) {
    selection->agility_frequency_score =
        std::max(selection->agility_frequency_score, 2.0f);
  }
  if (!has_detailed_facts || source_info.prf_lock_risk >= kHighPrfLockRisk) {
    selection->eccm_rejitter_score =
        std::max(selection->eccm_rejitter_score, 2.0f);
  }
  if (!has_detailed_facts || source_info.jammer_power_db >= kHighJammerPowerDb) {
    selection->burnthrough_gain_score =
        std::max(selection->burnthrough_gain_score, 2.0f);
  }
}

void AccumulateMultiSourceEccmFacts(
    const common::EccmJammerSourceInfo& source,
    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  if (source.confidence < kMinimumCredibleConfidence) {
    return;
  }

  selection->has_credible_multisource_evidence = true;
  const float confidence_weight = std::max(source.confidence, 0.5f);
  const float power_weight =
      std::min(source.jammer_power_db / kHighJammerPowerDb, 2.0f);
  const float js_weight =
      std::min(source.jammer_to_signal_db / kHighJammerToSignalDb, 2.0f);

  selection->adaptive_beamforming_score += 0.8f * confidence_weight;
  if (source.jammer_in_sidelobe) {
    selection->sidelobe_canceller_score += 2.0f * confidence_weight;
  }
  if (source.frequency_overlap_ratio >= kHighFrequencyOverlapRatio) {
    selection->agility_frequency_score +=
        source.frequency_overlap_ratio * 2.0f * confidence_weight;
  }
  if (source.prf_lock_risk >= kHighPrfLockRisk) {
    selection->eccm_rejitter_score +=
        source.prf_lock_risk * 2.0f * confidence_weight;
  }
  if (source.jammer_power_db >= kHighJammerPowerDb ||
      source.jammer_to_signal_db >= kHighJammerToSignalDb) {
    selection->burnthrough_gain_score +=
        std::max(power_weight, js_weight) * confidence_weight;
  }

  switch (source.technique) {
    case common::JammingTechnique::kNoiseSuppression:
      selection->adaptive_beamforming_score += 0.7f * confidence_weight;
      selection->burnthrough_gain_score += 0.8f * confidence_weight;
      if (source.jammer_in_sidelobe) {
        selection->sidelobe_canceller_score += 1.2f * confidence_weight;
      }
      break;
    case common::JammingTechnique::kDeception:
      selection->agility_frequency_score += 1.5f * confidence_weight;
      selection->eccm_rejitter_score += 1.8f * confidence_weight;
      selection->adaptive_beamforming_score += 0.5f * confidence_weight;
      break;
    case common::JammingTechnique::kRepeater:
      selection->eccm_rejitter_score += 1.6f * confidence_weight;
      selection->adaptive_beamforming_score += 1.0f * confidence_weight;
      selection->agility_frequency_score += 0.6f * confidence_weight;
      break;
    case common::JammingTechnique::kUnknown:
    default:
      selection->adaptive_beamforming_score += 0.4f * confidence_weight;
      break;
  }
}

void AccumulateAssociationDrivenBias(
    const common::AssociationQualityInfo& association_quality_info,
    EccmProposalSelection* selection) {
  if (selection == nullptr ||
      !HasMeaningfulAssociationPressure(association_quality_info)) {
    return;
  }

  const float severity = ClampUnit(association_quality_info.jamming_severity);
  const float stress = ClampUnit(association_quality_info.association_stress);
  const float cost_pressure =
      ClampUnit(std::max(association_quality_info.mean_match_cost / 3.0f,
                         association_quality_info.p95_match_cost / 4.0f));
  const float combined_weight =
      0.45f * severity + 0.40f * stress + 0.15f * cost_pressure;

  switch (association_quality_info.dominant_jamming_semantic) {
    case common::JammingSemantic::kDeception:
      selection->association_semantic = common::JammingSemantic::kDeception;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 3.5f * combined_weight;
      selection->eccm_rejitter_score += 3.8f * combined_weight;
      selection->adaptive_beamforming_score += 0.8f * combined_weight;
      break;
    case common::JammingSemantic::kRepeater:
      selection->association_semantic = common::JammingSemantic::kRepeater;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 1.8f * combined_weight;
      selection->eccm_rejitter_score += 3.6f * combined_weight;
      selection->adaptive_beamforming_score += 0.9f * combined_weight;
      break;
    case common::JammingSemantic::kMixed:
      selection->association_semantic = common::JammingSemantic::kMixed;
      selection->association_supports_frequency_agility = true;
      selection->association_supports_rejitter = true;
      selection->association_supports_beam_adaptation = true;
      selection->agility_frequency_score += 2.8f * combined_weight;
      selection->eccm_rejitter_score += 3.0f * combined_weight;
      selection->adaptive_beamforming_score += 0.8f * combined_weight;
      break;
    case common::JammingSemantic::kNoiseSuppression:
    case common::JammingSemantic::kNone:
    default:
      break;
  }
}

/// @brief 将关联语义转换为可读描述文本。
/// @param semantic 当前周期主导干扰语义。
/// @return 用于 rationale 的简短描述。
std::string DescribeAssociationSemantic(common::JammingSemantic semantic) {
  switch (semantic) {
    case common::JammingSemantic::kDeception:
      return "deception-like association stress";
    case common::JammingSemantic::kRepeater:
      return "repeater-like association stress";
    case common::JammingSemantic::kMixed:
      return "mixed association stress";
    case common::JammingSemantic::kNoiseSuppression:
      return "noise-like association stress";
    case common::JammingSemantic::kNone:
    default:
      return "association stress";
  }
}

std::string BuildProposalRationale(
    common::ControlDirectiveType type,
    const EccmProposalSelection& selection) {
  std::string rationale;
  switch (type) {
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      rationale = "jamming facts favor sidelobe cancellation";
      break;
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      rationale = "jamming environment requires adaptive beamforming";
      if (selection.association_supports_beam_adaptation) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " also degrades association stability";
      }
      break;
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      rationale = "jamming facts favor agility frequency";
      if (selection.association_supports_frequency_agility) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " raises frequency-agility priority";
      }
      break;
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      rationale = "jamming facts favor rejitter";
      if (selection.association_supports_rejitter) {
        rationale += "; ";
        rationale += DescribeAssociationSemantic(selection.association_semantic);
        rationale += " raises PRF rejitter priority";
      }
      break;
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      rationale = "jamming facts favor burnthrough gain";
      break;
    default:
      rationale = "jamming facts require countermeasure";
      break;
  }
  return rationale;
}

void AppendProposal(common::ControlDirectiveType type, int priority,
                    const std::string& rationale,
                    std::vector<pipeline::TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(
      pipeline::TacticalProposal{BuildDirective(type), priority, rationale});
}

void AppendEccmProposals(const common::EccmSourceInfo& source_info,
                         const common::AssociationQualityInfo& association_quality_info,
                         bool environment_jamming_detected,
                         bool hold_only,
                         std::vector<pipeline::TacticalProposal>* proposals) {
  EccmProposalSelection selection;
  if (!source_info.jammer_sources.empty()) {
    for (std::size_t i = 0; i < source_info.jammer_sources.size(); ++i) {
      AccumulateMultiSourceEccmFacts(source_info.jammer_sources[i], &selection);
    }
    if (!selection.has_credible_multisource_evidence) {
      AccumulateCautiousFallback(&selection);
    }
  } else {
    const bool has_detailed_facts = HasDetailedEccmFacts(source_info);
    const bool should_use_cautious_fallback =
        !has_detailed_facts && !environment_jamming_detected &&
        (HasMeaningfulAssociationPressure(association_quality_info) ||
         hold_only);
    if (should_use_cautious_fallback) {
      AccumulateCautiousFallback(&selection);
    } else {
      AccumulateLegacyEccmFacts(source_info, &selection);
    }
  }
  AccumulateAssociationDrivenBias(association_quality_info, &selection);

  if (selection.sidelobe_canceller_score >= kThresholdSidelobeCanceller) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
        ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                 selection.sidelobe_canceller_score),
        BuildProposalRationale(
            common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
            selection),
        proposals);
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
        ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                 selection.adaptive_beamforming_score),
        BuildProposalRationale(
            common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            selection),
        proposals);
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                   ResolvePriorityFromScore(kBasePriorityAgilityFrequency,
                                            selection.agility_frequency_score),
                   BuildProposalRationale(
                       common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                       selection),
                   proposals);
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(common::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                   ResolvePriorityFromScore(kBasePriorityEccmRejitter,
                                            selection.eccm_rejitter_score),
                   BuildProposalRationale(
                       common::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                       selection),
                   proposals);
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
        ResolvePriorityFromScore(kBasePriorityBurnthroughGain,
                                 selection.burnthrough_gain_score),
        BuildProposalRationale(
            common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
            selection),
        proposals);
  }
}

} // namespace

void SurvivabilityEvaluator::Evaluate(
    const common::DecisionInputFrame& input_frame, pipeline::TacticalStateStore& state_store,
    pipeline::TacticalEvaluationState& evaluation_state) const {
  bool should_enable_eccm =
      evaluation_state.eccm_source_info.has_jamming_signal ||
      input_frame.environment_jamming_detected;
  const bool has_current_eccm_evidence = should_enable_eccm;
  if (!should_enable_eccm && state_store.eccm_hold_cycles_remaining > 0U) {
    should_enable_eccm = true;
    --state_store.eccm_hold_cycles_remaining;
  }

  evaluation_state.should_enable_eccm = should_enable_eccm;
  if (!should_enable_eccm) {
    spdlog::info(
        "[SurvivabilityEvaluator] Environment is clear. Continuing nominal operation.");
    return;
  }

  state_store.eccm_hold_cycles_remaining = kEccmHoldCycles;
  AppendEccmProposals(evaluation_state.eccm_source_info,
                      input_frame.association_quality_info,
                      input_frame.environment_jamming_detected,
                      !has_current_eccm_evidence,
                      &evaluation_state.proposals);
  spdlog::info(
      "[SurvivabilityEvaluator] Active jamming detected. Appending ECCM proposals.");
}

} // namespace eccm
} // namespace decision
} // namespace airborne_radar
