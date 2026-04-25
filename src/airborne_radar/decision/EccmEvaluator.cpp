#include "airborne_radar/decision/EccmEvaluator.h"

#include <algorithm>
#include <string>

namespace airborne_radar {
namespace decision {

namespace {

// ===== 多源干扰事实评分阈值 =====
constexpr float kHighFrequencyOverlapRatio = 0.5f;
constexpr float kHighPrfLockRisk = 0.5f;
constexpr float kHighJammerPowerDb = 8.0f;
constexpr float kHighJammerToSignalDb = 6.0f;
constexpr float kMinimumCredibleConfidence = 0.35f;
constexpr float kMinimumAssociationSeverity = 0.30f;
constexpr float kMinimumAssociationStress = 0.18f;

// ===== 基础优先级 =====
constexpr int kBasePrioritySidelobeCanceller = 86;
constexpr int kBasePriorityAdaptiveBeamforming = 80;
constexpr int kBasePriorityAgilityFrequency = 84;
constexpr int kBasePriorityEccmRejitter = 83;
constexpr int kBasePriorityBurnthroughGain = 82;

// ===== 输出阈值 =====
constexpr float kThresholdSidelobeCanceller = 1.5f;
constexpr float kThresholdAdaptiveBeamforming = 0.8f;
constexpr float kThresholdAgilityFrequency = 1.5f;
constexpr float kThresholdEccmRejitter = 1.5f;
constexpr float kThresholdBurnthroughGain = 1.5f;

}  // namespace

// ===== private static helpers =====

extension::control::ControlDirective EccmEvaluator::BuildDirective(
    extension::control::ControlDirectiveType type) {
  return extension::control::ControlDirective(
      type, extension::control::ControlDirectiveSource::SURVIVABILITY);
}

int EccmEvaluator::ResolvePriorityFromScore(int base_priority, float score) {
  return base_priority + static_cast<int>(score * 10.0f);
}

float EccmEvaluator::ClampUnit(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

bool EccmEvaluator::HasMeaningfulAssociationPressure(
    float jamming_severity, float association_stress) {
  return association_stress >= kMinimumAssociationStress &&
         jamming_severity >= kMinimumAssociationSeverity;
}

void EccmEvaluator::AccumulateCautiousFallback(
    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  selection->adaptive_beamforming_score =
      std::max(selection->adaptive_beamforming_score, 1.0f);
}

void EccmEvaluator::AccumulateMultiSourceEccmFacts(
    const model::EccmJammerSourceInfo& source,
    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  if (source.confidence < kMinimumCredibleConfidence - 1e-5f) {
    return;
  }

  selection->has_credible_multisource_evidence = true;
  const float confidence_weight = ClampUnit(source.confidence);
  const float power_weight =
      std::max(0.0f, std::min(source.jammer_power_db / kHighJammerPowerDb, 2.0f));
  const float js_weight = std::max(
      0.0f, std::min(source.jammer_to_signal_db / kHighJammerToSignalDb, 2.0f));

  selection->adaptive_beamforming_score += 0.8f * confidence_weight;
  if (source.jammer_in_sidelobe) {
    selection->sidelobe_canceller_score += 2.0f * confidence_weight;
  }
  if (source.frequency_overlap_ratio >= kHighFrequencyOverlapRatio - 1e-5f) {
    selection->agility_frequency_score +=
        source.frequency_overlap_ratio * 2.0f * confidence_weight;
  }
  if (source.prf_lock_risk >= kHighPrfLockRisk - 1e-5f) {
    selection->eccm_rejitter_score +=
        source.prf_lock_risk * 2.0f * confidence_weight;
  }
  if (source.jammer_power_db >= kHighJammerPowerDb - 1e-5f ||
      source.jammer_to_signal_db >= kHighJammerToSignalDb - 1e-5f) {
    selection->burnthrough_gain_score +=
        std::max(power_weight, js_weight) * confidence_weight;
  }

  switch (source.technique) {
    case model::JammingTechnique::kNoiseSuppression:
      selection->adaptive_beamforming_score += 0.7f * confidence_weight;
      selection->burnthrough_gain_score += 0.8f * confidence_weight;
      if (source.jammer_in_sidelobe) {
        selection->sidelobe_canceller_score += 1.2f * confidence_weight;
      }
      break;
    case model::JammingTechnique::kDeception:
      selection->agility_frequency_score += 1.5f * confidence_weight;
      selection->eccm_rejitter_score += 1.8f * confidence_weight;
      selection->adaptive_beamforming_score += 0.5f * confidence_weight;
      break;
    case model::JammingTechnique::kRepeater:
      selection->eccm_rejitter_score += 1.6f * confidence_weight;
      selection->adaptive_beamforming_score += 1.0f * confidence_weight;
      selection->agility_frequency_score += 0.6f * confidence_weight;
      break;
    case model::JammingTechnique::kUnknown:
    default:
      selection->adaptive_beamforming_score += 0.4f * confidence_weight;
      break;
  }
}

std::string EccmEvaluator::BuildProposalRationale(
    extension::control::ControlDirectiveType type,
    const EccmProposalSelection& selection) {
  (void)selection;
  switch (type) {
    case extension::control::ControlDirectiveType::
        REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return "jamming facts favor sidelobe cancellation";
    case extension::control::ControlDirectiveType::
        REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return "jamming environment requires adaptive beamforming";
    case extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return "jamming facts favor agility frequency";
    case extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return "jamming facts favor rejitter";
    case extension::control::ControlDirectiveType::
        REQUEST_ECCM_BURNTHROUGH_GAIN:
      return "jamming facts favor burnthrough gain";
    default:
      return "jamming facts require countermeasure";
  }
}

void EccmEvaluator::AppendProposal(
    extension::control::ControlDirectiveType type, int priority,
    const std::string& rationale,
    std::vector<extension::TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(
      extension::TacticalProposal{BuildDirective(type), priority, rationale});
}

// ===== public Evaluate =====

EccmEvaluator::Result EccmEvaluator::Evaluate(
    const model::EccmSourceInfo& eccm_source_info, bool hold_only,
    std::vector<extension::TacticalProposal>* proposals) {
  Result result;

  if (proposals == nullptr) {
    return result;
  }

  EccmProposalSelection selection;

  if (hold_only) {
    // 持有期路径：仅保守回退，评分权重低于新鲜证据
    AccumulateCautiousFallback(&selection);
  } else {
    if (!eccm_source_info.jammer_sources.empty()) {
      for (std::size_t i = 0; i < eccm_source_info.jammer_sources.size(); ++i) {
        AccumulateMultiSourceEccmFacts(eccm_source_info.jammer_sources[i],
                                       &selection);
      }
      if (!selection.has_credible_multisource_evidence) {
        AccumulateCautiousFallback(&selection);
      }
    } else {
      AccumulateCautiousFallback(&selection);
    }
  }

  // 各项达标时追加 proposal
  if (selection.sidelobe_canceller_score >= kThresholdSidelobeCanceller) {
    AppendProposal(
        extension::control::ControlDirectiveType::
            REQUEST_ENABLE_SIDELOBE_CANCELLER,
        ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                 selection.sidelobe_canceller_score),
        BuildProposalRationale(
            extension::control::ControlDirectiveType::
                REQUEST_ENABLE_SIDELOBE_CANCELLER,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(
        extension::control::ControlDirectiveType::
            REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
        ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                 selection.adaptive_beamforming_score),
        BuildProposalRationale(
            extension::control::ControlDirectiveType::
                REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(
        extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
        ResolvePriorityFromScore(kBasePriorityAgilityFrequency,
                                 selection.agility_frequency_score),
        BuildProposalRationale(
            extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(
        extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER,
        ResolvePriorityFromScore(kBasePriorityEccmRejitter,
                                 selection.eccm_rejitter_score),
        BuildProposalRationale(
            extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(
        extension::control::ControlDirectiveType::
            REQUEST_ECCM_BURNTHROUGH_GAIN,
        ResolvePriorityFromScore(kBasePriorityBurnthroughGain,
                                 selection.burnthrough_gain_score),
        BuildProposalRationale(
            extension::control::ControlDirectiveType::
                REQUEST_ECCM_BURNTHROUGH_GAIN,
            selection),
        proposals);
    result.eccm_activated = true;
  }

  // 有干扰事实但未生成任何达标提案时，最低激活自适应波束形成作为回退
  if (!result.eccm_activated && !hold_only) {
    AccumulateCautiousFallback(&selection);
    if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
      AppendProposal(
          extension::control::ControlDirectiveType::
              REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
          ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                   selection.adaptive_beamforming_score),
          BuildProposalRationale(
              extension::control::ControlDirectiveType::
                  REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
              selection),
          proposals);
      result.eccm_activated = true;
    }
  }

  return result;
}

}  // namespace decision
}  // namespace airborne_radar
