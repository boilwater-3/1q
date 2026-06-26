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

session::ControlDirective EccmEvaluator::BuildDirective(
    session::ControlDirectiveType type) {
  return session::ControlDirective(
      type, session::ControlDirectiveSource::SURVIVABILITY);
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

void EccmEvaluator::AccumulateAssociationPressureFacts(
    const session::AssociationQualityInfo& association_quality,
    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  if (!HasMeaningfulAssociationPressure(association_quality.jamming_severity,
                                        association_quality.association_stress)) {
    return;
  }

  // 等效于物理参数恰在阈值处（jammer_power_db = kHighJammerPowerDb,
  // jammer_to_signal_db = kHighJammerToSignalDb, jammer_in_sidelobe = false）。
  const float severity = ClampUnit(association_quality.jamming_severity);
  selection->has_credible_multisource_evidence = true;

  // 通用：自适应波束形成在所有干扰类型下均适用
  selection->adaptive_beamforming_score += 0.8f * severity;

  // 频率重叠和 PRF 锁风险以 severity 代替（物理映射）
  if (severity >= kHighFrequencyOverlapRatio) {
    selection->agility_frequency_score += 2.0f * severity * severity;
  }
  if (severity >= kHighPrfLockRisk) {
    selection->eccm_rejitter_score += 2.0f * severity * severity;
  }
  // 穿透增益（等效 power_weight = js_weight = 1.0）
  selection->burnthrough_gain_score += severity;

  // 干扰语义特化评分
  switch (association_quality.dominant_jamming_semantic) {
    case config::JammingSemantic::kDeception:
    case config::JammingSemantic::kMixed:
      selection->agility_frequency_score += 1.5f * severity;
      selection->eccm_rejitter_score += 1.8f * severity;
      selection->adaptive_beamforming_score += 0.5f * severity;
      break;
    case config::JammingSemantic::kRepeater:
      selection->eccm_rejitter_score += 1.6f * severity;
      selection->adaptive_beamforming_score += 1.0f * severity;
      selection->agility_frequency_score += 0.6f * severity;
      break;
    case config::JammingSemantic::kNoiseSuppression:
      selection->adaptive_beamforming_score += 0.7f * severity;
      selection->burnthrough_gain_score += 0.8f * severity;
      break;
    default:
      selection->adaptive_beamforming_score += 0.4f * severity;
      break;
  }
}

void EccmEvaluator::AccumulateMultiSourceEccmFacts(
    const session::EccmJammerSourceInfo& source,
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
    case session::JammingTechnique::kNoiseSuppression:
      selection->adaptive_beamforming_score += 0.7f * confidence_weight;
      selection->burnthrough_gain_score += 0.8f * confidence_weight;
      if (source.jammer_in_sidelobe) {
        selection->sidelobe_canceller_score += 1.2f * confidence_weight;
      }
      break;
    case session::JammingTechnique::kDeception:
      selection->agility_frequency_score += 1.5f * confidence_weight;
      selection->eccm_rejitter_score += 1.8f * confidence_weight;
      selection->adaptive_beamforming_score += 0.5f * confidence_weight;
      break;
    case session::JammingTechnique::kRepeater:
      selection->eccm_rejitter_score += 1.6f * confidence_weight;
      selection->adaptive_beamforming_score += 1.0f * confidence_weight;
      selection->agility_frequency_score += 0.6f * confidence_weight;
      break;
    case session::JammingTechnique::kUnknown:
    default:
      selection->adaptive_beamforming_score += 0.4f * confidence_weight;
      break;
  }
}

std::string EccmEvaluator::BuildProposalRationale(
    session::ControlDirectiveType type,
    const EccmProposalSelection& selection) {
  (void)selection;
  switch (type) {
    case session::ControlDirectiveType::
        REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return "jamming facts favor sidelobe cancellation";
    case session::ControlDirectiveType::
        REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return "jamming environment requires adaptive beamforming";
    case session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return "jamming facts favor agility frequency";
    case session::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return "jamming facts favor rejitter";
    case session::ControlDirectiveType::
        REQUEST_ECCM_BURNTHROUGH_GAIN:
      return "jamming facts favor burnthrough gain";
    default:
      return "jamming facts require countermeasure";
  }
}

void EccmEvaluator::AppendProposal(
    session::ControlDirectiveType type, int priority,
    const std::string& rationale,
    std::vector<session::TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(
      session::TacticalProposal{BuildDirective(type), priority, rationale});
}

// ===== public Evaluate =====

EccmEvaluator::Result EccmEvaluator::Evaluate(
    const session::EccmSourceInfo& eccm_source_info, bool hold_only,
    std::vector<session::TacticalProposal>* proposals) {
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
        session::ControlDirectiveType::
            REQUEST_ENABLE_SIDELOBE_CANCELLER,
        ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                 selection.sidelobe_canceller_score),
        BuildProposalRationale(
            session::ControlDirectiveType::
                REQUEST_ENABLE_SIDELOBE_CANCELLER,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(
        session::ControlDirectiveType::
            REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
        ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                 selection.adaptive_beamforming_score),
        BuildProposalRationale(
            session::ControlDirectiveType::
                REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
        ResolvePriorityFromScore(kBasePriorityAgilityFrequency,
                                 selection.agility_frequency_score),
        BuildProposalRationale(
            session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
        ResolvePriorityFromScore(kBasePriorityEccmRejitter,
                                 selection.eccm_rejitter_score),
        BuildProposalRationale(
            session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(
        session::ControlDirectiveType::
            REQUEST_ECCM_BURNTHROUGH_GAIN,
        ResolvePriorityFromScore(kBasePriorityBurnthroughGain,
                                 selection.burnthrough_gain_score),
        BuildProposalRationale(
            session::ControlDirectiveType::
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
          session::ControlDirectiveType::
              REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
          ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                   selection.adaptive_beamforming_score),
          BuildProposalRationale(
              session::ControlDirectiveType::
                  REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
              selection),
          proposals);
      result.eccm_activated = true;
    }
  }

  return result;
}

// ===== Evaluate 重载（关联压力路径）=====

EccmEvaluator::Result EccmEvaluator::Evaluate(
    const session::EccmSourceInfo& eccm_source_info,
    const session::AssociationQualityInfo& association_quality, bool hold_only,
    std::vector<session::TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return Result();
  }

  EccmProposalSelection selection;

  if (hold_only) {
    AccumulateCautiousFallback(&selection);
  } else {
    if (!eccm_source_info.jammer_sources.empty()) {
      for (std::size_t i = 0; i < eccm_source_info.jammer_sources.size(); ++i) {
        AccumulateMultiSourceEccmFacts(eccm_source_info.jammer_sources[i], &selection);
      }
      if (!selection.has_credible_multisource_evidence) {
        AccumulateCautiousFallback(&selection);
      }
    } else if (HasMeaningfulAssociationPressure(association_quality.jamming_severity,
                                                association_quality.association_stress)) {
      AccumulateAssociationPressureFacts(association_quality, &selection);
    } else {
      AccumulateCautiousFallback(&selection);
    }
  }

  // 评分→提案（共享逻辑，与基础 Evaluate 相同）
  Result result;
  if (selection.sidelobe_canceller_score >= kThresholdSidelobeCanceller) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
        ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                 selection.sidelobe_canceller_score),
        BuildProposalRationale(
            session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
        ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                 selection.adaptive_beamforming_score),
        BuildProposalRationale(
            session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
        ResolvePriorityFromScore(kBasePriorityAgilityFrequency,
                                 selection.agility_frequency_score),
        BuildProposalRationale(
            session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY, selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
        ResolvePriorityFromScore(kBasePriorityEccmRejitter, selection.eccm_rejitter_score),
        BuildProposalRationale(session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                               selection),
        proposals);
    result.eccm_activated = true;
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(
        session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
        ResolvePriorityFromScore(kBasePriorityBurnthroughGain,
                                 selection.burnthrough_gain_score),
        BuildProposalRationale(
            session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN, selection),
        proposals);
    result.eccm_activated = true;
  }
  if (!result.eccm_activated && !hold_only) {
    AccumulateCautiousFallback(&selection);
    if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
      AppendProposal(
          session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
          ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                   selection.adaptive_beamforming_score),
          BuildProposalRationale(
              session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
              selection),
          proposals);
      result.eccm_activated = true;
    }
  }
  return result;
}

}  // namespace decision
}  // namespace airborne_radar
