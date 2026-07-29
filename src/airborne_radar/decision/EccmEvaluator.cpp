#include "airborne_radar/decision/EccmEvaluator.h"

#include <algorithm>
#include <string>

namespace airborne_radar {
namespace decision {

namespace {

constexpr double kSidelobeObservationAngleDeg = 3.0;
constexpr double kStrongJammerToNoiseDb = 6.0;

constexpr int kBasePrioritySidelobeCanceller = 86;
constexpr int kBasePriorityAdaptiveBeamforming = 80;
constexpr int kBasePriorityAgilityFrequency = 84;
constexpr int kBasePriorityEccmRejitter = 83;
constexpr int kBasePriorityBurnthroughGain = 82;
constexpr int kBasePriorityAntiRgpo = 89;
constexpr int kBasePriorityAntiVgpo = 85;
constexpr int kBasePriorityAntiFalseTarget = 81;

constexpr float kThresholdSidelobeCanceller = 1.5f;
constexpr float kThresholdAdaptiveBeamforming = 0.8f;
constexpr float kThresholdAgilityFrequency = 1.5f;
constexpr float kThresholdEccmRejitter = 1.5f;
constexpr float kThresholdBurnthroughGain = 1.5f;
constexpr float kThresholdAntiRgpo = 1.5f;
constexpr float kThresholdAntiVgpo = 1.5f;
constexpr float kThresholdAntiFalseTarget = 1.5f;

void MarkActivated(EccmEvaluator::Result* result) {
  if (result == nullptr) {
    return;
  }
  result->eccm_activated = true;
  result->activation_source = EccmEvaluator::ActivationSource::kReceiverRf;
}

}  // namespace

session::ControlDirective EccmEvaluator::BuildDirective(
    session::ControlDirectiveType type) {
  return session::ControlDirective(type, session::ControlDirectiveSource::SURVIVABILITY);
}

int EccmEvaluator::ResolvePriorityFromScore(int base_priority, float score) {
  return base_priority + static_cast<int>(score * 10.0f);
}

float EccmEvaluator::ResolveBurnthroughGain(float score) {
  return std::max(1.0f, std::min(2.0f, 1.0f + 0.25f * score));
}

void EccmEvaluator::AccumulateInterferenceObservation(
    const session::ArInterferenceObservation& observation,
    EccmProposalSelection* selection) {
  if (selection == nullptr) {
    return;
  }
  selection->adaptive_beamforming_score += 1.0f;
  selection->agility_frequency_score += 2.0f;
  if (observation.estimated_off_boresight_deg >= kSidelobeObservationAngleDeg) {
    selection->sidelobe_canceller_score += 2.0f;
  }
  if (observation.estimated_waveform_kind ==
          oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain ||
      observation.estimated_waveform_kind ==
          oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    selection->eccm_rejitter_score += 2.0f;
  }
  if (observation.jammer_to_noise_db >= kStrongJammerToNoiseDb) {
    const double normalized_strength =
        std::min(2.0, observation.jammer_to_noise_db / kStrongJammerToNoiseDb);
    selection->burnthrough_gain_score += static_cast<float>(normalized_strength);
  }
  // 反欺骗评分：按可观测特征路由 RGPO/VGPO，而非让任意 kPulseTrain 同时贡献两者。
  // 此前实现把 kPulseTrain 对称地给 anti_rgpo_score 与 anti_vgpo_score 各 +1.5，没有可观测
  // 特征区分两种技术，导致单一脉冲列干扰同时触发两类对抗。现按以下特征路由：
  //   - VGPO（速度波门拖引）：estimated_range_rate_mps 显著非零 → 加速度限幅；
  //   - RGPO（距离波门拖引）：coherent_emission_count >= 2（多假目标同方向）→ 前沿跟踪；
  //   - 单纯 kPulseTrain 无上述特征时不触发任一反欺骗分数。
  constexpr double kVgpoRangeRateGateMps = 15.0;  // 显著径向速度门限（m/s）
  if (std::fabs(observation.estimated_range_rate_mps) >= kVgpoRangeRateGateMps) {
    selection->anti_vgpo_score += 1.5f;
  }
  if (observation.coherent_emission_count >= 2U) {
    selection->anti_rgpo_score += 1.5f;
  }
  // 假目标：观测分类为疑似假目标 → 假目标鉴别。
  if (observation.deception_class == session::DeceptionClass::kLikelyFalseTarget) {
    selection->anti_false_target_score += 2.0f;
  }
}

std::string EccmEvaluator::BuildProposalRationale(
    session::ControlDirectiveType type, const EccmProposalSelection& selection) {
  (void)selection;
  switch (type) {
    case session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return "RF observation favors sidelobe cancellation";
    case session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return "RF observation requires adaptive beamforming";
    case session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return "RF observation favors frequency agility";
    case session::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return "RF observation favors pulse rejitter";
    case session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return "RF observation favors burnthrough gain";
    case session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE:
      return "RF observation suggests RGPO deception — enable leading-edge tracking";
    case session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND:
      return "RF observation suggests VGPO deception — enable acceleration bound";
    case session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION:
      return "RF observation suggests false targets — enable discrimination";
    default:
      return "RF observation requires countermeasure";
  }
}

void EccmEvaluator::AppendProposal(
    session::ControlDirectiveType type, int priority, const std::string& rationale,
    std::vector<session::TacticalProposal>* proposals, bool has_requested_value,
    float requested_value) {
  if (proposals == nullptr) {
    return;
  }
  const session::ControlDirective directive =
      has_requested_value
          ? session::ControlDirective(type, session::ControlDirectiveSource::SURVIVABILITY,
                                      requested_value)
          : BuildDirective(type);
  proposals->push_back(session::TacticalProposal{directive, priority, rationale});
}

EccmEvaluator::Result EccmEvaluator::Evaluate(
    const session::ArInterferenceObservationList& observations,
    std::vector<session::TacticalProposal>* proposals) {
  Result result;
  if (proposals == nullptr || observations.empty()) {
    return result;
  }

  EccmProposalSelection selection;
  for (const session::ArInterferenceObservation& observation : observations) {
    AccumulateInterferenceObservation(observation, &selection);
  }
  if (selection.sidelobe_canceller_score >= kThresholdSidelobeCanceller) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
                   ResolvePriorityFromScore(kBasePrioritySidelobeCanceller,
                                            selection.sidelobe_canceller_score),
                   BuildProposalRationale(
                       session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
                       selection),
                   proposals);
    MarkActivated(&result);
  }
  if (selection.adaptive_beamforming_score >= kThresholdAdaptiveBeamforming) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                   ResolvePriorityFromScore(kBasePriorityAdaptiveBeamforming,
                                            selection.adaptive_beamforming_score),
                   BuildProposalRationale(
                       session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                       selection),
                   proposals);
    MarkActivated(&result);
  }
  if (selection.agility_frequency_score >= kThresholdAgilityFrequency) {
    AppendProposal(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                   ResolvePriorityFromScore(kBasePriorityAgilityFrequency,
                                            selection.agility_frequency_score),
                   BuildProposalRationale(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                          selection),
                   proposals);
    MarkActivated(&result);
  }
  if (selection.eccm_rejitter_score >= kThresholdEccmRejitter) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                   ResolvePriorityFromScore(kBasePriorityEccmRejitter,
                                            selection.eccm_rejitter_score),
                   BuildProposalRationale(session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                                          selection),
                   proposals);
    MarkActivated(&result);
  }
  if (selection.burnthrough_gain_score >= kThresholdBurnthroughGain) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                   ResolvePriorityFromScore(kBasePriorityBurnthroughGain,
                                            selection.burnthrough_gain_score),
                   BuildProposalRationale(
                       session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN, selection),
                   proposals, true, ResolveBurnthroughGain(selection.burnthrough_gain_score));
    MarkActivated(&result);
  }
  if (selection.anti_rgpo_score >= kThresholdAntiRgpo) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
                   ResolvePriorityFromScore(kBasePriorityAntiRgpo,
                                            selection.anti_rgpo_score),
                   BuildProposalRationale(
                       session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
                       selection),
                   proposals);
    MarkActivated(&result);
  }
  if (selection.anti_vgpo_score >= kThresholdAntiVgpo) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND,
                   ResolvePriorityFromScore(kBasePriorityAntiVgpo,
                                            selection.anti_vgpo_score),
                   BuildProposalRationale(
                       session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND,
                       selection),
                   proposals);
    MarkActivated(&result);
  }
  if (selection.anti_false_target_score >= kThresholdAntiFalseTarget) {
    AppendProposal(session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION,
                   ResolvePriorityFromScore(kBasePriorityAntiFalseTarget,
                                            selection.anti_false_target_score),
                   BuildProposalRationale(
                       session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION,
                       selection),
                   proposals);
    MarkActivated(&result);
  }
  return result;
}

}  // namespace decision
}  // namespace airborne_radar
