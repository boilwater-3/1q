// Copyright 2026. All Rights Reserved.
//
// Description: SurvivabilityEvaluator 的实现。

#include "1q/airborne_radar/decision/eccm/SurvivabilityEvaluator.h"

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace eccm {

namespace {

const std::uint32_t kEccmHoldCycles = 2;
const float kHighFrequencyOverlapRatio = 0.5f;
const float kHighPrfLockRisk = 0.5f;
const float kHighJammerPowerDb = 8.0f;

common::ControlDirective BuildDirective(common::ControlDirectiveType type) {
  return common::ControlDirective(type,
                                  common::ControlDirectiveSource::SURVIVABILITY);
}

bool HasDetailedEccmFacts(const common::EccmSourceInfo& source_info) {
  return source_info.jammer_power_db > 0.0f ||
         source_info.frequency_overlap_ratio > 0.0f ||
         source_info.prf_lock_risk > 0.0f || source_info.jammer_in_sidelobe;
}

void AppendProposal(common::ControlDirectiveType type, int priority,
                    const char* rationale,
                    std::vector<TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(
      TacticalProposal{BuildDirective(type), priority, rationale});
}

void AppendEccmProposals(const common::EccmSourceInfo& source_info,
                         std::vector<TacticalProposal>* proposals) {
  const bool has_detailed_facts = HasDetailedEccmFacts(source_info);
  const bool enable_sidelobe_canceller =
      !has_detailed_facts || source_info.jammer_in_sidelobe;
  const bool enable_frequency_agility =
      !has_detailed_facts ||
      source_info.frequency_overlap_ratio >= kHighFrequencyOverlapRatio;
  const bool enable_rejitter =
      !has_detailed_facts || source_info.prf_lock_risk >= kHighPrfLockRisk;
  const bool enable_burnthrough =
      !has_detailed_facts || source_info.jammer_power_db >= kHighJammerPowerDb;

  if (enable_sidelobe_canceller) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER, 90,
        "jamming facts favor sidelobe cancellation", proposals);
  }
  AppendProposal(
      common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING, 85,
      "jamming environment requires adaptive beamforming", proposals);
  if (enable_frequency_agility) {
    AppendProposal(common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY, 84,
                   "jamming facts favor agility frequency", proposals);
  }
  if (enable_rejitter) {
    AppendProposal(common::ControlDirectiveType::REQUEST_ECCM_REJITTER, 83,
                   "jamming facts favor rejitter", proposals);
  }
  if (enable_burnthrough) {
    AppendProposal(
        common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN, 82,
        "jamming facts favor burnthrough gain", proposals);
  }
}

} // namespace

void SurvivabilityEvaluator::Evaluate(
    const common::DecisionInputFrame& input_frame, TacticalStateStore& state_store,
    TacticalEvaluationState& evaluation_state) const {
  bool should_enable_eccm =
      evaluation_state.eccm_source_info.has_jamming_signal ||
      input_frame.environment_jamming_detected;
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
                      &evaluation_state.proposals);
  spdlog::info(
      "[SurvivabilityEvaluator] Active jamming detected. Appending ECCM proposals.");
}

} // namespace eccm
} // namespace decision
} // namespace airborne_radar
