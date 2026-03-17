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

common::ControlDirective BuildDirective(common::ControlDirectiveType type) {
  return common::ControlDirective(type,
                                  common::ControlDirectiveSource::SURVIVABILITY);
}

void AppendEccmProposals(std::vector<TacticalProposal>* proposals) {
  if (proposals == nullptr) {
    return;
  }
  proposals->push_back(TacticalProposal{
      BuildDirective(common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER),
      90,
      "jamming environment requires sidelobe cancellation"});
  proposals->push_back(TacticalProposal{
      BuildDirective(common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING),
      85,
      "jamming environment requires adaptive beamforming"});
  proposals->push_back(TacticalProposal{
      BuildDirective(common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY),
      84,
      "jamming environment requires agility frequency"});
  proposals->push_back(TacticalProposal{
      BuildDirective(common::ControlDirectiveType::REQUEST_ECCM_REJITTER),
      83,
      "jamming environment requires rejitter"});
  proposals->push_back(TacticalProposal{
      BuildDirective(common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN),
      82,
      "jamming environment requires burnthrough gain"});
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
  AppendEccmProposals(&evaluation_state.proposals);
  spdlog::info(
      "[SurvivabilityEvaluator] Active jamming detected. Appending ECCM proposals.");
}

} // namespace eccm
} // namespace decision
} // namespace airborne_radar
