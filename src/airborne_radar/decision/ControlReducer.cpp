// Copyright 2026. All Rights Reserved.
//
// Description: ControlReducer 的实现。

#include "1q/airborne_radar/decision/ControlReducer.h"

#include <algorithm>
#include <set>
#include <vector>

namespace airborne_radar {
namespace decision {

namespace {

bool CompareProposalPriority(const TacticalProposal& lhs,
                             const TacticalProposal& rhs) {
  return lhs.priority > rhs.priority;
}

} // namespace

ControlReductionResult ControlReducer::Reduce(
    const common::RadarControlProfile& previous_profile,
    const std::vector<TacticalProposal>& proposals) const {
  ControlReductionResult result;
  result.profile = previous_profile;
  if (proposals.empty()) {
    return result;
  }

  result.profile.version = previous_profile.version + 1U;
  std::vector<TacticalProposal> sorted_proposals = proposals;
  std::stable_sort(sorted_proposals.begin(), sorted_proposals.end(),
                   CompareProposalPriority);

  std::set<common::ControlDirectiveType> applied_types;
  for (std::size_t i = 0; i < sorted_proposals.size(); ++i) {
    const common::ControlDirective& directive = sorted_proposals[i].directive;
    if (applied_types.find(directive.type) != applied_types.end()) {
      result.rejected_directives.push_back(directive);
      continue;
    }

    switch (directive.type) {
      case common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
        result.profile.enable_lpi_power_control = true;
        result.profile.lpi_power_scale = 0.5f;
        break;
      case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
        result.profile.enable_lpi_beamforming = true;
        break;
      case common::ControlDirectiveType::REQUEST_LPI_DWELL:
        result.profile.lpi_dwell_scale = 0.75f;
        break;
      case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
        result.profile.enable_sidelobe_canceller = true;
        break;
      case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
        result.profile.enable_adaptive_beamforming = true;
        break;
      case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
        result.profile.enable_agility_frequency = true;
        break;
      case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
        result.profile.enable_eccm_rejitter = true;
        break;
      case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
        result.profile.eccm_burnthrough_gain = 1.5f;
        break;
      case common::ControlDirectiveType::NONE:
      default:
        result.rejected_directives.push_back(directive);
        continue;
    }

    applied_types.insert(directive.type);
    result.applied_directives.push_back(directive);
  }

  return result;
}

} // namespace decision
} // namespace airborne_radar
