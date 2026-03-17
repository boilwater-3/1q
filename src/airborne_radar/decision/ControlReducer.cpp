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

void ResolveEmissionSurvivabilityConflict(
    const ControlReducerConfig& config, common::RadarControlProfile* profile) {
  if (profile == nullptr) {
    return;
  }

  // 烧穿增益属于生存性优先路径，应对 LPI 功率压低设置保护下限，
  // 避免 profile 同时要求“强穿透”和“显著降功率”而互相抵消。
  if (config.prefer_survivability_in_power_conflict &&
      profile->eccm_burnthrough_gain > 1.0f &&
      profile->enable_lpi_power_control) {
    profile->lpi_power_scale =
        std::max(profile->lpi_power_scale, config.burnthrough_lpi_power_floor);
  }
}

} // namespace

ControlReducer::ControlReducer(ControlReducerConfig config)
    : config_(config) {}

void ControlReducer::UpdateConfig(ControlReducerConfig config) {
  config_ = config;
}

ControlReducerConfig ControlReducer::GetConfig() const {
  return config_;
}

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
        result.profile.lpi_power_scale = config_.lpi_power_scale_on_reduction;
        break;
      case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
        result.profile.enable_lpi_beamforming = true;
        break;
      case common::ControlDirectiveType::REQUEST_LPI_DWELL:
        result.profile.lpi_dwell_scale = config_.lpi_dwell_scale;
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
        result.profile.eccm_burnthrough_gain = config_.eccm_burnthrough_gain;
        break;
      case common::ControlDirectiveType::NONE:
      default:
        result.rejected_directives.push_back(directive);
        continue;
    }

    applied_types.insert(directive.type);
    result.applied_directives.push_back(directive);
  }

  ResolveEmissionSurvivabilityConflict(config_, &result.profile);
  return result;
}

} // namespace decision
} // namespace airborne_radar
