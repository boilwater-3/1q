// Copyright 2026. All Rights Reserved.
//
// @file ControlReducer.cpp
// @brief 实现 ControlReducer 的控制意图归并逻辑。

#include "airborne_radar/decision/pipeline/ControlReducer.h"

#include <algorithm>
#include <set>
#include <vector>

namespace airborne_radar {
namespace decision {
namespace pipeline {

namespace {

/// @brief 按 proposal 优先级降序比较。
/// @param lhs 左侧 proposal。
/// @param rhs 右侧 proposal。
/// @return 左侧优先级更高时返回 true。
bool CompareProposalPriority(const TacticalProposal& lhs,
                             const TacticalProposal& rhs) {
  return lhs.priority > rhs.priority;
}

/// @brief 判断控制意图是否属于 LPI 域。
/// @param type 控制意图类型。
/// @return 属于 LPI 域时返回 true。
bool IsLpiDirective(common::ControlDirectiveType type) {
  switch (type) {
    case common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
    case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
    case common::ControlDirectiveType::REQUEST_LPI_DWELL:
      return true;
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
    case common::ControlDirectiveType::NONE:
    default:
      return false;
  }
}

/// @brief 判断控制意图是否属于 ECCM 域。
/// @param type 控制意图类型。
/// @return 属于 ECCM 域时返回 true。
bool IsEccmDirective(common::ControlDirectiveType type) {
  switch (type) {
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return true;
    case common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
    case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
    case common::ControlDirectiveType::REQUEST_LPI_DWELL:
    case common::ControlDirectiveType::NONE:
    default:
      return false;
  }
}

/// @brief 判断当前控制真值中 LPI 域是否处于激活状态。
/// @param profile 当前控制真值。
/// @return LPI 任一子域生效时返回 true。
bool IsLpiDomainActive(const common::RadarControlProfile& profile) {
  return profile.enable_lpi_power_control || profile.enable_lpi_beamforming ||
         profile.lpi_dwell_scale != 1.0f;
}

/// @brief 判断当前控制真值中 ECCM 域是否处于激活状态。
/// @param profile 当前控制真值。
/// @return ECCM 任一子域生效时返回 true。
bool IsEccmDomainActive(const common::RadarControlProfile& profile) {
  return profile.enable_agility_frequency || profile.enable_sidelobe_canceller ||
         profile.enable_adaptive_beamforming || profile.enable_eccm_rejitter ||
         profile.eccm_burnthrough_gain > 1.0f;
}

/// @brief 将控制真值中的 LPI 域恢复为默认关闭态。
/// @param profile 待修改的控制真值。
void ResetLpiDomain(common::RadarControlProfile* profile) {
  if (profile == nullptr) {
    return;
  }
  profile->enable_lpi_power_control = false;
  profile->lpi_power_scale = 1.0f;
  profile->enable_lpi_beamforming = false;
  profile->lpi_dwell_scale = 1.0f;
}

/// @brief 将控制真值中的 ECCM 域恢复为默认关闭态。
/// @param profile 待修改的控制真值。
void ResetEccmDomain(common::RadarControlProfile* profile) {
  if (profile == nullptr) {
    return;
  }
  profile->enable_agility_frequency = false;
  profile->enable_sidelobe_canceller = false;
  profile->enable_adaptive_beamforming = false;
  profile->enable_eccm_rejitter = false;
  profile->eccm_burnthrough_gain = 1.0f;
}

void CopyLpiDomain(const common::RadarControlProfile& from,
                   common::RadarControlProfile* to) {
  if (to == nullptr) {
    return;
  }
  to->enable_lpi_power_control = from.enable_lpi_power_control;
  to->lpi_power_scale = from.lpi_power_scale;
  to->enable_lpi_beamforming = from.enable_lpi_beamforming;
  to->lpi_dwell_scale = from.lpi_dwell_scale;
}

void CopyEccmDomain(const common::RadarControlProfile& from,
                    common::RadarControlProfile* to) {
  if (to == nullptr) {
    return;
  }
  to->enable_agility_frequency = from.enable_agility_frequency;
  to->enable_sidelobe_canceller = from.enable_sidelobe_canceller;
  to->enable_adaptive_beamforming = from.enable_adaptive_beamforming;
  to->enable_eccm_rejitter = from.enable_eccm_rejitter;
  to->eccm_burnthrough_gain = from.eccm_burnthrough_gain;
}

void ApplyDirectiveToProfile(const ControlReducerConfig& config,
                             const common::ControlDirective& directive,
                             common::RadarControlProfile* profile,
                             std::vector<common::ControlDirective>* applied,
                             std::vector<common::ControlDirective>* rejected) {
  if (profile == nullptr || applied == nullptr || rejected == nullptr) {
    return;
  }

  switch (directive.type) {
    case common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      profile->enable_lpi_power_control = true;
      profile->lpi_power_scale = config.lpi_power_scale_on_reduction;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      profile->enable_lpi_beamforming = true;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_LPI_DWELL:
      profile->lpi_dwell_scale = config.lpi_dwell_scale;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      profile->enable_sidelobe_canceller = true;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      profile->enable_adaptive_beamforming = true;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      profile->enable_agility_frequency = true;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      profile->enable_eccm_rejitter = true;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      profile->eccm_burnthrough_gain = config.eccm_burnthrough_gain;
      applied->push_back(directive);
      break;
    case common::ControlDirectiveType::NONE:
    default:
      rejected->push_back(directive);
      break;
  }
}

void MoveAppliedDirectiveToRejected(
    common::ControlDirectiveType type,
    std::vector<common::ControlDirective>* applied,
    std::vector<common::ControlDirective>* rejected) {
  if (applied == nullptr || rejected == nullptr) {
    return;
  }
  std::vector<common::ControlDirective>::iterator found =
      std::find_if(applied->begin(), applied->end(),
                   [type](const common::ControlDirective& directive) {
                     return directive.type == type;
                   });
  if (found == applied->end()) {
    return;
  }
  rejected->push_back(*found);
  applied->erase(found);
}

void ResolveEmissionSurvivabilityConflict(
    const ControlReducerConfig& config, common::RadarControlProfile* profile,
    std::vector<common::ControlDirective>* applied,
    std::vector<common::ControlDirective>* rejected) {
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

  if (profile->enable_lpi_beamforming &&
      profile->enable_adaptive_beamforming) {
    if (config.prefer_survivability_in_beam_conflict) {
      profile->enable_lpi_beamforming = false;
      MoveAppliedDirectiveToRejected(
          common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING, applied,
          rejected);
    } else {
      profile->enable_adaptive_beamforming = false;
      MoveAppliedDirectiveToRejected(
          common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
          applied, rejected);
    }
  }
}

bool HasOperationalProfileChanged(const common::RadarControlProfile& previous,
                                  const common::RadarControlProfile& next) {
  return previous.enable_lpi_power_control != next.enable_lpi_power_control ||
         previous.lpi_power_scale != next.lpi_power_scale ||
         previous.enable_lpi_beamforming != next.enable_lpi_beamforming ||
         previous.lpi_dwell_scale != next.lpi_dwell_scale ||
         previous.enable_agility_frequency != next.enable_agility_frequency ||
         previous.enable_sidelobe_canceller != next.enable_sidelobe_canceller ||
         previous.enable_adaptive_beamforming !=
             next.enable_adaptive_beamforming ||
         previous.enable_eccm_rejitter != next.enable_eccm_rejitter ||
         previous.eccm_burnthrough_gain != next.eccm_burnthrough_gain;
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
  common::RadarControlProfile next_profile = previous_profile;
  std::vector<TacticalProposal> sorted_proposals = proposals;
  std::stable_sort(sorted_proposals.begin(), sorted_proposals.end(),
                   CompareProposalPriority);

  const bool previous_lpi_active = IsLpiDomainActive(previous_profile);
  const bool previous_eccm_active = IsEccmDomainActive(previous_profile);
  const std::uint32_t previous_lpi_hold =
      previous_profile.lpi_hold_cycles_remaining;
  const std::uint32_t previous_eccm_hold =
      previous_profile.eccm_hold_cycles_remaining;
  std::uint32_t next_lpi_cooldown =
      previous_profile.lpi_cooldown_cycles_remaining;
  std::uint32_t next_eccm_cooldown =
      previous_profile.eccm_cooldown_cycles_remaining;

  std::set<common::ControlDirectiveType> applied_types;
  std::vector<common::ControlDirective> accepted_directives;
  bool has_lpi_requests = false;
  bool has_eccm_requests = false;
  for (std::size_t i = 0; i < sorted_proposals.size(); ++i) {
    const common::ControlDirective& directive = sorted_proposals[i].directive;
    if (applied_types.find(directive.type) != applied_types.end()) {
      result.rejected_directives.push_back(directive);
      continue;
    }

    const bool is_lpi = IsLpiDirective(directive.type);
    const bool is_eccm = IsEccmDirective(directive.type);
    if (is_lpi && !previous_lpi_active && next_lpi_cooldown > 0U) {
      result.rejected_directives.push_back(directive);
      continue;
    }
    if (is_eccm && !previous_eccm_active && next_eccm_cooldown > 0U) {
      result.rejected_directives.push_back(directive);
      continue;
    }

    applied_types.insert(directive.type);
    accepted_directives.push_back(directive);
    has_lpi_requests = has_lpi_requests || is_lpi;
    has_eccm_requests = has_eccm_requests || is_eccm;
  }

  if (has_lpi_requests) {
    ResetLpiDomain(&next_profile);
    next_profile.lpi_hold_cycles_remaining =
        config_.lpi_hold_cycles_after_request;
    next_profile.lpi_cooldown_cycles_remaining = 0U;
    for (std::size_t i = 0; i < accepted_directives.size(); ++i) {
      if (IsLpiDirective(accepted_directives[i].type)) {
        ApplyDirectiveToProfile(config_, accepted_directives[i], &next_profile,
                               &result.applied_directives,
                               &result.rejected_directives);
      }
    }
  } else if (previous_lpi_active && previous_lpi_hold > 0U) {
    CopyLpiDomain(previous_profile, &next_profile);
    next_profile.lpi_hold_cycles_remaining = previous_lpi_hold - 1U;
    next_profile.lpi_cooldown_cycles_remaining = 0U;
  } else {
    ResetLpiDomain(&next_profile);
    next_profile.lpi_hold_cycles_remaining = 0U;
    next_profile.lpi_cooldown_cycles_remaining =
        previous_lpi_active ? config_.lpi_cooldown_cycles_after_release
                            : (next_lpi_cooldown > 0U ? next_lpi_cooldown - 1U
                                                      : 0U);
  }

  if (has_eccm_requests) {
    ResetEccmDomain(&next_profile);
    next_profile.eccm_hold_cycles_remaining =
        config_.eccm_hold_cycles_after_request;
    next_profile.eccm_cooldown_cycles_remaining = 0U;
    for (std::size_t i = 0; i < accepted_directives.size(); ++i) {
      if (IsEccmDirective(accepted_directives[i].type)) {
        ApplyDirectiveToProfile(config_, accepted_directives[i], &next_profile,
                               &result.applied_directives,
                               &result.rejected_directives);
      }
    }
  } else if (previous_eccm_active && previous_eccm_hold > 0U) {
    CopyEccmDomain(previous_profile, &next_profile);
    next_profile.eccm_hold_cycles_remaining = previous_eccm_hold - 1U;
    next_profile.eccm_cooldown_cycles_remaining = 0U;
  } else {
    ResetEccmDomain(&next_profile);
    next_profile.eccm_hold_cycles_remaining = 0U;
    next_profile.eccm_cooldown_cycles_remaining =
        previous_eccm_active ? config_.eccm_cooldown_cycles_after_release
                             : (next_eccm_cooldown > 0U
                                    ? next_eccm_cooldown - 1U
                                    : 0U);
  }

  ResolveEmissionSurvivabilityConflict(config_, &next_profile,
                                       &result.applied_directives,
                                       &result.rejected_directives);
  next_profile.version =
      HasOperationalProfileChanged(previous_profile, next_profile)
          ? previous_profile.version + 1U
          : previous_profile.version;
  result.profile = next_profile;
  return result;
}

} // namespace pipeline
} // namespace decision
} // namespace airborne_radar
