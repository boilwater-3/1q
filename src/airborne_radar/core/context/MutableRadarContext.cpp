// Copyright 2026. All Rights Reserved.
//
// @file MutableRadarContext.cpp
// @brief 实现面向外部接入的可变雷达上下文默认实现。

#include "airborne_radar/core/context/MutableRadarContext.h"

#include <utility>

namespace airborne_radar {
namespace core {
namespace context {

void MutableRadarContext::BeginCycle(const RadarCycleInput& input) {
  SetTargetFeatures(input.target_features);
  SetPlatformAttitude(input.platform_attitude_deg);
  SetCycleDeltaTimeSec(input.dt_sec);
  ResetCycleOutputs();
}

void MutableRadarContext::SetTargetFeatures(
    common::TargetFeatureList target_features) {
  target_features_ = std::move(target_features);
}

void MutableRadarContext::SetPlatformAttitude(
    const common::PlatformAttitudeDeg& platform_attitude_deg) {
  platform_attitude_deg_ = platform_attitude_deg;
}

void MutableRadarContext::SetCycleDeltaTimeSec(float dt_sec) {
  cycle_dt_sec_ = dt_sec;
}

void MutableRadarContext::ResetCycleOutputs() {
  submitted_commands_.clear();
}

const std::vector<common::RadarCommand>&
MutableRadarContext::GetSubmittedCommands() const {
  return submitted_commands_;
}

bool MutableRadarContext::HasLatestControlProfile() const {
  return has_latest_control_profile_;
}

const common::RadarControlProfile&
MutableRadarContext::GetLatestControlProfile() const {
  return latest_control_profile_;
}

common::TargetFeatureList MutableRadarContext::GetTargetFeatures() const {
  return target_features_;
}

common::PlatformAttitudeDeg MutableRadarContext::GetPlatformAttitude() const {
  return platform_attitude_deg_;
}

float MutableRadarContext::GetCycleDeltaTimeSec() const {
  return cycle_dt_sec_;
}

void MutableRadarContext::SubmitControlCommand(common::RadarCommand cmd) {
  submitted_commands_.push_back(std::move(cmd));
}

void MutableRadarContext::UpdateRadarControlProfile(
    const common::RadarControlProfile& profile) {
  latest_control_profile_ = profile;
  has_latest_control_profile_ = true;
}

} // namespace context
} // namespace core
} // namespace airborne_radar
