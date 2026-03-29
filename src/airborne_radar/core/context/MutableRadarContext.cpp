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

void MutableRadarContext::SetTargetFeatures(common::model::TargetFeatureList target_features) {
  target_features_ = std::move(target_features);
}

void MutableRadarContext::SetPlatformAttitude(
    const common::config::PlatformAttitudeDeg& platform_attitude_deg) {
  platform_attitude_deg_ = platform_attitude_deg;
}

void MutableRadarContext::SetCycleDeltaTimeSec(float dt_sec) { cycle_dt_sec_ = dt_sec; }

void MutableRadarContext::ResetCycleOutputs() { submitted_commands_.clear(); }

const std::vector<common::control::RadarCommand>& MutableRadarContext::GetSubmittedCommands() const {
  return submitted_commands_;
}

bool MutableRadarContext::HasLatestControlProfile() const { return has_latest_control_profile_; }

const common::control::RadarControlProfile& MutableRadarContext::GetLatestControlProfile() const {
  return latest_control_profile_;
}

const common::model::TargetFeatureList& MutableRadarContext::GetTargetFeatures() const {
  return target_features_;
}

common::config::PlatformAttitudeDeg MutableRadarContext::GetPlatformAttitude() const {
  return platform_attitude_deg_;
}

float MutableRadarContext::GetCycleDeltaTimeSec() const { return cycle_dt_sec_; }

void MutableRadarContext::SubmitControlCommand(common::control::RadarCommand cmd) {
  submitted_commands_.push_back(std::move(cmd));
}

void MutableRadarContext::UpdateRadarControlProfile(const common::control::RadarControlProfile& profile) {
  latest_control_profile_ = profile;
  has_latest_control_profile_ = true;
}

}  // namespace context
}  // namespace core
}  // namespace airborne_radar
