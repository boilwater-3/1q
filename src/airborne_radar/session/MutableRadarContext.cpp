#include "airborne_radar/session/MutableRadarContext.h"

#include <utility>

namespace airborne_radar {
namespace session {

struct MutableRadarContext::RuntimeSnapshot {
  std::shared_ptr<model::TargetFeatureList> target_features;
  model::PlatformAttitudeDeg platform_attitude_deg{};
  float cycle_dt_sec{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands{};
  extension::control::RadarControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

void MutableRadarContext::BeginCycle(const RadarCycleInput& input) {
  SetTargetFeatures(input.target_features);
  SetPlatformAttitude(input.platform_attitude_deg);
  SetCycleDeltaTimeSec(input.dt_sec);
  ResetCycleOutputs();
}

void MutableRadarContext::SetTargetFeatures(model::TargetFeatureList target_features) {
  target_features_.reset(new model::TargetFeatureList(std::move(target_features)));
}

void MutableRadarContext::SetPlatformAttitude(
    const model::PlatformAttitudeDeg& platform_attitude_deg) {
  platform_attitude_deg_ = platform_attitude_deg;
}

void MutableRadarContext::SetCycleDeltaTimeSec(float dt_sec) { cycle_dt_sec_ = dt_sec; }

void MutableRadarContext::ResetCycleOutputs() { submitted_commands_.clear(); }

const std::vector<extension::control::RadarCommand>& MutableRadarContext::GetSubmittedCommands()
    const {
  return submitted_commands_;
}

bool MutableRadarContext::HasLatestControlProfile() const { return has_latest_control_profile_; }

const extension::control::RadarControlProfile& MutableRadarContext::GetLatestControlProfile() const {
  return latest_control_profile_;
}

extension::RadarContextRuntimeState MutableRadarContext::CaptureRuntimeState() const {
  extension::RadarContextRuntimeState state;
  std::shared_ptr<RuntimeSnapshot> snapshot(new RuntimeSnapshot());
  snapshot->target_features = target_features_;
  snapshot->platform_attitude_deg = platform_attitude_deg_;
  snapshot->cycle_dt_sec = cycle_dt_sec_;
  snapshot->submitted_commands = submitted_commands_;
  snapshot->latest_control_profile = latest_control_profile_;
  snapshot->has_latest_control_profile = has_latest_control_profile_;
  state.owner_identity = this;
  state.schema_version = 1U;
  state.opaque = snapshot;
  return state;
}

void MutableRadarContext::RestoreRuntimeState(const extension::RadarContextRuntimeState& state) {
  if (state.owner_identity == this && state.schema_version == 1U) {
    const std::shared_ptr<RuntimeSnapshot> snapshot =
        std::static_pointer_cast<RuntimeSnapshot>(state.opaque);
    if (snapshot != nullptr) {
      target_features_ = snapshot->target_features;
      platform_attitude_deg_ = snapshot->platform_attitude_deg;
      cycle_dt_sec_ = snapshot->cycle_dt_sec;
      submitted_commands_ = snapshot->submitted_commands;
      latest_control_profile_ = snapshot->latest_control_profile;
      has_latest_control_profile_ = snapshot->has_latest_control_profile;
      return;
    }
  }

  target_features_.reset(new model::TargetFeatureList(state.target_features));
  platform_attitude_deg_ = state.platform_attitude_deg;
  cycle_dt_sec_ = state.cycle_dt_sec;
  submitted_commands_ = state.submitted_commands;
  latest_control_profile_ = state.latest_control_profile;
  has_latest_control_profile_ = state.has_latest_control_profile;
}

const model::TargetFeatureList& MutableRadarContext::GetTargetFeatures() const {
  static const model::TargetFeatureList kEmptyTargetFeatures;
  return target_features_ != nullptr ? *target_features_ : kEmptyTargetFeatures;
}

model::PlatformAttitudeDeg MutableRadarContext::GetPlatformAttitude() const {
  return platform_attitude_deg_;
}

float MutableRadarContext::GetCycleDeltaTimeSec() const { return cycle_dt_sec_; }

void MutableRadarContext::SubmitControlCommand(extension::control::RadarCommand cmd) {
  submitted_commands_.push_back(std::move(cmd));
}

void MutableRadarContext::UpdateRadarControlProfile(
    const extension::control::RadarControlProfile& profile) {
  latest_control_profile_ = profile;
  has_latest_control_profile_ = true;
}

}  // namespace session
}  // namespace airborne_radar
