#include "airborne_radar/session/MutableRadarContext.h"

#include <utility>

namespace airborne_radar {
namespace session {

struct MutableRadarContext::RuntimeSnapshot {
  std::shared_ptr<RadarSceneTargetList> scene_targets;
  oneq::foundation::PoseState platform_pose{};
  float platform_altitude_m{0.0f};
  float cycle_dt_sec{1.0f};
  std::uint32_t cycle_index{0U};
  std::vector<extension::control::RadarCommand> submitted_commands{};
  extension::control::RadarControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

MutableRadarContext::MutableRadarContext(RadarSceneTargetList scene_targets)
    : scene_targets_(new RadarSceneTargetList(std::move(scene_targets))), cycle_index_(1U) {}

void MutableRadarContext::BeginCycle(const RadarCycleInput& input) {
  SetSceneTargets(input.scene);
  platform_pose_ = input.platform_pose;
  platform_altitude_m_ = input.platform_altitude_m;
  SetCycleDeltaTimeSec(input.dt_sec);
  cycle_index_ = input.cycle_index;
  ResetCycleOutputs();
}

void MutableRadarContext::SetSceneTargets(RadarSceneTargetList scene_targets) {
  scene_targets_.reset(new RadarSceneTargetList(std::move(scene_targets)));
}

void MutableRadarContext::SetPlatformAttitude(
    const model::PlatformAttitudeDeg& platform_attitude_deg) {
  platform_pose_.attitude_deg = platform_attitude_deg;
}

void MutableRadarContext::SetCycleDeltaTimeSec(float dt_sec) { cycle_dt_sec_ = dt_sec; }

void MutableRadarContext::SetCycleIndex(std::uint32_t cycle_index) { cycle_index_ = cycle_index; }

void MutableRadarContext::ResetCycleOutputs() { submitted_commands_.clear(); }

const std::vector<extension::control::RadarCommand>& MutableRadarContext::GetSubmittedCommands()
    const {
  return submitted_commands_;
}

bool MutableRadarContext::HasLatestControlProfile() const { return has_latest_control_profile_; }

const extension::control::RadarControlProfile& MutableRadarContext::GetLatestControlProfile()
    const {
  return latest_control_profile_;
}

const std::vector<extension::control::RadarCommand>& MutableRadarContext::SubmittedCommands()
    const {
  return submitted_commands_;
}

const extension::control::RadarControlProfile& MutableRadarContext::LatestControlProfile() const {
  return latest_control_profile_;
}

RadarContextRuntimeState MutableRadarContext::CaptureRuntimeState() const {
  RadarContextRuntimeState state;
  std::shared_ptr<RuntimeSnapshot> snapshot(new RuntimeSnapshot());
  snapshot->scene_targets = scene_targets_;
  snapshot->platform_pose = platform_pose_;
  snapshot->platform_altitude_m = platform_altitude_m_;
  snapshot->cycle_dt_sec = cycle_dt_sec_;
  snapshot->cycle_index = cycle_index_;
  snapshot->submitted_commands = submitted_commands_;
  snapshot->latest_control_profile = latest_control_profile_;
  snapshot->has_latest_control_profile = has_latest_control_profile_;
  state.owner_identity = this;
  state.schema_version = 1U;
  state.opaque = snapshot;
  state.scene_targets = scene_targets_ != nullptr ? *scene_targets_ : RadarSceneTargetList();
  state.platform_pose = platform_pose_;
  state.platform_altitude_m = platform_altitude_m_;
  state.cycle_dt_sec = cycle_dt_sec_;
  state.cycle_index = cycle_index_;
  state.submitted_commands = submitted_commands_;
  state.latest_control_profile = latest_control_profile_;
  state.has_latest_control_profile = has_latest_control_profile_;
  return state;
}

void MutableRadarContext::RestoreRuntimeState(const RadarContextRuntimeState& state) {
  if (state.owner_identity == this && state.schema_version == 1U) {
    const std::shared_ptr<RuntimeSnapshot> snapshot =
        std::static_pointer_cast<RuntimeSnapshot>(state.opaque);
    if (snapshot != nullptr) {
      scene_targets_ = snapshot->scene_targets;
      platform_pose_ = snapshot->platform_pose;
      platform_altitude_m_ = snapshot->platform_altitude_m;
      cycle_dt_sec_ = snapshot->cycle_dt_sec;
      cycle_index_ = snapshot->cycle_index;
      submitted_commands_ = snapshot->submitted_commands;
      latest_control_profile_ = snapshot->latest_control_profile;
      has_latest_control_profile_ = snapshot->has_latest_control_profile;
      return;
    }
  }

  scene_targets_.reset(new RadarSceneTargetList(state.scene_targets));
  platform_pose_ = state.platform_pose;
  platform_altitude_m_ = state.platform_altitude_m;
  cycle_dt_sec_ = state.cycle_dt_sec;
  cycle_index_ = state.cycle_index;
  submitted_commands_ = state.submitted_commands;
  latest_control_profile_ = state.latest_control_profile;
  has_latest_control_profile_ = state.has_latest_control_profile;
}

const RadarSceneTargetList& MutableRadarContext::GetSceneTargets() const {
  static const RadarSceneTargetList kEmptySceneTargets;
  return scene_targets_ != nullptr ? *scene_targets_ : kEmptySceneTargets;
}

model::PlatformAttitudeDeg MutableRadarContext::GetPlatformAttitude() const {
  return platform_pose_.attitude_deg;
}

float MutableRadarContext::GetPlatformAltitudeM() const { return platform_altitude_m_; }

float MutableRadarContext::GetCycleDeltaTimeSec() const { return cycle_dt_sec_; }

std::uint32_t MutableRadarContext::GetCycleIndex() const { return cycle_index_; }

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
