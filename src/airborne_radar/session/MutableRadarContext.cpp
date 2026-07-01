#include "airborne_radar/session/MutableRadarContext.h"

#include <utility>

namespace airborne_radar {
namespace session {

struct MutableArContext::RuntimeSnapshot {
  std::shared_ptr<ArSceneTargetList> scene_targets;
  oneq::foundation::PoseState platform_pose{};
  float platform_altitude_m{0.0f};
  float cycle_dt_sec{1.0f};
  std::uint32_t cycle_index{0U};
  std::vector<session::ArCommand> submitted_commands{};
  session::ArControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

MutableArContext::MutableArContext(ArSceneTargetList scene_targets)
    : scene_targets_(new ArSceneTargetList(std::move(scene_targets))), cycle_index_(1U) {}

void MutableArContext::BeginCycle(const ArCycleInput& input) {
  SetSceneTargets(input.scene);
  platform_pose_ = input.platform_pose;
  platform_altitude_m_ = input.platform_altitude_m;
  SetCycleDeltaTimeSec(input.dt_sec);
  cycle_index_ = input.cycle_index;
  ResetCycleOutputs();
}

void MutableArContext::SetSceneTargets(ArSceneTargetList scene_targets) {
  scene_targets_.reset(new ArSceneTargetList(std::move(scene_targets)));
}

void MutableArContext::SetPlatformAttitude(
    const config::PlatformAttitudeDeg& platform_attitude_deg) {
  platform_pose_.attitude_deg = platform_attitude_deg;
}

void MutableArContext::SetCycleDeltaTimeSec(float dt_sec) { cycle_dt_sec_ = dt_sec; }

void MutableArContext::SetCycleIndex(std::uint32_t cycle_index) { cycle_index_ = cycle_index; }

void MutableArContext::ResetCycleOutputs() { submitted_commands_.clear(); }

const std::vector<session::ArCommand>& MutableArContext::GetSubmittedCommands()
    const {
  return submitted_commands_;
}

bool MutableArContext::HasLatestControlProfile() const { return has_latest_control_profile_; }

const session::ArControlProfile& MutableArContext::GetLatestControlProfile()
    const {
  return latest_control_profile_;
}

const std::vector<session::ArCommand>& MutableArContext::SubmittedCommands()
    const {
  return submitted_commands_;
}

const session::ArControlProfile& MutableArContext::LatestControlProfile() const {
  return latest_control_profile_;
}

ArContextRuntimeState MutableArContext::CaptureRuntimeState() const {
  ArContextRuntimeState state;
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
  state.scene_targets = scene_targets_ != nullptr ? *scene_targets_ : ArSceneTargetList();
  state.platform_pose = platform_pose_;
  state.platform_altitude_m = platform_altitude_m_;
  state.cycle_dt_sec = cycle_dt_sec_;
  state.cycle_index = cycle_index_;
  state.submitted_commands = submitted_commands_;
  state.latest_control_profile = latest_control_profile_;
  state.has_latest_control_profile = has_latest_control_profile_;
  return state;
}

void MutableArContext::RestoreRuntimeState(const ArContextRuntimeState& state) {
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

  scene_targets_.reset(new ArSceneTargetList(state.scene_targets));
  platform_pose_ = state.platform_pose;
  platform_altitude_m_ = state.platform_altitude_m;
  cycle_dt_sec_ = state.cycle_dt_sec;
  cycle_index_ = state.cycle_index;
  submitted_commands_ = state.submitted_commands;
  latest_control_profile_ = state.latest_control_profile;
  has_latest_control_profile_ = state.has_latest_control_profile;
}

const ArSceneTargetList& MutableArContext::GetSceneTargets() const {
  static const ArSceneTargetList kEmptySceneTargets;
  return scene_targets_ != nullptr ? *scene_targets_ : kEmptySceneTargets;
}

config::PlatformAttitudeDeg MutableArContext::GetPlatformAttitude() const {
  return platform_pose_.attitude_deg;
}

float MutableArContext::GetPlatformAltitudeM() const { return platform_altitude_m_; }

float MutableArContext::GetCycleDeltaTimeSec() const { return cycle_dt_sec_; }

std::uint32_t MutableArContext::GetCycleIndex() const { return cycle_index_; }

void MutableArContext::SubmitControlCommand(session::ArCommand cmd) {
  submitted_commands_.push_back(std::move(cmd));
}

void MutableArContext::UpdateRadarControlProfile(
    const session::ArControlProfile& profile) {
  latest_control_profile_ = profile;
  has_latest_control_profile_ = true;
}

}  // namespace session
}  // namespace airborne_radar
