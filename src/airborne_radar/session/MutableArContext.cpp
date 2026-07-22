#include "airborne_radar/session/MutableArContext.h"

#include <utility>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace session {

struct ArContextRuntimeIdentity {};

struct ArContextRuntimeSnapshot {
  std::shared_ptr<ArSceneTargetList> scene_targets;
  oneq::foundation::PoseState platform_pose{};
  float platform_altitude_m{0.0f};
  float cycle_dt_sec{1.0f};
  std::uint32_t cycle_index{0U};
  std::vector<session::ArCommand> submitted_commands{};
  session::ArControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

MutableArContext::MutableArContext()
    : owner_identity_(new ArContextRuntimeIdentity()) {}

MutableArContext::MutableArContext(ArSceneTargetList scene_targets)
    : owner_identity_(new ArContextRuntimeIdentity()),
      scene_targets_(new ArSceneTargetList(std::move(scene_targets))),
      cycle_index_(1U) {}

void MutableArContext::BeginCycle(ArSceneTargetList scene_targets,
                                  const oneq::foundation::PoseState& platform_pose,
                                  float platform_altitude_m, float dt_sec,
                                  std::uint32_t cycle_index) {
  SetSceneTargets(std::move(scene_targets));
  platform_pose_ = platform_pose;
  platform_altitude_m_ = platform_altitude_m;
  SetCycleDeltaTimeSec(dt_sec);
  cycle_index_ = cycle_index;
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
  std::shared_ptr<ArContextRuntimeSnapshot> snapshot(new ArContextRuntimeSnapshot());
  snapshot->scene_targets = scene_targets_;
  snapshot->platform_pose = platform_pose_;
  snapshot->platform_altitude_m = platform_altitude_m_;
  snapshot->cycle_dt_sec = cycle_dt_sec_;
  snapshot->cycle_index = cycle_index_;
  snapshot->submitted_commands = submitted_commands_;
  snapshot->latest_control_profile = latest_control_profile_;
  snapshot->has_latest_control_profile = has_latest_control_profile_;
  return ArContextRuntimeState(owner_identity_, std::move(snapshot));
}

bool MutableArContext::RestoreRuntimeState(const ArContextRuntimeState& state) {
  if (state.owner_identity_ != owner_identity_ || state.snapshot_ == nullptr) {
    PROJECT_LOG_ERROR(
        "[MutableArContext] context runtime state restore rejected: "
        "owner/snapshot mismatch.");
    return false;
  }

  scene_targets_ = state.snapshot_->scene_targets;
  platform_pose_ = state.snapshot_->platform_pose;
  platform_altitude_m_ = state.snapshot_->platform_altitude_m;
  cycle_dt_sec_ = state.snapshot_->cycle_dt_sec;
  cycle_index_ = state.snapshot_->cycle_index;
  submitted_commands_ = state.snapshot_->submitted_commands;
  latest_control_profile_ = state.snapshot_->latest_control_profile;
  has_latest_control_profile_ = state.snapshot_->has_latest_control_profile;
  return true;
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
