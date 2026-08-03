#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"

namespace airborne_radar {
namespace session {

namespace {

ArDebugTrackStatus ToDebugStatus(session::TrackStatus status) {
  switch (status) {
    case session::TrackStatus::kConfirmed:
      return ArDebugTrackStatus::kConfirmed;
    case session::TrackStatus::kLost:
      return ArDebugTrackStatus::kLost;
    case session::TrackStatus::kTentative:
    default:
      return ArDebugTrackStatus::kTentative;
  }
}

const session::TrackStateSnapshot* FindTrackByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (const session::TrackStateSnapshot& track : frame.tracks) {
    if (track.external_target_id == external_target_id && external_target_id != 0U) {
      return &track;
    }
  }
  return nullptr;
}

ArDebugTrackState BuildTrackState(const ArTargetInput& target,
                                  const ArCycleResult& cycle_result) {
  ArDebugTrackState state;
  state.external_target_id = target.target_id;
  state.target_name = target.target_name;
  state.present_in_input = true;
  if (cycle_result.status != ArCycleStatus::kCompleted) {
    state.status = ArDebugTrackStatus::kCycleNotCompleted;
    return state;
  }
  const session::TrackStateSnapshot* track =
      FindTrackByExternalTargetId(cycle_result.track_output_frame, target.target_id);
  if (track == nullptr) {
    // external_target_id 为 0（未知）的输入目标无法按 ID 关联到 track。
    state.status = ArDebugTrackStatus::kNotInOutput;
    return state;
  }
  state.has_track = true;
  state.association_key = track->association_key;
  state.status = ToDebugStatus(track->status);
  state.position_x = track->position_x;
  state.position_y = track->position_y;
  state.position_z = track->position_z;
  state.speed = track->speed;
  state.rcs = track->rcs;
  state.hit_count = track->hit_count;
  state.miss_count = track->miss_count;
  state.target_type = track->target_type;
  return state;
}

}  // namespace

ArTrackOutputDebugView ArTrackOutputDebugViewBuilder::Build(const ArTargetInputList& targets,
                                                            const ArCycleResult& result) {
  ArTrackOutputDebugView view;
  view.world_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.track_output_frame.cycle_index;
  view.completed_this_cycle = result.status == ArCycleStatus::kCompleted;
  view.receiver_impairment = result.receiver_impairment;
  view.diagnostics = result.diagnostics;
  view.tracks.reserve(targets.size());
  for (const ArTargetInput& target : targets) {
    view.tracks.push_back(BuildTrackState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace airborne_radar
