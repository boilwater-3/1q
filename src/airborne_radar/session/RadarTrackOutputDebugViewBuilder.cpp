#include "1q/airborne_radar/session/RadarTrackOutputDebugView.h"

#include "1q/airborne_radar/session/RadarCycleInput.h"

namespace airborne_radar {
namespace session {

namespace {

RadarDebugTrackStatus ToDebugStatus(model::TrackStatus status) {
  switch (status) {
    case model::TrackStatus::kConfirmed:
      return RadarDebugTrackStatus::kConfirmed;
    case model::TrackStatus::kLost:
      return RadarDebugTrackStatus::kLost;
    case model::TrackStatus::kTentative:
    default:
      return RadarDebugTrackStatus::kTentative;
  }
}

const model::TrackStateSnapshot* FindTrackByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (const model::TrackStateSnapshot& track : frame.tracks) {
    if (track.external_target_id == external_target_id && external_target_id != 0U) {
      return &track;
    }
  }
  return nullptr;
}

RadarDebugTrackState BuildTrackState(const RadarSceneTarget& target, const RadarCycleResult& cycle_result) {
  RadarDebugTrackState state;
  state.external_target_id = target.external_target_id;
  state.target_name = target.target_name;
  state.present_in_input = true;
  if (!cycle_result.executed_this_cycle) {
    state.status = RadarDebugTrackStatus::kCycleNotExecuted;
    return state;
  }
  const model::TrackStateSnapshot* track =
      FindTrackByExternalTargetId(cycle_result.track_output_frame, target.external_target_id);
  if (track == nullptr) {
    // external_target_id 为 0（未知）的输入目标无法按 ID 关联到 track。
    state.status = RadarDebugTrackStatus::kNotInOutput;
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

RadarTrackOutputDebugView RadarTrackOutputDebugViewBuilder::Build(const RadarCycleInput& input,
                                                                   const RadarCycleResult& result) {
  RadarTrackOutputDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.track_output_frame.cycle_index;
  view.executed_this_cycle = result.executed_this_cycle;
  view.reused_previous_output = result.reused_previous_output;
  view.has_validation_error = result.has_validation_error;
  view.abort_reason = result.abort_reason;
  view.tracks.reserve(input.scene.size());
  for (const RadarSceneTarget& target : input.scene) {
    view.tracks.push_back(BuildTrackState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace airborne_radar
