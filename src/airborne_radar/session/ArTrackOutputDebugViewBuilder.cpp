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
  // 输入实体回填（规则 12）：无轨迹/未完成周期时以 input 侧 RCS 真值填充
  // （轨迹位置为雷达局部系，input 为 ECEF，不跨参考系回填）。
  state.rcs = target.rcs;
  if (cycle_result.status != ArCycleStatus::kCompleted) {
    state.status = ArDebugTrackStatus::kCycleNotCompleted;
    return state;
  }
  const session::TrackStateSnapshot* track =
      FindTrackByExternalTargetId(cycle_result.output_frame, target.target_id);
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

ArTrackOutputDebugView ArTrackOutputDebugViewBuilder::Build(const ArCycleInput& input,
                                                            const ArCycleResult& result) {
  ArTrackOutputDebugView view;
  view.world_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.output_frame.cycle_index;
  view.completed_this_cycle = result.status == ArCycleStatus::kCompleted;
  view.receiver_impairment = result.receiver_impairment;
  view.issues = result.issues;
  // STT 指定航迹状态直接转写 L2 结果字段（只读组合，不反向影响 pipeline）。
  view.effective_work_mode = result.effective_work_mode;
  view.designation_active = result.designation_active;
  view.designated_target_id = result.designated_target_id;
  view.designation_reverted_to_tws = result.designation_reverted_to_tws;
  view.designation_revert_reason = result.designation_revert_reason;
  view.tracks.reserve(input.targets.size());
  for (const ArTargetInput& target : input.targets) {
    view.tracks.push_back(BuildTrackState(target, result));
  }
  return view;
}

}  // namespace session
}  // namespace airborne_radar
