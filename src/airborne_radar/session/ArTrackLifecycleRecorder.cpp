#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"

#include <unordered_map>
#include <utility>

namespace airborne_radar {
namespace session {

namespace {

struct TargetState {
  bool confirmed{false};
  std::string target_name{};
};

const session::TrackStateSnapshot* FindTrackByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (const session::TrackStateSnapshot& track : frame.tracks) {
    if (track.external_target_id == external_target_id && external_target_id != 0U) {
      return &track;
    }
  }
  return nullptr;
}

ArTrackLifecycleReason InferReason(const session::TrackStateSnapshot* track) {
  if (track == nullptr) {
    return ArTrackLifecycleReason::kNoTrack;
  }
  return ArTrackLifecycleReason::kUnknown;
}

ArTrackLifecycleEvent MakeBaseEvent(const ArTargetInput& target,
                                    const ArCycleResult& result) {
  ArTrackLifecycleEvent event;
  event.world_cycle_index = result.input_cycle_index;
  event.external_target_id = target.target_id;
  event.target_name = target.target_name;
  return event;
}

}  // namespace

struct ArTrackLifecycleRecorder::Impl {
  ArTrackLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, TargetState> states;
};

ArTrackLifecycleRecorder::ArTrackLifecycleRecorder(ArTrackLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

ArTrackLifecycleRecorder::~ArTrackLifecycleRecorder() = default;

ArTrackLifecycleRecorder::ArTrackLifecycleRecorder(ArTrackLifecycleRecorder&&) noexcept = default;
ArTrackLifecycleRecorder& ArTrackLifecycleRecorder::operator=(ArTrackLifecycleRecorder&&) noexcept =
    default;

std::vector<ArTrackLifecycleEvent> ArTrackLifecycleRecorder::Update(
    const ArTargetInputList& targets, const ArCycleResult& result) {
  std::vector<ArTrackLifecycleEvent> events;
  if (result.status != ArCycleStatus::kCompleted) {
    return events;
  }
  events.reserve(targets.size());
  for (const ArTargetInput& target : targets) {
    // external_target_id 为 0 的输入目标无法按 ID 关联，跳过生命周期记录。
    if (target.target_id == 0U) {
      continue;
    }
    TargetState& state = impl_->states[target.target_id];
    const session::TrackStateSnapshot* track =
        FindTrackByExternalTargetId(result.track_output_frame, target.target_id);
    const bool confirmed_now =
        track != nullptr && track->status == session::TrackStatus::kConfirmed;

    if (confirmed_now) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.confirmed ? ArTrackLifecycleEventKind::kUpdated
                                   : ArTrackLifecycleEventKind::kFirstConfirmed;
      event.reason = ArTrackLifecycleReason::kNone;
      event.association_key = track->association_key;
      event.track_status = track->status;
      event.speed = track->speed;
      events.push_back(event);
      state.confirmed = true;
      state.target_name = target.target_name;
      continue;
    }

    const bool lost_now = track != nullptr && track->status == session::TrackStatus::kLost;
    if (lost_now && state.confirmed) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = ArTrackLifecycleEventKind::kLost;
      event.reason = ArTrackLifecycleReason::kNone;
      event.association_key = track->association_key;
      event.track_status = track->status;
      event.speed = track->speed;
      events.push_back(event);
    } else if (!state.confirmed && impl_->config.emit_not_tracked_events) {
      ArTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = ArTrackLifecycleEventKind::kNotTracked;
      event.reason = InferReason(track);
      if (track != nullptr) {
        event.association_key = track->association_key;
        event.track_status = track->status;
        event.speed = track->speed;
      }
      events.push_back(event);
    }

    // 进入 lost 后不再视为已确认，后续若无 track 可重新触发首次确认。
    if (lost_now) {
      state.confirmed = false;
    }
    state.target_name = target.target_name;
  }
  return events;
}

void ArTrackLifecycleRecorder::Reset() { impl_->states.clear(); }

}  // namespace session
}  // namespace airborne_radar
