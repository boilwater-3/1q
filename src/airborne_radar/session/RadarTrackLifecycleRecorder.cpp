#include "1q/airborne_radar/session/RadarTrackLifecycleRecorder.h"

#include <unordered_map>
#include <utility>

#include "1q/airborne_radar/session/RadarCycleInput.h"

namespace airborne_radar {
namespace session {

namespace {

struct TargetState {
  bool confirmed{false};
  std::string target_name{};
};

const model::TrackStateSnapshot* FindTrackByExternalTargetId(
    const TrackOutputFrame& frame, std::uint64_t external_target_id) {
  for (const model::TrackStateSnapshot& track : frame.tracks) {
    if (track.external_target_id == external_target_id && external_target_id != 0U) {
      return &track;
    }
  }
  return nullptr;
}

RadarTrackLifecycleReason InferReason(const RadarCycleResult& result, const model::TrackStateSnapshot* track) {
  if (result.has_validation_error) {
    return RadarTrackLifecycleReason::kValidationRejected;
  }
  if (!result.executed_this_cycle) {
    return RadarTrackLifecycleReason::kCycleNotExecuted;
  }
  if (track == nullptr) {
    return RadarTrackLifecycleReason::kNoTrack;
  }
  return RadarTrackLifecycleReason::kUnknown;
}

RadarTrackLifecycleEvent MakeBaseEvent(const RadarSceneTarget& target, const RadarCycleResult& result) {
  RadarTrackLifecycleEvent event;
  event.cycle_index = result.input_cycle_index;
  event.external_target_id = target.external_target_id;
  event.target_name = target.target_name;
  return event;
}

}  // namespace

struct RadarTrackLifecycleRecorder::Impl {
  RadarTrackLifecycleRecorderConfig config;
  std::unordered_map<std::uint64_t, TargetState> states;
};

RadarTrackLifecycleRecorder::RadarTrackLifecycleRecorder(RadarTrackLifecycleRecorderConfig config)
    : impl_(new Impl{config, {}}) {}

RadarTrackLifecycleRecorder::~RadarTrackLifecycleRecorder() = default;

RadarTrackLifecycleRecorder::RadarTrackLifecycleRecorder(RadarTrackLifecycleRecorder&&) noexcept = default;
RadarTrackLifecycleRecorder& RadarTrackLifecycleRecorder::operator=(RadarTrackLifecycleRecorder&&) noexcept =
    default;

std::vector<RadarTrackLifecycleEvent> RadarTrackLifecycleRecorder::Update(
    const RadarCycleInput& input, const RadarCycleResult& result) {
  std::vector<RadarTrackLifecycleEvent> events;
  events.reserve(input.scene.size());
  for (const RadarSceneTarget& target : input.scene) {
    // external_target_id 为 0 的输入目标无法按 ID 关联，跳过生命周期记录。
    if (target.external_target_id == 0U) {
      continue;
    }
    TargetState& state = impl_->states[target.external_target_id];
    const model::TrackStateSnapshot* track =
        FindTrackByExternalTargetId(result.track_output_frame, target.external_target_id);
    const bool confirmed_now =
        result.executed_this_cycle && track != nullptr && track->status == model::TrackStatus::kConfirmed;

    if (confirmed_now) {
      RadarTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = state.confirmed ? RadarTrackLifecycleEventKind::kUpdated
                                   : RadarTrackLifecycleEventKind::kFirstConfirmed;
      event.reason = RadarTrackLifecycleReason::kNone;
      event.association_key = track->association_key;
      event.track_status = track->status;
      event.speed = track->speed;
      events.push_back(event);
      state.confirmed = true;
      state.target_name = target.target_name;
      continue;
    }

    const bool lost_now =
        result.executed_this_cycle && track != nullptr && track->status == model::TrackStatus::kLost;
    if (lost_now && state.confirmed) {
      RadarTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = RadarTrackLifecycleEventKind::kLost;
      event.reason = RadarTrackLifecycleReason::kNone;
      event.association_key = track->association_key;
      event.track_status = track->status;
      event.speed = track->speed;
      events.push_back(event);
    } else if (!state.confirmed && impl_->config.emit_not_tracked_events) {
      RadarTrackLifecycleEvent event = MakeBaseEvent(target, result);
      event.kind = RadarTrackLifecycleEventKind::kNotTracked;
      event.reason = InferReason(result, track);
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

void RadarTrackLifecycleRecorder::Reset() { impl_->states.clear(); }

}  // namespace session
}  // namespace airborne_radar
