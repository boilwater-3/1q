// Copyright 2026. All Rights Reserved.
//
// Description: TrackLifecycleManager 的实现。

#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace tracking {

TrackLifecycleManager::TrackLifecycleManager(ITrackPool &pool,
                                             const LifecycleConfig &config)
    : pool_(pool), config_(config), next_track_id_(1), tracks_by_key_() {}

void TrackLifecycleManager::Update(
    const CycleContext &cycle,
    const std::vector<TrackMeasurement> &measurements) {
  std::map<std::uint64_t, bool> hit_flags;

  for (std::vector<TrackMeasurement>::const_iterator it = measurements.begin();
       it != measurements.end(); ++it) {
    const TrackMeasurement &measurement = *it;

    std::map<std::uint64_t, common::TrackState *>::iterator found =
        tracks_by_key_.find(measurement.association_key);

    common::TrackState *track = nullptr;
    if (found == tracks_by_key_.end()) {
      track = pool_.Acquire();
      if (track == nullptr) {
        continue;
      }

      ResetForReuse(*track);
      track->track_id = next_track_id_++;
      track->batch_id = cycle.batch_id;
      track->status = common::TrackStatus::kTentative;
      track->first_cycle = cycle.cycle_index;
      track->last_update_cycle = cycle.cycle_index;
      track->hit_count = 1;
      track->miss_count = 0;

      tracks_by_key_[measurement.association_key] = track;
    } else {
      track = found->second;
      track->last_update_cycle = cycle.cycle_index;
      track->hit_count += 1;
      track->miss_count = 0;
    }

    track->position = measurement.position;
    track->velocity = measurement.velocity;
    track->acceleration = measurement.acceleration;
    track->rcs = measurement.rcs;
    track->jamming_detected = measurement.jamming_detected;

    PromoteState(*track, cycle.cycle_index, true);
    hit_flags[measurement.association_key] = true;
  }

  std::vector<std::uint64_t> keys_to_recycle;
  for (std::map<std::uint64_t, common::TrackState *>::iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const std::uint64_t key = it->first;
    common::TrackState *track = it->second;

    if (hit_flags.find(key) != hit_flags.end()) {
      continue;
    }

    PromoteState(*track, cycle.cycle_index, false);

    if (track->status == common::TrackStatus::kRecycled) {
      keys_to_recycle.push_back(key);
    }
  }

  for (std::vector<std::uint64_t>::const_iterator it = keys_to_recycle.begin();
       it != keys_to_recycle.end(); ++it) {
    std::map<std::uint64_t, common::TrackState *>::iterator found =
        tracks_by_key_.find(*it);
    if (found == tracks_by_key_.end()) {
      continue;
    }

    ResetForReuse(*found->second);
    pool_.Release(found->second);
    tracks_by_key_.erase(found);
  }
}

std::vector<const common::TrackState *> TrackLifecycleManager::GetActiveTracks()
    const {
  std::vector<const common::TrackState *> result;
  result.reserve(tracks_by_key_.size());

  for (std::map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const common::TrackState *track = it->second;
    if (track->status != common::TrackStatus::kRecycled) {
      result.push_back(track);
    }
  }

  return result;
}

common::TargetFeatureList TrackLifecycleManager::BuildFeatureSnapshot() const {
  common::TargetFeatureList features;
  features.reserve(tracks_by_key_.size());

  for (std::map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const common::TrackState *track = it->second;
    if (track->status == common::TrackStatus::kRecycled) {
      continue;
    }

    const float speed = track->velocity.norm();
    const float accel = track->acceleration.norm();
    features.push_back(common::TargetFeature(speed, track->rcs,
                                             track->jamming_detected, accel));
  }

  return features;
}

void TrackLifecycleManager::PromoteState(common::TrackState &track,
                                         std::uint32_t cycle_index,
                                         bool hit_this_cycle) {
  if (hit_this_cycle) {
    if (track.status == common::TrackStatus::kTentative &&
        track.hit_count >= config_.confirm_hits) {
      track.status = common::TrackStatus::kConfirmed;
    } else if (track.status == common::TrackStatus::kLost) {
      track.status = common::TrackStatus::kConfirmed;
    }
    return;
  }

  track.miss_count += 1;
  if (track.status == common::TrackStatus::kTentative ||
      track.status == common::TrackStatus::kConfirmed) {
    if (track.miss_count > config_.max_miss_before_lost) {
      track.status = common::TrackStatus::kLost;
    }
    return;
  }

  if (track.status == common::TrackStatus::kLost) {
    const std::uint32_t lost_cycles =
        cycle_index >= track.last_update_cycle
            ? (cycle_index - track.last_update_cycle)
            : 0;
    if (lost_cycles > config_.max_lost_cycles) {
      track.status = common::TrackStatus::kRecycled;
      track.generation += 1;
    }
  }
}

void TrackLifecycleManager::ResetForReuse(common::TrackState &track) const {
  track.batch_id = 0;
  track.status = common::TrackStatus::kTentative;
  track.first_cycle = 0;
  track.last_update_cycle = 0;
  track.miss_count = 0;
  track.hit_count = 0;
  track.position.setZero();
  track.velocity.setZero();
  track.acceleration.setZero();
  track.rcs = 0.0f;
  track.jamming_detected = false;
  track.covariance.clear();
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
