// Copyright 2026. All Rights Reserved.
//
// Description: TrackLifecycleManager 的实现。

#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"

#include <algorithm>

#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

TrackLifecycleManager::TrackLifecycleManager(ITrackPool &pool,
                                             const LifecycleConfig &config,
                                             const IKalmanPredictor *predictor,
                                             const IKalmanUpdater *updater)
    : pool_(pool), config_(config), next_track_id_(1), tracks_by_key_(),
      kalman_predictor_(predictor), kalman_updater_(updater) {}

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

    if (measurement.has_cartesian_position) {
      track->position = measurement.position;
    }

    if (measurement.velocity != Eigen::Vector3f::Zero()) {
      track->velocity = measurement.velocity;
    } else {
      track->velocity = Eigen::Vector3f(measurement.observed_speed, 0.0f, 0.0f);
    }

    if (measurement.acceleration != Eigen::Vector3f::Zero()) {
      track->acceleration = measurement.acceleration;
    } else {
      track->acceleration =
          Eigen::Vector3f(measurement.observed_acceleration, 0.0f, 0.0f);
    }
    track->rcs = measurement.rcs;
    track->jamming_detected = measurement.jamming_detected;

    PromoteState(*track, cycle.cycle_index, true);
    hit_flags[measurement.association_key] = true;

    // ---- Kalman 滤波集成 ----
    if (kalman_predictor_ != nullptr && kalman_updater_ != nullptr &&
        measurement.has_cartesian_position) {
      if (!measurement.matched_existing_track) {
        // 新轨迹：从量测初始化高斯状态
        GaussianTrackState init;
        init.mean(0) = measurement.position(0);  // x
        init.mean(1) = measurement.velocity(0);   // vx
        init.mean(2) = measurement.position(1);  // y
        init.mean(3) = measurement.velocity(1);   // vy
        init.mean(4) = measurement.position(2);  // z
        init.mean(5) = measurement.velocity(2);   // vz
        init.covariance = StateCovariance::Identity() * 100.0f;
        track->gaussian_state = init;
      } else {
        // 已有轨迹：predict → update
        GaussianTrackState predicted =
            kalman_predictor_->Predict(track->gaussian_state, cycle_dt_);
        MeasurementVector z;
        z(0) = measurement.position(0);
        z(1) = measurement.position(1);
        z(2) = measurement.position(2);
        KalmanUpdateResult result = kalman_updater_->Update(predicted, z);
        track->gaussian_state = result.posterior;
      }

      // 将 Kalman 后验均值写回 TrackState 的位置/速度字段
      track->position(0) = track->gaussian_state.mean(0);
      track->position(1) = track->gaussian_state.mean(2);
      track->position(2) = track->gaussian_state.mean(4);
      track->velocity(0) = track->gaussian_state.mean(1);
      track->velocity(1) = track->gaussian_state.mean(3);
      track->velocity(2) = track->gaussian_state.mean(5);
    }
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
    } else if (kalman_predictor_ != nullptr) {
      // 未命中的轨迹也需要做预测（协方差膨胀，均值外推）
      track->gaussian_state =
          kalman_predictor_->Predict(track->gaussian_state, cycle_dt_);
      track->position(0) = track->gaussian_state.mean(0);
      track->position(1) = track->gaussian_state.mean(2);
      track->position(2) = track->gaussian_state.mean(4);
      track->velocity(0) = track->gaussian_state.mean(1);
      track->velocity(1) = track->gaussian_state.mean(3);
      track->velocity(2) = track->gaussian_state.mean(5);
    }
  }

  // 更新周期时间信息
  if (cycle.cycle_index > last_cycle_index_ && last_cycle_index_ > 0) {
    cycle_dt_ = static_cast<float>(cycle.cycle_index - last_cycle_index_);
  }
  last_cycle_index_ = cycle.cycle_index;

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
  track.gaussian_state = GaussianTrackState();
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
