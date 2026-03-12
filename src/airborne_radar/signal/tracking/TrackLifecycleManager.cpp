// Copyright 2026. All Rights Reserved.
//
// Description: TrackLifecycleManager 的实现。

#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"

#include <algorithm>

#include "airborne_radar/signal/tracking/ImmFilter.h"
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

TrackLifecycleManager::TrackLifecycleManager(
    ITrackPool &pool, const LifecycleConfig &config,
    const std::vector<const IKalmanPredictor *> &imm_predictors,
    const std::vector<const IKalmanUpdater *> &imm_updaters,
    const Eigen::MatrixXf &imm_transition_probability,
    const Eigen::VectorXf &imm_initial_weights)
    : pool_(pool), config_(config), next_track_id_(1), tracks_by_key_(),
      imm_predictors_(imm_predictors), imm_updaters_(imm_updaters),
      imm_transition_probability_(imm_transition_probability),
      imm_initial_weights_(imm_initial_weights) {
  const int num_models = static_cast<int>(imm_predictors_.size());
  if (num_models <= 0 || imm_updaters_.size() != imm_predictors_.size()) {
    imm_predictors_.clear();
    imm_updaters_.clear();
    imm_transition_probability_.resize(0, 0);
    imm_initial_weights_.resize(0);
    return;
  }

  if (imm_transition_probability_.rows() != num_models ||
      imm_transition_probability_.cols() != num_models) {
    imm_transition_probability_ = Eigen::MatrixXf::Constant(
        num_models, num_models,
        num_models > 1 ? 0.05f / static_cast<float>(num_models - 1) : 1.0f);
    imm_transition_probability_.diagonal().setConstant(num_models > 1 ? 0.95f : 1.0f);
  }

  if (imm_initial_weights_.size() != num_models) {
    imm_initial_weights_ = Eigen::VectorXf::Constant(
        num_models, 1.0f / static_cast<float>(num_models));
  }
}

TrackLifecycleManager::~TrackLifecycleManager() = default;

bool TrackLifecycleManager::IsImmEnabled() const {
  return !imm_predictors_.empty() && imm_predictors_.size() == imm_updaters_.size() &&
         imm_transition_probability_.rows() ==
             static_cast<Eigen::Index>(imm_predictors_.size()) &&
         imm_initial_weights_.size() ==
             static_cast<Eigen::Index>(imm_predictors_.size());
}

GaussianTrackState TrackLifecycleManager::BuildInitialGaussianState(
    const TrackMeasurement &measurement) const {
  GaussianTrackState init;
  init.mean(0) = measurement.position(0);
  init.mean(1) = measurement.velocity(0);
  init.mean(2) = measurement.position(1);
  init.mean(3) = measurement.velocity(1);
  init.mean(4) = measurement.position(2);
  init.mean(5) = measurement.velocity(2);
  init.covariance = StateCovariance::Identity() * 100.0f;
  return init;
}

ImmFilter *TrackLifecycleManager::GetOrCreateImmFilter(
    std::uint64_t association_key, const GaussianTrackState &initial_state) {
  std::map<std::uint64_t, std::unique_ptr<ImmFilter> >::iterator found =
      imm_filters_by_key_.find(association_key);
  if (found != imm_filters_by_key_.end()) {
    return found->second.get();
  }

  ImmConfig config;
  config.transition_probability = imm_transition_probability_;
  config.initial_weights = imm_initial_weights_;

  std::unique_ptr<ImmFilter> filter(
      new ImmFilter(config, imm_predictors_, imm_updaters_));
  std::vector<ImmModelState> initial_states;
  initial_states.reserve(imm_predictors_.size());
  for (std::size_t i = 0; i < imm_predictors_.size(); ++i) {
    initial_states.push_back(
        ImmModelState(initial_state, imm_initial_weights_(static_cast<Eigen::Index>(i))));
  }
  filter->SetModelStates(initial_states);
  ImmFilter *filter_ptr = filter.get();
  imm_filters_by_key_[association_key] = std::move(filter);
  return filter_ptr;
}

void TrackLifecycleManager::ApplyGaussianState(common::TrackState &track,
                                               const GaussianTrackState &state) const {
  track.gaussian_state = state;
  track.position(0) = state.mean(0);
  track.position(1) = state.mean(2);
  track.position(2) = state.mean(4);
  track.velocity(0) = state.mean(1);
  track.velocity(1) = state.mean(3);
  track.velocity(2) = state.mean(5);
}

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

    // ---- Kalman/IMM 滤波集成 ----
    if (measurement.has_cartesian_position) {
      if (IsImmEnabled()) {
        const GaussianTrackState initial_state = BuildInitialGaussianState(measurement);
        ImmFilter *imm_filter = GetOrCreateImmFilter(
            measurement.association_key,
            measurement.matched_existing_track ? track->gaussian_state : initial_state);
        if (!measurement.matched_existing_track) {
          ApplyGaussianState(*track, initial_state);
        } else {
          MeasurementVector z;
          z(0) = measurement.position(0);
          z(1) = measurement.position(1);
          z(2) = measurement.position(2);
          imm_filter->Process(z, cycle_dt_);
          ApplyGaussianState(*track, imm_filter->GetCombinedState());
        }
      } else if (kalman_predictor_ != nullptr && kalman_updater_ != nullptr) {
        if (!measurement.matched_existing_track) {
          ApplyGaussianState(*track, BuildInitialGaussianState(measurement));
        } else {
          GaussianTrackState predicted =
              kalman_predictor_->Predict(track->gaussian_state, cycle_dt_);
          MeasurementVector z;
          z(0) = measurement.position(0);
          z(1) = measurement.position(1);
          z(2) = measurement.position(2);
          KalmanUpdateResult result =
              kalman_updater_->Update(predicted, z,
                                      measurement.measurement_covariance);
          ApplyGaussianState(*track, result.posterior);
        }
      }
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
      imm_filters_by_key_.erase(key);
      keys_to_recycle.push_back(key);
    } else if (IsImmEnabled()) {
      std::map<std::uint64_t, std::unique_ptr<ImmFilter> >::iterator imm_found =
          imm_filters_by_key_.find(key);
      if (imm_found != imm_filters_by_key_.end()) {
        imm_found->second->Predict(cycle_dt_);
        ApplyGaussianState(*track, imm_found->second->GetCombinedState());
      }
    } else if (kalman_predictor_ != nullptr) {
      // 未命中的轨迹也需要做预测（协方差膨胀，均值外推）
      ApplyGaussianState(
          *track, kalman_predictor_->Predict(track->gaussian_state, cycle_dt_));
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
    common::TargetFeature feature(speed, track->rcs,
                    track->jamming_detected, accel);
    feature.position_x = track->position(0);
    feature.position_y = track->position(1);
    feature.position_z = track->position(2);
    features.push_back(feature);
  }

  return features;
}

std::vector<AssociationTrackSeed> TrackLifecycleManager::BuildAssociationSeeds()
    const {
  std::vector<AssociationTrackSeed> seeds;
  seeds.reserve(tracks_by_key_.size());

  for (std::map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const common::TrackState *track = it->second;
    if (track->status == common::TrackStatus::kRecycled) {
      continue;
    }

    AssociationTrackSeed seed;
    seed.association_key = it->first;
    seed.legacy_feature =
        Eigen::Vector3f(track->velocity.norm(), track->rcs,
                        track->acceleration.norm());
    seed.has_position = true;
    seed.position = track->position;
    seed.has_gaussian_state = true;
    seed.gaussian_state = track->gaussian_state;
    seeds.push_back(seed);
  }

  return seeds;
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
