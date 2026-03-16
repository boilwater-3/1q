// Copyright 2026. All Rights Reserved.
//
// Description: TrackLifecycleManager 的实现。

#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"

#include <algorithm>
#include <unordered_set>

#include <spdlog/spdlog.h>

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
      kalman_predictor_(imm_predictors.empty() ? nullptr : imm_predictors.front()),
      kalman_updater_(imm_updaters.empty() ? nullptr : imm_updaters.front()),
      imm_predictors_(imm_predictors), imm_updaters_(imm_updaters),
      imm_transition_probability_(imm_transition_probability),
      imm_initial_weights_(imm_initial_weights) {
  const int num_models = static_cast<int>(imm_predictors_.size());
  if (num_models <= 0 || imm_updaters_.size() != imm_predictors_.size()) {
    kalman_predictor_ = nullptr;
    kalman_updater_ = nullptr;
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
  init.mean(0) = measurement.raw_measurement.position(0);
  init.mean(1) = measurement.filtered_feature.velocity(0);
  init.mean(2) = measurement.raw_measurement.position(1);
  init.mean(3) = measurement.filtered_feature.velocity(1);
  init.mean(4) = measurement.raw_measurement.position(2);
  init.mean(5) = measurement.filtered_feature.velocity(2);
  init.covariance = StateCovariance::Identity() * 100.0f;
  return init;
}

bool TrackLifecycleManager::ShouldUseImmForMeasurement(
    bool track_existed_before_cycle,
    common::TrackStatus status_before_update,
    const TrackMeasurement &measurement) const {
  if (!IsImmEnabled() || !measurement.raw_measurement.has_cartesian_position) {
    return false;
  }

  if (config_.imm_activation_policy == ImmActivationPolicy::kAllTracks) {
    return true;
  }

  return track_existed_before_cycle &&
         status_before_update == common::TrackStatus::kConfirmed &&
         measurement.raw_measurement.matched_existing_track;
}

bool TrackLifecycleManager::ShouldUseImmForMiss(
    common::TrackStatus status_before_prediction) const {
  if (!IsImmEnabled()) {
    return false;
  }

  if (config_.imm_activation_policy == ImmActivationPolicy::kAllTracks) {
    return true;
  }

  return status_before_prediction == common::TrackStatus::kConfirmed;
}

ImmFilter *TrackLifecycleManager::GetOrCreateImmFilter(
    std::uint64_t association_key, const GaussianTrackState &initial_state) {
  std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter> >::iterator found =
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

ImmFilter *TrackLifecycleManager::FindImmFilter(
    std::uint64_t association_key) const {
  std::unordered_map<std::uint64_t, std::unique_ptr<ImmFilter> >::const_iterator
      found = imm_filters_by_key_.find(association_key);
  if (found == imm_filters_by_key_.end()) {
    return nullptr;
  }
  return found->second.get();
}

void TrackLifecycleManager::ApplyGaussianState(common::TrackState &track,
                                               const GaussianTrackState &state,
                                               const Eigen::Vector3f &previous_velocity,
                                               float dt) const {
  track.gaussian_state = state;
  track.position(0) = state.mean(0);
  track.position(1) = state.mean(2);
  track.position(2) = state.mean(4);
  track.velocity(0) = state.mean(1);
  track.velocity(1) = state.mean(3);
  track.velocity(2) = state.mean(5);

  if (dt > 1e-6f) {
    track.acceleration = (track.velocity - previous_velocity) / dt;
  }
}

void TrackLifecycleManager::Update(
    const CycleContext &cycle,
    const std::vector<TrackMeasurement> &measurements) {
  bool dt_fallback_used = false;
  const float effective_dt_sec =
      ResolveEffectiveCycleDeltaTimeSec(cycle, &dt_fallback_used);
  std::unordered_set<std::uint64_t> hit_flags;
  hit_flags.reserve(measurements.size());
  std::size_t new_track_count = 0;
  std::size_t updated_track_count = 0;

  for (std::vector<TrackMeasurement>::const_iterator it = measurements.begin();
       it != measurements.end(); ++it) {
    const TrackMeasurement &measurement = *it;

    std::unordered_map<std::uint64_t, common::TrackState *>::iterator found =
        tracks_by_key_.find(measurement.raw_measurement.association_key);

    common::TrackState *track = nullptr;
    const bool track_existed_before_cycle = found != tracks_by_key_.end();
    common::TrackStatus status_before_update = common::TrackStatus::kTentative;
    if (found == tracks_by_key_.end()) {
      track = pool_.Acquire();
      if (track == nullptr) {
        spdlog::warn(
            "[TrackLifecycleManager] failed to acquire track from pool for association_key={}",
            measurement.raw_measurement.association_key);
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

      tracks_by_key_[measurement.raw_measurement.association_key] = track;
      ++new_track_count;
    } else {
      track = found->second;
      status_before_update = track->status;
      track->last_update_cycle = cycle.cycle_index;
      track->hit_count += 1;
      track->miss_count = 0;
      ++updated_track_count;
    }

    if (measurement.raw_measurement.has_cartesian_position) {
      track->position = measurement.raw_measurement.position;
    }

    if (measurement.raw_measurement.external_target_id != 0U) {
      track->external_target_id = measurement.raw_measurement.external_target_id;
    }

    if (measurement.filtered_feature.velocity != Eigen::Vector3f::Zero()) {
      track->velocity = measurement.filtered_feature.velocity;
    } else {
      track->velocity = Eigen::Vector3f(
          measurement.filtered_feature.observed_speed, 0.0f, 0.0f);
    }

    if (measurement.filtered_feature.acceleration != Eigen::Vector3f::Zero()) {
      track->acceleration = measurement.filtered_feature.acceleration;
    } else {
      track->acceleration = Eigen::Vector3f(
          measurement.filtered_feature.observed_acceleration, 0.0f, 0.0f);
    }
    track->rcs = measurement.filtered_feature.rcs;
    track->jamming_detected = measurement.filtered_feature.jamming_detected;

    PromoteState(*track, cycle.cycle_index, true);
    hit_flags.insert(measurement.raw_measurement.association_key);

    // ---- Kalman/IMM 滤波集成 ----
    if (measurement.raw_measurement.has_cartesian_position) {
      const Eigen::Vector3f velocity_before_filter = track->velocity;
      if (ShouldUseImmForMeasurement(track_existed_before_cycle,
                                     status_before_update, measurement)) {
        const GaussianTrackState initial_state = BuildInitialGaussianState(measurement);
        ImmFilter *imm_filter = GetOrCreateImmFilter(
            measurement.raw_measurement.association_key,
            measurement.raw_measurement.matched_existing_track
                ? track->gaussian_state
                : initial_state);
        if (!measurement.raw_measurement.matched_existing_track) {
          ApplyGaussianState(*track, initial_state, velocity_before_filter,
                             effective_dt_sec);
        } else {
          MeasurementVector z;
          z(0) = measurement.raw_measurement.position(0);
          z(1) = measurement.raw_measurement.position(1);
          z(2) = measurement.raw_measurement.position(2);
          imm_filter->Process(z, effective_dt_sec);
          ApplyGaussianState(*track, imm_filter->GetCombinedState(),
                             velocity_before_filter, effective_dt_sec);
        }
      } else if (kalman_predictor_ != nullptr && kalman_updater_ != nullptr) {
        if (!measurement.raw_measurement.matched_existing_track) {
          ApplyGaussianState(*track, BuildInitialGaussianState(measurement),
                             velocity_before_filter, effective_dt_sec);
        } else {
          GaussianTrackState predicted =
              kalman_predictor_->Predict(track->gaussian_state,
                                         effective_dt_sec);
          MeasurementVector z;
          z(0) = measurement.raw_measurement.position(0);
          z(1) = measurement.raw_measurement.position(1);
          z(2) = measurement.raw_measurement.position(2);
          KalmanUpdateResult result =
              kalman_updater_->Update(predicted, z,
                                      measurement.raw_measurement
                                          .measurement_covariance);
          ApplyGaussianState(*track, result.posterior, velocity_before_filter,
                             effective_dt_sec);
        }
      }
    }
  }

  std::vector<std::uint64_t> keys_to_recycle;
  std::size_t predicted_without_hit_count = 0;
  keys_to_recycle.reserve(tracks_by_key_.size());
  for (std::unordered_map<std::uint64_t, common::TrackState *>::iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const std::uint64_t key = it->first;
    common::TrackState *track = it->second;
    const common::TrackStatus status_before_prediction = track->status;

    if (hit_flags.find(key) != hit_flags.end()) {
      continue;
    }

    ++predicted_without_hit_count;

    PromoteState(*track, cycle.cycle_index, false);

    if (track->status == common::TrackStatus::kRecycled) {
      imm_filters_by_key_.erase(key);
      keys_to_recycle.push_back(key);
    } else if (ShouldUseImmForMiss(status_before_prediction)) {
      ImmFilter *imm_filter = FindImmFilter(key);
      if (imm_filter != nullptr) {
        const Eigen::Vector3f velocity_before_filter = track->velocity;
        imm_filter->Predict(effective_dt_sec);
        ApplyGaussianState(*track, imm_filter->GetCombinedState(),
                           velocity_before_filter, effective_dt_sec);
      } else if (kalman_predictor_ != nullptr) {
        const Eigen::Vector3f velocity_before_filter = track->velocity;
        ApplyGaussianState(
            *track,
            kalman_predictor_->Predict(track->gaussian_state, effective_dt_sec),
            velocity_before_filter, effective_dt_sec);
      }
    } else if (kalman_predictor_ != nullptr) {
      // 未命中的轨迹也需要做预测（协方差膨胀，均值外推）
      const Eigen::Vector3f velocity_before_filter = track->velocity;
      ApplyGaussianState(
          *track,
          kalman_predictor_->Predict(track->gaussian_state, effective_dt_sec),
          velocity_before_filter, effective_dt_sec);
    }
  }

  last_cycle_index_ = cycle.cycle_index;

  for (std::vector<std::uint64_t>::const_iterator it = keys_to_recycle.begin();
       it != keys_to_recycle.end(); ++it) {
    std::unordered_map<std::uint64_t, common::TrackState *>::iterator found =
        tracks_by_key_.find(*it);
    if (found == tracks_by_key_.end()) {
      continue;
    }

    ResetForReuse(*found->second);
    pool_.Release(found->second);
    tracks_by_key_.erase(found);
  }

  spdlog::debug(
      "[TrackLifecycleManager] cycle summary: cycle_index={} measurements={} new_tracks={} updated_tracks={} predicted_without_hit={} recycled_tracks={} active_tracks={} imm_enabled={} dt_sec={:.3f} dt_fallback_used={}",
      cycle.cycle_index, measurements.size(), new_track_count, updated_track_count,
      predicted_without_hit_count, keys_to_recycle.size(), tracks_by_key_.size(),
      IsImmEnabled() ? "true" : "false", effective_dt_sec,
      dt_fallback_used ? "true" : "false");
}

float TrackLifecycleManager::ResolveEffectiveCycleDeltaTimeSec(
    const CycleContext &cycle,
    bool *dt_fallback_used) const {
  if (dt_fallback_used != nullptr) {
    *dt_fallback_used = false;
  }

  if (cycle.dt_sec > 0.0f) {
    return cycle.dt_sec;
  }

  if (dt_fallback_used != nullptr) {
    *dt_fallback_used = true;
  }

  spdlog::warn(
      "[TrackLifecycleManager] invalid external dt_sec={}, fallback to "
      "cycle-index-derived step",
      cycle.dt_sec);

  if (last_cycle_index_ > 0 && cycle.cycle_index > last_cycle_index_) {
    return static_cast<float>(cycle.cycle_index - last_cycle_index_);
  }
  return 1.0f;
}

std::vector<const common::TrackState *> TrackLifecycleManager::GetActiveTracks()
    const {
  std::vector<const common::TrackState *> result;
  result.reserve(tracks_by_key_.size());

  for (std::unordered_map<std::uint64_t, common::TrackState *>::const_iterator it =
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

  for (std::unordered_map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const common::TrackState *track = it->second;
    if (track->status == common::TrackStatus::kRecycled) {
      continue;
    }

    common::TargetFeature feature(track->velocity(0),
                    track->velocity(1),
                    track->velocity(2),
                    track->rcs,
                    track->acceleration(0),
                    track->acceleration(1),
                    track->acceleration(2));
    feature.position_x = track->position(0);
    feature.position_y = track->position(1);
    feature.position_z = track->position(2);
    feature.external_target_id = track->external_target_id;
    features.push_back(feature);
  }

  return features;
}

std::vector<AssociationTrackSeed> TrackLifecycleManager::BuildAssociationSeeds()
    const {
  std::vector<AssociationTrackSeed> seeds;
  seeds.reserve(tracks_by_key_.size());

  for (std::unordered_map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const common::TrackState *track = it->second;
    if (track->status == common::TrackStatus::kRecycled) {
      continue;
    }

    AssociationTrackSeed seed;
    seed.association_key = it->first;
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
    if (track.status == common::TrackStatus::kLost ||
        (track.status == common::TrackStatus::kTentative &&
         track.hit_count >= config_.confirm_hits)) {
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
  track.external_target_id = 0;
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
