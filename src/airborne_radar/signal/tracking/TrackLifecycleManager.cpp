// Copyright 2026. All Rights Reserved.
//
// Description: TrackLifecycleManager 的实现。

#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"

#include <algorithm>
#include <mutex>
#include <unordered_set>

#include <spdlog/spdlog.h>

#include "airborne_radar/signal/tracking/ImmFilter.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

common::DecisionTrackStatus ToDecisionTrackStatus(common::TrackStatus status) {
  switch (status) {
    case common::TrackStatus::kConfirmed:
      return common::DecisionTrackStatus::kConfirmed;
    case common::TrackStatus::kLost:
      return common::DecisionTrackStatus::kLost;
    case common::TrackStatus::kTentative:
    case common::TrackStatus::kRecycled:
    default:
      return common::DecisionTrackStatus::kTentative;
  }
}

/// @brief SynchronizedTrackPool 为任意对象池提供全局互斥包装。
class SynchronizedTrackPool : public ITrackPool {
public:
  explicit SynchronizedTrackPool(ITrackPool &inner) : inner_(inner) {}

  common::TrackState *Acquire() override {
    std::lock_guard<std::mutex> lock(mutex_);
    return inner_.Acquire();
  }

  void Release(common::TrackState *track) override {
    std::lock_guard<std::mutex> lock(mutex_);
    inner_.Release(track);
  }

  std::size_t Capacity() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return inner_.Capacity();
  }

  std::size_t InUseCount() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return inner_.InUseCount();
  }

private:
  ITrackPool &inner_;
  mutable std::mutex mutex_;
};

/// @brief ConfigureEffectiveTrackPool 根据配置选择直接池或加锁包装池。
void ConfigureEffectiveTrackPool(
    ITrackPool &pool, TrackPoolThreadSafetyMode mode,
    std::unique_ptr<ITrackPool> *owned_pool_wrapper, ITrackPool **effective_pool) {
  if (owned_pool_wrapper == nullptr || effective_pool == nullptr) {
    return;
  }

  owned_pool_wrapper->reset();
  if (mode == TrackPoolThreadSafetyMode::kMultiThreadGlobalLock) {
    owned_pool_wrapper->reset(new SynchronizedTrackPool(pool));
    *effective_pool = owned_pool_wrapper->get();
    return;
  }

  *effective_pool = &pool;
}

/// @brief WorkItemKind 描述单条轨迹工作单元的命中类型。
enum class WorkItemKind {
  kHit = 0,
  kMiss
};

/// @brief TrackUpdateWorkItem 表示单条轨迹在计算阶段的只读输入。
struct TrackUpdateWorkItem {
  std::uint64_t association_key{0};
  WorkItemKind kind{WorkItemKind::kMiss};
  common::TrackState *track{nullptr};
  const TrackMeasurement *measurement{nullptr};
  bool track_existed_before_cycle{false};
  bool use_imm{false};
  ImmFilter *imm_filter{nullptr};
  common::TrackStatus status_before_update{common::TrackStatus::kTentative};
  common::TrackState track_before_update;
};

/// @brief TrackUpdateResult 表示单条轨迹计算阶段产出的写回结果。
struct TrackUpdateResult {
  std::uint64_t association_key{0};
  common::TrackState *track{nullptr};
  common::TrackState track_after_update;
  bool should_recycle{false};
};

/// @brief LifecycleUpdateScratch 聚合单周期生命周期更新的中间缓存。
struct LifecycleUpdateScratch {
  std::unordered_map<std::uint64_t, const TrackMeasurement *> measurement_by_key;
  std::unordered_map<std::uint64_t, common::TrackState> track_snapshots;
  std::unordered_set<std::uint64_t> hit_keys;
  std::vector<TrackUpdateWorkItem> work_items;
  std::vector<TrackUpdateResult> results;
  std::vector<std::uint64_t> keys_to_recycle;
  std::size_t new_track_count{0};
  std::size_t updated_track_count{0};
  std::size_t predicted_without_hit_count{0};
};

/// @brief BuildMeasurementVector 将位置量测写入滤波观测向量。
MeasurementVector BuildMeasurementVector(const TrackMeasurement &measurement) {
  MeasurementVector z;
  z(0) = measurement.raw_measurement.position(0);
  z(1) = measurement.raw_measurement.position(1);
  z(2) = measurement.raw_measurement.position(2);
  return z;
}

} // namespace

TrackLifecycleManager::TrackLifecycleManager(ITrackPool &pool,
                                             const LifecycleConfig &config,
                                             const IKalmanPredictor *predictor,
                                             const IKalmanUpdater *updater)
    : config_(config), next_track_id_(1), tracks_by_key_(),
      kalman_predictor_(predictor), kalman_updater_(updater) {
  ConfigureEffectiveTrackPool(pool, config_.track_pool_thread_safety_mode,
                              &owned_pool_wrapper_, &pool_);
}

TrackLifecycleManager::TrackLifecycleManager(
    ITrackPool &pool, const LifecycleConfig &config,
    const std::vector<const IKalmanPredictor *> &imm_predictors,
    const std::vector<const IKalmanUpdater *> &imm_updaters,
    const Eigen::MatrixXf &imm_transition_probability,
    const Eigen::VectorXf &imm_initial_weights)
    : config_(config), next_track_id_(1), tracks_by_key_(),
      kalman_predictor_(imm_predictors.empty() ? nullptr : imm_predictors.front()),
      kalman_updater_(imm_updaters.empty() ? nullptr : imm_updaters.front()),
      imm_predictors_(imm_predictors), imm_updaters_(imm_updaters),
      imm_transition_probability_(imm_transition_probability),
      imm_initial_weights_(imm_initial_weights) {
  ConfigureEffectiveTrackPool(pool, config_.track_pool_thread_safety_mode,
                              &owned_pool_wrapper_, &pool_);
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
  LifecycleUpdateScratch scratch;
  scratch.measurement_by_key.reserve(measurements.size());
  scratch.track_snapshots.reserve(tracks_by_key_.size());
  scratch.hit_keys.reserve(measurements.size());
  scratch.keys_to_recycle.reserve(tracks_by_key_.size());

  // Phase 1: PreparePhase，建立量测路由并快照周期开始时的轨迹状态。
  for (std::unordered_map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    scratch.track_snapshots[it->first] = *it->second;
  }
  for (std::vector<TrackMeasurement>::const_iterator it = measurements.begin();
       it != measurements.end(); ++it) {
    const std::uint64_t association_key =
        it->raw_measurement.association_key;
    std::unordered_map<std::uint64_t, const TrackMeasurement *>::iterator found =
        scratch.measurement_by_key.find(association_key);
    if (found != scratch.measurement_by_key.end()) {
      spdlog::warn(
          "[TrackLifecycleManager] duplicate measurement for association_key={} in one cycle, use last measurement for staged update",
          association_key);
    }
    scratch.measurement_by_key[association_key] = &(*it);
    scratch.hit_keys.insert(association_key);
  }

  scratch.work_items.reserve(tracks_by_key_.size() +
                             scratch.measurement_by_key.size());
  scratch.results.reserve(tracks_by_key_.size() +
                          scratch.measurement_by_key.size());

  // Phase 2: EnsurePhase，串行补齐本周期所需的轨迹对象和 IMM 运行态。
  for (std::vector<TrackMeasurement>::const_iterator it = measurements.begin();
       it != measurements.end(); ++it) {
    const TrackMeasurement &measurement = *it;
    const std::uint64_t association_key =
        measurement.raw_measurement.association_key;
    if (scratch.measurement_by_key[association_key] != &measurement) {
      continue;
    }

    std::unordered_map<std::uint64_t, common::TrackState *>::iterator found =
        tracks_by_key_.find(association_key);
    const bool track_existed_before_cycle = found != tracks_by_key_.end();
    common::TrackState *track = nullptr;
    common::TrackState track_before_update;
    if (!track_existed_before_cycle) {
      track = pool_ != nullptr ? pool_->Acquire() : nullptr;
      if (track == nullptr) {
        spdlog::warn(
            "[TrackLifecycleManager] failed to acquire track from pool for association_key={}",
            association_key);
        continue;
      }

      ResetForReuse(*track);
      track->track_id = next_track_id_++;
      track->batch_id = cycle.batch_id;
      track->status = common::TrackStatus::kTentative;
      track->first_cycle = cycle.cycle_index;
      track->last_update_cycle = 0;
      track->hit_count = 0;
      track->miss_count = 0;
      tracks_by_key_[association_key] = track;
      track_before_update = *track;
      ++scratch.new_track_count;
    } else {
      track = found->second;
      track_before_update = scratch.track_snapshots[association_key];
      ++scratch.updated_track_count;
    }

    const common::TrackStatus status_before_update =
        track_existed_before_cycle
            ? track_before_update.status
            : common::TrackStatus::kTentative;
    const bool use_imm = ShouldUseImmForMeasurement(
        track_existed_before_cycle, status_before_update, measurement);
    ImmFilter *imm_filter = nullptr;
    if (use_imm) {
      const GaussianTrackState initial_state =
          measurement.raw_measurement.matched_existing_track
              ? track_before_update.gaussian_state
              : BuildInitialGaussianState(measurement);
      imm_filter = GetOrCreateImmFilter(association_key, initial_state);
    }

    TrackUpdateWorkItem work_item;
    work_item.association_key = association_key;
    work_item.kind = WorkItemKind::kHit;
    work_item.track = track;
    work_item.measurement = &measurement;
    work_item.track_existed_before_cycle = track_existed_before_cycle;
    work_item.use_imm = use_imm;
    work_item.imm_filter = imm_filter;
    work_item.status_before_update = status_before_update;
    work_item.track_before_update = track_before_update;
    scratch.work_items.push_back(work_item);
  }

  for (std::unordered_map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    if (scratch.hit_keys.find(it->first) != scratch.hit_keys.end()) {
      continue;
    }

    const common::TrackState &track_before_update =
        scratch.track_snapshots[it->first];
    const common::TrackStatus status_before_prediction =
        track_before_update.status;
    ImmFilter *imm_filter = nullptr;
    bool use_imm = false;
    if (ShouldUseImmForMiss(status_before_prediction)) {
      imm_filter = FindImmFilter(it->first);
      use_imm = imm_filter != nullptr;
    }

    TrackUpdateWorkItem work_item;
    work_item.association_key = it->first;
    work_item.kind = WorkItemKind::kMiss;
    work_item.track = it->second;
    work_item.track_existed_before_cycle = true;
    work_item.use_imm = use_imm;
    work_item.imm_filter = imm_filter;
    work_item.status_before_update = status_before_prediction;
    work_item.track_before_update = track_before_update;
    scratch.work_items.push_back(work_item);
    ++scratch.predicted_without_hit_count;
  }

  // Phase 3: ComputePhase，只读容器并按工作单元计算写回结果。
  for (std::vector<TrackUpdateWorkItem>::const_iterator it =
           scratch.work_items.begin();
       it != scratch.work_items.end(); ++it) {
    const TrackUpdateWorkItem &work_item = *it;
    TrackUpdateResult result;
    result.association_key = work_item.association_key;
    result.track = work_item.track;
    result.track_after_update = work_item.track_before_update;

    common::TrackState &track = result.track_after_update;
    if (work_item.kind == WorkItemKind::kHit) {
      const TrackMeasurement &measurement = *work_item.measurement;
      track.last_update_cycle = cycle.cycle_index;
      track.hit_count += 1;
      track.miss_count = 0;

      if (measurement.raw_measurement.has_cartesian_position) {
        track.position = measurement.raw_measurement.position;
      }

      if (measurement.raw_measurement.external_target_id != 0U) {
        track.external_target_id =
            measurement.raw_measurement.external_target_id;
      }

      if (measurement.filtered_feature.velocity != Eigen::Vector3f::Zero()) {
        track.velocity = measurement.filtered_feature.velocity;
      } else {
        track.velocity = Eigen::Vector3f(
            measurement.filtered_feature.observed_speed, 0.0f, 0.0f);
      }

      if (measurement.filtered_feature.acceleration != Eigen::Vector3f::Zero()) {
        track.acceleration = measurement.filtered_feature.acceleration;
      } else {
        track.acceleration = Eigen::Vector3f(
            measurement.filtered_feature.observed_acceleration, 0.0f, 0.0f);
      }
      track.rcs = measurement.filtered_feature.rcs;
      track.jamming_detected = measurement.filtered_feature.jamming_detected;

      PromoteState(track, cycle.cycle_index, true,
                   cycle.extra_miss_tolerance);

      if (measurement.raw_measurement.has_cartesian_position) {
        const Eigen::Vector3f velocity_before_filter = track.velocity;
        if (work_item.use_imm && work_item.imm_filter != nullptr) {
          const GaussianTrackState initial_state =
              BuildInitialGaussianState(measurement);
          if (!measurement.raw_measurement.matched_existing_track) {
            ApplyGaussianState(track, initial_state, velocity_before_filter,
                               effective_dt_sec);
          } else {
            work_item.imm_filter->Process(BuildMeasurementVector(measurement),
                                          effective_dt_sec);
            ApplyGaussianState(track, work_item.imm_filter->GetCombinedState(),
                               velocity_before_filter, effective_dt_sec);
          }
        } else if (kalman_predictor_ != nullptr &&
                   kalman_updater_ != nullptr) {
          if (!measurement.raw_measurement.matched_existing_track) {
            ApplyGaussianState(track, BuildInitialGaussianState(measurement),
                               velocity_before_filter, effective_dt_sec);
          } else {
            const GaussianTrackState predicted =
                kalman_predictor_->Predict(track.gaussian_state,
                                           effective_dt_sec);
            const KalmanUpdateResult update_result =
                kalman_updater_->Update(
                    predicted, BuildMeasurementVector(measurement),
                    measurement.raw_measurement.measurement_covariance);
            ApplyGaussianState(track, update_result.posterior,
                               velocity_before_filter, effective_dt_sec);
          }
        }
      }
    } else {
      PromoteState(track, cycle.cycle_index, false,
                   cycle.extra_miss_tolerance);

      if (track.status == common::TrackStatus::kRecycled) {
        result.should_recycle = true;
      } else if (work_item.use_imm && work_item.imm_filter != nullptr) {
        const Eigen::Vector3f velocity_before_filter = track.velocity;
        work_item.imm_filter->Predict(effective_dt_sec);
        ApplyGaussianState(track, work_item.imm_filter->GetCombinedState(),
                           velocity_before_filter, effective_dt_sec);
      } else if (kalman_predictor_ != nullptr) {
        const Eigen::Vector3f velocity_before_filter = track.velocity;
        ApplyGaussianState(
            track,
            kalman_predictor_->Predict(track.gaussian_state, effective_dt_sec),
            velocity_before_filter, effective_dt_sec);
      }
    }

    scratch.results.push_back(result);
  }

  // Phase 4: CommitPhase，串行写回所有轨迹结果。
  for (std::vector<TrackUpdateResult>::const_iterator it =
           scratch.results.begin();
       it != scratch.results.end(); ++it) {
    if (it->track == nullptr) {
      continue;
    }
    *(it->track) = it->track_after_update;
    if (it->should_recycle) {
      scratch.keys_to_recycle.push_back(it->association_key);
    }
  }

  last_cycle_index_ = cycle.cycle_index;

  // Phase 5: RecyclePhase，串行回收已进入 recycled 状态的轨迹与 IMM 运行态。
  for (std::vector<std::uint64_t>::const_iterator it =
           scratch.keys_to_recycle.begin();
       it != scratch.keys_to_recycle.end(); ++it) {
    imm_filters_by_key_.erase(*it);

    std::unordered_map<std::uint64_t, common::TrackState *>::iterator found =
        tracks_by_key_.find(*it);
    if (found == tracks_by_key_.end()) {
      continue;
    }

    ResetForReuse(*found->second);
    if (pool_ != nullptr) {
      pool_->Release(found->second);
    }
    tracks_by_key_.erase(found);
  }

  latest_evidence_by_key_.clear();
  latest_evidence_by_key_.reserve(scratch.measurement_by_key.size());
  for (std::unordered_map<std::uint64_t, const TrackMeasurement *>::const_iterator
           it = scratch.measurement_by_key.begin();
       it != scratch.measurement_by_key.end(); ++it) {
    if (tracks_by_key_.find(it->first) == tracks_by_key_.end()) {
      continue;
    }

    const TrackMeasurement &measurement = *(it->second);
    common::DecisionMeasurementEvidence evidence;
    evidence.has_measurement_evidence = true;
    evidence.updated_this_cycle = true;
    evidence.predicted_only_this_cycle = false;
    evidence.matched_existing_track =
        measurement.raw_measurement.matched_existing_track;
    evidence.association_cost = measurement.raw_measurement.association_cost;
    evidence.detection_margin_db =
        measurement.raw_measurement.detection_margin_db;
    evidence.used_position_association =
        measurement.raw_measurement.used_position_association;
    evidence.used_external_association_seeds =
        measurement.raw_measurement.used_external_association_seeds;
    latest_evidence_by_key_[it->first] = evidence;
  }

  spdlog::debug(
      "[TrackLifecycleManager] cycle summary: cycle_index={} measurements={} new_tracks={} updated_tracks={} predicted_without_hit={} recycled_tracks={} active_tracks={} imm_enabled={} dt_sec={:.3f} dt_fallback_used={}",
      cycle.cycle_index, measurements.size(), scratch.new_track_count,
      scratch.updated_track_count, scratch.predicted_without_hit_count,
      scratch.keys_to_recycle.size(), tracks_by_key_.size(),
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

common::DecisionTrackSnapshotList
TrackLifecycleManager::BuildDecisionSnapshot() const {
  common::DecisionTrackSnapshotList snapshots;
  snapshots.reserve(tracks_by_key_.size());

  for (std::unordered_map<std::uint64_t, common::TrackState *>::const_iterator it =
           tracks_by_key_.begin();
       it != tracks_by_key_.end(); ++it) {
    const common::TrackState *track = it->second;
    if (track->status == common::TrackStatus::kRecycled) {
      continue;
    }

    common::DecisionTrackSnapshot snapshot(
        track->velocity(0), track->velocity(1), track->velocity(2), track->rcs,
        track->acceleration(0), track->acceleration(1), track->acceleration(2),
        track->jamming_detected, track->external_target_id, it->first);
    snapshot.state.status = ToDecisionTrackStatus(track->status);
    snapshot.state.position_x = track->position(0);
    snapshot.state.position_y = track->position(1);
    snapshot.state.position_z = track->position(2);
    snapshot.state.hit_count = track->hit_count;
    snapshot.state.miss_count = track->miss_count;

    std::unordered_map<std::uint64_t, common::DecisionMeasurementEvidence>::
        const_iterator evidence_found = latest_evidence_by_key_.find(it->first);
    if (evidence_found != latest_evidence_by_key_.end()) {
      snapshot.evidence = evidence_found->second;
    } else {
      snapshot.evidence.has_measurement_evidence = false;
      snapshot.evidence.updated_this_cycle = false;
      snapshot.evidence.predicted_only_this_cycle =
          track->last_update_cycle != last_cycle_index_;
    }
    snapshots.push_back(snapshot);
  }

  return snapshots;
}

common::DecisionInputFrame TrackLifecycleManager::BuildDecisionFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    bool environment_jamming_detected) const {
  common::DecisionInputFrame frame;
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
  frame.environment_jamming_detected = environment_jamming_detected;
  frame.tracks = BuildDecisionSnapshot();
  return frame;
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
                                         bool hit_this_cycle,
                                         std::uint32_t extra_miss_tolerance) {
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
    const std::uint32_t max_miss_before_lost =
        config_.max_miss_before_lost + extra_miss_tolerance;
    if (track.miss_count > max_miss_before_lost) {
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
