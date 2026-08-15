/**
 * @file RirTrackLifecycle.cpp
 * @brief RIR 轻量跟踪子集的航迹生命周期管理器实现（阶段 2-T T3）。
 */

#include "remote_identification_radar/tracking/RirTrackLifecycle.h"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "common/logging/ProjectLog.h"

namespace remote_identification_radar {
namespace tracking {

namespace {

/** @brief 生命周期保留键：未关联量测。 */
constexpr std::uint64_t kUnassociatedKey = 0U;

/** @brief 加速度估计的最小有效步长（s）。 */
constexpr float kMinimumAccelerationDtSec = 1.0e-6f;

}  // namespace

RirTrackLifecycle::RirTrackLifecycle(RirLifecycleConfig config, RirTrackFilterConfig filter_config)
    : lifecycle_config_(config), filter_(filter_config) {}

void RirTrackLifecycle::Update(const RirCycleContext& cycle,
                               const std::vector<RirTrackMeasurement>& measurements) {
  if (!std::isfinite(cycle.dt_sec) || cycle.dt_sec <= 0.0f) {
    // 中译：周期时间步长非法，本周期航迹更新被跳过且状态不变。
    // 标识：时间保护——无效 dt 不推进状态机与滤波器，防止协方差传播被污染。
    PROJECT_LOG_ERROR(
        "[RirTrackLifecycle] invalid cycle dt_sec={}, update skipped with no state changes.",
        cycle.dt_sec);
    return;
  }

  // 同键多量测取最后一条（与 AR 子集一致）；按输入顺序建工作单元，
  // 保证同周期多个新航迹的 track_id 分配与输入顺序确定一致。
  std::unordered_map<std::uint64_t, std::size_t> measurement_slot_by_key;
  std::vector<const RirTrackMeasurement*> unique_measurements;
  std::unordered_set<std::uint64_t> hit_keys;
  measurement_slot_by_key.reserve(measurements.size());
  unique_measurements.reserve(measurements.size());
  hit_keys.reserve(measurements.size());
  for (const RirTrackMeasurement& measurement : measurements) {
    if (measurement.association_key == kUnassociatedKey) {
      // 中译：忽略未分配关联键的量测。
      // 标识：输入契约——生命周期只接受关联器已回填键的量测。
      PROJECT_LOG_WARN(
          "[RirTrackLifecycle] ignored measurement with association_key=0 at source_index={}",
          measurement.source_index);
      continue;
    }
    const auto existing = measurement_slot_by_key.find(measurement.association_key);
    if (existing != measurement_slot_by_key.end()) {
      // 中译：同一周期内关联键 {} 出现重复量测，暂存更新使用最后一条。
      // 标识：输入去重——同键多量测取最后一条，防止一次更新写多次。
      PROJECT_LOG_WARN(
          "[RirTrackLifecycle] duplicate measurement for association_key={} in one cycle, use "
          "last measurement for staged update",
          measurement.association_key);
      unique_measurements[existing->second] = &measurement;
    } else {
      measurement_slot_by_key[measurement.association_key] = unique_measurements.size();
      unique_measurements.push_back(&measurement);
    }
    hit_keys.insert(measurement.association_key);
  }

  std::map<std::uint64_t, RirTrackState> snapshots = tracks_;
  std::vector<WorkItem> work_items;
  work_items.reserve(snapshots.size() + unique_measurements.size());

  for (const RirTrackMeasurement* measurement : unique_measurements) {
    const std::uint64_t key = measurement->association_key;
    WorkItem work_item;
    work_item.association_key = key;
    work_item.kind = WorkItemKind::kHit;
    work_item.measurement = measurement;

    const std::map<std::uint64_t, RirTrackState>::const_iterator found = snapshots.find(key);
    if (found == snapshots.end()) {
      RirTrackState track;
      track.association_key = key;
      track.track_id = next_track_id_++;
      track.batch_id = cycle.batch_id;
      track.status = RirTrackStatus::kTentative;
      track.first_cycle = cycle.cycle_index;
      work_item.track_before = track;
      work_item.status_before = RirTrackStatus::kTentative;
      work_item.track_existed_before_cycle = false;
    } else {
      work_item.track_before = found->second;
      work_item.status_before = found->second.status;
      work_item.track_existed_before_cycle = true;
    }
    work_items.push_back(work_item);
  }

  for (const auto& entry : snapshots) {
    if (hit_keys.find(entry.first) != hit_keys.end()) {
      continue;
    }
    WorkItem work_item;
    work_item.association_key = entry.first;
    work_item.kind = WorkItemKind::kMiss;
    work_item.track_before = entry.second;
    work_item.status_before = entry.second.status;
    work_item.track_existed_before_cycle = true;
    work_items.push_back(work_item);
  }

  std::vector<WorkResult> results;
  results.reserve(work_items.size());
  for (const WorkItem& work_item : work_items) {
    WorkResult result;
    result.association_key = work_item.association_key;
    result.track_after = work_item.track_before;
    RirTrackState& track = result.track_after;

    if (work_item.kind == WorkItemKind::kHit) {
      const RirTrackMeasurement& measurement = *work_item.measurement;
      track.last_update_cycle = cycle.cycle_index;
      track.hit_count += 1U;
      track.miss_count = 0U;

      track.position = measurement.position;
      if (measurement.external_target_id != 0U) {
        track.external_target_id = measurement.external_target_id;
      }
      if (!measurement.target_name.empty()) {
        track.target_name = measurement.target_name;
      }
      // 先写入本周期速度种子，再执行 KF 更新；加速度为滤波修正/新息速度差。
      track.velocity = measurement.velocity;
      track.rcs = measurement.rcs;

      PromoteState(&track, cycle.cycle_index, true);
      ApplyHitFilter(&track, measurement, work_item.status_before, cycle.dt_sec);
    } else {
      const bool should_recycle = PromoteState(&track, cycle.cycle_index, false);
      if (should_recycle) {
        result.should_recycle = true;
      } else {
        const Eigen::Vector3f previous_velocity = track.velocity;
        const RirGaussianState predicted = filter_.Predict(track.gaussian_state, cycle.dt_sec);
        ApplyGaussianState(&track, predicted, previous_velocity, cycle.dt_sec);
      }
    }
    results.push_back(result);
  }

  tracks_.clear();
  for (const WorkResult& result : results) {
    if (result.should_recycle) {
      continue;
    }
    tracks_[result.association_key] = result.track_after;
  }
  last_cycle_index_ = cycle.cycle_index;
}

RirTrackSnapshotList RirTrackLifecycle::BuildTrackSnapshots() const {
  RirTrackSnapshotList snapshots;
  snapshots.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    RirTrackState snapshot = entry.second;
    snapshot.association_key = entry.first;
    snapshots.push_back(snapshot);
  }
  return snapshots;
}

std::vector<RirTrackSeed> RirTrackLifecycle::BuildAssociationSeeds() const {
  std::vector<RirTrackSeed> seeds;
  seeds.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    const RirTrackState& track = entry.second;
    RirTrackSeed seed;
    seed.association_key = entry.first;
    seed.has_position = true;
    seed.position = track.position;
    seed.has_gaussian_state = true;
    seed.gaussian_state = track.gaussian_state;
    seeds.push_back(seed);
  }
  return seeds;
}

const RirTrackState* RirTrackLifecycle::FindTrack(std::uint64_t association_key) const {
  const std::map<std::uint64_t, RirTrackState>::const_iterator found =
      tracks_.find(association_key);
  if (found == tracks_.end()) {
    return nullptr;
  }
  return &found->second;
}

void RirTrackLifecycle::Reset() {
  tracks_.clear();
  next_track_id_ = 1U;
  last_cycle_index_ = 0U;
}

void RirTrackLifecycle::UpdateConfig(RirLifecycleConfig lifecycle_config,
                                     RirTrackFilterConfig filter_config) {
  lifecycle_config_ = lifecycle_config;
  filter_.UpdateConfig(filter_config);
}

RirLifecycleRuntimeState RirTrackLifecycle::CaptureRuntimeState() const {
  RirLifecycleRuntimeState state;
  state.next_track_id = next_track_id_;
  state.last_cycle_index = last_cycle_index_;
  state.tracks = BuildTrackSnapshots();
  return state;
}

void RirTrackLifecycle::RestoreRuntimeState(const RirLifecycleRuntimeState& state) {
  tracks_.clear();
  for (const RirTrackState& track : state.tracks) {
    tracks_[track.association_key] = track;
  }
  next_track_id_ = state.next_track_id;
  last_cycle_index_ = state.last_cycle_index;
}

bool RirTrackLifecycle::PromoteState(RirTrackState* track, std::uint32_t cycle_index,
                                     bool hit_this_cycle) {
  if (hit_this_cycle) {
    if (track->status == RirTrackStatus::kLost ||
        (track->status == RirTrackStatus::kTentative &&
         track->hit_count >= lifecycle_config_.confirm_hits)) {
      track->status = RirTrackStatus::kConfirmed;
    }
    return false;
  }

  track->miss_count += 1U;
  if (track->status == RirTrackStatus::kTentative || track->status == RirTrackStatus::kConfirmed) {
    if (track->miss_count > lifecycle_config_.max_miss_before_lost) {
      track->status = RirTrackStatus::kLost;
    }
    return false;
  }

  if (track->status == RirTrackStatus::kLost) {
    const std::uint32_t lost_cycles =
        cycle_index >= track->last_update_cycle ? (cycle_index - track->last_update_cycle) : 0U;
    if (lost_cycles > lifecycle_config_.max_lost_cycles) {
      return true;
    }
  }
  return false;
}

void RirTrackLifecycle::ApplyGaussianState(RirTrackState* track, const RirGaussianState& state,
                                           const Eigen::Vector3f& previous_velocity,
                                           float dt_sec) const {
  track->gaussian_state = state;
  track->position = Eigen::Vector3f(state.mean(0), state.mean(2), state.mean(4));
  track->velocity = Eigen::Vector3f(state.mean(1), state.mean(3), state.mean(5));
  if (dt_sec > kMinimumAccelerationDtSec) {
    track->acceleration = (track->velocity - previous_velocity) / dt_sec;
  }
}

void RirTrackLifecycle::ApplyHitFilter(RirTrackState* track, const RirTrackMeasurement& measurement,
                                       RirTrackStatus status_before, float dt_sec) const {
  // 与 AR 子集一致：加速度 = KF 后验速度与本周期速度种子的差/dt。
  const Eigen::Vector3f velocity_before_filter = track->velocity;
  const bool reset_filter_on_hit =
      !measurement.matched_existing_track || status_before == RirTrackStatus::kLost;
  if (reset_filter_on_hit) {
    ApplyGaussianState(track, filter_.Initialize(measurement), velocity_before_filter, dt_sec);
    return;
  }

  const RirGaussianState predicted = filter_.Predict(track->gaussian_state, dt_sec);
  const RirKalmanUpdateResult update_result = filter_.Update(predicted, measurement);
  ApplyGaussianState(track, update_result.posterior, velocity_before_filter, dt_sec);
}

}  // namespace tracking
}  // namespace remote_identification_radar
