/**
 * @file RecognitionTracker.cpp
 * @brief 识别积累与判定状态机实现。
 */

#include "airborne_radar/recognition/RecognitionTracker.h"

#include <algorithm>
#include <cmath>

#include "airborne_radar/recognition/RecognitionMatcher.h"
#include "airborne_radar/recognition/RecognitionObservationBuilder.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace recognition {

namespace {

/** @brief 型号确认所需最小有效维度数（运动不能单独确认型号）。 */
constexpr std::uint32_t kMinValidDimensionsForModel = 2U;
/** @brief 有效观测质量下限：全部维度质量低于此值（如短驻留）不计为观测。 */
constexpr float kMinimumObservationQuality = 0.05f;

float Median(std::vector<float> values) {
  if (values.empty()) {
    return 0.0f;
  }
  std::sort(values.begin(), values.end());
  return values[values.size() / 2U];
}

float Mean(const std::vector<float>& values) {
  if (values.empty()) {
    return 0.0f;
  }
  float sum = 0.0f;
  for (std::size_t i = 0U; i < values.size(); ++i) {
    sum += values[i];
  }
  return sum / static_cast<float>(values.size());
}

std::uint8_t CountValidDimensions(const RecognitionFeatureSet& set) {
  std::uint8_t count = 0U;
  if (set.rcs.valid && set.rcs.quality > 0.0f) {
    ++count;
  }
  if (set.motion.valid && set.motion.quality > 0.0f) {
    ++count;
  }
  if (set.polarization.valid && set.polarization.quality > 0.0f) {
    ++count;
  }
  if (set.range_profile.valid && set.range_profile.quality > 0.0f) {
    ++count;
  }
  return count;
}

bool MotionOnly(const RecognitionFeatureSet& set) {
  return CountValidDimensions(set) == 1U && set.motion.valid && set.motion.quality > 0.0f;
}

/** @brief 数据库 category_id 字符串 → 公共大类枚举；未知映射为 kUnknown。 */
session::ArRecognitionCategory CategoryToPublic(const std::string& category_id) {
  if (category_id == "BALLISTIC") {
    return session::ArRecognitionCategory::kBallistic;
  }
  if (category_id == "NEAR_SPACE") {
    return session::ArRecognitionCategory::kNearSpace;
  }
  if (category_id == "OTHER") {
    return session::ArRecognitionCategory::kOther;
  }
  return session::ArRecognitionCategory::kUnknown;
}

/**
 * @brief 滑动窗口聚合：运动子特征取中位数，其余取均值；质量取均值。
 */
RecognitionFeatureSet AggregateWindow(const std::vector<RecognitionFeatureSet>& window) {
  RecognitionFeatureSet aggregate;
  std::vector<float> rcs_values;
  std::vector<float> speed_values;
  std::vector<float> altitude_values;
  std::vector<float> acceleration_values;
  std::vector<float> turn_radius_values;
  std::vector<float> pol_difference_values;
  std::vector<float> pol_relative_values;
  std::vector<float> pol_sum_values;
  std::vector<float> rp_length_values;
  std::vector<float> rp_peak_count_values;
  std::vector<float> rp_concentration_values;
  std::vector<float> rcs_quality;
  std::vector<float> motion_quality;
  std::vector<float> polarization_quality;
  std::vector<float> range_profile_quality;

  for (std::size_t i = 0U; i < window.size(); ++i) {
    const RecognitionFeatureSet& set = window[i];
    if (set.rcs.valid) {
      rcs_values.push_back(set.rcs.mean_dbsm);
      rcs_quality.push_back(set.rcs.quality);
    }
    if (set.motion.valid) {
      speed_values.push_back(set.motion.speed_mps);
      altitude_values.push_back(set.motion.altitude_m);
      acceleration_values.push_back(set.motion.acceleration_mps2);
      if (!set.motion.is_straight && set.motion.turn_radius_m > 0.0f) {
        turn_radius_values.push_back(set.motion.turn_radius_m);
      }
      motion_quality.push_back(set.motion.quality);
    }
    if (set.polarization.valid) {
      pol_difference_values.push_back(set.polarization.energy_difference_db);
      pol_relative_values.push_back(set.polarization.relative_difference_db);
      pol_sum_values.push_back(set.polarization.energy_sum_db);
      polarization_quality.push_back(set.polarization.quality);
    }
    if (set.range_profile.valid) {
      rp_length_values.push_back(set.range_profile.length_m);
      rp_peak_count_values.push_back(static_cast<float>(set.range_profile.peak_count));
      rp_concentration_values.push_back(set.range_profile.peak_energy_concentration);
      range_profile_quality.push_back(set.range_profile.quality);
      aggregate.range_profile.resolution_m = set.range_profile.resolution_m;
    }
  }

  if (!rcs_values.empty()) {
    aggregate.rcs.valid = true;
    aggregate.rcs.mean_dbsm = Mean(rcs_values);
    aggregate.rcs.quality = Mean(rcs_quality);
  }
  if (!speed_values.empty()) {
    aggregate.motion.valid = true;
    aggregate.motion.speed_mps = Median(speed_values);
    aggregate.motion.altitude_m = Median(altitude_values);
    aggregate.motion.acceleration_mps2 = Median(acceleration_values);
    if (!turn_radius_values.empty()) {
      aggregate.motion.turn_radius_m = Median(turn_radius_values);
      aggregate.motion.is_straight = false;
    } else {
      aggregate.motion.is_straight = true;
    }
    aggregate.motion.quality = Mean(motion_quality);
  }
  if (!pol_difference_values.empty()) {
    aggregate.polarization.valid = true;
    aggregate.polarization.energy_difference_db = Mean(pol_difference_values);
    aggregate.polarization.relative_difference_db = Mean(pol_relative_values);
    aggregate.polarization.energy_sum_db = Mean(pol_sum_values);
    aggregate.polarization.quality = Mean(polarization_quality);
  }
  if (!rp_length_values.empty()) {
    aggregate.range_profile.valid = true;
    aggregate.range_profile.length_m = Mean(rp_length_values);
    aggregate.range_profile.peak_count =
        static_cast<std::uint32_t>(std::lround(Mean(rp_peak_count_values)));
    aggregate.range_profile.peak_energy_concentration = Mean(rp_concentration_values);
    aggregate.range_profile.quality = Mean(range_profile_quality);
  }
  aggregate.valid_feature_mask = 0U;
  if (aggregate.rcs.valid && aggregate.rcs.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kRcs);
  }
  if (aggregate.motion.valid && aggregate.motion.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kMotion);
  }
  if (aggregate.polarization.valid && aggregate.polarization.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kPolarization);
  }
  if (aggregate.range_profile.valid && aggregate.range_profile.quality > 0.0f) {
    aggregate.valid_feature_mask |=
        static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kRangeProfile);
  }
  return aggregate;
}

}  // namespace

void RecognitionTracker::ExpireIfHeld(RecognitionTrackState* state, float sim_time_sec) {
  if (state != nullptr && state->has_conclusion && state->conclusion_time_sec >= 0.0f &&
      sim_time_sec - state->conclusion_time_sec > options_.result_hold_sec) {
    state->result.state = session::ArRecognitionState::kStale;
  }
}

void RecognitionTracker::ExitRecognitionMode() {
  for (std::unordered_map<std::uint64_t, RecognitionTrackState>::iterator it = tracks_.begin();
       it != tracks_.end(); ++it) {
    // 保留结论进入保持期；清空积累。
    it->second.window.clear();
    it->second.window_timestamps_sec.clear();
    it->second.confirmed_hit_count = 0U;
    it->second.observation_count = 0U;
    it->second.first_observation_sec = 0.0f;
  }
}

void RecognitionTracker::UpdateCycle(
    const session::TrackStateSnapshotList& tracks,
    const std::unordered_map<std::uint64_t, TrackObservationInput>& observations_by_key,
    const RecognitionFeatureDatabase& database,
    const config::ArRecognitionFeatureWeights& weights, float sim_time_sec,
    std::uint32_t cycle_index, std::uint64_t batch_id) {
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const session::TrackStateSnapshot& snapshot = tracks[i];
    const std::uint64_t key = snapshot.association_key;
    RecognitionTrackState& state = tracks_[key];
    state.association_key = key;

    // 键重分配检测：hit_count 回落视为新目标，清空旧状态。
    if (state.last_seen_hit_count > 0U && snapshot.hit_count < state.last_seen_hit_count) {
      state = RecognitionTrackState{};
      state.association_key = key;
    }
    state.last_seen_hit_count = snapshot.hit_count;

    const std::unordered_map<std::uint64_t, TrackObservationInput>::const_iterator found =
        observations_by_key.find(key);
    if (found == observations_by_key.end() || found->second.target == nullptr) {
      // 本周期无观测输入：结论按 result_hold_sec 过期为 kStale。
      ExpireIfHeld(&state, sim_time_sec);
      continue;  // 无场景目标特征输入 → 本周期无观测
    }
    const TrackObservationInput& input = found->second;
    if (input.context.range_m > options_.max_range_m) {
      ExpireIfHeld(&state, sim_time_sec);
      continue;  // 超出最大识别距离
    }

    const RecognitionFeatureSet features =
        RecognitionObservationBuilder::Build(*input.target, snapshot, input.context);
    if (features.valid_feature_mask == 0U) {
      ExpireIfHeld(&state, sim_time_sec);
      continue;  // 无有效特征维度 → 本周期不积累
    }
    // 观测质量下限：全部维度质量低于下限（如短驻留）的周期不计为有效观测。
    const float max_quality =
        std::max(std::max(features.rcs.quality, features.motion.quality),
                 std::max(features.polarization.quality, features.range_profile.quality));
    if (max_quality < kMinimumObservationQuality) {
      ExpireIfHeld(&state, sim_time_sec);
      continue;
    }

    // 时间窗维护：按 accumulation_window_sec 裁剪。
    if (state.window.empty()) {
      state.first_observation_sec = sim_time_sec;
    }
    state.window.push_back(features);
    state.window_timestamps_sec.push_back(sim_time_sec);
    while (state.window.size() > 1U &&
           sim_time_sec - state.window_timestamps_sec.front() > options_.accumulation_window_sec) {
      state.window.erase(state.window.begin());
      state.window_timestamps_sec.erase(state.window_timestamps_sec.begin());
    }
    ++state.observation_count;
    if (snapshot.status == session::TrackStatus::kConfirmed) {
      ++state.confirmed_hit_count;
    }

    session::ArRecognitionResult judgement;
    judgement.observation_count = state.observation_count;
    judgement.accumulation_sec = sim_time_sec - state.first_observation_sec;
    judgement.database_version = active_database_version_;
    judgement.source_cycle_index = cycle_index;
    judgement.source_batch_id = batch_id;

    // 判定门槛：确认命中数与有效观测数。
    if (state.confirmed_hit_count < options_.min_confirmed_hits ||
        state.observation_count < options_.min_observation_count) {
      judgement.state = session::ArRecognitionState::kAccumulating;
      state.result = judgement;
      continue;
    }

    const RecognitionFeatureSet aggregate = AggregateWindow(state.window);
    if (aggregate.valid_feature_mask == 0U) {
      judgement.state = session::ArRecognitionState::kUnknown;
      state.result = judgement;
      continue;
    }

    const RecognitionMatchResult match =
        RecognitionMatcher::QueryBestMatch(aggregate, input.context, database, weights);
    const std::uint8_t valid_dimensions = CountValidDimensions(aggregate);
    const bool motion_only = MotionOnly(aggregate);

    judgement.valid_feature_mask = aggregate.valid_feature_mask;
    judgement.feature_scores.rcs_quality = aggregate.rcs.quality;
    judgement.feature_scores.motion_quality = aggregate.motion.quality;
    judgement.feature_scores.polarization_quality = aggregate.polarization.quality;
    judgement.feature_scores.range_profile_quality = aggregate.range_profile.quality;

    if (!match.has_candidates) {
      judgement.state = session::ArRecognitionState::kUnknown;
    } else {
      // 分项相似度：与最佳候选模板比对（best model 的适用 profile）。
      const RecognitionModel* best_model = nullptr;
      for (std::size_t m = 0U; m < database.models().size(); ++m) {
        if (database.models()[m].model_id == match.best_model_id) {
          best_model = &database.models()[m];
          break;
        }
      }
      if (best_model != nullptr && !best_model->profiles.empty()) {
        const std::array<float, 4> similarities = RecognitionMatcher::ComputeFeatureSimilarities(
            aggregate, best_model->profiles.front());
        judgement.feature_scores.rcs_similarity = similarities[0];
        judgement.feature_scores.motion_similarity = similarities[1];
        judgement.feature_scores.polarization_similarity = similarities[2];
        judgement.feature_scores.range_profile_similarity = similarities[3];
      }
      judgement.best_score = match.best_score;
      judgement.runner_up_score = match.runner_up_score;
      judgement.confidence = match.confidence;
      const float margin = judgement.best_score - judgement.runner_up_score;
      const bool margin_ok = margin >= options_.minimum_margin;
      const bool dimensions_ok = valid_dimensions >= kMinValidDimensionsForModel && !motion_only;
      if (judgement.best_score < options_.acceptance_score) {
        judgement.state = session::ArRecognitionState::kUnknown;
      } else if (margin_ok && dimensions_ok) {
        judgement.state = session::ArRecognitionState::kModelConfirmed;
        judgement.target_model = match.best_model_id;
        judgement.target_category = CategoryToPublic(match.best_category_id);
      } else {
        judgement.state = session::ArRecognitionState::kCategoryConfirmed;
        judgement.target_category = CategoryToPublic(match.best_category_id);
      }
    }

    state.result = judgement;
    state.conclusion_time_sec = sim_time_sec;
    state.has_conclusion = true;
  }
}

void RecognitionTracker::HoldCycle(const session::TrackStateSnapshotList& tracks,
                                   float sim_time_sec) {
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const session::TrackStateSnapshot& snapshot = tracks[i];
    const std::uint64_t key = snapshot.association_key;
    const std::unordered_map<std::uint64_t, RecognitionTrackState>::iterator found =
        tracks_.find(key);
    if (found == tracks_.end()) {
      continue;
    }
    RecognitionTrackState& state = found->second;
    if (state.last_seen_hit_count > 0U && snapshot.hit_count < state.last_seen_hit_count) {
      tracks_.erase(found);
      continue;
    }
    state.last_seen_hit_count = snapshot.hit_count;
    ExpireIfHeld(&state, sim_time_sec);
  }
}

const session::ArRecognitionResult* RecognitionTracker::FindResult(
    std::uint64_t association_key) const {
  const std::unordered_map<std::uint64_t, RecognitionTrackState>::const_iterator found =
      tracks_.find(association_key);
  return found == tracks_.end() ? nullptr : &found->second.result;
}

session::ArRecognitionCycleSummary RecognitionTracker::BuildSummary(
    const session::TrackStateSnapshotList& tracks) const {
  session::ArRecognitionCycleSummary summary;
  std::uint32_t confirmed_count = 0U;
  std::uint32_t rcs_available = 0U;
  std::uint32_t motion_available = 0U;
  std::uint32_t polarization_available = 0U;
  std::uint32_t range_profile_available = 0U;
  float first_confirmation_sum = 0.0f;
  float confidence_sum = 0.0f;
  for (std::size_t i = 0U; i < tracks.size(); ++i) {
    const session::ArRecognitionResult* result = FindResult(tracks[i].association_key);
    if (result == nullptr || result->state == session::ArRecognitionState::kDisabled) {
      ++summary.disabled_count;
      continue;
    }
    ++summary.participating_track_count;
    switch (result->state) {
      case session::ArRecognitionState::kCategoryConfirmed:
        ++summary.category_confirmed_count;
        break;
      case session::ArRecognitionState::kModelConfirmed:
        ++summary.model_confirmed_count;
        break;
      case session::ArRecognitionState::kUnknown:
        ++summary.unknown_count;
        break;
      default:
        break;
    }
    if (result->state == session::ArRecognitionState::kCategoryConfirmed ||
        result->state == session::ArRecognitionState::kModelConfirmed) {
      ++confirmed_count;
      confidence_sum += result->confidence;
      const std::unordered_map<std::uint64_t, RecognitionTrackState>::const_iterator found =
          tracks_.find(tracks[i].association_key);
      if (found != tracks_.end() && found->second.first_observation_sec > 0.0f &&
          found->second.conclusion_time_sec >= found->second.first_observation_sec) {
        first_confirmation_sum +=
            found->second.conclusion_time_sec - found->second.first_observation_sec;
      }
    }
    if (result->feature_scores.rcs_quality > 0.0f) {
      ++rcs_available;
    }
    if (result->feature_scores.motion_quality > 0.0f) {
      ++motion_available;
    }
    if (result->feature_scores.polarization_quality > 0.0f) {
      ++polarization_available;
    }
    if (result->feature_scores.range_profile_quality > 0.0f) {
      ++range_profile_available;
    }
  }
  if (summary.participating_track_count > 0U) {
    const float participating = static_cast<float>(summary.participating_track_count);
    summary.rcs_availability_rate = static_cast<float>(rcs_available) / participating;
    summary.motion_availability_rate = static_cast<float>(motion_available) / participating;
    summary.polarization_availability_rate =
        static_cast<float>(polarization_available) / participating;
    summary.range_profile_availability_rate =
        static_cast<float>(range_profile_available) / participating;
  }
  if (confirmed_count > 0U) {
    summary.mean_confidence = confidence_sum / static_cast<float>(confirmed_count);
    summary.mean_first_confirmation_sec =
        first_confirmation_sum / static_cast<float>(confirmed_count);
  }
  return summary;
}

RecognitionTracker::Snapshot RecognitionTracker::Capture() const {
  Snapshot snapshot;
  snapshot.active_database_version = active_database_version_;
  snapshot.tracks.reserve(tracks_.size());
  for (const auto& entry : tracks_) {
    snapshot.tracks.push_back(entry.second);
  }
  return snapshot;
}

void RecognitionTracker::Restore(const Snapshot& snapshot) {
  tracks_.clear();
  for (std::size_t i = 0U; i < snapshot.tracks.size(); ++i) {
    tracks_[snapshot.tracks[i].association_key] = snapshot.tracks[i];
  }
  active_database_version_ = snapshot.active_database_version;
}

void RecognitionTracker::Reset() {
  tracks_.clear();
  active_database_version_.clear();
}

}  // namespace recognition
}  // namespace airborne_radar
