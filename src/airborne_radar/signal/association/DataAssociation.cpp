// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现基于距离度量、波门与指派求解的数据关联流程。

#include "airborne_radar/signal/association/DataAssociation.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace association {

namespace {

/// @brief 未关联目标使用的保留键值。
constexpr std::uint64_t kUnassociatedKey = 0;

} // namespace

/// @brief 构造数据关联引擎并初始化内部组件。
/// @param config 关联配置。
DataAssociationEngine::DataAssociationEngine(DataAssociationConfig config)
    : config_(config),
      distance_metric_(config.speed_sigma, config.rcs_sigma,
                       config.acceleration_sigma),
  gater_(config.unassigned_cost),
  hypothesiser_(&distance_metric_, &gater_),
      next_key_(1),
      previous_tracks_() {}

/// @brief 执行一次完整的数据关联。
/// @param targets 当前周期输入目标特征集合。
/// @param detection_succeeded 探测成功标记。
/// @return 结构化关联结果。
AssociationResult DataAssociationEngine::AssociateDetections(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded) {
  const std::size_t target_count = targets.size();
  AssociationResult result;
  result.target_keys.resize(target_count, kUnassociatedKey);

  std::vector<std::size_t> measurement_indices;
  measurement_indices.reserve(target_count);
  for (std::size_t i = 0; i < target_count; ++i) {
    if (i < detection_succeeded.size() && detection_succeeded[i] != 0U) {
      measurement_indices.push_back(i);
    }
  }

  if (measurement_indices.empty()) {
    result.missed_track_keys.reserve(previous_tracks_.size());
    for (const TrackSignature &track : previous_tracks_) {
      result.missed_track_keys.push_back(track.key);
    }
    previous_tracks_.clear();
    return result;
  }

  std::vector<Eigen::Vector3f> measurements;
  measurements.reserve(measurement_indices.size());
  for (std::size_t i = 0; i < measurement_indices.size(); ++i) {
    measurements.push_back(BuildFeatureVector(targets[measurement_indices[i]]));
  }

  std::vector<std::uint64_t> measurement_to_key(measurements.size(), kUnassociatedKey);
  std::vector<float> measurement_match_cost(measurements.size(), 0.0f);
  std::vector<std::uint8_t> track_matched(previous_tracks_.size(), 0U);

  if (!previous_tracks_.empty()) {
    const std::size_t rows = previous_tracks_.size();
    const std::size_t cols = measurements.size();
    const std::size_t dim = std::max(rows, cols);
    const float rejected_cost = config_.unassigned_cost + 1.0f;
    FeatureVectorList predicted_tracks;
    predicted_tracks.reserve(rows);
    for (const TrackSignature &track : previous_tracks_) {
      predicted_tracks.push_back(track.feature);
    }

    Eigen::MatrixXf cost_matrix =
        Eigen::MatrixXf::Constant(static_cast<Eigen::Index>(dim),
                                  static_cast<Eigen::Index>(dim),
                                  config_.unassigned_cost);
    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < cols; ++c) {
        cost_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
            rejected_cost;
      }
    }

    const std::vector<AssociationHypothesis> hypotheses =
        hypothesiser_.Generate(predicted_tracks, measurements);
    for (const AssociationHypothesis &hypothesis : hypotheses) {
      cost_matrix(static_cast<Eigen::Index>(hypothesis.track_index),
                  static_cast<Eigen::Index>(hypothesis.measurement_index)) =
          hypothesis.cost;
    }

    const std::vector<int> assignment = assignment_solver_.Solve(cost_matrix);

    for (std::size_t r = 0; r < rows; ++r) {
      const int assigned_col = assignment[r];
      if (assigned_col < 0 || static_cast<std::size_t>(assigned_col) >= cols) {
        continue;
      }

      const float matched_cost =
          cost_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(assigned_col));
      if (matched_cost <= config_.unassigned_cost) {
        const std::size_t measurement_index = static_cast<std::size_t>(assigned_col);
        measurement_to_key[measurement_index] = previous_tracks_[r].key;
        measurement_match_cost[measurement_index] = matched_cost;
        track_matched[r] = 1U;
      }
    }

    for (std::size_t r = 0; r < rows; ++r) {
      if (track_matched[r] == 0U) {
        result.missed_track_keys.push_back(previous_tracks_[r].key);
      }
    }
  }

  std::vector<TrackSignature> next_tracks;
  next_tracks.reserve(measurements.size());

  for (std::size_t m = 0; m < measurements.size(); ++m) {
    std::uint64_t key = measurement_to_key[m];
    const bool matched_existing_track = key != kUnassociatedKey;
    if (key == kUnassociatedKey) {
      key = next_key_++;
    }

    const std::size_t target_index = measurement_indices[m];
    result.target_keys[target_index] = key;
    if (matched_existing_track) {
      result.matches.push_back(
          AssociationMatch{key, target_index, measurement_match_cost[m]});
    } else {
      result.unassociated_target_indices.push_back(target_index);
    }

    next_tracks.push_back(TrackSignature{key, measurements[m]});
  }

  previous_tracks_.swap(next_tracks);
  return result;
}

/// @brief 执行一次兼容旧接口的数据关联。
/// @param targets 当前周期输入目标特征集合。
/// @param detection_succeeded 探测成功标记。
/// @return 与目标索引对齐的稳定关联键列表。
std::vector<std::uint64_t> DataAssociationEngine::Associate(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded) {
  return AssociateDetections(targets, detection_succeeded).target_keys;
}

/// @brief 从目标特征中提取关联使用的三维特征向量。
/// @param target 输入目标特征。
/// @return 由速度、RCS 和加速度组成的特征向量。
Eigen::Vector3f DataAssociationEngine::BuildFeatureVector(
    const common::TargetFeature &target) const {
  return Eigen::Vector3f(target.current_track_speed, target.current_track_rcs,
                         target.current_track_acceleration);
}

} // namespace association
} // namespace signal
} // namespace airborne_radar
