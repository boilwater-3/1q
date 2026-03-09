// Copyright 2026. All Rights Reserved.
//
// Description: Data association implementation.

#include "airborne_radar/signal/association/DataAssociation.h"

#include <algorithm>

#include "airborne_radar/signal/association/LapjvSolver.h"

namespace airborne_radar {
namespace signal {
namespace association {

namespace {

constexpr std::uint64_t kUnassociatedKey = 0;

} // namespace

DataAssociationEngine::DataAssociationEngine(DataAssociationConfig config)
    : config_(config), next_key_(1), previous_tracks_() {}

std::vector<std::uint64_t> DataAssociationEngine::Associate(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded) {
  const std::size_t target_count = targets.size();
  std::vector<std::uint64_t> association_keys(target_count, kUnassociatedKey);

  std::vector<std::size_t> measurement_indices;
  measurement_indices.reserve(target_count);
  for (std::size_t i = 0; i < target_count; ++i) {
    if (i < detection_succeeded.size() && detection_succeeded[i] != 0U) {
      measurement_indices.push_back(i);
    }
  }

  if (measurement_indices.empty()) {
    previous_tracks_.clear();
    return association_keys;
  }

  std::vector<Eigen::Vector3f> measurements;
  measurements.reserve(measurement_indices.size());
  for (std::size_t i = 0; i < measurement_indices.size(); ++i) {
    measurements.push_back(BuildFeatureVector(targets[measurement_indices[i]]));
  }

  std::vector<std::uint64_t> measurement_to_key(measurements.size(), kUnassociatedKey);

  if (!previous_tracks_.empty()) {
    const std::size_t rows = previous_tracks_.size();
    const std::size_t cols = measurements.size();
    const std::size_t dim = std::max(rows, cols);

    Eigen::MatrixXf cost_matrix =
        Eigen::MatrixXf::Constant(static_cast<Eigen::Index>(dim),
                                  static_cast<Eigen::Index>(dim),
                                  config_.unassigned_cost);

    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < cols; ++c) {
        cost_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
            ComputeMahalanobisSquared(previous_tracks_[r].feature, measurements[c]);
      }
    }

    LapjvSolver solver;
    const std::vector<int> assignment = solver.Solve(cost_matrix);

    for (std::size_t r = 0; r < rows; ++r) {
      const int assigned_col = assignment[r];
      if (assigned_col < 0 || static_cast<std::size_t>(assigned_col) >= cols) {
        continue;
      }

      const float matched_cost =
          cost_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(assigned_col));
      if (matched_cost <= config_.unassigned_cost) {
        measurement_to_key[static_cast<std::size_t>(assigned_col)] = previous_tracks_[r].key;
      }
    }
  }

  std::vector<TrackSignature> next_tracks;
  next_tracks.reserve(measurements.size());

  for (std::size_t m = 0; m < measurements.size(); ++m) {
    std::uint64_t key = measurement_to_key[m];
    if (key == kUnassociatedKey) {
      key = next_key_++;
    }

    const std::size_t target_index = measurement_indices[m];
    association_keys[target_index] = key;
    next_tracks.push_back(TrackSignature{key, measurements[m]});
  }

  previous_tracks_.swap(next_tracks);
  return association_keys;
}

Eigen::Vector3f DataAssociationEngine::BuildFeatureVector(
    const common::TargetFeature &target) const {
  return Eigen::Vector3f(target.current_track_speed, target.current_track_rcs,
                         target.current_track_acceleration);
}

float DataAssociationEngine::ComputeMahalanobisSquared(
    const Eigen::Vector3f &predicted,
    const Eigen::Vector3f &measurement) const {
  const Eigen::Vector3f innovation = measurement - predicted;

  Eigen::Array3f inv_diag;
  inv_diag << 1.0f / (config_.speed_sigma * config_.speed_sigma),
      1.0f / (config_.rcs_sigma * config_.rcs_sigma),
      1.0f / (config_.acceleration_sigma * config_.acceleration_sigma);

  return innovation.array().square().matrix().dot(inv_diag.matrix());
}

} // namespace association
} // namespace signal
} // namespace airborne_radar
