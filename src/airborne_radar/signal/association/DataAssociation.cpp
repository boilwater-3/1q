// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现基于距离度量、波门与指派求解的数据关联流程。

#include "airborne_radar/signal/association/DataAssociation.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace airborne_radar {
namespace signal {
namespace association {

namespace {

/// @brief 未关联目标使用的保留键值。
constexpr std::uint64_t kUnassociatedKey = 0;

[[noreturn]] void AbortContractViolation(const char *message,
                                        std::size_t index) {
  std::fprintf(stderr,
               "[DataAssociationEngine] Contract violation at target[%zu]: %s\n",
               index, message);
  std::fflush(stderr);
  std::abort();
}

tracking::MeasurementMatrix BuildPositionMeasurementMatrix() {
  tracking::MeasurementMatrix H = tracking::MeasurementMatrix::Zero();
  H(0, 0) = 1.0f;
  H(1, 2) = 1.0f;
  H(2, 4) = 1.0f;
  return H;
}

tracking::MeasurementCovariance BuildDefaultMeasurementCovariance(
    float measurement_noise_std) {
  return tracking::MeasurementCovariance::Identity() *
         measurement_noise_std * measurement_noise_std;
}

} // namespace

/// @brief 构造数据关联引擎并初始化内部组件。
/// @param config 关联配置。
DataAssociationEngine::DataAssociationEngine(DataAssociationConfig config)
  : config_(config),
    full_distance_metric_(Eigen::Matrix3f::Identity() *
              config.kalman_measurement_noise_std *
              config.kalman_measurement_noise_std),
    gater_(config.unassigned_cost),
    position_hypothesiser_(&full_distance_metric_, &gater_),
    kalman_predictor_(tracking::KalmanPredictorConfig()),
    kalman_updater_(tracking::KalmanUpdaterConfig()),
    next_key_(1),
    fallback_history_tracks_(),
    external_seed_tracks_(),
    association_seed_mode_(AssociationSeedMode::kFallbackHistoryCache) {
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  kalman_predictor_.UpdateConfig(predictor_config);

  tracking::KalmanUpdaterConfig updater_config;
  updater_config.measurement_noise_std = config.kalman_measurement_noise_std;
  kalman_updater_.UpdateConfig(updater_config);
}

void DataAssociationEngine::UpdateConfig(DataAssociationConfig config) {
  config_ = config;
  full_distance_metric_ = FullMahalanobisDistanceMetric(
    Eigen::Matrix3f::Identity() * config.kalman_measurement_noise_std *
    config.kalman_measurement_noise_std);
  gater_ = CostThresholdGater(config.unassigned_cost);
  position_hypothesiser_ = DenseCostHypothesiser(&full_distance_metric_, &gater_);
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  kalman_predictor_.UpdateConfig(predictor_config);

  tracking::KalmanUpdaterConfig updater_config;
  updater_config.measurement_noise_std = config.kalman_measurement_noise_std;
  kalman_updater_.UpdateConfig(updater_config);
}

AssociationResult DataAssociationEngine::AssociateDetections(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded) {
  std::vector<tracking::MeasurementCovariance> measurement_covariances(
      targets.size(),
      BuildDefaultMeasurementCovariance(config_.kalman_measurement_noise_std));
  return AssociateDetections(targets, detection_succeeded,
                             measurement_covariances);
}

/// @brief 执行一次完整的数据关联。
/// @param targets 当前周期输入目标特征集合。
/// @param detection_succeeded 探测成功标记。
/// @return 结构化关联结果。
AssociationResult DataAssociationEngine::AssociateDetections(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded,
    const std::vector<tracking::MeasurementCovariance> &measurement_covariances) {
  const std::size_t target_count = targets.size();
  AssociationResult result;
  result.target_keys.resize(target_count, kUnassociatedKey);

  if (measurement_covariances.size() != target_count) {
    AbortContractViolation(
        "measurement_covariances size must match targets size", target_count);
  }

  std::vector<std::size_t> measurement_indices;
  measurement_indices.reserve(target_count);
  for (std::size_t i = 0; i < target_count; ++i) {
    if (i < detection_succeeded.size() && detection_succeeded[i] != 0U) {
      measurement_indices.push_back(i);
    }
  }

  const std::vector<TrackSignature> &association_priors =
      association_seed_mode_ == AssociationSeedMode::kExternalSeeds
          ? external_seed_tracks_
          : fallback_history_tracks_;

  if (measurement_indices.empty()) {
    result.missed_track_keys.reserve(association_priors.size());
    for (const TrackSignature &track : association_priors) {
      result.missed_track_keys.push_back(track.key);
    }
    fallback_history_tracks_.clear();
    external_seed_tracks_.clear();
    association_seed_mode_ = AssociationSeedMode::kFallbackHistoryCache;
    return result;
  }

  ValidateDetectedTargetsHavePosition(targets, detection_succeeded);
  result.used_position_association = true;
  result.used_external_association_seeds =
      association_seed_mode_ == AssociationSeedMode::kExternalSeeds;

  std::vector<Eigen::Vector3f> measurements;
  measurements.reserve(measurement_indices.size());
  for (std::size_t i = 0; i < measurement_indices.size(); ++i) {
    const std::size_t target_index = measurement_indices[i];
    measurements.push_back(BuildPositionVector(targets[target_index]));
  }

  std::vector<std::uint64_t> measurement_to_key(measurements.size(), kUnassociatedKey);
  std::vector<float> measurement_match_cost(measurements.size(), 0.0f);
  std::vector<std::uint8_t> track_matched(association_priors.size(), 0U);

  if (!association_priors.empty()) {
    const std::size_t rows = association_priors.size();
    const std::size_t cols = measurements.size();
    const std::size_t dim = std::max(rows, cols);
    const float rejected_cost = config_.unassigned_cost + 1.0f;
    FeatureVectorList predicted_tracks;
    predicted_tracks.reserve(rows);
    std::vector<tracking::GaussianTrackState> predicted_states;
    std::vector<Eigen::Matrix3f> projected_measurement_covariances;
    predicted_states.reserve(rows);
    projected_measurement_covariances.reserve(rows);
    for (const TrackSignature &track : association_priors) {
      if (!track.has_position) {
        AbortContractViolation(
            "association prior missing cartesian position under position-only mode",
            track.key);
      }

      if (track.has_gaussian_state) {
        const tracking::GaussianTrackState predicted =
            kalman_predictor_.Predict(track.gaussian_state, 1.0f);
        predicted_states.push_back(predicted);
        predicted_tracks.push_back(
            Eigen::Vector3f(predicted.mean(0), predicted.mean(2), predicted.mean(4)));
        projected_measurement_covariances.push_back(
            ComputeProjectedMeasurementCovariance(predicted));
      } else {
        const tracking::GaussianTrackState predicted =
            InitializeGaussianState(track.position);
        predicted_states.push_back(predicted);
        predicted_tracks.push_back(track.position);
        projected_measurement_covariances.push_back(
            ComputeProjectedMeasurementCovariance(predicted));
      }
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

    std::vector<tracking::MeasurementCovariance> measurement_covariances_for_matches;
    measurement_covariances_for_matches.reserve(measurement_indices.size());
    for (std::size_t i = 0; i < measurement_indices.size(); ++i) {
      measurement_covariances_for_matches.push_back(
          measurement_covariances[measurement_indices[i]]);
    }

    const std::vector<AssociationHypothesis> hypotheses =
        position_hypothesiser_.Generate(predicted_tracks, measurements,
                                        projected_measurement_covariances,
                                        measurement_covariances_for_matches);
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
        measurement_to_key[measurement_index] = association_priors[r].key;
        measurement_match_cost[measurement_index] = matched_cost;
        track_matched[r] = 1U;
      }
    }

    for (std::size_t r = 0; r < rows; ++r) {
      if (track_matched[r] == 0U) {
        result.missed_track_keys.push_back(association_priors[r].key);
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

    TrackSignature signature(key);
    signature.has_position = true;
    signature.position = measurements[m];

    if (matched_existing_track) {
      for (std::size_t row = 0; row < association_priors.size(); ++row) {
        if (association_priors[row].key != key) {
          continue;
        }

        const tracking::GaussianTrackState predicted =
            association_priors[row].has_gaussian_state
                ? kalman_predictor_.Predict(
                      association_priors[row].gaussian_state, 1.0f)
                : InitializeGaussianState(association_priors[row].position);
        tracking::MeasurementVector z;
        z << measurements[m](0), measurements[m](1), measurements[m](2);
        const tracking::KalmanUpdateResult update_result =
            kalman_updater_.Update(predicted, z,
                                   measurement_covariances[target_index]);
        signature.has_gaussian_state = true;
        signature.gaussian_state = update_result.posterior;
        break;
      }
    } else {
      signature.has_gaussian_state = true;
      signature.gaussian_state = InitializeGaussianState(measurements[m]);
    }

    next_tracks.push_back(signature);
  }

  if (association_seed_mode_ == AssociationSeedMode::kExternalSeeds) {
    external_seed_tracks_.clear();
  } else {
    fallback_history_tracks_.swap(next_tracks);
  }
  association_seed_mode_ = AssociationSeedMode::kFallbackHistoryCache;
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

std::vector<std::uint64_t> DataAssociationEngine::Associate(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded,
    const std::vector<tracking::MeasurementCovariance> &measurement_covariances) {
  return AssociateDetections(targets, detection_succeeded,
                             measurement_covariances)
      .target_keys;
}

void DataAssociationEngine::SetAssociationSeeds(
    const std::vector<tracking::AssociationTrackSeed> &seeds) {
  fallback_history_tracks_.clear();
  external_seed_tracks_.clear();
  external_seed_tracks_.reserve(seeds.size());
  association_seed_mode_ = AssociationSeedMode::kExternalSeeds;

  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const tracking::AssociationTrackSeed &seed = seeds[i];
    TrackSignature signature(seed.association_key);
    signature.has_position = seed.has_position;
    signature.position = seed.position;
    signature.has_gaussian_state = seed.has_gaussian_state;
    signature.gaussian_state = seed.gaussian_state;
    external_seed_tracks_.push_back(signature);
  }
}

Eigen::Vector3f DataAssociationEngine::BuildPositionVector(
    const common::TargetFeature &target) const {
  return Eigen::Vector3f(target.position_x, target.position_y, target.position_z);
}

bool DataAssociationEngine::HasPositionMeasurement(
    const common::TargetFeature &target) const {
  return target.position_x != 0.0f || target.position_y != 0.0f ||
         target.position_z != 0.0f;
}

void DataAssociationEngine::ValidateDetectedTargetsHavePosition(
    const common::TargetFeatureList &targets,
    const std::vector<std::uint8_t> &detection_succeeded) const {
  for (std::size_t i = 0; i < targets.size() && i < detection_succeeded.size(); ++i) {
    if (detection_succeeded[i] == 0U) {
      continue;
    }
    if (!HasPositionMeasurement(targets[i])) {
      AbortContractViolation("detected target is missing cartesian position", i);
    }
  }
}

tracking::GaussianTrackState DataAssociationEngine::InitializeGaussianState(
    const Eigen::Vector3f &position) const {
  tracking::StateVector mean = tracking::StateVector::Zero();
  mean(0) = position(0);
  mean(2) = position(1);
  mean(4) = position(2);

  tracking::StateCovariance covariance = tracking::StateCovariance::Identity() * 100.0f;
  return tracking::GaussianTrackState(mean, covariance);
}

tracking::MeasurementCovariance DataAssociationEngine::ComputeProjectedMeasurementCovariance(
    const tracking::GaussianTrackState &predicted) const {
  const tracking::MeasurementMatrix H = BuildPositionMeasurementMatrix();
  return H * predicted.covariance * H.transpose();
}

} // namespace association
} // namespace signal
} // namespace airborne_radar
