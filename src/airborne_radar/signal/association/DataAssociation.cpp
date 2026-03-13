// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现基于距离度量、波门与指派求解的数据关联流程。

#include "airborne_radar/signal/association/DataAssociation.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace signal {
namespace association {

namespace {

/// @brief 未关联目标使用的保留键值。
constexpr std::uint64_t kUnassociatedKey = 0;

[[noreturn]] void AbortContractViolation(const char *message,
                                        std::size_t index) {
  if (spdlog::default_logger_raw() != nullptr) {
    spdlog::critical(
        "[DataAssociationEngine] Contract violation at target[{}]: {}",
        index, message);
    spdlog::default_logger_raw()->flush();
  }
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
  const float variance = measurement_noise_std * measurement_noise_std;
  return tracking::MeasurementCovariance::Identity() * variance;
}

const char *AssociationPriorSourceName(bool using_external_seeds,
                                       bool allow_internal_fallback_history) {
  if (using_external_seeds) {
    return "external-seeds";
  }
  if (allow_internal_fallback_history) {
    return "fallback-history";
  }
  return "stateless";
}

} // namespace

/// @brief 构造数据关联引擎并初始化内部组件。
/// @param config 关联配置。
DataAssociationEngine::DataAssociationEngine(DataAssociationConfig config)
  : config_(config),
    full_distance_metric_(Eigen::Matrix3f::Identity() *
              (config.kalman_measurement_noise_std *
               config.kalman_measurement_noise_std)),
    gater_(config.unassigned_cost),
    position_hypothesiser_(&full_distance_metric_, &gater_),
    kalman_predictor_(tracking::KalmanPredictorConfig()),
    next_key_(1),
    fallback_cache_(),
    external_seed_tracks_(),
    association_seed_mode_(AssociationSeedMode::kFallbackHistoryCache) {
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  kalman_predictor_.UpdateConfig(predictor_config);

}

void DataAssociationEngine::SetInternalHistoryFallbackEnabled(bool enabled) {
  fallback_cache_.SetEnabled(enabled);
  spdlog::info(
      "[DataAssociationEngine] internal history fallback {}",
      enabled ? "enabled" : "disabled");
}

void DataAssociationEngine::UpdateConfig(DataAssociationConfig config) {
  config_ = config;
  full_distance_metric_ = FullMahalanobisDistanceMetric(
    Eigen::Matrix3f::Identity() *
    (config.kalman_measurement_noise_std *
     config.kalman_measurement_noise_std));
  gater_ = CostThresholdGater(config.unassigned_cost);
  position_hypothesiser_ = DenseCostHypothesiser(&full_distance_metric_, &gater_);
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  kalman_predictor_.UpdateConfig(predictor_config);

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

  const bool using_external_seeds = UsingExternalSeeds();
  const bool persist_internal_fallback_history = AllowInternalFallbackHistory();
  const std::vector<ExternalSeedTrackSignature> &external_priors =
      external_seed_tracks_;
  const std::vector<FallbackTrackSignature> &fallback_priors =
      fallback_cache_.GetTracks();
  const std::size_t association_prior_count =
      using_external_seeds ? external_priors.size() : fallback_priors.size();
  const char *prior_source = AssociationPriorSourceName(
      using_external_seeds, persist_internal_fallback_history);

  if (measurement_indices.empty()) {
    result.missed_track_keys.reserve(association_prior_count);
    if (using_external_seeds) {
      for (std::size_t i = 0; i < external_priors.size(); ++i) {
        result.missed_track_keys.push_back(external_priors[i].key);
      }
    } else {
      for (std::size_t i = 0; i < fallback_priors.size(); ++i) {
        result.missed_track_keys.push_back(fallback_priors[i].key);
      }
    }
    spdlog::debug(
        "[DataAssociationEngine] cycle summary: prior_source={} priors={} detections=0 matches=0 missed_tracks={} new_tracks=0",
        prior_source, association_prior_count, result.missed_track_keys.size());
    ClearConsumedAssociationPriors();
    return result;
  }

  ValidateDetectedTargetsHavePosition(targets, detection_succeeded);
  result.used_position_association = true;
  result.used_external_association_seeds = using_external_seeds;

  std::vector<Eigen::Vector3f> measurements;
  measurements.reserve(measurement_indices.size());
  for (std::size_t i = 0; i < measurement_indices.size(); ++i) {
    const std::size_t target_index = measurement_indices[i];
    measurements.push_back(BuildPositionVector(targets[target_index]));
  }

  std::vector<std::uint64_t> measurement_to_key(measurements.size(), kUnassociatedKey);
  std::vector<float> measurement_match_cost(measurements.size(), 0.0f);
  std::vector<std::uint8_t> track_matched(association_prior_count, 0U);

  if (association_prior_count > 0U) {
    const PositionAssociationPriors priors =
        using_external_seeds
            ? BuildExternalPositionAssociationPriors(external_priors)
            : BuildFallbackPositionAssociationPriors(fallback_priors);
    const std::size_t rows = association_prior_count;
    const std::size_t cols = measurements.size();
    const std::size_t dim = std::max(rows, cols);
    const float rejected_cost = config_.unassigned_cost + 1.0f;

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
      position_hypothesiser_.Generate(
        priors.predicted_tracks, measurements,
        priors.projected_measurement_covariances,
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
        measurement_to_key[measurement_index] = priors.keys[r];
        measurement_match_cost[measurement_index] = matched_cost;
        track_matched[r] = 1U;
      }
    }

    for (std::size_t r = 0; r < rows; ++r) {
      if (track_matched[r] == 0U) {
        result.missed_track_keys.push_back(priors.keys[r]);
      }
    }
  }

  std::vector<FallbackTrackSignature> next_tracks;
  if (persist_internal_fallback_history) {
    next_tracks.reserve(measurements.size());
  }

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

    if (!persist_internal_fallback_history) {
      continue;
    }

    next_tracks.push_back(BuildFallbackTrackSignature(key, measurements[m]));
  }

  if (using_external_seeds) {
    ClearConsumedAssociationPriors();
  } else if (persist_internal_fallback_history) {
    ReplaceFallbackHistory(&next_tracks);
  } else {
    ClearConsumedAssociationPriors();
  }
  spdlog::debug(
      "[DataAssociationEngine] cycle summary: prior_source={} priors={} detections={} matches={} missed_tracks={} new_tracks={} persisted_fallback_history={}",
      prior_source, association_prior_count, measurement_indices.size(),
      result.matches.size(), result.missed_track_keys.size(),
      result.unassociated_target_indices.size(),
      persist_internal_fallback_history ? "true" : "false");
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
  fallback_cache_.Clear();
  external_seed_tracks_.clear();
  external_seed_tracks_.reserve(seeds.size());
  association_seed_mode_ = AssociationSeedMode::kExternalSeeds;

  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const tracking::AssociationTrackSeed &seed = seeds[i];
    if (!seed.has_position) {
      AbortContractViolation(
          "external association seed missing cartesian position", i);
    }
    if (!seed.has_gaussian_state) {
      AbortContractViolation(
          "external association seed missing gaussian state", i);
    }

    ExternalSeedTrackSignature signature(seed.association_key);
    signature.has_position = seed.has_position;
    signature.position = seed.position;
    signature.has_gaussian_state = seed.has_gaussian_state;
    signature.gaussian_state = seed.gaussian_state;
    external_seed_tracks_.push_back(signature);
  }
  spdlog::debug("[DataAssociationEngine] accepted {} external association seeds",
                external_seed_tracks_.size());
}

void DataAssociationEngine::ResetAssociationSeedModeToFallbackHistory() {
  external_seed_tracks_.clear();
  association_seed_mode_ = AssociationSeedMode::kFallbackHistoryCache;
}

bool DataAssociationEngine::UsingExternalSeeds() const {
  return association_seed_mode_ == AssociationSeedMode::kExternalSeeds;
}

bool DataAssociationEngine::AllowInternalFallbackHistory() const {
  return !UsingExternalSeeds() && fallback_cache_.IsEnabled();
}

void DataAssociationEngine::ClearConsumedAssociationPriors() {
  if (UsingExternalSeeds()) {
    external_seed_tracks_.clear();
  } else {
    fallback_cache_.Clear();
  }
  association_seed_mode_ = AssociationSeedMode::kFallbackHistoryCache;
}

void DataAssociationEngine::ReplaceFallbackHistory(
    std::vector<FallbackTrackSignature> *next_tracks) {
  fallback_cache_.Replace(next_tracks);
  external_seed_tracks_.clear();
  association_seed_mode_ = AssociationSeedMode::kFallbackHistoryCache;
}

const std::vector<DataAssociationEngine::FallbackTrackSignature> &
DataAssociationEngine::FallbackHistoryCache::EmptyTracks() {
  static const std::vector<DataAssociationEngine::FallbackTrackSignature> kEmptyTracks;
  return kEmptyTracks;
}

void DataAssociationEngine::FallbackHistoryCache::SetEnabled(bool enabled_value) {
  enabled = enabled_value;
  if (!enabled) {
    tracks.clear();
  }
}

bool DataAssociationEngine::FallbackHistoryCache::IsEnabled() const {
  return enabled;
}

const std::vector<DataAssociationEngine::FallbackTrackSignature> &
DataAssociationEngine::FallbackHistoryCache::GetTracks() const {
  return enabled ? tracks : EmptyTracks();
}

void DataAssociationEngine::FallbackHistoryCache::Clear() {
  tracks.clear();
}

void DataAssociationEngine::FallbackHistoryCache::Replace(
    std::vector<FallbackTrackSignature> *next_tracks) {
  if (!enabled) {
    tracks.clear();
    return;
  }
  tracks.swap(*next_tracks);
}

DataAssociationEngine::FallbackTrackSignature
DataAssociationEngine::BuildFallbackTrackSignature(
    std::uint64_t key,
    const Eigen::Vector3f &measurement) const {
  FallbackTrackSignature signature(key);
  signature.position = measurement;
  return signature;
}

DataAssociationEngine::PositionAssociationPriors
DataAssociationEngine::BuildExternalPositionAssociationPriors(
    const std::vector<ExternalSeedTrackSignature> &external_priors) const {
  PositionAssociationPriors priors;
  priors.keys.reserve(external_priors.size());
  priors.predicted_tracks.reserve(external_priors.size());
  priors.projected_measurement_covariances.reserve(external_priors.size());

  for (std::size_t i = 0; i < external_priors.size(); ++i) {
    const ExternalSeedTrackSignature &track = external_priors[i];
    if (!track.has_position) {
      AbortContractViolation(
          "association prior missing cartesian position under position-only mode",
          track.key);
    }
    if (!track.has_gaussian_state) {
      AbortContractViolation(
          "association prior missing gaussian state under external-seed mode",
          track.key);
    }

    const tracking::GaussianTrackState predicted =
        kalman_predictor_.Predict(track.gaussian_state, 1.0f);
    priors.keys.push_back(track.key);
    priors.predicted_tracks.push_back(
        Eigen::Vector3f(predicted.mean(0), predicted.mean(2), predicted.mean(4)));
    priors.projected_measurement_covariances.push_back(
        ComputeProjectedMeasurementCovariance(predicted));
  }

  return priors;
}

DataAssociationEngine::PositionAssociationPriors
DataAssociationEngine::BuildFallbackPositionAssociationPriors(
    const std::vector<FallbackTrackSignature> &fallback_priors) const {
  PositionAssociationPriors priors;
  priors.keys.reserve(fallback_priors.size());
  priors.predicted_tracks.reserve(fallback_priors.size());
  priors.projected_measurement_covariances.reserve(fallback_priors.size());

  for (std::size_t i = 0; i < fallback_priors.size(); ++i) {
    const FallbackTrackSignature &track = fallback_priors[i];
    const tracking::GaussianTrackState predicted =
        InitializeGaussianState(track.position);
    priors.keys.push_back(track.key);
    // fallback 明确为 position-only：直接用缓存位置作为预测位置。
    priors.predicted_tracks.push_back(track.position);
    priors.projected_measurement_covariances.push_back(
        ComputeProjectedMeasurementCovariance(predicted));
  }

  return priors;
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
