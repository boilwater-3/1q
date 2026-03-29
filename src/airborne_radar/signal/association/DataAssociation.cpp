#include "airborne_radar/signal/association/DataAssociation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace association {

namespace {
/**
 * @brief 未关联目标使用的保留键值。
 * @note 代码行为依据：关联结果会用该值区分“命中已有轨迹”与“需要分配新键”的目标。
 */
constexpr std::uint64_t kUnassociatedKey = 0;
/**
 * @brief 触发关联输入契约违例并终止进程。
 * @param message 违例信息。
 * @param index 违例目标索引或轨迹键。
 */
[[noreturn]] void AbortContractViolation(const char* message, std::size_t index) {
  if (PROJECT_LOG_HAS_DEFAULT_LOGGER()) {
    PROJECT_LOG_CRITICAL("[DataAssociationEngine] Contract violation at target[{}]: {}", index,
                         message);
    PROJECT_LOG_FLUSH_DEFAULT();
  }
  std::fprintf(stderr, "[DataAssociationEngine] Contract violation at target[%zu]: %s\n", index,
               message);
  std::fflush(stderr);
  std::abort();
}
/**
 * @brief 构建位置量测矩阵 H（3x6）。
 * @return 位置提取矩阵。
 */
tracking::MeasurementMatrix BuildPositionMeasurementMatrix() {
  tracking::MeasurementMatrix H = tracking::MeasurementMatrix::Zero();
  H(0, 0) = 1.0f;
  H(1, 2) = 1.0f;
  H(2, 4) = 1.0f;
  return H;
}
/**
 * @brief 构建默认量测协方差矩阵。
 * @param measurement_noise_std 量测标准差。
 * @return 对角协方差矩阵。
 */
tracking::MeasurementCovariance BuildDefaultMeasurementCovariance(float measurement_noise_std) {
  const float variance = measurement_noise_std * measurement_noise_std;
  return tracking::MeasurementCovariance::Identity() * variance;
}
/**
 * @brief 关联先验来源文本化。
 * @param using_external_seeds 是否使用外部 seeds。
 * @return 先验来源字符串。
 */
const char* AssociationPriorSourceName(bool using_external_seeds) {
  if (using_external_seeds) {
    return "external-seeds";
  }
  return "stateless";
}
/**
 * @brief 安全比值计算。
 * @param numerator 分子。
 * @param denominator 分母。
 * @return 分母为 0 时返回 0，否则返回浮点比值。
 */
float SafeRatio(std::size_t numerator, std::size_t denominator) {
  if (denominator == 0U) {
    return 0.0f;
  }
  return static_cast<float>(numerator) / static_cast<float>(denominator);
}
/**
 * @brief 计算 P95 的 nearest-rank 下标。
 * @param sample_size 样本数。
 * @return 排序后对应的 P95 下标。
 */
std::size_t ComputeNearestRankP95Index(std::size_t sample_size) {
  if (sample_size == 0U) {
    return 0U;
  }
  const std::size_t rank = (95U * sample_size + 99U) / 100U;
  return rank > 0U ? rank - 1U : 0U;
}
/**
 * @brief 构建关联质量指标汇总。
 * @param prior_track_count 先验轨迹数量。
 * @param detection_count 本周期检测数量。
 * @param matches 命中列表。
 * @param new_track_count 新建轨迹数量。
 * @param missed_track_count 失配轨迹数量。
 * @return 聚合后的质量指标。
 */
AssociationQualityMetrics BuildAssociationQualityMetrics(
    std::size_t prior_track_count, std::size_t detection_count,
    const std::vector<AssociationMatch>& matches, std::size_t new_track_count,
    std::size_t missed_track_count) {
  AssociationQualityMetrics metrics;
  metrics.prior_track_count = prior_track_count;
  metrics.detection_count = detection_count;
  metrics.matched_count = matches.size();
  metrics.new_track_count = new_track_count;
  metrics.missed_track_count = missed_track_count;
  metrics.match_rate = SafeRatio(metrics.matched_count, detection_count);
  metrics.new_track_rate = SafeRatio(new_track_count, detection_count);
  metrics.missed_track_rate = SafeRatio(missed_track_count, prior_track_count);

  if (matches.empty()) {
    return metrics;
  }

  float sum_match_cost = 0.0f;
  std::vector<float> sorted_costs;
  sorted_costs.reserve(matches.size());
  for (std::size_t i = 0; i < matches.size(); ++i) {
    sum_match_cost += matches[i].cost;
    sorted_costs.push_back(matches[i].cost);
  }
  metrics.mean_match_cost = sum_match_cost / static_cast<float>(matches.size());

  std::sort(sorted_costs.begin(), sorted_costs.end());
  metrics.p95_match_cost = sorted_costs[ComputeNearestRankP95Index(sorted_costs.size())];
  return metrics;
}

}  // namespace

DataAssociationEngine::DataAssociationEngine(DataAssociationConfig config)
    : config_(config),
      full_distance_metric_(Eigen::Matrix3f::Identity() * (config.kalman_measurement_noise_std *
                                                           config.kalman_measurement_noise_std)),
      gater_(config.unassigned_cost),
      position_hypothesiser_(&full_distance_metric_, &gater_),
      kalman_predictor_(tracking::KalmanPredictorConfig()) {
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  kalman_predictor_.UpdateConfig(predictor_config);
}

void DataAssociationEngine::UpdateConfig(DataAssociationConfig config) {
  config_ = config;
  full_distance_metric_ = FullMahalanobisDistanceMetric(
      Eigen::Matrix3f::Identity() *
      (config.kalman_measurement_noise_std * config.kalman_measurement_noise_std));
  gater_ = CostThresholdGater(config.unassigned_cost);
  position_hypothesiser_ = DenseCostHypothesiser(&full_distance_metric_, &gater_);
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  kalman_predictor_.UpdateConfig(predictor_config);
}

AssociationResult DataAssociationEngine::AssociateDetections(
    const common::model::TargetFeatureList& targets,
    const std::vector<std::uint8_t>& detection_succeeded, float dt_sec) {
  std::vector<tracking::MeasurementCovariance> measurement_covariances(
      targets.size(), BuildDefaultMeasurementCovariance(config_.kalman_measurement_noise_std));
  return AssociateDetections(targets, detection_succeeded, measurement_covariances, dt_sec);
}

AssociationResult DataAssociationEngine::AssociateDetections(
    const common::model::TargetFeatureList& targets, const std::vector<std::uint8_t>& detection_succeeded,
    const std::vector<tracking::MeasurementCovariance>& measurement_covariances, float dt_sec) {
  const std::size_t target_count = targets.size();
  AssociationResult result;
  result.target_keys.resize(target_count, kUnassociatedKey);

  if (measurement_covariances.size() != target_count) {
    AbortContractViolation("measurement_covariances size must match targets size", target_count);
  }

  std::vector<std::size_t> measurement_indices;
  measurement_indices.reserve(target_count);
  for (std::size_t i = 0; i < target_count; ++i) {
    if (i < detection_succeeded.size() && detection_succeeded[i] != 0U) {
      measurement_indices.push_back(i);
    }
  }

  const bool using_external_seeds = UsingExternalSeeds();
  const std::vector<ExternalSeedTrackSignature>& external_priors = external_seed_tracks_;
  const std::size_t association_prior_count = using_external_seeds ? external_priors.size() : 0U;
  const char* prior_source = AssociationPriorSourceName(using_external_seeds);

  if (measurement_indices.empty()) {
    result.missed_track_keys.reserve(association_prior_count);
    if (using_external_seeds) {
      for (std::size_t i = 0; i < external_priors.size(); ++i) {
        result.missed_track_keys.push_back(external_priors[i].key);
      }
    }
    result.quality_metrics = BuildAssociationQualityMetrics(
        association_prior_count, 0U, result.matches, 0U, result.missed_track_keys.size());
    PROJECT_LOG_DEBUG(
        "[DataAssociationEngine] cycle summary: prior_source={} priors={} detections=0 matches=0 "
        "missed_tracks={} new_tracks=0",
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
        BuildExternalPositionAssociationPriors(external_priors, dt_sec);
    const std::size_t rows = association_prior_count;
    const std::size_t cols = measurements.size();
    const std::size_t dim = std::max(rows, cols);
    const float rejected_cost = std::nextafter(config_.unassigned_cost,
                                               std::numeric_limits<float>::infinity());

    Eigen::MatrixXf cost_matrix = Eigen::MatrixXf::Constant(
        static_cast<Eigen::Index>(dim), static_cast<Eigen::Index>(dim), config_.unassigned_cost);
    for (std::size_t r = 0; r < rows; ++r) {
      for (std::size_t c = 0; c < cols; ++c) {
        cost_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = rejected_cost;
      }
    }

    std::vector<tracking::MeasurementCovariance> measurement_covariances_for_matches;
    measurement_covariances_for_matches.reserve(measurement_indices.size());
    for (std::size_t i = 0; i < measurement_indices.size(); ++i) {
      measurement_covariances_for_matches.push_back(
          measurement_covariances[measurement_indices[i]]);
    }

    const std::vector<AssociationHypothesis> hypotheses = position_hypothesiser_.Generate(
        priors.predicted_tracks, measurements, priors.projected_measurement_covariances,
        measurement_covariances_for_matches);
    for (const AssociationHypothesis& hypothesis : hypotheses) {
      cost_matrix(static_cast<Eigen::Index>(hypothesis.track_index),
                  static_cast<Eigen::Index>(hypothesis.measurement_index)) = hypothesis.cost;
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

  for (std::size_t m = 0; m < measurements.size(); ++m) {
    std::uint64_t key = measurement_to_key[m];
    const bool matched_existing_track = key != kUnassociatedKey;
    if (key == kUnassociatedKey) {
      key = next_key_++;
    }

    const std::size_t target_index = measurement_indices[m];
    result.target_keys[target_index] = key;
    if (matched_existing_track) {
      result.matches.push_back(AssociationMatch{key, target_index, measurement_match_cost[m]});
    } else {
      result.unassociated_target_indices.push_back(target_index);
    }
  }

  ClearConsumedAssociationPriors();

  result.quality_metrics = BuildAssociationQualityMetrics(
      association_prior_count, measurement_indices.size(), result.matches,
      result.unassociated_target_indices.size(), result.missed_track_keys.size());

  PROJECT_LOG_DEBUG(
      "[DataAssociationEngine] cycle summary: prior_source={} priors={} detections={} matches={} "
      "missed_tracks={} new_tracks={}",
      prior_source, association_prior_count, measurement_indices.size(), result.matches.size(),
      result.missed_track_keys.size(), result.unassociated_target_indices.size());
  return result;
}

std::vector<std::uint64_t> DataAssociationEngine::Associate(
    const common::model::TargetFeatureList& targets,
    const std::vector<std::uint8_t>& detection_succeeded) {
  return AssociateDetections(targets, detection_succeeded).target_keys;
}

std::vector<std::uint64_t> DataAssociationEngine::Associate(
    const common::model::TargetFeatureList& targets, const std::vector<std::uint8_t>& detection_succeeded,
    const std::vector<tracking::MeasurementCovariance>& measurement_covariances) {
  return AssociateDetections(targets, detection_succeeded, measurement_covariances).target_keys;
}

void DataAssociationEngine::SetAssociationSeeds(
    const std::vector<tracking::AssociationTrackSeed>& seeds) {
  external_seed_tracks_.clear();
  external_seed_tracks_.reserve(seeds.size());
  association_seed_mode_ = AssociationSeedMode::kExternalSeeds;

  for (std::size_t i = 0; i < seeds.size(); ++i) {
    const tracking::AssociationTrackSeed& seed = seeds[i];
    if (!seed.has_position) {
      AbortContractViolation("external association seed missing cartesian position", i);
    }
    if (!seed.has_gaussian_state) {
      AbortContractViolation("external association seed missing gaussian state", i);
    }

    ExternalSeedTrackSignature signature(seed.association_key);
    signature.has_position = seed.has_position;
    signature.position = seed.position;
    signature.has_gaussian_state = seed.has_gaussian_state;
    signature.gaussian_state = seed.gaussian_state;
    external_seed_tracks_.push_back(signature);
  }
  PROJECT_LOG_DEBUG("[DataAssociationEngine] accepted {} external association seeds",
                    external_seed_tracks_.size());
}

void DataAssociationEngine::ResetAssociationSeedModeToStateless() {
  external_seed_tracks_.clear();
  association_seed_mode_ = AssociationSeedMode::kStateless;
}

bool DataAssociationEngine::UsingExternalSeeds() const {
  return association_seed_mode_ == AssociationSeedMode::kExternalSeeds;
}

void DataAssociationEngine::ClearConsumedAssociationPriors() {
  external_seed_tracks_.clear();
  association_seed_mode_ = AssociationSeedMode::kStateless;
}

DataAssociationEngine::PositionAssociationPriors
DataAssociationEngine::BuildExternalPositionAssociationPriors(
    const std::vector<ExternalSeedTrackSignature>& external_priors, float dt_sec) const {
  PositionAssociationPriors priors;
  priors.keys.reserve(external_priors.size());
  priors.predicted_tracks.reserve(external_priors.size());
  priors.projected_measurement_covariances.reserve(external_priors.size());

  for (std::size_t i = 0; i < external_priors.size(); ++i) {
    const ExternalSeedTrackSignature& track = external_priors[i];
    if (!track.has_position) {
      AbortContractViolation(
          "association prior missing cartesian position under position-only mode", track.key);
    }
    if (!track.has_gaussian_state) {
      AbortContractViolation("association prior missing gaussian state under external-seed mode",
                             track.key);
    }

    const tracking::GaussianTrackState predicted =
        kalman_predictor_.Predict(track.gaussian_state, dt_sec);
    priors.keys.push_back(track.key);
    priors.predicted_tracks.push_back(
        Eigen::Vector3f(predicted.mean(0), predicted.mean(2), predicted.mean(4)));
    priors.projected_measurement_covariances.push_back(
        ComputeProjectedMeasurementCovariance(predicted));
  }

  return priors;
}

Eigen::Vector3f DataAssociationEngine::BuildPositionVector(
    const common::model::TargetFeature& target) const {
  return Eigen::Vector3f(target.position_x, target.position_y, target.position_z);
}

bool DataAssociationEngine::HasPositionMeasurement(const common::model::TargetFeature& target) const {
  return target.has_cartesian_position ||
         target.position_x != 0.0f || target.position_y != 0.0f || target.position_z != 0.0f;
}

void DataAssociationEngine::ValidateDetectedTargetsHavePosition(
    const common::model::TargetFeatureList& targets,
    const std::vector<std::uint8_t>& detection_succeeded) const {
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
    const Eigen::Vector3f& position) const {
  tracking::StateVector mean = tracking::StateVector::Zero();
  mean(0) = position(0);
  mean(2) = position(1);
  mean(4) = position(2);

  tracking::StateCovariance covariance = tracking::StateCovariance::Identity() * 100.0f;
  return tracking::GaussianTrackState(mean, covariance);
}

tracking::MeasurementCovariance DataAssociationEngine::ComputeProjectedMeasurementCovariance(
    const tracking::GaussianTrackState& predicted) const {
  const tracking::MeasurementMatrix H = BuildPositionMeasurementMatrix();
  return H * predicted.covariance * H.transpose();
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
