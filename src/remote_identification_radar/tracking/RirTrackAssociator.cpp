/**
 * @file RirTrackAssociator.cpp
 * @brief RIR 轻量跟踪子集的门限 + 最近邻关联器实现（阶段 2-T T2）。
 */

#include "remote_identification_radar/tracking/RirTrackAssociator.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <vector>

#include "common/estimation/IKalmanUpdater.h"
#include "common/logging/ProjectLog.h"

namespace remote_identification_radar {
namespace tracking {

namespace {

/** @brief 关联键保留值：未关联。 */
constexpr std::uint64_t kUnassociatedKey = 0U;

/** @brief 位置量测矩阵 H（状态序 [x, vx, y, vy, z, vz]）。 */
RirMeasurementMatrix BuildPositionMeasurementMatrix() {
  return ::oneq::common::estimation::IKalmanUpdater<6, 3>::BuildPositionMeasurementMatrix();
}

/** @brief 缺省对角量测协方差。 */
RirMeasurementCovariance BuildDefaultMeasurementCovariance(float std_dev) {
  const float variance = std_dev * std_dev;
  return RirMeasurementCovariance::Identity() * variance;
}

/** @brief 由高斯状态读取位置。 */
Eigen::Vector3f PositionOf(const RirGaussianState& state) {
  return Eigen::Vector3f(state.mean(0), state.mean(2), state.mean(4));
}

/** @brief 计算马氏距离平方；S 不可分解或结果非有限时返回无穷大。 */
float ComputeSquaredMahalanobisDistance(const Eigen::Vector3f& predicted_position,
                                        const Eigen::Vector3f& measurement_position,
                                        const RirMeasurementCovariance& innovation_covariance) {
  const Eigen::LLT<RirMeasurementCovariance> llt(innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<float>::infinity();
  }
  const Eigen::Vector3f innovation = measurement_position - predicted_position;
  const Eigen::Vector3f solved = llt.solve(innovation);
  const float distance = innovation.dot(solved);
  return std::isfinite(distance) ? distance : std::numeric_limits<float>::infinity();
}

}  // namespace

RirTrackAssociator::RirTrackAssociator(RirAssociationConfig config) : config_(config) {
  ::oneq::common::estimation::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  predictor_.UpdateConfig(predictor_config);
  next_key_ = config.initial_next_key == kUnassociatedKey ? 1U : config.initial_next_key;
}

RirAssociationResult RirTrackAssociator::Associate(
    const std::vector<RirTrackMeasurement>& measurements, const std::vector<RirTrackSeed>& seeds,
    float dt_sec) {
  RirAssociationResult result;
  const float effective_dt = (std::isfinite(dt_sec) && dt_sec > 0.0f) ? dt_sec : 0.0f;
  const RirMeasurementMatrix H = BuildPositionMeasurementMatrix();

  std::vector<std::size_t> viable_measurement_indices;
  std::vector<RirMeasurementCovariance> resolved_covariances;
  viable_measurement_indices.reserve(measurements.size());
  resolved_covariances.reserve(measurements.size());
  for (std::size_t i = 0U; i < measurements.size(); ++i) {
    if (!IsUsableMeasurement(measurements[i])) {
      // 中译：丢弃位置含非有限值的检测量测。
      // 标识：关联输入保护——该量测不进入门限与航迹更新，防止数值污染。
      PROJECT_LOG_WARN(
          "[RirTrackAssociator] dropped measurement at source_index={}: non-finite position",
          measurements[i].source_index);
      continue;
    }
    viable_measurement_indices.push_back(i);
    resolved_covariances.push_back(
        ResolveMeasurementCovariance(measurements[i].measurement_covariance));
  }

  struct SeedPrior {
    std::uint64_t key{0U};
    Eigen::Vector3f predicted_position{Eigen::Vector3f::Zero()};
    RirMeasurementCovariance projected_covariance{RirMeasurementCovariance::Zero()};
  };
  std::vector<SeedPrior> seed_priors;
  seed_priors.reserve(seeds.size());
  std::unordered_set<std::uint64_t> seen_seed_keys;
  for (std::size_t i = 0U; i < seeds.size(); ++i) {
    const RirTrackSeed& seed = seeds[i];
    if (seed.association_key == kUnassociatedKey) {
      // 中译：拒绝保留键 0 的关联种子。
      // 标识：关联输入契约——种子必须来自生命周期导出且携带有效键。
      PROJECT_LOG_WARN(
          "[RirTrackAssociator] rejected seed with reserved association_key=0 at index={}", i);
      continue;
    }
    if (!seed.has_gaussian_state) {
      // 中译：拒绝缺少高斯状态的关联种子，并将其计为本周期失配。
      // 标识：关联输入契约——无高斯状态无法预测波门。
      PROJECT_LOG_WARN("[RirTrackAssociator] rejected seed without gaussian state: key={}",
                       seed.association_key);
      result.missed_track_keys.push_back(seed.association_key);
      continue;
    }
    if (!seed.gaussian_state.mean.allFinite() || !seed.gaussian_state.covariance.allFinite()) {
      // 中译：拒绝含非有限高斯状态的关联种子。
      // 标识：数值保护——非有限状态无法形成有效波门。
      PROJECT_LOG_WARN("[RirTrackAssociator] rejected seed with non-finite gaussian state: key={}",
                       seed.association_key);
      result.missed_track_keys.push_back(seed.association_key);
      continue;
    }
    if (!seen_seed_keys.insert(seed.association_key).second) {
      // 中译：忽略重复关联键的种子。
      // 标识：输入去重——同键多种子只保留首个，防止重复分配同一量测。
      PROJECT_LOG_WARN("[RirTrackAssociator] ignored duplicate seed association_key={}",
                       seed.association_key);
      continue;
    }

    const RirGaussianState predicted = PredictSeed(seed, effective_dt);
    SeedPrior prior;
    prior.key = seed.association_key;
    prior.predicted_position = PositionOf(predicted);
    prior.projected_covariance = H * predicted.covariance * H.transpose();
    seed_priors.push_back(prior);
  }

  std::vector<Candidate> candidates;
  candidates.reserve(seed_priors.size() * viable_measurement_indices.size());
  for (std::size_t s = 0U; s < seed_priors.size(); ++s) {
    for (std::size_t m = 0U; m < viable_measurement_indices.size(); ++m) {
      const RirMeasurementCovariance innovation_covariance =
          seed_priors[s].projected_covariance + resolved_covariances[m];
      const RirTrackMeasurement& measurement = measurements[viable_measurement_indices[m]];
      const float cost = ComputeSquaredMahalanobisDistance(
          seed_priors[s].predicted_position, measurement.position, innovation_covariance);
      if (cost <= config_.gate_threshold) {
        candidates.push_back(Candidate{s, m, cost});
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
    if (lhs.cost != rhs.cost) {
      return lhs.cost < rhs.cost;
    }
    if (lhs.seed_index != rhs.seed_index) {
      return lhs.seed_index < rhs.seed_index;
    }
    return lhs.measurement_index < rhs.measurement_index;
  });

  std::vector<std::uint8_t> seed_matched(seed_priors.size(), 0U);
  std::vector<std::uint8_t> measurement_matched(viable_measurement_indices.size(), 0U);
  std::vector<std::uint64_t> matched_key_by_measurement(viable_measurement_indices.size(),
                                                        kUnassociatedKey);
  std::vector<float> matched_cost_by_measurement(viable_measurement_indices.size(), 0.0f);

  for (const Candidate& candidate : candidates) {
    if (seed_matched[candidate.seed_index] != 0U ||
        measurement_matched[candidate.measurement_index] != 0U) {
      continue;
    }
    seed_matched[candidate.seed_index] = 1U;
    measurement_matched[candidate.measurement_index] = 1U;
    matched_key_by_measurement[candidate.measurement_index] = seed_priors[candidate.seed_index].key;
    matched_cost_by_measurement[candidate.measurement_index] = candidate.cost;
  }

  for (std::size_t m = 0U; m < viable_measurement_indices.size(); ++m) {
    const std::size_t source_index = viable_measurement_indices[m];
    RirTrackMeasurement associated = measurements[source_index];
    if (matched_key_by_measurement[m] == kUnassociatedKey) {
      associated.association_key = next_key_++;
      associated.matched_existing_track = false;
    } else {
      associated.association_key = matched_key_by_measurement[m];
      associated.matched_existing_track = true;
      result.matches.push_back(RirAssociationMatch{associated.association_key, source_index,
                                                   matched_cost_by_measurement[m]});
    }
    result.measurements.push_back(associated);
  }

  for (std::size_t s = 0U; s < seed_priors.size(); ++s) {
    if (seed_matched[s] == 0U) {
      result.missed_track_keys.push_back(seed_priors[s].key);
    }
  }

  return result;
}

void RirTrackAssociator::UpdateConfig(RirAssociationConfig config) {
  config_ = config;
  ::oneq::common::estimation::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = config.kalman_noise_diff_coeff;
  predictor_.UpdateConfig(predictor_config);
}

void RirTrackAssociator::Reset() {
  next_key_ = config_.initial_next_key == kUnassociatedKey ? 1U : config_.initial_next_key;
}

RirAssociationRuntimeState RirTrackAssociator::CaptureRuntimeState() const {
  RirAssociationRuntimeState state;
  state.next_key = next_key_;
  return state;
}

void RirTrackAssociator::RestoreRuntimeState(const RirAssociationRuntimeState& state) {
  next_key_ = state.next_key;
}

bool RirTrackAssociator::IsUsableMeasurement(const RirTrackMeasurement& measurement) const {
  return measurement.position.allFinite();
}

RirMeasurementCovariance RirTrackAssociator::ResolveMeasurementCovariance(
    const RirMeasurementCovariance& covariance) const {
  // 零矩阵视为"未注入动态 R"，使用配置缺省对角 R（与 AR 无动态协方差重载一致）。
  if (!covariance.allFinite() || covariance.isZero(0.0f)) {
    return BuildDefaultMeasurementCovariance(config_.default_measurement_noise_std);
  }
  return covariance;
}

RirGaussianState RirTrackAssociator::PredictSeed(const RirTrackSeed& seed, float dt_sec) const {
  return predictor_.Predict(seed.gaussian_state, dt_sec);
}

}  // namespace tracking
}  // namespace remote_identification_radar
