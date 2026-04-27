#include "airborne_radar/signal/association/Hypothesiser.h"

#include <cstdio>
#include <cstdlib>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace association {

namespace {

[[noreturn]] void AbortContractViolation(const char* message) {
  if (PROJECT_LOG_HAS_DEFAULT_LOGGER()) {
    PROJECT_LOG_CRITICAL("[DenseCostHypothesiser] Contract violation: {}", message);
    PROJECT_LOG_FLUSH_DEFAULT();
  }
  std::fprintf(stderr, "[DenseCostHypothesiser] Contract violation: %s\n", message);
  std::fflush(stderr);
  std::abort();
}

}  // namespace

DenseCostHypothesiser::DenseCostHypothesiser(ICovarianceAwareDistanceMetric* distance_metric,
                                             float max_cost)
    : distance_metric_(distance_metric), covariance_metric_(distance_metric), max_cost_(max_cost) {
  if (distance_metric_ == nullptr) {
    AbortContractViolation("hypothesiser requires distance metric");
  }
}

DenseCostHypothesiser::DenseCostHypothesiser(const IDistanceMetric* distance_metric,
                                             float max_cost)
    : distance_metric_(distance_metric), max_cost_(max_cost) {
  if (distance_metric_ == nullptr) {
    AbortContractViolation("hypothesiser requires distance metric");
  }
}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements) const {
  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      const float cost =
          distance_metric_->Compute(predicted_tracks[track_index], measurements[measurement_index]);
      if (cost > max_cost_) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  return hypotheses;
}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements,
    const std::vector<Eigen::Matrix3f>& innovation_covariances) const {
  if (innovation_covariances.size() != predicted_tracks.size()) {
    AbortContractViolation("innovation_covariances size must match predicted_tracks size");
  }

  if (covariance_metric_ == nullptr) {
    AbortContractViolation(
        "track-wise innovation covariance requires FullMahalanobisDistanceMetric");
  }

  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    covariance_metric_->SetInnovationCovariance(innovation_covariances[track_index]);
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      const float cost =
          distance_metric_->Compute(predicted_tracks[track_index], measurements[measurement_index]);
      if (cost > max_cost_) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  // 重置 metric 内部 LLT 状态，防止下次调用基础 2-arg Generate 时继承过期协方差。
  covariance_metric_->SetInnovationCovariance(Eigen::Matrix3f::Identity());

  return hypotheses;
}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements,
    const std::vector<Eigen::Matrix3f>& projected_measurement_covariances,
    const std::vector<Eigen::Matrix3f>& measurement_covariances) const {
  if (projected_measurement_covariances.size() != predicted_tracks.size()) {
    AbortContractViolation(
        "projected_measurement_covariances size must match predicted_tracks size");
  }
  if (measurement_covariances.size() != measurements.size()) {
    AbortContractViolation("measurement_covariances size must match measurements size");
  }

  if (covariance_metric_ == nullptr) {
    AbortContractViolation("dynamic measurement covariance requires FullMahalanobisDistanceMetric");
  }

  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      covariance_metric_->SetInnovationCovariance(projected_measurement_covariances[track_index] +
                                                  measurement_covariances[measurement_index]);
      const float cost =
          distance_metric_->Compute(predicted_tracks[track_index], measurements[measurement_index]);
      if (cost > max_cost_) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  // 重置 metric 内部 LLT 状态，防止下次调用基础 2-arg Generate 时继承过期协方差。
  covariance_metric_->SetInnovationCovariance(Eigen::Matrix3f::Identity());

  return hypotheses;
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
