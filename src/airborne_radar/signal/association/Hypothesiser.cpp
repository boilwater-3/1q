#include "airborne_radar/signal/association/Hypothesiser.h"

#include <limits>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace association {

DenseCostHypothesiser::DenseCostHypothesiser(FullMahalanobisDistanceMetric* distance_metric,
                                             float max_cost)
    : full_metric_(distance_metric), mutable_full_metric_(distance_metric), max_cost_(max_cost) {
  if (full_metric_ == nullptr) {
    PROJECT_LOG_ERROR("[DenseCostHypothesiser] rejected null distance metric.");
  }
}

DenseCostHypothesiser::DenseCostHypothesiser(const FullMahalanobisDistanceMetric* distance_metric,
                                             float max_cost)
    : full_metric_(distance_metric), max_cost_(max_cost) {
  if (full_metric_ == nullptr) {
    PROJECT_LOG_ERROR("[DenseCostHypothesiser] rejected null distance metric.");
  }
}

DenseCostHypothesiser::DenseCostHypothesiser(const MahalanobisDistanceMetric* distance_metric,
                                             float max_cost)
    : simple_metric_(distance_metric), max_cost_(max_cost) {
  if (simple_metric_ == nullptr) {
    PROJECT_LOG_ERROR("[DenseCostHypothesiser] rejected null distance metric.");
  }
}

float DenseCostHypothesiser::ComputeCost(const Eigen::Vector3f& predicted,
                                         const Eigen::Vector3f& measurement) const {
  if (full_metric_ != nullptr) {
    return full_metric_->Compute(predicted, measurement);
  }
  if (simple_metric_ != nullptr) {
    return simple_metric_->Compute(predicted, measurement);
  }
  return std::numeric_limits<float>::infinity();
}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements) const {
  std::vector<AssociationHypothesis> hypotheses;
  if (full_metric_ == nullptr && simple_metric_ == nullptr) {
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] cannot generate hypotheses without distance metric.");
    return hypotheses;
  }
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      const float cost =
          ComputeCost(predicted_tracks[track_index], measurements[measurement_index]);
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
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] rejected innovation covariance count {} for {} tracks.",
        innovation_covariances.size(), predicted_tracks.size());
    return std::vector<AssociationHypothesis>();
  }

  if (mutable_full_metric_ == nullptr) {
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] track-wise innovation covariance requires mutable full "
        "Mahalanobis metric.");
    return std::vector<AssociationHypothesis>();
  }

  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    mutable_full_metric_->SetInnovationCovariance(innovation_covariances[track_index]);
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      const float cost =
          ComputeCost(predicted_tracks[track_index], measurements[measurement_index]);
      if (cost > max_cost_) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  // 重置 metric 内部 LLT 状态，防止下次调用基础 2-arg Generate 时继承过期协方差。
  mutable_full_metric_->SetInnovationCovariance(Eigen::Matrix3f::Identity());

  return hypotheses;
}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements,
    const std::vector<Eigen::Matrix3f>& projected_measurement_covariances,
    const std::vector<Eigen::Matrix3f>& measurement_covariances) const {
  if (projected_measurement_covariances.size() != predicted_tracks.size()) {
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] rejected projected covariance count {} for {} tracks.",
        projected_measurement_covariances.size(), predicted_tracks.size());
    return std::vector<AssociationHypothesis>();
  }
  if (measurement_covariances.size() != measurements.size()) {
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] rejected measurement covariance count {} for {} measurements.",
        measurement_covariances.size(), measurements.size());
    return std::vector<AssociationHypothesis>();
  }

  if (mutable_full_metric_ == nullptr) {
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] dynamic measurement covariance requires mutable full Mahalanobis "
        "metric.");
    return std::vector<AssociationHypothesis>();
  }

  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      mutable_full_metric_->SetInnovationCovariance(projected_measurement_covariances[track_index] +
                                                    measurement_covariances[measurement_index]);
      const float cost =
          ComputeCost(predicted_tracks[track_index], measurements[measurement_index]);
      if (cost > max_cost_) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  // 重置 metric 内部 LLT 状态，防止下次调用基础 2-arg Generate 时继承过期协方差。
  mutable_full_metric_->SetInnovationCovariance(Eigen::Matrix3f::Identity());

  return hypotheses;
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
