#include "airborne_radar/signal/association/Hypothesiser.h"

#include <stdexcept>

namespace airborne_radar {
namespace signal {
namespace association {

DenseCostHypothesiser::DenseCostHypothesiser(IDistanceMetric* distance_metric, const IGater* gater)
    : distance_metric_(distance_metric), gater_(gater) {
  if (distance_metric_ == nullptr || gater_ == nullptr) {
    throw std::invalid_argument("hypothesiser requires distance metric and gater");
  }
}

DenseCostHypothesiser::DenseCostHypothesiser(const IDistanceMetric* distance_metric,
                                             const IGater* gater)
    : DenseCostHypothesiser(const_cast<IDistanceMetric*>(distance_metric), gater) {}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements) const {
  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      const float cost =
          distance_metric_->Compute(predicted_tracks[track_index], measurements[measurement_index]);
      if (!gater_->Accept(cost)) {
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
    throw std::invalid_argument("innovation_covariances size must match predicted_tracks size");
  }

  FullMahalanobisDistanceMetric* full_metric =
      dynamic_cast<FullMahalanobisDistanceMetric*>(distance_metric_);
  if (full_metric == nullptr) {
    throw std::invalid_argument(
        "track-wise innovation covariance requires FullMahalanobisDistanceMetric");
  }

  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    full_metric->SetInnovationCovariance(innovation_covariances[track_index]);
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      const float cost =
          distance_metric_->Compute(predicted_tracks[track_index], measurements[measurement_index]);
      if (!gater_->Accept(cost)) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  return hypotheses;
}

std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements,
    const std::vector<Eigen::Matrix3f>& projected_measurement_covariances,
    const std::vector<Eigen::Matrix3f>& measurement_covariances) const {
  if (projected_measurement_covariances.size() != predicted_tracks.size()) {
    throw std::invalid_argument(
        "projected_measurement_covariances size must match predicted_tracks size");
  }
  if (measurement_covariances.size() != measurements.size()) {
    throw std::invalid_argument("measurement_covariances size must match measurements size");
  }

  FullMahalanobisDistanceMetric* full_metric =
      dynamic_cast<FullMahalanobisDistanceMetric*>(distance_metric_);
  if (full_metric == nullptr) {
    throw std::invalid_argument(
        "dynamic measurement covariance requires FullMahalanobisDistanceMetric");
  }

  std::vector<AssociationHypothesis> hypotheses;
  hypotheses.reserve(predicted_tracks.size() * measurements.size());

  for (std::size_t track_index = 0; track_index < predicted_tracks.size(); ++track_index) {
    for (std::size_t measurement_index = 0; measurement_index < measurements.size();
         ++measurement_index) {
      full_metric->SetInnovationCovariance(projected_measurement_covariances[track_index] +
                                           measurement_covariances[measurement_index]);
      const float cost =
          distance_metric_->Compute(predicted_tracks[track_index], measurements[measurement_index]);
      if (!gater_->Accept(cost)) {
        continue;
      }

      hypotheses.push_back(AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  return hypotheses;
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
