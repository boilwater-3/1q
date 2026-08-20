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
    // 中译：拒绝空距离度量指针。
    // 标识：装配保护——未注入距离度量时记录错误，后续计算将退化，
    //       排查构造参数与依赖注入。
    PROJECT_LOG_ERROR("[DenseCostHypothesiser] rejected null distance metric.");
  }
}

DenseCostHypothesiser::DenseCostHypothesiser(const FullMahalanobisDistanceMetric* distance_metric,
                                             float max_cost)
    : full_metric_(distance_metric), max_cost_(max_cost) {
  if (full_metric_ == nullptr) {
    // 中译：拒绝空距离度量指针。
    // 标识：装配保护——未注入距离度量时记录错误，后续计算将退化，
    //       排查构造参数与依赖注入。
    PROJECT_LOG_ERROR("[DenseCostHypothesiser] rejected null distance metric.");
  }
}

DenseCostHypothesiser::DenseCostHypothesiser(const MahalanobisDistanceMetric* distance_metric,
                                             float max_cost)
    : simple_metric_(distance_metric), max_cost_(max_cost) {
  if (simple_metric_ == nullptr) {
    // 中译：拒绝空距离度量指针。
    // 标识：装配保护——未注入距离度量时记录错误，后续计算将退化，
    //       排查构造参数与依赖注入。
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
    // 中译：没有距离度量，无法生成关联假设。
    // 标识：装配保护——度量指针全部为空时返回空假设集，
    //       排查假设生成器构造参数。
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
    // 中译：创新协方差数量 {} 与航迹数 {} 不匹配，拒绝生成。
    // 标识：输入契约校验——协方差表必须逐航迹对应，防止错配计算距离。
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] rejected innovation covariance count {} for {} tracks.",
        innovation_covariances.size(), predicted_tracks.size());
    return std::vector<AssociationHypothesis>();
  }

  if (mutable_full_metric_ == nullptr) {
    // 中译：逐航迹创新协方差需要可变的全量马氏距离度量（未注入）。
    // 标识：能力边界——协方差逐航迹更新模式要求可变度量，
    //       缺失时返回空假设集。
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
    // 中译：投影协方差数量 {} 与航迹数 {} 不匹配，拒绝生成。
    // 标识：输入契约校验——投影协方差表必须逐航迹对应。
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] rejected projected covariance count {} for {} tracks.",
        projected_measurement_covariances.size(), predicted_tracks.size());
    return std::vector<AssociationHypothesis>();
  }
  if (measurement_covariances.size() != measurements.size()) {
    // 中译：量测协方差数量 {} 与量测数 {} 不匹配，拒绝生成。
    // 标识：输入契约校验——量测协方差表必须逐量测对应。
    PROJECT_LOG_ERROR(
        "[DenseCostHypothesiser] rejected measurement covariance count {} for {} measurements.",
        measurement_covariances.size(), measurements.size());
    return std::vector<AssociationHypothesis>();
  }

  if (mutable_full_metric_ == nullptr) {
    // 中译：动态量测协方差需要可变的全量马氏距离度量（未注入）。
    // 标识：能力边界——逐量测协方差模式要求可变度量，缺失时返回空假设集。
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
