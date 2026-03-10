// Copyright 2026. All Rights Reserved.
//
// 文件说明：实现数据关联候选假设的生成逻辑。

#include "airborne_radar/signal/association/Hypothesiser.h"

#include <stdexcept>

namespace airborne_radar {
namespace signal {
namespace association {

/// @brief 构造候选假设生成器。
/// @param distance_metric 距离度量器。
/// @param gater 波门裁剪器。
DenseCostHypothesiser::DenseCostHypothesiser(const IDistanceMetric *distance_metric,
                                             const IGater *gater)
    : distance_metric_(distance_metric), gater_(gater) {
  if (distance_metric_ == nullptr || gater_ == nullptr) {
    throw std::invalid_argument("hypothesiser requires distance metric and gater");
  }
}

/// @brief 生成所有通过波门的轨迹-量测候选假设。
/// @param predicted_tracks 历史轨迹预测特征集合。
/// @param measurements 当前量测特征集合。
/// @return 候选假设列表。
std::vector<AssociationHypothesis> DenseCostHypothesiser::Generate(
    const FeatureVectorList &predicted_tracks,
    const FeatureVectorList &measurements) const {
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

      hypotheses.push_back(
          AssociationHypothesis{track_index, measurement_index, cost});
    }
  }

  return hypotheses;
}

} // namespace association
} // namespace signal
} // namespace airborne_radar