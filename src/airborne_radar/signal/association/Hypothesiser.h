// Copyright 2026. All Rights Reserved.
//
// 文件说明：定义数据关联阶段的候选假设生成接口与实现。

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Gater.h"

namespace airborne_radar {
namespace signal {
namespace association {

/// @brief 单个关联候选假设。
struct AssociationHypothesis {
  /// @brief 默认构造。
  AssociationHypothesis() = default;
  /// @brief 参数构造。
  AssociationHypothesis(std::size_t t, std::size_t m, float c)
      : track_index(t), measurement_index(m), cost(c) {}

  /// @brief 历史轨迹索引。
  std::size_t track_index{0};
  /// @brief 当前量测索引。
  std::size_t measurement_index{0};
  /// @brief 该候选对应的关联代价。
  float cost{0.0f};
};

/// @brief 用于关联计算的特征向量列表。
using FeatureVectorList = std::vector<Eigen::Vector3f>;

/// @brief 关联候选假设生成器抽象接口。
class IHypothesiser {
public:
  /// @brief 析构函数。
  virtual ~IHypothesiser() = default;

  /// @brief 生成轨迹与量测之间的候选关联假设。
  /// @param predicted_tracks 历史轨迹预测特征集合。
  /// @param measurements 当前量测特征集合。
  /// @return 满足波门约束的候选假设列表。
  virtual std::vector<AssociationHypothesis> Generate(
      const FeatureVectorList &predicted_tracks,
      const FeatureVectorList &measurements) const = 0;
};

/// @brief 稠密代价矩阵假设生成器。
class DenseCostHypothesiser final : public IHypothesiser {
public:
  /// @brief 构造候选假设生成器。
  /// @param distance_metric 距离度量器。
  /// @param gater 波门裁剪器。
  DenseCostHypothesiser(const IDistanceMetric *distance_metric,
                        const IGater *gater);

  /// @brief 生成所有通过波门的轨迹-量测候选。
  /// @param predicted_tracks 历史轨迹预测特征集合。
  /// @param measurements 当前量测特征集合。
  /// @return 候选假设列表。
  std::vector<AssociationHypothesis> Generate(
      const FeatureVectorList &predicted_tracks,
      const FeatureVectorList &measurements) const override;

private:
  /// @brief 距离度量器。
  const IDistanceMetric *distance_metric_{nullptr};
  /// @brief 波门裁剪器。
  const IGater *gater_{nullptr};
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_