/**
 * @file Hypothesiser.h
 * @brief 定义数据关联阶段的候选假设生成接口与实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "airborne_radar/signal/association/DistanceMetric.h"
#include "airborne_radar/signal/association/Gater.h"

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief 单个关联候选假设。
 */
struct AssociationHypothesis {
  AssociationHypothesis() = default;
  /**
   * @brief 参数构造。
   */
  AssociationHypothesis(std::size_t t, std::size_t m, float c)
      : track_index(t), measurement_index(m), cost(c) {}
  std::size_t track_index{0};       /**< 历史轨迹索引。 */
  std::size_t measurement_index{0}; /**< 当前量测索引。 */
  float cost{0.0f};                 /**< 该候选对应的关联代价。 */
};
/**
 * @brief 用于关联计算的特征向量列表。
 */
using FeatureVectorList = std::vector<Eigen::Vector3f>;
/**
 * @brief 关联候选假设生成器抽象接口。
 */
class IHypothesiser {
 public:
  virtual ~IHypothesiser() = default;
  /**
   * @brief 生成轨迹与量测之间的候选关联假设。
   * @param predicted_tracks 历史轨迹预测特征集合。
   * @param measurements 当前量测特征集合。
   * @return 满足波门约束的候选假设列表。
   */
  virtual std::vector<AssociationHypothesis> Generate(
      const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements) const = 0;
};
/**
 * @brief 稠密代价矩阵假设生成器。
 */
class DenseCostHypothesiser final : public IHypothesiser {
 public:
  /**
   * @brief 构造候选假设生成器（协方差注入路径）。
   * @param distance_metric 支持协方差注入的距离度量器，用于带新息协方差的 Generate() 重载。
   * @param gater 波门裁剪器。
   */
  DenseCostHypothesiser(ICovarianceAwareDistanceMetric* distance_metric, const IGater* gater);
  /**
   * @brief 构造候选假设生成器（只读基础路径）。
   * @param distance_metric 距离度量器，仅支持基础 Generate() 重载。
   * @param gater 波门裁剪器。
   */
  DenseCostHypothesiser(const IDistanceMetric* distance_metric, const IGater* gater);
  /**
   * @brief 生成所有通过波门的轨迹-量测候选。
   * @param predicted_tracks 历史轨迹预测特征集合。
   * @param measurements 当前量测特征集合。
   * @return 候选假设列表。
   */
  std::vector<AssociationHypothesis> Generate(const FeatureVectorList& predicted_tracks,
                                              const FeatureVectorList& measurements) const override;
  /**
   * @brief 使用逐轨迹新息协方差生成候选假设。
   * @param predicted_tracks 历史轨迹预测特征集合。
   * @param measurements 当前量测特征集合。
   * @param innovation_covariances 与轨迹索引对齐的 3×3 新息协方差列表。
   * @return 候选假设列表。
   */
  std::vector<AssociationHypothesis> Generate(
      const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements,
      const std::vector<Eigen::Matrix3f>& innovation_covariances) const;
  /**
   * @brief 使用逐轨迹预测协方差和逐量测动态协方差生成候选假设。
   * @param predicted_tracks 历史轨迹预测特征集合。
   * @param measurements 当前量测特征集合。
   * @param projected_measurement_covariances 与轨迹索引对齐的 HPH^T 列表。
   * @param measurement_covariances 与量测索引对齐的动态量测协方差 R 列表。
   * @return 候选假设列表。
   */
  std::vector<AssociationHypothesis> Generate(
      const FeatureVectorList& predicted_tracks, const FeatureVectorList& measurements,
      const std::vector<Eigen::Matrix3f>& projected_measurement_covariances,
      const std::vector<Eigen::Matrix3f>& measurement_covariances) const;

 private:
  const IDistanceMetric* distance_metric_{nullptr};            /**< 距离度量器（基础路径）。 */
  ICovarianceAwareDistanceMetric* covariance_metric_{nullptr}; /**< 协方差注入度量器（可空）。 */
  const IGater* gater_{nullptr};                               /**< 波门裁剪器。 */
};

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_
