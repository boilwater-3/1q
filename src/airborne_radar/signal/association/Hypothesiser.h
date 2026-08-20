/**
 * @file Hypothesiser.h
 * @brief 定义数据关联阶段的候选假设生成内部端口与实现。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "airborne_radar/signal/association/DistanceMetric.h"

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
 * @brief 稠密代价矩阵假设生成器。
 */
class DenseCostHypothesiser final {
 public:
  /**
   * @brief 构造候选假设生成器（协方差注入路径）。
   * @param distance_metric 完整协方差距离度量器，用于带新息协方差的 Generate() 重载。
   * @param max_cost 最大允许代价值阈值。
   */
  DenseCostHypothesiser(FullMahalanobisDistanceMetric* distance_metric, float max_cost);
  /**
   * @brief 构造候选假设生成器（只读完整协方差路径）。
   * @param distance_metric 距离度量器，仅支持基础 Generate() 重载。
   * @param max_cost 最大允许代价值阈值。
   */
  DenseCostHypothesiser(const FullMahalanobisDistanceMetric* distance_metric, float max_cost);
  /**
   * @brief 构造候选假设生成器（只读对角协方差路径）。
   * @param distance_metric 距离度量器，仅支持基础 Generate() 重载。
   * @param max_cost 最大允许代价值阈值。
   */
  DenseCostHypothesiser(const MahalanobisDistanceMetric* distance_metric, float max_cost);
  /**
   * @brief 生成所有通过波门的轨迹-量测候选。
   * @param predicted_tracks 历史轨迹预测特征集合。
   * @param measurements 当前量测特征集合。
   * @return 候选假设列表。
   */
  std::vector<AssociationHypothesis> Generate(const FeatureVectorList& predicted_tracks,
                                              const FeatureVectorList& measurements) const;
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
  float ComputeCost(const Eigen::Vector3f& predicted, const Eigen::Vector3f& measurement) const;

  const MahalanobisDistanceMetric* simple_metric_{nullptr};     /**< 对角协方差度量器。 */
  const FullMahalanobisDistanceMetric* full_metric_{nullptr};   /**< 完整协方差度量器。 */
  FullMahalanobisDistanceMetric* mutable_full_metric_{nullptr}; /**< 可注入协方差度量器。 */
  float max_cost_{0.0f};                                        /**< 最大允许代价阈值。 */
};

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_HYPOTHESISER_H_
