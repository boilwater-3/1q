/**
 * @file KdTreeClusterer.h
 * @brief 定义基于 KD-tree 的 ESR 观测聚类器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_KD_TREE_CLUSTERER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_KD_TREE_CLUSTERER_H_

#include <cstddef>
#include <vector>

#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief KdTreeClusterResult 描述聚类输出。
 */
struct KdTreeClusterResult {
  std::vector<std::vector<std::size_t>> clusters{}; /**< 聚类簇索引集合 */
  std::vector<std::size_t> noise_indices{};         /**< 未满足簇条件的噪声索引 */
};

/**
 * @brief KdTreeClusterer 使用半径邻域搜索执行简化 DBSCAN 聚类。
 */
class KdTreeClusterer final {
 public:
  /**
   * @brief 对特征向量执行聚类。
   * @param[in] features 输入特征向量列表。
   * @param[in] config 聚类配置。
   * @return 聚类结果。
   */
  KdTreeClusterResult Cluster(const std::vector<ObservationFeatureVector>& features,
                              const extension::InterceptClusterConfig& config) const;
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_KD_TREE_CLUSTERER_H_
