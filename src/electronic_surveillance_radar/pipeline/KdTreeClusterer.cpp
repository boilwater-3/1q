#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include <nanoflann.hpp>

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

namespace {

/**
 * @brief KdTreeDatasetAdaptor 把特征数组适配为 nanoflann 数据集接口。
 */
class KdTreeDatasetAdaptor {
 public:
  explicit KdTreeDatasetAdaptor(
      const std::vector<ObservationFeatureVector>& features)
      : features_(features) {}

  inline std::size_t kdtree_get_point_count() const { return features_.size(); }

  inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    return features_[idx].values[dim];
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }

 private:
  const std::vector<ObservationFeatureVector>& features_;
};

/**
 * @brief 执行 KD-tree 半径搜索。
 * @param[in] index KD-tree 索引。
 * @param[in] feature 查询特征。
 * @param[in] radius 半径阈值。
 * @return 邻域索引列表（升序）。
 */
template <typename IndexType>
std::vector<std::size_t> RadiusSearch(const IndexType& index,
                                      const ObservationFeatureVector& feature,
                                      float radius) {
  typedef typename IndexType::DistanceType KdDistanceType;
// nanoflann 1.3.x 与 1.5+ 在搜索参数与结果容器类型上存在 API 差异。
#if defined(NANOFLANN_VERSION) && (NANOFLANN_VERSION >= 0x150)
  typedef typename IndexType::IndexType KdIndexType;
  std::vector<nanoflann::ResultItem<KdIndexType, KdDistanceType> > matches;
  nanoflann::SearchParameters search_params;
#else
  typedef std::size_t KdIndexType;
  std::vector<std::pair<KdIndexType, KdDistanceType> > matches;
  nanoflann::SearchParams search_params;
#endif
  search_params.sorted = true;
  const KdDistanceType radius_sq =
      static_cast<KdDistanceType>(radius * radius);
  index.radiusSearch(feature.values.data(), radius_sq, matches, search_params);

  std::vector<std::size_t> neighbor_indices;
  neighbor_indices.reserve(matches.size());
  for (std::size_t i = 0; i < matches.size(); ++i) {
    neighbor_indices.push_back(static_cast<std::size_t>(matches[i].first));
  }
  return neighbor_indices;
}

/**
 * @brief 判断某索引是否已在队列中。
 * @param[in] values 查询队列。
 * @param[in] target 目标索引。
 * @return 存在时返回 `true`。
 */
bool ContainsIndex(const std::deque<std::size_t>& values, std::size_t target) {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i] == target) {
      return true;
    }
  }
  return false;
}

}  // namespace

KdTreeClusterResult KdTreeClusterer::Cluster(
    const std::vector<ObservationFeatureVector>& features,
    const InterceptClusterConfig& config) const {
  KdTreeClusterResult result;
  if (features.empty()) {
    return result;
  }

  const float cluster_radius = config.radius > 0.0f ? config.radius : 1.0f;
  const std::size_t min_points =
      config.min_points > 0U ? static_cast<std::size_t>(config.min_points) : 1U;

  KdTreeDatasetAdaptor adaptor(features);
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, KdTreeDatasetAdaptor>,
      KdTreeDatasetAdaptor, kObservationFeatureDimension, std::size_t>
      KdTreeIndex;
  KdTreeIndex index(kObservationFeatureDimension, adaptor,
                    nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  std::vector<std::int32_t> labels(features.size(), -1);
  std::vector<std::uint8_t> visited(features.size(), 0U);

  for (std::size_t i = 0; i < features.size(); ++i) {
    if (visited[i] != 0U) {
      continue;
    }
    visited[i] = 1U;

    std::vector<std::size_t> neighbors =
        RadiusSearch(index, features[i], cluster_radius);
    if (neighbors.size() < min_points) {
      result.noise_indices.push_back(i);
      continue;
    }

    const std::size_t cluster_id = result.clusters.size();
    result.clusters.push_back(std::vector<std::size_t>());

    std::deque<std::size_t> queue(neighbors.begin(), neighbors.end());
    while (!queue.empty()) {
      const std::size_t candidate = queue.front();
      queue.pop_front();

      if (visited[candidate] == 0U) {
        visited[candidate] = 1U;
        const std::vector<std::size_t> candidate_neighbors =
            RadiusSearch(index, features[candidate], cluster_radius);
        if (candidate_neighbors.size() >= min_points) {
          for (std::size_t j = 0; j < candidate_neighbors.size(); ++j) {
            if (!ContainsIndex(queue, candidate_neighbors[j])) {
              queue.push_back(candidate_neighbors[j]);
            }
          }
        }
      }

      if (labels[candidate] == -1) {
        labels[candidate] = static_cast<std::int32_t>(cluster_id);
        result.clusters[cluster_id].push_back(candidate);
      }
    }
  }

  for (std::size_t i = 0; i < result.clusters.size(); ++i) {
    std::sort(result.clusters[i].begin(), result.clusters[i].end());
  }
  std::sort(result.noise_indices.begin(), result.noise_indices.end());
  return result;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
