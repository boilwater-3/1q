/**
 * @file HypothesisAssociator.h
 * @brief 定义 ESR 聚类结果到辐射源假设的关联器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_HYPOTHESIS_ASSOCIATOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_HYPOTHESIS_ASSOCIATOR_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/common/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

/**
 * @brief HypothesisAssociator 负责簇级观测与历史假设的关联更新。
 */
class HypothesisAssociator final {
 public:
  /**
   * @brief 使用配置构造关联器。
   * @param[in] config 假设关联配置。
   */
  explicit HypothesisAssociator(InterceptAssociationConfig config = {});

  /**
   * @brief 更新配置。
   * @param[in] config 新关联配置。
   */
  void UpdateConfig(InterceptAssociationConfig config);

  /**
   * @brief 基于当前周期簇摘要更新并导出假设列表。
   * @param[in] cycle_index 当前周期号。
   * @param[in] clusters 当前周期簇摘要列表。
   * @param[in,out] next_hypothesis_id 假设 ID 分配器。
   * @return 当前周期对外导出的假设列表。
   */
  common::EmitterHypothesisList Update(
      std::uint32_t cycle_index, const std::vector<ClusterSummary>& clusters,
      std::uint64_t* next_hypothesis_id);

  /**
   * @brief 重置内部历史状态。
   */
 void Reset();

 private:
  struct TrackState {
    std::uint64_t hypothesis_id{0U};
    ObservationFeatureVector feature{};
    std::vector<std::string> candidate_classes{};
    common::EmitterMode mode{common::EmitterMode::kUnknown};
    common::ThreatLevel threat_level{common::ThreatLevel::kLow};
    float bearing_az_deg{0.0f};
    float bearing_el_deg{0.0f};
    float bearing_std_deg{8.0f};
    float confidence{0.0f};
    std::uint32_t last_seen_cycle{0U};
    std::uint32_t hit_streak{0U};
    std::uint32_t missed_cycles{0U};
    std::uint32_t age_cycles{0U};
    std::size_t support_count{0U};
    bool confirmed{false};
  };

  InterceptAssociationConfig config_{};
  std::vector<TrackState> tracks_{};
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_HYPOTHESIS_ASSOCIATOR_H_
