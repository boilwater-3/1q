/**
 * @file ArDeceptionCluster.h
 * @brief 定义由相干脉冲串聚类产生的欺骗簇摘要。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DECEPTION_CLUSTER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DECEPTION_CLUSTER_H_

#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace signal {
namespace detection {

/**
 * @brief 一组在同一分辨单元内相干相关的欺骗发射形成的簇摘要。
 *
 * 由 ArInterferenceObservationResolver 在 Complete 阶段产生，作为
 * detection→pipeline 的内部端口；不进入公开 result 或 replay。
 * association_key_seed 由 equipment signature 和簇序号混合而成，
 * 保证跨周期确定性且不依赖 observation_id 或本周期簇顺序。
 */
struct ArDeceptionCluster {
  std::uint64_t representative_observation_id{0U};
  std::uint32_t emission_count{0U};
  std::uint64_t association_key_seed{0U};
};

using ArDeceptionClusterList = std::vector<ArDeceptionCluster>;

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DECEPTION_CLUSTER_H_
