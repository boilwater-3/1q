/**
 * @file BeamSchedulerConfig.h
 * @brief 定义 expert 波束调度配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_SCHEDULER_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_SCHEDULER_CONFIG_H_

#include <cstdint>

namespace airborne_radar {
namespace config {
namespace expert {
namespace beam {

/**
 * @brief expert 波束扫描调度提示配置。
 */
struct BeamSchedulerConfig {
  std::uint32_t azimuth_step_count_hint{0U}; /**< 方位步进数提示。 */
  std::uint32_t elevation_step_count_hint{0U}; /**< 俯仰步进数提示。 */
  bool prefer_dense_tas_sampling{false}; /**< 是否偏好更密的 TAS 采样。 */
};

}  // namespace beam
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_BEAM_BEAM_SCHEDULER_CONFIG_H_
