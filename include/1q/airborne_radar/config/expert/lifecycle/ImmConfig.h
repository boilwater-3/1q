/**
 * @file ImmConfig.h
 * @brief 定义 expert IMM 配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_LIFECYCLE_IMM_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_LIFECYCLE_IMM_CONFIG_H_

#include <cstdint>

namespace airborne_radar {
namespace config {
namespace expert {
namespace lifecycle {

/**
 * @brief expert IMM 提示配置。
 */
struct ImmConfig {
  bool enable_imm_lifecycle{false}; /**< 是否启用 IMM 生命周期路径。 */
  std::uint32_t model_count_hint{2U}; /**< IMM 模型数提示值。 */
};

}  // namespace lifecycle
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_LIFECYCLE_IMM_CONFIG_H_
