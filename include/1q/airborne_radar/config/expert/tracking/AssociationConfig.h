/**
 * @file AssociationConfig.h
 * @brief 定义 expert 关联配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_ASSOCIATION_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_ASSOCIATION_CONFIG_H_

namespace airborne_radar {
namespace config {
namespace expert {
namespace tracking {

/**
 * @brief expert 量测关联参数。
 */
struct AssociationConfig {
  float unassigned_cost{9.0f}; /**< 未分配量测代价。 */
  bool use_distance_gate_hint{false}; /**< 是否启用距离门限 sigma 提示。为 false 时库内自适应计算门限。 */
  float distance_gate_sigma_hint{0.0f}; /**< 距离门限 sigma 提示值（单位：m）。仅当 use_distance_gate_hint=true 时生效。 */
};

}  // namespace tracking
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_ASSOCIATION_CONFIG_H_
