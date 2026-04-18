/**
 * @file RcsPhysicsConfig.h
 * @brief 定义 expert RCS 物理建模配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_RCS_PHYSICS_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_RCS_PHYSICS_CONFIG_H_

namespace airborne_radar {
namespace config {
namespace expert {
namespace detection {

/**
 * @brief expert RCS 物理建模参数。
 */
struct RcsPhysicsConfig {
  bool enable_physical_rcs{false}; /**< 是否启用物理 RCS 估计。 */
  float frequency_hz{0.0f}; /**< 物理 RCS 估计使用的频率。 */
  float physics_mix_ratio{0.0f}; /**< 物理估计与经验值的混合比例。 */
  float cylinder_weight{0.5f}; /**< 圆柱散射模型权重。 */
  float min_equivalent_radius_m{0.05f}; /**< 等效半径下界。 */
  float max_equivalent_radius_m{5.0f}; /**< 等效半径上界。 */
  float min_rcs_m2{0.01f}; /**< RCS 裁剪下界。 */
  float max_rcs_m2{1000.0f}; /**< RCS 裁剪上界。 */
  float bistatic_psi_offset_deg{5.0f}; /**< 双站角偏移补偿。 */
};

}  // namespace detection
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_DETECTION_RCS_PHYSICS_CONFIG_H_
