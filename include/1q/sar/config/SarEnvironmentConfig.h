/**
 * @file SarEnvironmentConfig.h
 * @brief 定义 SAR 环境与传播默认配置。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief SAR 环境默认配置。
 *
 * @note 保留域（reserved domain）：当前 Phase 1 计算链路（raw echo 生成、L1 RDA、
 *       L3 BP、质量摘要）**不消费**本域任何字段。地形参考高程、大气损耗、表面后向
 *       散射系数、扁平地球几何与大气衰减开关均不影响成像输出。本域存在的目的是
 *       replay 保真（`SarReplayFlatbufferCodec` 完整序列化/反序列化该域以重建记录
 *       快照）与 config 透传（`SarSessionConfigBuilder` 保留其值），供未来按域接入
 *       计算时直接启用。参照 `SarPolicyConfig` 中 `retain_raw_phase_history`、
 *       `max_allowed_squint_angle_deg` 的“保留字段”先例。
 */
struct ONEQ_API SarEnvironmentConfig {
  double terrain_reference_altitude_m{0.0};   /**< 保留字段：地形参考高程，当前不驱动任何计算阶段 */
  double atmospheric_loss_db_per_km{0.0};     /**< 保留字段：大气损耗，当前不驱动任何计算阶段 */
  double surface_backscatter_sigma0_db{-12.0}; /**< 保留字段：表面后向散射系数，当前不驱动任何计算阶段 */
  bool use_flat_earth_geometry{true};         /**< 保留字段：扁平地球几何开关，当前不驱动任何计算阶段 */
  bool enable_atmospheric_attenuation{false}; /**< 保留字段：大气衰减开关，当前不驱动任何计算阶段 */
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_
