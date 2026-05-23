/**
 * @file EsrPolicyConfig.h
 * @brief 定义 ESR 策略域配置与探测策略参数。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_POLICY_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_POLICY_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrDetectionProfile 描述对外探测策略档位。
 */
enum class ONEQ_API EsrDetectionProfile {
  kConservative = 0, /**< 保守策略，降低虚警 */
  kBalanced,         /**< 均衡策略 */
  kSensitive         /**< 高灵敏策略，提升检出 */
};

/**
 * @brief EsrDetectionPolicyConfig 描述对外探测策略语义与详细参数。
 *
 * 支持双模式：
 * - 预设档位模式（use_profile_defaults == true）：profile 映射到内部参数；
 * - 详细参数模式（use_profile_defaults == false）：直接使用用户提供的详细参数。
 */
struct ONEQ_API EsrDetectionPolicyConfig {
  EsrDetectionProfile profile{EsrDetectionProfile::kBalanced}; /**< 探测策略档位 */
  bool use_profile_defaults{true}; /**< true 表示使用 profile 的默认参数映射 */

  float min_detect_snr_db{6.0f};           /**< 详细参数：最小截获信噪比门限（单位：dB） */
  float pfa{1.0e-6f};                      /**< 详细参数：期望虚警概率，范围 (0, 1) */
  std::uint32_t pulse_count{8U};           /**< 详细参数：脉冲积累数量 */
  float threshold_scale{1.0f};             /**< 详细参数：门限缩放系数 */
  bool enable_statistical_detection{true}; /**< 详细参数：是否启用统计检测模型 */
};

/**
 * @brief EsrPolicyConfig 描述 ESR 的统一策略域输入。
 */
struct ONEQ_API EsrPolicyConfig {
  EsrDetectionPolicyConfig detection{}; /**< 探测策略子域 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_POLICY_CONFIG_H_
