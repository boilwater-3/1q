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
 * @brief EsrDetectionPolicyConfig 描述对外探测策略详细参数。
 */
struct ONEQ_API EsrDetectionPolicyConfig {
  float min_detect_snr_db{6.0f};           /**< 最小截获信噪比门限（单位：dB） */
  float pfa{1.0e-6f};                      /**< 期望虚警概率，范围 (0, 1) */
  std::uint32_t pulse_count{8U};           /**< 脉冲积累数量 */
  float threshold_scale{1.0f};             /**< 门限缩放系数 */
  bool enable_statistical_detection{true}; /**< 是否启用统计检测模型 */
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
