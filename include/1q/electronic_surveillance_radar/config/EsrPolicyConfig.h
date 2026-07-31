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
 *
 * @note 工作模式（EsrWorkMode）会在会话初始化时静默调整 pulse_count 和 threshold_scale：
 *       kHgesm → pulse_count ×4（上限 4096），threshold_scale ×0.85（下限 0.1）；
 *       kRwr   → pulse_count ÷2（下限 1），threshold_scale ×1.25（下限 0.1）。
 *       本结构存储调整前的基线值。见 EsrResolutionRules::ApplyWorkModeAdjustment。
 */
struct ONEQ_API EsrDetectionPolicyConfig {
  float minimum_snr_db{6.0f};           /**< 最小截获信噪比门限（单位：dB） */
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
