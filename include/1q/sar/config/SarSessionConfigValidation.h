/**
 * @file SarSessionConfigValidation.h
 * @brief SAR 会话配置校验工具。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sar/config/SarSessionConfig.h"

namespace sar {
namespace config {

/**
 * @brief ConfigValidationCode 表示 SAR 会话配置校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kCarrierFrequencyNotPositive,          /**< 载频 <= 0 */
  kBandwidthNotPositive,                 /**< 带宽 <= 0 */
  kPulseRepetitionFrequencyNotPositive,  /**< PRF <= 0 */
  kSampleRateNotPositive,                /**< 采样率 <= 0 */
  kAntennaLengthNotPositive,             /**< 方位孔径长度 <= 0 */
  kNominalSlantRangeNotPositive,         /**< 标称斜距 <= 0 */
  kPlatformSpeedNotPositive,             /**< 平台速度 <= 0 */
  kAzimuthPulseCountZero,                /**< 方位脉冲数为 0 */
  kRangeSampleCountZero,                 /**< 距离采样点数为 0 */
  kDesiredResolutionNotPositive,         /**< 期望分辨率 <= 0 */
  kSampleWindowTooSmallForPulse,         /**< 距离采样窗口装不下脉冲宽度（ceil(pulse_width*sample_rate) > range_sample_count）*/
  kRetainRawHistoryRequiresRawEcho,      /**< 请求返回 raw history 但 raw echo generation 关闭 */
  kSquintAngleInvalid,                   /**< squint 上限非有限或不在 [0, 90) */
  kHardwareLinkBudgetInvalid             /**< 发射功率、天线增益、噪声系数或系统损耗非法 */
};

/**
 * @brief ConfigValidationIssue 描述一条 SAR 会话配置校验结果。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题编码 */
  std::string field{};                                    /**< 关联字段名 */
  std::string message{};                                  /**< 简短说明 */
};

/** @brief ValidationIssueList 表示 SAR 会话配置校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

/**
 * @brief 校验最终 SAR 会话配置的合法性。
 *
 * 检查项包括：
 * - 载频、带宽、PRF、采样率为正；
 * - 方位孔径长度、标称斜距、平台速度为正；
 * - 发射功率为正，天线增益、接收机噪声系数与系统损耗为有限值；
 * - 方位脉冲数、距离采样点数非零；
 * - 期望分辨率（方位/地距）为正；
 * - 距离采样窗口能容纳完整脉冲宽度（ceil(pulse_width*sample_rate) <= range_sample_count）。
 *
 * @param config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateSarSessionConfig(
    const config::SarSessionConfig& config) noexcept;

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_VALIDATION_H_
