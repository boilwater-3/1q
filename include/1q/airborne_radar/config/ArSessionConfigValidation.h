/**
 * @file ArSessionConfigValidation.h
 * @brief 机载雷达会话配置校验入口。
 *
 * 会话配置静态校验的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief 会话配置校验问题分类码。
 *
 * 校验通过时返回空列表；存在问题时每个条目对应一项配置缺陷。
 */
enum class ConfigValidationCode {
  kNone = 0,                          /**< 无问题（默认占位）。 */
  kCommandedBeamwidthAzNotPositive,   /**< 指令态方位波束宽度启用时非正数。 */
  kCommandedBeamwidthElNotPositive,   /**< 指令态俯仰波束宽度启用时非正数。 */
  kMechanicalScanLimitsSwappedAz,     /**< 机械方位扫描下限大于上限。 */
  kMechanicalScanLimitsSwappedEl,     /**< 机械俯仰扫描下限大于上限。 */
  kElectronicScanLimitsSwappedAz,     /**< 电子方位扫描下限大于上限。 */
  kElectronicScanLimitsSwappedEl,     /**< 电子俯仰扫描下限大于上限。 */
  kRobustTrackingWithoutImm           /**< 选用抗干扰鲁棒跟踪后端但未启用 IMM 生命周期融合。 */
};

/**
 * @brief 单条配置校验问题描述。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题分类码。 */
  std::string field{};                                    /**< 触发问题的配置字段路径。 */
  std::string message{};                                  /**< 问题说明文本。 */
};

/** @brief 配置校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

/**
 * @brief 对会话初始化配置执行静态结构校验。
 *
 * 校验范围：指令态波束宽度启用时须为正数；机械/电子扫描限位下限不大于上限；
 * 选用 UD 分解（kUdKf）等抗干扰鲁棒跟踪后端时应同时启用 IMM 生命周期融合。
 *
 * @param[in] config 待校验的会话初始化配置。
 * @return 校验问题列表；为空表示配置通过校验。
 * @note 该函数为 noexcept，仅做只读检查，不修改输入配置。
 */
ONEQ_API ValidationIssueList
ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
