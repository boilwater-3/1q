/**
 * @file EosSessionConfigValidation.h
 * @brief EOS 会话配置校验工具。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief ConfigValidationCode 表示 EOS 会话配置校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kHorizontalFovNotPositive, /**< 水平视场角 <= 0 */
  kVerticalFovNotPositive,   /**< 垂直视场角 <= 0 */
  kScanRateNotPositive,      /**< 扫描角速度 <= 0 */
  kFrameRateNotPositive,     /**< 帧率 <= 0 */
  kScanRangeAzSwapped        /**< 扫描方位起止颠倒（起始 >= 结束） */
};

/**
 * @brief ConfigValidationIssue 描述一条 EOS 会话配置校验结果。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题编码 */
  std::string field{};                                    /**< 关联字段名 */
  std::string message{};                                  /**< 简短说明 */
};

/** @brief ValidationIssueList 表示 EOS 会话配置校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

/**
 * @brief 校验最终 EOS 会话配置的合法性。
 *
 * 检查项包括：
 * - 视场角为正；
 * - 扫描角速度与帧率为正；
 * - 扫描方位起止角一致。
 *
 * @param config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList
ValidateEosSessionConfig(const config::EosSessionConfig& config) noexcept;

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_
