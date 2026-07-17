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
  kScanRangeAzSwapped,       /**< 扫描方位起止颠倒（起始 >= 结束） */
  kEnvironmentModelTypeInvalid, /**< 环境模型策略枚举非法 */
  kEnvironmentPresetInvalid, /**< 环境预设枚举非法 */
  kRadiativeTransferModelInvalid, /**< 自定义辐射传输模型枚举非法 */
  kAerosolDensityFactorInvalid, /**< 自定义气溶胶因子非有限或 <= 0 */
  kTurbulenceFactorInvalid /**< 自定义湍流因子非有限或 <= 0 */
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
 * - 环境模型与 preset 枚举有效；
 * - 启用 custom 整组覆盖时，辐射模型枚举有效且两个因子有限并大于零。
 *
 * @param config 待校验的最终会话配置。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList
ValidateEosSessionConfig(const config::EosSessionConfig& config) noexcept;

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_VALIDATION_H_
