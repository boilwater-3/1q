/**
 * @file EosInputValidation.h
 * @brief 定义光学传感器周期输入校验接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_INPUT_VALIDATION_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"

namespace electro_optical_sensor {
namespace core {
namespace context {

/**
 * @brief EosValidationSeverity 表示输入校验问题严重级别。
 */
enum class EosValidationSeverity {
  kInfo = 0, /**< 信息级问题，不阻断执行 */
  kWarning,  /**< 警告级问题，建议调用方关注 */
  kError     /**< 错误级问题，建议阻断执行 */
};

/**
 * @brief EosValidationCode 表示结构化输入校验编码。
 */
enum class EosValidationCode {
  kNone = 0,                        /**< 无问题占位值 */
  kInvalidCycleDeltaTime,           /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,         /**< 周期步长非有限值 */
  kNonFinitePlatformNumericField,   /**< 平台位姿存在非有限值 */
  kInvalidSolarIrradiance,          /**< 太阳辐照度非法（< 0） */
  kInvalidAtmosphericTransmittance, /**< 大气透明度非法（不在 [0, 1]） */
  kInvalidCloudCoverageRatio,       /**< 云量非法（不在 [0, 1]） */
  kInvalidAmbientWindSpeed,         /**< 环境风速非法（非有限值或 < 0） */
  kInvalidBackgroundTemperature,    /**< 背景温度非法（<= 0） */
  kInvalidTargetId,                 /**< 目标标识非法（== 0） */
  kNonFiniteTargetNumericField,     /**< 目标存在非有限值字段 */
  kInvalidTargetRange,              /**< 目标斜距非法（<= 0） */
  kInvalidTargetTemperature,        /**< 目标温度非法（<= 0） */
  kInvalidTargetEmissivity,         /**< 目标辐射效率非法（不在 [0, 1]） */
  kInvalidTargetReflectance,        /**< 目标反射率非法（不在 [0, 1]） */
  kInvalidTargetProjectedArea       /**< 目标投影面积非法（<= 0） */
};

/**
 * @brief EosValidationIssue 描述单条输入校验结果。
 */
struct ONEQ_API EosValidationIssue {
  EosValidationSeverity severity{EosValidationSeverity::kInfo}; /**< 问题严重级别 */
  EosValidationCode code{EosValidationCode::kNone};             /**< 结构化编码 */
  std::size_t target_index{
      static_cast<std::size_t>(-1)}; /**< 目标索引；若无特定目标则为 `size_t(-1)` */
  std::string message{};             /**< 面向调用方的简短说明 */
};

/** @brief EosValidationIssueList 表示输入校验问题列表。 */
using EosValidationIssueList = std::vector<EosValidationIssue>;

/**
 * @brief 校验单周期光学传感器输入。
 * @param[in] input 单周期输入。
 * @return 校验问题列表。
 */
ONEQ_API EosValidationIssueList ValidateEosCycleInput(const EosCycleInput& input);

/**
 * @brief 判断校验列表中是否存在 error 级问题。
 * @param[in] issues 校验问题列表。
 * @return 若存在 error 级问题则返回 `true`。
 */
ONEQ_API bool HasEosValidationError(const EosValidationIssueList& issues);

}  // namespace context
}  // namespace core
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_INPUT_VALIDATION_H_
