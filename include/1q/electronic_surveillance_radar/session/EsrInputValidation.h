/**
 * @file EsrInputValidation.h
 * @brief 定义电子侦察周期输入校验接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_INPUT_VALIDATION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_INPUT_VALIDATION_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ValidationSeverity 表示校验问题严重级别。
 */
enum class ValidationSeverity {
  kInfo = 0, /**< 信息级问题，不阻断执行 */
  kWarning,  /**< 警告级问题，建议调用方关注 */
  kError     /**< 错误级问题，建议阻断执行 */
};

/**
 * @brief ValidationCode 表示结构化校验编码。
 */
enum class ValidationCode {
  kNone = 0,                      /**< 无问题占位值 */
  kInvalidCycleDeltaTime,         /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,       /**< 周期步长非有限值 */
  kNonFinitePlatformNumericField, /**< 平台存在非有限数值字段 */
  kEmptyEmitterId,                /**< 辐射源标识为空 */
  kInvalidEmitterFrequency,       /**< 辐射源频率非法（<= 0） */
  kInvalidEmitterBandwidth,       /**< 辐射源带宽非法（<= 0） */
  kInvalidEmitterPower,           /**< 辐射源功率非法（<= 0） */
  kInvalidEmitterPulseWidth,      /**< 辐射源脉宽非法（<= 0） */
  kInvalidEmitterPri,             /**< 辐射源 PRI 非法（<= 0） */
  kEmitterPriLessThanPulseWidth,  /**< 辐射源 PRI 小于脉宽 */
  kInvalidEmitterBeamwidth,       /**< 辐射源波束宽度非法（<= 0） */
  kNonFiniteEmitterNumericField   /**< 辐射源存在非有限数值字段 */
};

/**
 * @brief ValidationIssue 描述单条输入校验结果。
 */
struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 问题严重级别 */
  ValidationCode code{ValidationCode::kNone};             /**< 结构化编码 */
  std::size_t entity_index{static_cast<std::size_t>(-1)}; /**< 实体索引；若与特定实体无关则为 `size_t(-1)` */
  std::string message{};             /**< 面向调用方的简短说明 */
};

/** @brief ValidationIssueList 表示输入校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

/**
 * @brief 校验单周期电子侦察输入。
 * @param[in] input 单周期输入。
 * @return 校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateEsrCycleInput(const EsrCycleInput& input);

/**
 * @brief 判断校验列表中是否存在 error 级问题。
 * @param[in] issues 校验问题列表。
 * @return 若存在 error 级问题则返回 `true`。
 */
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_INPUT_VALIDATION_H_
