/**
 * @file EosInputValidation.h
 * @brief 定义光学传感器周期输入校验接口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_INPUT_VALIDATION_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/foundation/validation_types.h"

namespace electro_optical_sensor {
namespace session {

using oneq::foundation::ValidationSeverity;
using oneq::foundation::ValidationLocation;
using oneq::foundation::ValidationLocationKind;

/**
 * @brief ValidationCode 表示结构化输入校验编码。
 *
 * @note 仅保留 `EosCycleInput` 实际校验路径会触发的编码。环境观测字段
 *       （太阳辐照度、云量、风速、背景温度、太阳角、昼夜类型）已迁入
 *       `config::EosEnvironmentScenarioConfig`，不再属于周期输入域，故不在此
 *       声明对应的校验编码，以免误导调用方以为可以在 CycleInput 上校验这些字段。
 */
enum class ONEQ_API ValidationCode {
  kNone = 0,                        /**< 无问题占位值 */
  kInvalidCycleDeltaTime,           /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,         /**< 周期步长非有限值 */
  kNonFinitePlatformNumericField,   /**< 平台位姿存在非有限值 */
  kNonFiniteTargetNumericField,     /**< 目标存在非有限值字段 */
  kInvalidTargetRange,              /**< 目标斜距非法（<= 0） */
  kInvalidTargetTemperature,        /**< 目标温度非法（<= 0） */
  kInvalidTargetEmissivity,         /**< 目标辐射效率非法（不在 [0, 1]） */
  kInvalidTargetReflectance,        /**< 目标反射率非法（不在 [0, 1]） */
  kInconsistentTargetEnergyBalance, /**< 目标 emissivity + reflectance 不一致（> 1） */
  kInvalidTargetProjectedArea,      /**< 目标投影面积非法（<= 0） */
  kCycleDeltaTimeExceedsFramePeriod, /**< 周期步长超出帧率合理范围 */
  kCount                            /**< 枚举哨兵值（非实际错误码） */
};

/**
 * @brief ValidationIssue 描述单条输入校验结果。
 */
struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 问题严重级别 */
  ValidationCode code{ValidationCode::kNone};             /**< 结构化编码 */
  ValidationLocation location{};       /**< 结构化定位信息 */
  std::string field{};                 /**< 触发问题的字段名；为空表示跨字段或域级问题 */
  std::string message{};             /**< 面向调用方的简短说明 */
};

/** @brief ValidationIssueList 表示输入校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

/**
 * @brief 校验单周期光学传感器输入。
 * @param[in] input 单周期输入。
 * @param[in] frame_rate_hz 传感器帧率（Hz），用于 dt_sec 上界校验；必须正有限。
 * @return 校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateEosCycleInput(
    const ::electro_optical_sensor::session::EosCycleInput& input, float frame_rate_hz);

/**
 * @brief 判断校验列表中是否存在 error 级问题。
 * @param[in] issues 校验问题列表。
 * @return 若存在 error 级问题则返回 `true`。
 */
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_INPUT_VALIDATION_H_
