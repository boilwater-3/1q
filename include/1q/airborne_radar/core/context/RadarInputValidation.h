// Copyright 2026. All Rights Reserved.
//
// @file RadarInputValidation.h
// @brief 定义雷达周期输入的显式校验接口。

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"

namespace airborne_radar {
namespace core {
namespace context {

/// @brief ValidationSeverity 表示校验结果严重级别。
enum class ValidationSeverity {
  /// @brief 仅提示语义，不阻断执行。
  kInfo = 0,

  /// @brief 调用方应显式关注的潜在问题。
  kWarning,

  /// @brief 明确的错误输入，建议阻断执行。
  kError
};

/// @brief ValidationCode 表示结构化校验问题类型。
enum class ValidationCode {
  /// @brief 无问题占位值。
  kNone = 0,

  /// @brief 周期步长非法（<= 0）。
  kInvalidCycleDeltaTime,

  /// @brief 周期步长不是有限值。
  kNonFiniteCycleDeltaTime,

  /// @brief 目标字段存在非有限值。
  kNonFiniteTargetField,

  /// @brief 目标既没有有效斜距，也没有有效笛卡尔位置。
  kMissingRangeAndCartesianPosition,

  /// @brief 目标外部标识符未知。
  kUnknownExternalTargetId,

  /// @brief 外部标识符重复。
  kDuplicateExternalTargetId,

  /// @brief 目标 RCS 为负值。
  kNegativeRcs
};

/// @brief ValidationIssue 描述一条结构化输入校验结果。
struct ValidationIssue {
  /// @brief 严重级别。
  ValidationSeverity severity{ValidationSeverity::kInfo};

  /// @brief 问题类型编码。
  ValidationCode code{ValidationCode::kNone};

  /// @brief 目标索引；若与具体目标无关，则为 `size_t(-1)`。
  std::size_t target_index{static_cast<std::size_t>(-1)};

  /// @brief 面向外部调用方的简短说明。
  std::string message{};
};

/// @brief 校验完整周期输入。
/// @param input 当前周期输入。
/// @return 按发现顺序返回的校验问题列表。
std::vector<ValidationIssue>
ValidateRadarCycleInput(const RadarCycleInput& input);

/// @brief 校验目标特征列表。
/// @param targets 当前周期目标列表。
/// @return 按发现顺序返回的校验问题列表。
std::vector<ValidationIssue>
ValidateTargetFeatures(const common::TargetFeatureList& targets);

/// @brief 判断是否包含 error 级别问题。
/// @param issues 校验问题列表。
/// @return 至少存在一个 `kError` 时返回 true。
bool HasValidationError(const std::vector<ValidationIssue>& issues);

} // namespace context
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_
