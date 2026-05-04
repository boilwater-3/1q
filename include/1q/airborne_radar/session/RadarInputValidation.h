/**
 * @file RadarInputValidation.h
 * @brief 定义雷达周期输入的显式校验接口。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ValidationSeverity 表示校验结果严重级别。
 */
enum class ONEQ_API ValidationSeverity {
  kInfo = 0, /**< 仅提示语义，不阻断执行 */
  kWarning,  /**< 调用方应显式关注的潜在问题 */
  kError     /**< 明确的错误输入，建议阻断执行 */
};

/**
 * @brief ValidationCode 表示结构化校验问题类型。
 */
enum class ONEQ_API ValidationCode {
  kNone = 0,                         /**< 无问题占位值 */
  kInvalidCycleDeltaTime,            /**< 周期步长非法（<= 0） */
  kNonFiniteCycleDeltaTime,          /**< 周期步长不是有限值 */
  kNonFinitePlatformNumericField,    /**< 平台位姿字段存在非有限值 */
  kNonFiniteTargetField,             /**< 目标字段存在非有限值 */
  kMissingRangeAndCartesianPosition, /**< 目标既没有有效斜距，也没有有效笛卡尔位置 */
  kUnknownExternalTargetId,          /**< 目标外部标识符未知 */
  kDuplicateExternalTargetId,        /**< 外部标识符重复 */
  kNegativeRcs,                      /**< 目标 RCS 为负值 */
  kInvalidEnvironmentObservation     /**< 环境观测字段非法 */
};

/**
 * @brief ValidationLocationKind 表示校验问题定位域。
 */
enum class ONEQ_API ValidationLocationKind {
  kGlobal = 0,  /**< 与具体域或实体无关 */
  kPlatform,    /**< 平台位姿域 */
  kEnvironment, /**< 环境输入域 */
  kSceneEntity  /**< 场景实体域（需配合 `entity_index`） */
};

/**
 * @brief ValidationLocation 描述校验问题定位信息。
 */
struct ONEQ_API ValidationLocation {
  ValidationLocationKind kind{ValidationLocationKind::kGlobal}; /**< 定位域 */
  std::size_t entity_index{
      static_cast<std::size_t>(-1)}; /**< 场景实体索引；非场景实体问题时为 `size_t(-1)` */
};

/**
 * @brief ValidationIssue 描述一条结构化输入校验结果。
 */
struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo}; /**< 严重级别 */
  ValidationCode code{ValidationCode::kNone};             /**< 问题类型编码 */
  ValidationLocation location{};                          /**< 结构化定位信息 */
  std::string field{};   /**< 触发问题的字段名；为空表示跨字段或域级问题 */
  std::string message{}; /**< 面向外部调用方的简短说明 */
};

/** @brief ValidationIssueList 表示输入校验问题列表。 */
using ValidationIssueList = std::vector<ValidationIssue>;

/**
 * @brief 校验周期步长字段。
 * @param[in] dt_sec 当前周期步长（单位：秒）。
 * @return 校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateRadarCycleDeltaTime(float dt_sec);

/**
 * @brief 校验完整周期输入。
 * @param[in] input 当前周期输入。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateRadarCycleInput(const RadarCycleInput& input);

/**
 * @brief 校验场景目标列表。
 * @param[in] targets 当前周期场景目标列表。
 * @return 按发现顺序返回的校验问题列表。
 */
ONEQ_API ValidationIssueList ValidateRadarSceneTargets(const RadarSceneTargetList& targets);

/**
 * @brief 判断是否包含 error 级别问题。
 * @param[in] issues 校验问题列表。
 * @return 至少存在一个 `kError` 时返回 true。
 */
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_RADAR_INPUT_VALIDATION_H_
