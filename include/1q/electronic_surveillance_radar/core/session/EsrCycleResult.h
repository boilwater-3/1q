/**
 * @file EsrCycleResult.h
 * @brief 定义电子侦察会话单周期聚合结果类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_CYCLE_RESULT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_CYCLE_RESULT_H_

#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {

/**
 * @brief EsrCycleResult 描述电子侦察会话单周期聚合结果。
 */
struct ONEQ_API EsrCycleResult {
  common::EsrOutputFrame output_frame{}; /**< 当前周期输出帧 */
  context::EsrValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false}; /**< 是否存在 error 级输入问题 */
};

}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CORE_SESSION_ESR_CYCLE_RESULT_H_
