/**
 * @file EosCycleResult.h
 * @brief 定义光学传感器会话单周期聚合结果类型。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_CYCLE_RESULT_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_CYCLE_RESULT_H_

#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"

namespace electro_optical_sensor {
namespace core {
namespace session {

/**
 * @brief EosCycleResult 描述光学传感器单周期聚合结果。
 */
struct ONEQ_API EosCycleResult {
  common::EosOutputFrame output_frame{};               /**< 当前周期输出帧 */
  context::EosValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false};                    /**< 是否存在 error 级输入问题 */
};

}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_CYCLE_RESULT_H_
