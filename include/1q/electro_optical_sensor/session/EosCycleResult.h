/**
 * @file EosCycleResult.h
 * @brief 定义光学传感器会话单周期聚合结果类型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_RESULT_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_RESULT_H_

#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosOutputFrame 表示单周期聚合探测输出帧。
 */
struct ONEQ_API EosOutputFrame {
  std::uint32_t cycle_index{0U};               /**< 当前周期号 */
  float scan_azimuth_deg{0.0f};                /**< 当前周期扫描中心方位角（单位：deg） */
  output::EosDetectionRecordList detections{}; /**< 当前周期探测结果 */
};

/**
 * @brief EosCycleResult 描述光学传感器单周期聚合结果。
 * @note `output_frame` 与 `detection_attributions` 只有在 `executed_this_cycle=true`
 *       时才代表本周期有效计算结果；失败/abort 周期会保留默认值或上一有效输出，
 *       不能按真实零值参与统计。
 */
struct ONEQ_API EosCycleResult {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号，用于失败结果与 trace 归属 */
  EosOutputFrame output_frame{};       /**< 当前周期原始系统输出帧 */
  attribution::EosDetectionAttributionRecordList
      detection_attributions{};            /**< 探测记录到仿真目标的归属映射，非真实传感器输出 */
  ValidationIssueList validation_issues{}; /**< 当前周期输入校验结果 */
  bool has_validation_error{false};        /**< 是否存在 error 级输入问题 */
  bool executed_this_cycle{false};         /**< 当前周期是否实际执行了核心 pipeline */
  bool reused_previous_output{false};      /**< 当前周期是否复用了上一有效输出 */
  session::EosPipelineAbortReason abort_reason{
      session::EosPipelineAbortReason::kNone}; /**< 当前周期终止原因 */
};

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_RESULT_H_
