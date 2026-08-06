/**
 * @file EosPipelineRuntimeTypes.h
 * @brief 定义 EOS 管线内部运行态与执行结果类型。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_RUNTIME_TYPES_H_
#define ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_RUNTIME_TYPES_H_

#include <cstdint>

#include "1q/electro_optical_sensor/config/EosMissionConfig.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"

namespace electro_optical_sensor {
namespace extension {

using session::EosPipelineAbortReason;

/**
 * @brief EosPipelineWorkMode 描述核心探测评估模式。
 * @note 等价于 config::EosWorkMode，限定为内部 pipeline 使用。
 */
using EosPipelineWorkMode = config::EosWorkMode;

/**
 * @brief EosPipelineRuntimeState 描述 EOS 管线运行态快照。
 */
struct EosPipelineRuntimeState {
  const void* owner_identity{nullptr};       /**< 所有者实例身份（用于恢复时匹配原管线） */
  std::uint32_t schema_version{0U};          /**< 快照结构版本号 */
  float current_scan_azimuth_deg{0.0f};      /**< 当前扫描方位角（单位：deg） */
  float scan_start_az_deg{0.0f};             /**< 扫描起始方位角（单位：deg） */
  float scan_end_az_deg{0.0f};               /**< 扫描结束方位角（单位：deg） */
  float scan_rate_deg_per_sec{0.0f};         /**< 扫描角速度（单位：deg/s） */
};

/**
 * @brief EosPipelineExecuteResult 描述核心管线单周期执行结果。
 */
struct EosPipelineExecuteResult {
  output::EosDetectionRecordList detections{};                          /**< 本周期探测记录列表 */
  attribution::EosDetectionAttributionRecordList detection_attributions{}; /**< 探测记录到仿真目标的归属映射 */
  float scan_azimuth_deg{0.0f};                                         /**< 本周期扫描中心方位角（单位：deg） */
  bool executed_this_cycle{false};                                      /**< 本周期是否实际执行了核心处理 */
  EosPipelineAbortReason abort_reason{EosPipelineAbortReason::kNone};   /**< 本周期终止原因 */
  session::EosDiagnosticIssueList diagnostics{}; /**< 正常执行周期按目标排除的 kInfo 诊断（规则 13b），
                                                      经 controller 转写进 EosCycleResult。 */
};

}  // namespace extension
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_RUNTIME_TYPES_H_
