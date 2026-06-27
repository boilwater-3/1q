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
 * @brief EosPipelineEnvironmentModelType 描述环境模型策略。
 * @note 等价于 config::EosEnvironmentModelType，限定为内部 pipeline 使用。
 */
using EosPipelineEnvironmentModelType = config::EosEnvironmentModelType;

/**
 * @brief EosPipelineRuntimeState 描述 EOS 管线运行态快照。
 */
struct EosPipelineRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  float current_scan_azimuth_deg{0.0f};
  float scan_start_az_deg{0.0f};
  float scan_end_az_deg{0.0f};
  float scan_rate_deg_per_sec{0.0f};
};

/**
 * @brief EosPipelineExecuteResult 描述核心管线单周期执行结果。
 */
struct EosPipelineExecuteResult {
  output::EosDetectionRecordList detections{};
  attribution::EosDetectionAttributionRecordList detection_attributions{};
  float scan_azimuth_deg{0.0f};
  bool executed_this_cycle{false};
  EosPipelineAbortReason abort_reason{EosPipelineAbortReason::kNone};
};

}  // namespace extension
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_RUNTIME_TYPES_H_
