/**
 * @file IEosPipeline.h
 * @brief EOS 管线扩展接口，允许外部接管核心探测执行。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_PIPELINE_IEOS_PIPELINE_H_
#define ELECTRO_OPTICAL_SENSOR_PIPELINE_IEOS_PIPELINE_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"

namespace electro_optical_sensor {
namespace extension {

/**
 * @brief IEosPipeline 定义 EOS 核心管线扩展点。
 */
class ONEQ_API IEosPipeline {
 public:
  virtual ~IEosPipeline() = default;

  /**
   * @brief 更新核心处理层配置。
   * @param[in] config 新配置。
   * @param[in] reset_scan_phase 是否重置扫描相位。
   */
  virtual void UpdateConfig(const EosPipelineConfig& config, bool reset_scan_phase = true) = 0;

  /**
   * @brief 捕获核心处理层运行态快照。
   * @return 当前管线运行态快照。
   */
  virtual EosPipelineRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 恢复核心处理层运行态快照。
   * @param[in] state 待恢复运行态快照。
   * @return 恢复成功返回 true；快照不兼容或恢复拒绝返回 false。
   * @note 建议校验 owner_identity/schema_version，拒绝跨实例或跨 schema 快照恢复。
   */
  virtual bool RestoreRuntimeState(const EosPipelineRuntimeState& state) = 0;

  /**
   * @brief 执行单周期核心处理并输出探测结果。
   * @param[in] input 当前周期输入。
   * @return 单周期执行结果。
   */
  virtual EosPipelineExecuteResult RunCycle(const ::electro_optical_sensor::session::EosCycleInput& input) = 0;
};

}  // namespace extension

namespace pipeline {
using ::electro_optical_sensor::extension::IEosPipeline;
using ::electro_optical_sensor::extension::EosPipelineConfig;
using ::electro_optical_sensor::extension::EosPipelineWorkMode;
using ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType;
}  // namespace pipeline

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_PIPELINE_IEOS_PIPELINE_H_
