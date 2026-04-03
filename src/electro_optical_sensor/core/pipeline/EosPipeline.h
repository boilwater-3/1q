/**
 * @file EosPipeline.h
 * @brief 定义 EOS 核心处理层管线（扫描递推、视场判定、探测评估）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_PIPELINE_EOS_PIPELINE_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_PIPELINE_EOS_PIPELINE_H_

#include <memory>

#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/pipeline/IEosPipeline.h"

namespace electro_optical_sensor {
namespace core {
namespace pipeline {

using ::electro_optical_sensor::pipeline::EosPipelineConfig;
using ::electro_optical_sensor::pipeline::EosPipelineEnvironmentModelType;
using ::electro_optical_sensor::pipeline::EosPipelineWorkMode;

/**
 * @brief EosPipeline 封装核心处理层执行。
 * @note 线程模型：实例维护可变扫描相位状态，不是线程安全类型；并发访问需外部同步。
 */
class EosPipeline : public ::electro_optical_sensor::pipeline::IEosPipeline {
 public:
  explicit EosPipeline(
      const EosPipelineConfig& config,
      std::shared_ptr<environment::IEosEnvironmentService> environment_service = nullptr);

  /**
   * @brief 更新核心处理层配置。
   * @param[in] config 新配置。
   * @param[in] reset_scan_phase 是否重置扫描相位。
   * @note 非线程安全：会修改内部扫描状态；并发调用需外部同步。
   */
  void UpdateConfig(const EosPipelineConfig& config, bool reset_scan_phase = true) override;

  /**
   * @brief 执行单周期核心处理并输出探测结果。
   * @param[in] input 当前周期输入。
   * @return 探测输出帧。
   * @note 非线程安全：会推进内部扫描相位（`current_scan_azimuth_deg_`）。
   */
  common::EosOutputFrame Execute(const context::EosCycleInput& input) override;

 private:
  void AdvanceScan(float dt_sec);
  bool IsTargetInCurrentFov(const context::EosTargetState& target) const;
  common::EosDetectionRecord BuildDetectionRecord(const context::EosTargetState& target,
                                                  const context::EosCycleInput& input) const;

  EosPipelineConfig config_{};
  float current_scan_azimuth_deg_{0.0f};
  std::shared_ptr<environment::IEosEnvironmentService> environment_service_;
};

}  // namespace pipeline
}  // namespace core
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_PIPELINE_EOS_PIPELINE_H_
