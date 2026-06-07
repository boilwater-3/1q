/**
 * @file EosPipeline.h
 * @brief 定义 EOS 核心处理层管线（扫描递推、视场判定、探测评估）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_H_
#define ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_H_

#include <memory>

#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "electro_optical_sensor/config/EosInternalExecutionConfig.h"

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {

using ::electro_optical_sensor::extension::EosPipelineEnvironmentModelType;
using ::electro_optical_sensor::extension::EosPipelineWorkMode;

/** @brief 帧级别环境计算上下文，目标无关字段的聚合（完整定义见 cpp）。 */
struct FrameContext;

/**
 * @brief EosPipeline 封装核心处理层执行。
 * @note 线程模型：实例维护可变扫描相位状态，不是线程安全类型；并发访问需外部同步。
 */
class EosPipeline : public ::electro_optical_sensor::extension::IEosPipeline {
 public:
  explicit EosPipeline(
      const config::execution::EosInternalExecutionConfig& config,
      std::shared_ptr<environment::IEosEnvironmentService> environment_service = nullptr);

  // ---- 内部接口 (直接操作 EosInternalExecutionConfig, 无转换开销) ----
  void ApplyInternalConfig(const config::execution::EosInternalExecutionConfig& config,
                           bool reset_scan_phase = true);

  extension::EosPipelineRuntimeState CaptureRuntimeState() const override;
  bool RestoreRuntimeState(const extension::EosPipelineRuntimeState& state) override;

  extension::EosPipelineExecuteResult RunCycle(
      const ::electro_optical_sensor::session::EosCycleInput& input) override;

 private:
  void AdvanceScan(float dt_sec);
  bool IsTargetInCurrentFov(
      const ::electro_optical_sensor::session::EosSceneTarget& target) const;
  FrameContext BuildFrameContext(
      const ::electro_optical_sensor::session::EosCycleInput& input) const;
  output::EosDetectionRecord BuildDetectionRecord(
      const ::electro_optical_sensor::session::EosSceneTarget& target,
      const ::electro_optical_sensor::session::EosCycleInput& input,
      const FrameContext& frame_ctx) const;

  config::execution::EosInternalExecutionConfig config_{};
  float current_scan_azimuth_deg_{0.0f};
  std::shared_ptr<environment::IEosEnvironmentService> environment_service_;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SIGNAL_PIPELINE_EOS_PIPELINE_H_
