/**
 * @file EosController.h
 * @brief 定义光学传感器核心调度控制器接口。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_RUNTIME_EOS_CONTROLLER_H_
#define ELECTRO_OPTICAL_SENSOR_RUNTIME_EOS_CONTROLLER_H_

#include <cstdint>
#include <memory>

#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "electro_optical_sensor/pipeline/EosPipelineRuntimeTypes.h"

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {
class EosPipeline;
}
}  // namespace signal
namespace extension {

/**
 * @brief EosControllerRuntimeState 描述控制器运行态快照。
 */
struct EosControllerRuntimeState {
  const void* owner_identity{nullptr};       /**< 所有者实例身份（用于恢复时匹配原控制器） */
  std::uint32_t schema_version{0U};          /**< 快照结构版本号 */
  session::EosOutputFrame latest_output{};   /**< 最近一帧有效输出（失败周期可能复用上一帧） */
  attribution::EosDetectionAttributionRecordList latest_detection_attributions{}; /**< 最近一帧归属映射 */
  session::ValidationIssueList last_validation_issues{}; /**< 最近一次输入校验问题列表 */
  bool has_latest_output{false};             /**< 是否已有可读取的最新输出帧 */
  bool has_validation_error{false};          /**< 最近一次校验是否存在 error 级问题 */
  bool last_cycle_executed{false};           /**< 最近一次 RunOnce 是否实际执行了核心 pipeline */
  bool last_cycle_reused_previous_output{false};          /**< 最近一次是否复用了上一有效输出 */
  session::EosPipelineAbortReason last_abort_reason{session::EosPipelineAbortReason::kNone}; /**< 最近一次终止原因 */
  EosPipelineRuntimeState pipeline_state{};  /**< 内嵌的管线运行态快照 */
};

/**
 * @brief EosController 负责调度输入校验、核心管线执行与输出缓存。
 */
class EosController {
 public:
  /**
   * @brief 构造光学传感器控制器。
   * @param[in] pipeline 核心管线实现。
   */
  explicit EosController(signal::pipeline::EosPipeline& pipeline);
  ~EosController();

  EosController(const EosController&) = delete;
  EosController& operator=(const EosController&) = delete;

  /**
   * @brief 执行一次光学传感器处理周期。
   * @param[in] input 当前周期输入。
   */
  void RunOnce(const ::electro_optical_sensor::session::EosCycleInput& input);

  /**
   * @brief 判断当前是否有可读取的最新输出帧。
   * @return 若已有输出帧则返回 true。
   */
  bool HasLatestDetectionOutputFrame() const;

  /**
   * @brief 获取最新输出帧。
   * @return 最新输出帧。
   */
  const session::EosOutputFrame& GetLatestDetectionOutputFrame() const;

  /**
   * @brief 获取最近一次输入校验结果。
   * @return 最近一次输入校验问题列表。
   */
  const session::ValidationIssueList& GetLastValidationIssues() const;

  /**
   * @brief 判断最近一次输入校验是否存在 error 级问题。
   * @return 若存在 error 级问题则返回 true。
   */
  bool HasValidationError() const;

  /**
   * @brief 最近一次 RunOnce 是否执行了核心 pipeline。
   * @return 若执行了核心 pipeline 则返回 true。
   */
  bool ExecutedLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 是否复用了上一有效输出。
   * @return 若复用了上一有效输出则返回 true。
   */
  bool ReusedPreviousDetectionOutputLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 的周期终止原因。
   * @return 周期终止原因。
   */
  session::EosPipelineAbortReason GetLastDetectionCycleAbortReason() const;

  /**
   * @brief 基于最近一次 RunOnce 状态构建单周期聚合结果。
   * @param[in] input 当前周期输入，仅在无可复用输出时用于回填 cycle_index。
   * @return 当前周期聚合结果。
   */
  ::electro_optical_sensor::session::EosCycleResult BuildCycleResult(
      const ::electro_optical_sensor::session::EosCycleInput& input) const;

  /**
   * @brief 捕获控制器运行态快照。
   * @return 控制器运行态快照。
   */
  EosControllerRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 恢复控制器运行态快照。
   * @param[in] state 待恢复运行态快照。
   */
  bool RestoreRuntimeState(const EosControllerRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_RUNTIME_EOS_CONTROLLER_H_
