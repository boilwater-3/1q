/**
 * @file EsrController.h
 * @brief 定义电子侦察核心调度控制器接口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_ESR_CONTROLLER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_ESR_CONTROLLER_H_

#include <memory>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace environment {
class IEsrEnvironmentService;
}
namespace extension {
class IInterceptPipeline;
}

/**
 * @brief EsrControllerRuntimeState 描述 ESR 控制器运行态快照。
 */
struct ONEQ_API EsrControllerRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  bool has_latest_output{false};
  session::EsrOutputFrame latest_output{};
  session::ValidationIssueList last_validation_issues{};
  std::uint64_t next_batch_id{0U};
  bool last_cycle_executed{false};
  extension::EsrPipelineAbortReason last_abort_reason{
      extension::EsrPipelineAbortReason::kNone};
  extension::InterceptPipelineRuntimeState pipeline_state{};
};

namespace extension {

/**
 * @brief EsrController 负责调度环境冻结、侦察流水线执行与输出缓存。
 */
class ONEQ_API EsrController {
 public:
  /**
   * @brief 构造电子侦察控制器。
   * @param[in] pipeline 侦察流水线接口实现。
   * @param[in] environment_service 环境服务接口实现。
   */
  EsrController(extension::IInterceptPipeline& pipeline,
                environment::IEsrEnvironmentService& environment_service);
  ~EsrController();

  EsrController(const EsrController&) = delete;
  EsrController& operator=(const EsrController&) = delete;

  /**
   * @brief 执行一次电子侦察处理周期。
   * @param[in] input 当前周期输入。
   */
  void RunOnce(const session::EsrCycleInput& input);

  /**
   * @brief 判断当前是否有可读取的最新输出帧。
   * @return 若已有输出帧则返回 `true`。
   */
  bool HasLatestOutputFrame() const;

  /**
   * @brief 获取最新输出帧。
   * @return 最新电子侦察输出帧。
   */
  const session::EsrOutputFrame& GetLatestOutputFrame() const;

  /**
   * @brief 获取最近一次输入校验结果。
   * @return 最近一次输入校验问题列表。
   */
  const session::ValidationIssueList& GetLastValidationIssues() const;

  /**
   * @brief 获取当前控制器绑定的流水线实例。
   */
  extension::IInterceptPipeline& GetPipeline();

  /**
   * @brief 最近一次 RunOnce 是否执行了核心 pipeline。
   * @return 若执行了核心 pipeline 则返回 true。
   */
  bool ExecutedLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 是否复用了上一有效输出。
   * @return 若复用了上一有效输出则返回 true。
   */
  bool ReusedPreviousOutputLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 的周期终止原因。
   * @return 周期终止原因。
   */
  extension::EsrPipelineAbortReason GetLastAbortReason() const;

  /**
   * @brief 捕获当前控制器运行态快照（含流水线快照）。
   * @return 当前运行态快照。
   */
  EsrControllerRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 使用快照恢复控制器运行态。
   * @param[in] state 待恢复的快照。
   * @return 恢复成功返回 `true`。
   */
  bool RestoreRuntimeState(const EsrControllerRuntimeState& state);

  /**
   * @brief 获取当前控制器绑定的环境服务实例。
   */
  environment::IEsrEnvironmentService& GetEnvironmentService();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_ESR_CONTROLLER_H_
