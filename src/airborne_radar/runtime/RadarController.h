/**
 * @file RadarController.h
 * @brief 核心处理层 AR 调度控制器（内部实现细节，不对外暴露）。
 */

#ifndef AIRBORNE_RADAR_RUNTIME_RADAR_CONTROLLER_H_
#define AIRBORNE_RADAR_RUNTIME_RADAR_CONTROLLER_H_

#include <cstddef>
#include <memory>

#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}
namespace session {
class MutableArContext;
}
namespace session {
class ITacticalDecisionEngine;
}  // namespace session

namespace extension {

struct ArControllerRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  session::TrackOutputFrame latest_output{};
  bool has_latest_output{false};
  session::ValidationIssueList last_validation_issues{};
  std::uint64_t next_batch_id{1U};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  session::SignalCycleAbortReason last_signal_abort_reason{session::SignalCycleAbortReason::kNone};
  signal::SignalPipelineRuntimeState signal_pipeline_state{};
};
}  // namespace extension
}  // namespace airborne_radar

namespace airborne_radar {
namespace extension {

/**
 * @brief ArController 负责调度信号处理、行为决策与指令下发。
 * @details 采用 PIMPL 模式隐藏实现细节，保证 ABI 稳定性；
 *          内部状态变更不会触发外部项目重编。
 */
class ArController {
 public:
  ~ArController();

  /**
   * @brief 构造函数，使用默认战术协调器。
   * @param[in] ar_context AR 上下文引用。
   * @param[in] signal_pipeline 信号处理流水线引用。
   * @param[in] environment_service 环境服务引用。
   */
  ArController(session::MutableArContext& ar_context,
               signal::ISignalPipeline& signal_pipeline,
               environment::IEnvironmentService& environment_service);

  /**
   * @brief 构造函数，显式注入新的决策引擎。
   * @param[in] ar_context AR 上下文引用。
   * @param[in] signal_pipeline 信号处理流水线引用。
   * @param[in] decision_engine 战术决策引擎引用。
   * @param[in] environment_service 环境服务引用。
   */
  ArController(session::MutableArContext& ar_context,
               signal::ISignalPipeline& signal_pipeline,
               session::ITacticalDecisionEngine& decision_engine,
               environment::IEnvironmentService& environment_service);

  /** @brief 执行一次 AR 处理循环 */
  void RunOnce();

  /**
   * @brief 执行指定次数的处理循环（用于仿真或测试）。
   * @param[in] cycles 循环次数。
   */
  void RunCycles(std::size_t cycles);

  /**
   * @brief 判断当前是否已有可读取的最新轨迹输出帧。
   * @return 若已完成至少一次输出帧装配则返回 true。
   */
  bool HasLatestTrackOutputFrame() const;

  /**
   * @brief 获取最近一次已缓存的轨迹输出帧。
   * @return 最近一次运行周期产生的轨迹输出帧引用。
   */
  const session::TrackOutputFrame& GetLatestTrackOutputFrame() const;

  /**
   * @brief 获取最近一次输入校验问题列表。
   * @return 最近一次 RunOnce 记录的校验问题。
   */
  const session::ValidationIssueList& GetLastValidationIssues() const;

  /**
   * @brief 判断最近一次输入校验是否存在 error 级问题。
   * @return 若存在 error 级问题则返回 true。
   */
  bool HasValidationError() const;

  /**
   * @brief 最近一次 RunOnce 是否真正执行了 signal/decision/control 主链路。
   * @return 若最近一次周期完成主链路执行则返回 true。
   */
  bool ExecutedLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 是否复用了上一有效轨迹输出。
   * @return 若最近一次周期未完成执行且复用了上一有效输出则返回 true。
   */
  bool ReusedPreviousTrackOutputLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 若未执行成功，返回 signal pipeline 的 abort 原因。
   */
  session::SignalCycleAbortReason GetLastSignalCycleAbortReason() const;

  /**
   * @brief 捕获当前控制器运行态快照。
   * @return 可用于失败回滚的控制器运行态快照。
   */
  ArControllerRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 恢复此前捕获的控制器运行态快照。
   * @param state 待恢复的控制器运行态快照。
   */
  bool RestoreRuntimeState(const ArControllerRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// 兼容别名：旧名称在 wrapper 阶段保留。
using RadarController = ArController;
using RadarControllerRuntimeState = ArControllerRuntimeState;

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RUNTIME_RADAR_CONTROLLER_H_
