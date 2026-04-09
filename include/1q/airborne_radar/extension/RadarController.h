/**
 * @file RadarController.h
 * @brief 定义核心处理层的雷达调度控制器接口。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_

#include <cstddef>
#include <memory>

#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/extension/IRadarOutputReader.h"
#include "1q/airborne_radar/extension/ControlReducerTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {
class IRadarContext;
class ITacticalDecisionEngine;
class IEnvironmentService;

struct RadarControllerRuntimeState {
  output::TrackOutputFrame latest_output{};
  bool has_latest_output{false};
  session::ValidationIssueList last_validation_issues{};
  std::uint64_t next_batch_id{1U};
  std::uint32_t cycle_index{1U};
  bool last_cycle_executed{false};
  bool last_cycle_reused_previous_output{false};
  SignalCycleAbortReason last_signal_abort_reason{SignalCycleAbortReason::kNone};
  SignalPipelineRuntimeState signal_pipeline_state{};
};
}  // namespace extension
}  // namespace airborne_radar

namespace airborne_radar {
namespace extension {

/**
 * @brief RadarController 负责调度信号处理、行为决策与指令下发。
 * @details 采用 PIMPL 模式隐藏实现细节，保证 ABI 稳定性；
 *          内部状态变更不会触发外部项目重编。
 */
class ONEQ_API RadarController : public IRadarOutputReader {
 public:
  ~RadarController() override;

  /**
   * @brief 构造函数，使用默认战术协调器。
   * @param[in] radar_context 雷达上下文引用。
   * @param[in] signal_pipeline 信号处理流水线引用。
   * @param[in] environment_service 环境服务引用。
   */
  RadarController(extension::IRadarContext& radar_context,
                  extension::ISignalPipeline& signal_pipeline,
                  extension::IEnvironmentService& environment_service);

  /**
   * @brief 构造函数，显式注入新的决策引擎。
   * @param[in] radar_context 雷达上下文引用。
   * @param[in] signal_pipeline 信号处理流水线引用。
   * @param[in] decision_engine 战术决策引擎引用。
   * @param[in] environment_service 环境服务引用。
   */
  RadarController(extension::IRadarContext& radar_context,
                  extension::ISignalPipeline& signal_pipeline,
                  extension::ITacticalDecisionEngine& decision_engine,
                  extension::IEnvironmentService& environment_service);

  /** @brief 执行一次雷达处理循环 */
  void RunOnce();

  /**
   * @brief 执行指定次数的处理循环（用于仿真或测试）。
   * @param[in] cycles 循环次数。
   */
  void RunCycles(std::size_t cycles);

  /**
   * @brief 更新控制归并器配置。
   * @param[in] config 控制归并器配置。
   */
  void UpdateControlReducerConfig(const extension::ControlReducerConfig& config);

  /**
   * @brief 判断当前是否已有可读取的最新轨迹输出帧。
   * @return 若已完成至少一次输出帧装配则返回 true。
   */
  bool HasLatestTrackOutputFrame() const override;

  /**
   * @brief 获取最近一次已缓存的轨迹输出帧。
   * @return 最近一次运行周期产生的轨迹输出帧引用。
   */
  const output::TrackOutputFrame& GetLatestTrackOutputFrame() const override;

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
  SignalCycleAbortReason GetLastSignalCycleAbortReason() const;

  /**
   * @brief 捕获当前控制器运行态快照。
   * @return 可用于失败回滚的控制器运行态快照。
   */
  RadarControllerRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 恢复此前捕获的控制器运行态快照。
   * @param state 待恢复的控制器运行态快照。
   */
  void RestoreRuntimeState(const RadarControllerRuntimeState& state);

  /**
   * @brief 获取当前控制器绑定的上下文实例。
   */
  extension::IRadarContext& GetRadarContext();

  /**
   * @brief 获取当前控制器绑定的信号流水线实例。
   */
  extension::ISignalPipeline& GetSignalPipeline();

  /**
   * @brief 获取当前控制器绑定的环境服务实例。
   */
  extension::IEnvironmentService& GetEnvironmentService();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CONTROLLER_H_
