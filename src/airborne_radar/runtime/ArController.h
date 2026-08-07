/**
 * @file ArController.h
 * @brief 核心处理层 AR 调度控制器（内部实现细节，不对外暴露）。
 */

#ifndef AIRBORNE_RADAR_RUNTIME_AR_CONTROLLER_H_
#define AIRBORNE_RADAR_RUNTIME_AR_CONTROLLER_H_

#include <cstddef>
#include <memory>

#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArTrackOutput.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/decision/ControlReducerTypes.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/recognition/RecognitionTracker.h"
#include "airborne_radar/signal/detection/ArDeceptionMeasurementCandidate.h"
#include "airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/pipeline/SignalCycleInput.h"

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}
namespace session {
class MutableArContext;
}
namespace extension {

/**
 * @brief ArController 运行态快照，用于失败回滚等场景的整快照捕获/恢复。
 * @note owner_identity 标识捕获方实例，RestoreRuntimeState 会拒绝跨实例恢复；
 *       schema_version 用于校验快照格式兼容性（7：识别状态纳入回滚边界）。
 */
struct ArControllerRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  session::TrackOutputFrame latest_output{};
  bool has_latest_output{false};
  std::uint64_t next_batch_id{1U};
  bool last_cycle_executed{false};
  session::SignalCycleAbortReason last_signal_abort_reason{session::SignalCycleAbortReason::kNone};
  session::ArControlProfile control_profile{};
  extension::ControlReducerConfig control_reducer_config{};
  decision::ControlReducerRuntimeState control_reducer_state{};
  bool has_pending_internal_decision{false};
  std::uint32_t pending_internal_cycle_index{0U};
  std::uint64_t pending_internal_batch_id{0U};
  std::vector<session::TacticalProposal> pending_internal_proposals{};
  bool has_pending_external_override{false};
  session::ExternalDecisionOverride pending_external_override{};
  bool has_latest_decision_observation{false};
  session::DecisionObservation latest_decision_observation{};
  session::DecisionControlSource last_applied_decision_source{
      session::DecisionControlSource::kNone};
  std::uint32_t last_applied_decision_cycle_index{0U};
  std::uint64_t last_applied_decision_batch_id{0U};
  std::vector<session::TacticalProposal> last_applied_decision_proposals{};
  bool control_prepared_for_cycle{false};
  recognition::RecognitionTracker::Snapshot recognition_tracker_state{}; /**< 识别积累/结论快照。 */
  config::ArWorkMode work_mode{config::ArWorkMode::kTws}; /**< 快照时工作模式。 */
  config::ArRecognitionConfig recognition_config{}; /**< 快照时识别策略配置。 */
  std::string recognition_database_path{};          /**< 快照时生效数据库路径。 */
  session::ArRecognitionCycleSummary latest_recognition_summary{}; /**< 最近周期识别摘要。 */
  bool has_latest_recognition_summary{false}; /**< 最近周期是否发布了识别摘要。 */
};
}  // namespace extension
}  // namespace airborne_radar

namespace airborne_radar {
namespace extension {

/**
 * @brief ArRecognitionStaticContext 识别链路的静态物理上下文（来自会话 hardware 域）。
 */
struct ArRecognitionStaticContext {
  config::engineering::TransmitterConfig transmitter{};
  config::engineering::ReceiverConfig receiver{};
  config::engineering::AntennaConfig antenna{};
};

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
   * @param[in] decision_control_config 决策控制配置。
   * @param[in] recognition_static_context 识别链路静态物理上下文（可为空默认）。
   */
  ArController(session::MutableArContext& ar_context, signal::ISignalPipeline& signal_pipeline,
               environment::IEnvironmentService& environment_service,
               config::DecisionControlConfig decision_control_config = {},
               ArRecognitionStaticContext recognition_static_context = {});

  /** @brief 原子更新后续成功周期使用的控制保持/冷却配置。 */
  void UpdateDecisionControlConfig(const config::DecisionControlConfig& decision_control_config);

  /**
   * @brief 原子更新识别运行期上下文（工作模式 + 识别策略配置）。
   * @note 由会话在运行期配置提交边界调用；识别数据库在路径变化时按需加载，
   *       加载失败保持原库并记录日志（识别降级为 kDisabled）。
   */
  void UpdateRecognitionRuntime(config::ArWorkMode work_mode,
                                const config::ArRecognitionConfig& recognition_config);

  /** @brief 最近周期是否发布了识别效能摘要。 */
  bool HasLatestRecognitionSummary() const;

  /** @brief 获取最近周期识别效能摘要。 */
  const session::ArRecognitionCycleSummary& GetLatestRecognitionSummary() const;

  /** @brief 当前生效识别特征数据库版本（供 replay 溯源）。 */
  std::string GetActiveRecognitionDatabaseVersion() const;

  /**
   * @brief 执行一次 AR 处理循环。
   * @param[in] cycle_input 本周期输入结构体（捆绑 scene_targets、RF v2 detection 上下文、
   *                         干扰观测与欺骗候选量测）。
   */
  void RunOnce(const signal::pipeline::SignalCycleInput& cycle_input);

  /**
   * @brief 在发射发布前消费上一成功周期的待决策并冻结本周期控制真值。
   * @return 本周期首次冻结返回 true；重复调用返回 false。
   */
  bool PrepareEmissionControl();

  /** @brief 在 Abandon 后释放控制冻结标记，不回滚已消费的控制真值。 */
  void ReleasePreparedEmissionControl();

  /** @brief 获取当前已冻结的控制真值。 */
  const session::ArControlProfile& GetControlProfile() const;

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
   * @brief 获取最近一次正常执行周期的 kInfo 排除诊断（规则 13b）。
   * @note 仅完成路径有内容；中止路径诊断由三写经 RecordAbort 写入。
   * @return 最近一次周期的按目标排除诊断列表。
   */
  const session::ArIssueList& GetLatestIssues() const;

  /**
   * @brief 最近一次 RunOnce 是否真正执行了 signal/decision/control 主链路。
   * @return 若最近一次周期完成主链路执行则返回 true。
   */
  bool ExecutedLatestCycle() const;

  /**
   * @brief 最近一次 RunOnce 若未执行成功，返回 signal pipeline 的 abort 原因。
   * @return 最近一次周期的 signal pipeline 终止原因；正常执行时为 kNone。
   */
  session::SignalCycleAbortReason GetLastSignalCycleAbortReason() const;

  /** @brief 获取最近成功周期发布的决策观测。 */
  const session::DecisionObservation& GetLatestDecisionObservation() const;

  /** @brief 当前是否存在可供外部模块响应的决策观测。 */
  bool HasLatestDecisionObservation() const;

  /** @brief 提交外部 profile 覆盖（整包替换值，绕过 TacticalProposal 管线与 hold/cooldown）。 */
  session::ExternalDecisionSubmitStatus SubmitExternalDecision(
      session::ExternalDecisionOverride override_decision);

  session::DecisionControlSource GetLastAppliedDecisionSource() const;
  std::uint32_t GetLastAppliedDecisionCycleIndex() const;
  std::uint64_t GetLastAppliedDecisionBatchId() const;
  const std::vector<session::TacticalProposal>& GetLastAppliedDecisionProposals() const;

  /**
   * @brief 捕获当前控制器运行态快照。
   * @return 可用于失败回滚的控制器运行态快照。
   */
  ArControllerRuntimeState CaptureRuntimeState() const;

  /**
   * @brief 恢复此前捕获的控制器运行态快照。
   * @param[in] state 待恢复的控制器运行态快照。
   * @return owner_identity 与 schema_version 校验通过并成功恢复时返回 true。
   */
  bool RestoreRuntimeState(const ArControllerRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RUNTIME_AR_CONTROLLER_H_
