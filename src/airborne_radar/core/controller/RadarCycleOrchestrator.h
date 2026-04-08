/**
 * @file RadarCycleOrchestrator.h
 * @brief 封装单周期信号流水线、输出装配与决策引擎的编排职责。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CYCLE_ORCHESTRATOR_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CYCLE_ORCHESTRATOR_H_

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace signal {
namespace assembly {
class IDataOutputManager;
}  // namespace assembly
}  // namespace signal

namespace extension {

/**
 * @brief CycleExecutionResult 聚合信号+决策主链路的单周期输出。
 */
struct CycleExecutionResult {
  extension::SignalCycleResult signal_result;     /**< 信号流水线原始结果。 */
  output::TrackOutputFrame track_output_frame;  /**< 装配后的中性输出帧。 */
  extension::TacticalDecisionResult decision_result; /**< 决策引擎输出。 */
};

/**
 * @brief 负责驱动单周期"环境冻结 → 信号流水线 → 决策引擎"主链路。
 *
 * 不持有控制真值所有权，控制真值由调用方（Impl）持有并在 Apply 阶段更新。
 */
class RadarCycleOrchestrator {
 public:
  /**
   * @brief 构造 orchestrator，所有依赖均为外部生命周期管理。
   * @param signal_pipeline   信号流水线引用。
   * @param decision_engine   决策引擎指针（可为 nullptr，为空时跳过决策）。
   * @param tactical_state_store 跨周期战术内存指针（可为 nullptr）。
   * @param environment_service  环境服务引用。
   * @param output_manager    数据输出装配接口引用。
   */
  RadarCycleOrchestrator(
      extension::ISignalPipeline& signal_pipeline,
      extension::ITacticalDecisionEngine* decision_engine,
      extension::TacticalStateStore* tactical_state_store,
      extension::IEnvironmentService& environment_service,
      signal::assembly::IDataOutputManager& output_manager);

  /**
   * @brief 冻结本周期环境，驱动 IEnvironmentService::BeginCycle。
   * @param cycle_dt_sec 当前周期步长（秒）。
   * @param stamp        当前周期时间戳。
   */
  void FreezeEnvironment(float cycle_dt_sec,
                         const oneq::internal::runtime::RuntimeCycleStamp& stamp);

  /**
   * @brief 执行信号流水线与决策引擎，返回完整周期执行结果。
   * @param target_features   当前周期目标特征列表（可为 nullptr，视为空列表）。
   * @param platform_attitude 当前平台姿态。
   * @param current_profile   本周期生效的控制真值（只读）。
   * @param stamp             当前周期时间戳。
   * @return 信号+决策的聚合结果。
   */
  CycleExecutionResult Execute(
      const model::TargetFeatureList* target_features,
      const model::PlatformAttitudeDeg& platform_attitude,
      const extension::control::RadarControlProfile& current_profile,
      const oneq::internal::runtime::RuntimeCycleStamp& stamp);

 private:
  extension::ISignalPipeline& signal_pipeline_;
  extension::ITacticalDecisionEngine* decision_engine_{nullptr};
  extension::TacticalStateStore* tactical_state_store_{nullptr};
  extension::IEnvironmentService& environment_service_;
  signal::assembly::IDataOutputManager& output_manager_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_RADAR_CYCLE_ORCHESTRATOR_H_
