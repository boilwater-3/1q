#include "1q/airborne_radar/core/controller/RadarController.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/core/controller/ControlCommandMapper.h"
#include "airborne_radar/core/controller/CycleTelemetryLogger.h"
#include "airborne_radar/core/controller/RadarCycleOrchestrator.h"
#include "airborne_radar/decision/pipeline/ControlReducer.h"
#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"
#include "airborne_radar/signal/assembly/DataOutputManager.h"
#include "airborne_radar/signal/assembly/IDataOutputManager.h"
#include "common/logging/ProjectLog.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace core {
namespace controller {

namespace {

/**
 * @brief AirborneRuntimeInput 描述单周期骨架执行需要的输入快照。
 */
struct AirborneRuntimeInput {
  const common::model::TargetFeatureList* target_features{
      nullptr};                                            /**< 当前周期目标输入只读视图。 */
  common::model::PlatformAttitudeDeg platform_attitude{}; /**< 当前平台姿态。 */
  float cycle_dt_sec{1.0f};                                /**< 当前周期步长。 */
};

}  // namespace

/**
 * @brief RadarController 内部实现体，持有所有运行时状态。
 */
struct RadarController::Impl {
  core::context::IRadarContext& radar_context;
  signal::pipeline::ISignalPipeline& signal_pipeline;
  decision::pipeline::ITacticalDecisionEngine* decision_engine{nullptr};
  std::unique_ptr<decision::pipeline::ITacticalDecisionEngine> owned_decision_engine;
  environment::IEnvironmentService& environment_service;
  std::unique_ptr<common::control::RadarControlProfile> owned_control_profile;
  std::reference_wrapper<common::control::RadarControlProfile> control_profile;
  std::unique_ptr<decision::pipeline::TacticalStateStore> tactical_state_store;
  std::unique_ptr<decision::pipeline::ControlReducer> control_reducer;
  std::unique_ptr<signal::assembly::IDataOutputManager> output_manager;
  std::unique_ptr<controller::ControlCommandMapper> command_mapper;
  std::unique_ptr<controller::RadarCycleOrchestrator> cycle_orchestrator;
  oneq::internal::runtime::RuntimeCycleState<common::output::TrackOutputFrame,
                                             context::ValidationIssueList>
      runtime_state{};
  std::uint32_t cycle_index{1};

  Impl(core::context::IRadarContext& ctx, signal::pipeline::ISignalPipeline& sig,
       environment::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        owned_decision_engine(new decision::pipeline::TacticalCoordinator()),
        environment_service(env),
        owned_control_profile(new common::control::RadarControlProfile()),
        control_profile(*owned_control_profile),
        tactical_state_store(new decision::pipeline::TacticalStateStore()),
        control_reducer(new decision::pipeline::ControlReducer()),
        output_manager(new signal::assembly::DataOutputManager()),
        command_mapper(new controller::ControlCommandMapper(
            *control_reducer, tactical_state_store.get(), ctx)) {
    decision_engine = owned_decision_engine.get();
    cycle_orchestrator.reset(new controller::RadarCycleOrchestrator(
        sig, decision_engine, tactical_state_store.get(), env, *output_manager));
  }

  Impl(core::context::IRadarContext& ctx, signal::pipeline::ISignalPipeline& sig,
       decision::pipeline::ITacticalDecisionEngine& ext_engine,
       environment::IEnvironmentService& env)
      : radar_context(ctx),
        signal_pipeline(sig),
        decision_engine(&ext_engine),
        environment_service(env),
        owned_control_profile(new common::control::RadarControlProfile()),
        control_profile(*owned_control_profile),
        tactical_state_store(new decision::pipeline::TacticalStateStore()),
        control_reducer(new decision::pipeline::ControlReducer()),
        output_manager(new signal::assembly::DataOutputManager()),
        command_mapper(new controller::ControlCommandMapper(
            *control_reducer, tactical_state_store.get(), ctx)),
        cycle_orchestrator(new controller::RadarCycleOrchestrator(
            sig, &ext_engine, tactical_state_store.get(), env, *output_manager)) {}
};

RadarController::RadarController(core::context::IRadarContext& radar_context,
                                 signal::pipeline::ISignalPipeline& signal_pipeline,
                                 environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, environment_service)) {}

RadarController::RadarController(core::context::IRadarContext& radar_context,
                                 signal::pipeline::ISignalPipeline& signal_pipeline,
                                 decision::pipeline::ITacticalDecisionEngine& decision_engine,
                                 environment::IEnvironmentService& environment_service)
    : impl_(new Impl(radar_context, signal_pipeline, decision_engine, environment_service)) {}

RadarController::~RadarController() = default;

void RadarController::RunOnce() {
  AirborneRuntimeInput runtime_input;
  runtime_input.target_features = &impl_->radar_context.GetTargetFeatures();
  runtime_input.platform_attitude = impl_->radar_context.GetPlatformAttitude();
  runtime_input.cycle_dt_sec = impl_->radar_context.GetCycleDeltaTimeSec();

  struct AirborneRuntimeHooks {
    Impl* impl{nullptr};

    oneq::internal::runtime::RuntimeValidationResult<context::ValidationIssueList> Validate(
        const AirborneRuntimeInput& input) const {
      oneq::internal::runtime::RuntimeValidationResult<context::ValidationIssueList> result;
      result.issues = context::ValidateRadarCycleDeltaTime(input.cycle_dt_sec);
      if (input.target_features != nullptr) {
        const context::ValidationIssueList target_issues =
            context::ValidateTargetFeatures(*input.target_features);
        result.issues.insert(result.issues.end(), target_issues.begin(), target_issues.end());
      }
      result.has_error = context::HasValidationError(result.issues);
      return result;
    }

    void FreezeEnvironment(const AirborneRuntimeInput& input,
                           const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      if (impl == nullptr) {
        return;
      }
      impl->cycle_orchestrator->FreezeEnvironment(input.cycle_dt_sec, stamp);
    }

    common::output::TrackOutputFrame Execute(
        const AirborneRuntimeInput& input,
        const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      const CycleExecutionResult exec_result = impl->cycle_orchestrator->Execute(
          input.target_features, input.platform_attitude, impl->control_profile.get(), stamp);

      const decision::pipeline::ControlReductionResult reduction_result =
          impl->command_mapper->Apply(&impl->control_profile.get(),
                                      exec_result.decision_result.proposals);

      const std::size_t input_target_count =
          input.target_features != nullptr ? input.target_features->size() : 0u;
      CycleTelemetryLogger::LogCycleSummary(
          CycleTelemetryPayload(
              stamp, input_target_count,
              exec_result.signal_result.decision_frame.tracks.size(),
              reduction_result.applied_directives.size(),
              exec_result.signal_result.decision_frame.environment_jamming_detected,
              impl->control_profile.get().version,
              exec_result.signal_result.decision_frame.perception_quality_info,
              exec_result.signal_result.association_quality_metrics));
      return exec_result.track_output_frame;
    }

    common::output::TrackOutputFrame BuildErrorOutput(
        const AirborneRuntimeInput& input,
        const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      (void)input;
      common::output::TrackOutputFrame frame;
      frame.cycle_index = stamp.cycle_index;
      frame.batch_id = stamp.batch_id;
      return frame;
    }
  };

  AirborneRuntimeHooks hooks;
  hooks.impl = impl_.get();
  oneq::internal::runtime::ExecuteRuntimeCycle(runtime_input, impl_->cycle_index,
                                               &impl_->runtime_state, &hooks);
  ++impl_->cycle_index;
}

void RadarController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce();
  }
}

void RadarController::UpdateControlReducerConfig(
    const decision::pipeline::ControlReducerConfig& config) {
  if (impl_->control_reducer == nullptr) {
    return;
  }
  impl_->control_reducer->UpdateConfig(config);
  PROJECT_LOG_INFO(
      "[RadarController] control reducer config updated: "
      "lpi_power_scale={} dwell_scale={} burnthrough_gain={} "
      "burnthrough_power_floor={} lpi_hold={} eccm_hold={} "
      "lpi_cooldown={} eccm_cooldown={} prefer_survivability_power={} "
      "prefer_survivability_beam={}",
      config.lpi_power_scale_on_reduction, config.lpi_dwell_scale, config.eccm_burnthrough_gain,
      config.burnthrough_lpi_power_floor, config.lpi_hold_cycles_after_request,
      config.eccm_hold_cycles_after_request, config.lpi_cooldown_cycles_after_release,
      config.eccm_cooldown_cycles_after_release,
      config.prefer_survivability_in_power_conflict ? "true" : "false",
      config.prefer_survivability_in_beam_conflict ? "true" : "false");
}

bool RadarController::HasLatestTrackOutputFrame() const {
  return impl_->runtime_state.has_latest_output;
}

const common::output::TrackOutputFrame& RadarController::GetLatestTrackOutputFrame() const {
  return impl_->runtime_state.latest_output;
}

const context::ValidationIssueList& RadarController::GetLastValidationIssues() const {
  return impl_->runtime_state.last_validation_issues;
}

bool RadarController::HasValidationError() const {
  return context::HasValidationError(impl_->runtime_state.last_validation_issues);
}

core::context::IRadarContext& RadarController::GetRadarContext() { return impl_->radar_context; }

signal::pipeline::ISignalPipeline& RadarController::GetSignalPipeline() {
  return impl_->signal_pipeline;
}

environment::IEnvironmentService& RadarController::GetEnvironmentService() {
  return impl_->environment_service;
}

}  // namespace controller
}  // namespace core
}  // namespace airborne_radar
