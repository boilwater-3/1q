#include "1q/airborne_radar/core/controller/RadarController.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "1q/airborne_radar/extension/control/ControlDirective.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
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
 * @brief 将干扰语义枚举转换为日志可读字符串。
 * @param semantic 当前周期主导干扰语义。
 * @return 供日志输出使用的短字符串。
 */
const char* JammingSemanticName(common::utils::JammingSemantic semantic) {
  switch (semantic) {
    case common::utils::JammingSemantic::kNoiseSuppression:
      return "noise";
    case common::utils::JammingSemantic::kDeception:
      return "deception";
    case common::utils::JammingSemantic::kRepeater:
      return "repeater";
    case common::utils::JammingSemantic::kMixed:
      return "mixed";
    case common::utils::JammingSemantic::kNone:
    default:
      return "none";
  }
}

/**
 * @brief 将控制意图类型映射为底座命令类型。
 * @param type 控制意图类型。
 * @return 对应的雷达命令类型。
 */
common::control::RadarCommandType ToRadarCommandType(common::control::ControlDirectiveType type) {
  switch (type) {
    case common::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return common::control::RadarCommandType::SET_LPI_POWER;
    case common::control::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      return common::control::RadarCommandType::SET_LPI_BEAMFORMING;
    case common::control::ControlDirectiveType::REQUEST_LPI_DWELL:
      return common::control::RadarCommandType::SET_LPI_DWELL;
    case common::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return common::control::RadarCommandType::ENABLE_SIDELOBE_CANCELLER;
    case common::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return common::control::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING;
    case common::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return common::control::RadarCommandType::SET_AGILITY_FREQ;
    case common::control::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return common::control::RadarCommandType::SET_ECCM_REJITTER;
    case common::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return common::control::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN;
    case common::control::ControlDirectiveType::NONE:
    default:
      return common::control::RadarCommandType::NONE;
  }
}

/**
 * @brief 将控制意图来源映射为底座命令来源。
 * @param source 控制意图来源。
 * @return 对应的雷达命令来源。
 */
common::control::RadarCommandSource ToRadarCommandSource(
    common::control::ControlDirectiveSource source) {
  switch (source) {
    case common::control::ControlDirectiveSource::THREAT_ASSESSMENT:
      return common::control::RadarCommandSource::CLASSIFIER;
    case common::control::ControlDirectiveSource::EMISSION_CONTROL:
      return common::control::RadarCommandSource::LPI;
    case common::control::ControlDirectiveSource::SURVIVABILITY:
      return common::control::RadarCommandSource::ECCM;
    case common::control::ControlDirectiveSource::UNKNOWN:
    default:
      return common::control::RadarCommandSource::UNKNOWN;
  }
}

/**
 * @brief 将单条控制意图转换为可提交的雷达命令。
 * @param directive 单条控制意图。
 * @return 与控制意图等价的雷达命令。
 */
common::control::RadarCommand ToRadarCommand(const common::control::ControlDirective& directive) {
  return common::control::RadarCommand(ToRadarCommandType(directive.type),
                                       ToRadarCommandSource(directive.source));
}

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
        output_manager(new signal::assembly::DataOutputManager()) {
    decision_engine = owned_decision_engine.get();
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
        output_manager(new signal::assembly::DataOutputManager()) {}

  void ExecuteCommands(const std::vector<common::control::ControlDirective>& directives) {
    for (std::size_t i = 0; i < directives.size(); ++i) {
      const common::control::RadarCommand command = ToRadarCommand(directives[i]);
      if (command.type == common::control::RadarCommandType::NONE) {
        continue;
      }
      radar_context.SubmitControlCommand(command);
    }
  }
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
      environment::EnvironmentCycleContext environment_cycle_context;
      environment_cycle_context.cycle_index = stamp.cycle_index;
      environment_cycle_context.dt_sec = input.cycle_dt_sec;
      impl->environment_service.BeginCycle(environment_cycle_context);
    }

    common::output::TrackOutputFrame Execute(
        const AirborneRuntimeInput& input,
        const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
      const common::model::TargetFeatureList kEmptyTargets;
      const common::model::TargetFeatureList& target_features =
          input.target_features != nullptr ? *input.target_features : kEmptyTargets;
      impl->signal_pipeline.SetControlProfile(impl->control_profile.get());
      impl->signal_pipeline.UpdatePlatformAttitude(input.platform_attitude);

      const signal::pipeline::SignalCycleResult signal_result =
          impl->signal_pipeline.RunCycle(target_features, impl->environment_service);
      const signal::pipeline::AssociationQualityMetrics association_metrics =
          signal_result.association_quality_metrics;
      common::model::DecisionInputFrame decision_frame = signal_result.decision_frame;
      decision_frame.cycle_index = stamp.cycle_index;
      decision_frame.batch_id = stamp.batch_id;
      common::output::TrackOutputFrame track_output_frame =
          impl->output_manager->BuildTrackOutputFrame(stamp.cycle_index, stamp.batch_id,
                                                      decision_frame.tracks);

      decision::pipeline::TacticalDecisionResult decision_result;
      if (impl->decision_engine != nullptr && impl->tactical_state_store != nullptr) {
        decision_result =
            impl->decision_engine->Evaluate(decision_frame, *impl->tactical_state_store);
      }

      const decision::pipeline::ControlReductionResult reduction_result =
          impl->control_reducer != nullptr
              ? impl->control_reducer->Reduce(impl->control_profile.get(),
                                              decision_result.proposals)
              : decision::pipeline::ControlReductionResult();
      if (impl->tactical_state_store != nullptr && impl->control_reducer != nullptr) {
        const decision::pipeline::ControlReducerRuntimeState reducer_state =
            impl->control_reducer->GetRuntimeState();
        impl->tactical_state_store->lpi_hold_cycles_remaining =
            reducer_state.lpi_hold_cycles_remaining;
        impl->tactical_state_store->eccm_hold_cycles_remaining =
            reducer_state.eccm_hold_cycles_remaining;
      }
      impl->control_profile.get() = reduction_result.profile;
      impl->radar_context.UpdateRadarControlProfile(impl->control_profile.get());
      impl->ExecuteCommands(reduction_result.applied_directives);

      PROJECT_LOG_DEBUG(
          "[RadarController] cycle summary: cycle_index={} batch_id={} "
          "input_targets={} decision_features={} directives={} "
          "jamming_detected={} profile_version={} detect_rate={:.3f} "
          "detect_stress={:.3f} assoc_priors={} assoc_detections={} "
          "assoc_matches={} assoc_new_tracks={} assoc_missed_tracks={} "
          "assoc_match_rate={:.3f} assoc_new_track_rate={:.3f} "
          "assoc_missed_rate={:.3f} assoc_mean_cost={:.3f} "
          "assoc_p95_cost={:.3f} assoc_jam_semantic={} "
          "assoc_jam_severity={:.3f} assoc_stress={:.3f}",
          stamp.cycle_index, stamp.batch_id, target_features.size(), decision_frame.tracks.size(),
          reduction_result.applied_directives.size(),
          decision_frame.environment_jamming_detected ? "true" : "false",
          impl->control_profile.get().version,
          decision_frame.perception_quality_info.detection_rate,
          decision_frame.perception_quality_info.detection_stress,
          association_metrics.prior_track_count, association_metrics.detection_count,
          association_metrics.matched_count, association_metrics.new_track_count,
          association_metrics.missed_track_count, association_metrics.match_rate,
          association_metrics.new_track_rate, association_metrics.missed_track_rate,
          association_metrics.mean_match_cost, association_metrics.p95_match_cost,
          JammingSemanticName(association_metrics.dominant_jamming_semantic),
          association_metrics.jamming_severity, association_metrics.association_stress);
      return track_output_frame;
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
