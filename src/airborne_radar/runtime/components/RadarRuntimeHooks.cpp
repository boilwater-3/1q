#include "airborne_radar/runtime/components/RadarRuntimeHooks.h"

#include "airborne_radar/session/RadarSceneTargetConversion.h"

#include <cstddef>

#include "airborne_radar/runtime/CycleTelemetryLogger.h"

namespace airborne_radar {
namespace runtime {
namespace components {

RadarRuntimeHooks::RadarRuntimeHooks(
    extension::RadarCycleOrchestrator& cycle_orchestrator,
    extension::ControlCommandMapper& command_mapper,
    extension::control::RadarControlProfile& control_profile,
    oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                               session::ValidationIssueList>& runtime_state,
    bool& last_cycle_executed, bool& last_cycle_reused_previous_output,
    extension::SignalCycleAbortReason& last_signal_abort_reason)
    : cycle_orchestrator_(cycle_orchestrator),
      command_mapper_(command_mapper),
      control_profile_(control_profile),
      runtime_state_(runtime_state),
      last_cycle_executed_(last_cycle_executed),
      last_cycle_reused_previous_output_(last_cycle_reused_previous_output),
      last_signal_abort_reason_(last_signal_abort_reason) {}

oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList>
RadarRuntimeHooks::Validate(const AirborneRuntimeInput& input) const {
  oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> result;
  result.issues = session::ValidateRadarCycleDeltaTime(input.cycle_dt_sec);
  if (input.target_features != nullptr) {
    const session::RadarSceneTargetList scene_targets = session::ToSceneTargetList(*input.target_features);
    const session::ValidationIssueList target_issues = session::ValidateTargetFeatures(scene_targets);
    result.issues.insert(result.issues.end(), target_issues.begin(), target_issues.end());
  }
  result.has_error = session::HasValidationError(result.issues);
  return result;
}

void RadarRuntimeHooks::FreezeEnvironment(
    const AirborneRuntimeInput& input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  cycle_orchestrator_.FreezeEnvironment(input.cycle_dt_sec, stamp);
}

output::TrackOutputFrame RadarRuntimeHooks::Execute(
    const AirborneRuntimeInput& input,
  const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  const extension::CycleExecutionResult exec_result = cycle_orchestrator_.Execute(
      input.target_features, input.platform_attitude, control_profile_, stamp);
  last_cycle_executed_ = exec_result.signal_result.executed_this_cycle;
  last_signal_abort_reason_ = exec_result.signal_result.abort_reason;
  last_cycle_reused_previous_output_ = !last_cycle_executed_ && runtime_state_.has_latest_output;
  if (!last_cycle_executed_) {
    return BuildErrorOutput(input, stamp);
  }

  const extension::ControlReductionResult reduction_result =
      command_mapper_.Apply(&control_profile_, exec_result.decision_result.proposals);

  const std::size_t input_target_count =
      input.target_features != nullptr ? input.target_features->size() : 0U;
  extension::CycleTelemetryLogger::LogCycleSummary(
      extension::CycleTelemetryPayload(
          stamp, input_target_count, exec_result.signal_result.decision_frame.tracks.size(),
          reduction_result.applied_directives.size(),
          exec_result.signal_result.decision_frame.environment_jamming_detected,
          control_profile_.version,
          exec_result.signal_result.decision_frame.perception_quality_info,
          exec_result.signal_result.association_quality_metrics));

  return exec_result.track_output_frame;
}

output::TrackOutputFrame RadarRuntimeHooks::BuildErrorOutput(
    const AirborneRuntimeInput& input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  (void)input;
  last_cycle_executed_ = false;
  last_cycle_reused_previous_output_ = runtime_state_.has_latest_output;
  if (runtime_state_.has_latest_output) {
    return runtime_state_.latest_output;
  }
  output::TrackOutputFrame frame;
  frame.cycle_index = stamp.cycle_index;
  frame.batch_id = stamp.batch_id;
  return frame;
}

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar
