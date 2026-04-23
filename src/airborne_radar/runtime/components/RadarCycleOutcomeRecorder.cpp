#include "airborne_radar/runtime/components/RadarCycleOutcomeRecorder.h"

namespace airborne_radar {
namespace runtime {
namespace components {

RadarCycleOutcomeRecorder::RadarCycleOutcomeRecorder(
    environment::IEnvironmentService& environment_service,
    extension::ISignalPipeline& signal_pipeline,
    oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                               session::ValidationIssueList>& runtime_state,
    std::uint32_t& cycle_index, bool& last_cycle_executed,
    bool& last_cycle_reused_previous_output,
    extension::SignalCycleAbortReason& last_signal_abort_reason)
    : environment_service_(environment_service),
      signal_pipeline_(signal_pipeline),
      runtime_state_(runtime_state),
      cycle_index_(cycle_index),
      last_cycle_executed_(last_cycle_executed),
      last_cycle_reused_previous_output_(last_cycle_reused_previous_output),
      last_signal_abort_reason_(last_signal_abort_reason) {}

RadarCycleSnapshot RadarCycleOutcomeRecorder::CaptureSnapshot() const {
  RadarCycleSnapshot snapshot;
  snapshot.environment_state = environment_service_.CaptureRuntimeState();
  snapshot.previous_output = runtime_state_.latest_output;
  snapshot.had_previous_output = runtime_state_.has_latest_output;
  snapshot.previous_batch_id = runtime_state_.next_batch_id;
  snapshot.previous_cycle_index = cycle_index_;
  snapshot.pipeline_state = signal_pipeline_.CaptureRuntimeState();
  return snapshot;
}

void RadarCycleOutcomeRecorder::ResetPerCycleFlags() const {
  last_cycle_executed_ = false;
  last_cycle_reused_previous_output_ = false;
  last_signal_abort_reason_ = extension::SignalCycleAbortReason::kNone;
}

void RadarCycleOutcomeRecorder::RestoreFromFailedCycle(
    const RadarCycleSnapshot& snapshot) const {
  environment_service_.RestoreRuntimeState(snapshot.environment_state);
  signal_pipeline_.RestoreRuntimeState(snapshot.pipeline_state);
  runtime_state_.latest_output = snapshot.previous_output;
  runtime_state_.has_latest_output = snapshot.had_previous_output;
  runtime_state_.next_batch_id = snapshot.previous_batch_id;
  cycle_index_ = snapshot.previous_cycle_index;
}

void RadarCycleOutcomeRecorder::CommitSuccessfulCycle() const { ++cycle_index_; }

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar
