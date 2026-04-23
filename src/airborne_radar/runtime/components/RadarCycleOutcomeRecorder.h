#ifndef AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_CYCLE_OUTCOME_RECORDER_H_
#define AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_CYCLE_OUTCOME_RECORDER_H_

#include <cstdint>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace runtime {
namespace components {

struct RadarCycleSnapshot {
  environment::EnvironmentServiceRuntimeState environment_state{};
  output::TrackOutputFrame previous_output{};
  bool had_previous_output{false};
  std::uint64_t previous_batch_id{0U};
  std::uint32_t previous_cycle_index{0U};
  extension::SignalPipelineRuntimeState pipeline_state{};
};

class RadarCycleOutcomeRecorder {
 public:
  RadarCycleOutcomeRecorder(
      environment::IEnvironmentService& environment_service,
      extension::ISignalPipeline& signal_pipeline,
      oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                                 session::ValidationIssueList>& runtime_state,
      std::uint32_t& cycle_index, bool& last_cycle_executed,
      bool& last_cycle_reused_previous_output,
      extension::SignalCycleAbortReason& last_signal_abort_reason);

  RadarCycleSnapshot CaptureSnapshot() const;

  void ResetPerCycleFlags() const;

  void RestoreFromFailedCycle(const RadarCycleSnapshot& snapshot) const;

  void CommitSuccessfulCycle() const;

 private:
  environment::IEnvironmentService& environment_service_;
  extension::ISignalPipeline& signal_pipeline_;
  oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                             session::ValidationIssueList>& runtime_state_;
  std::uint32_t& cycle_index_;
  bool& last_cycle_executed_;
  bool& last_cycle_reused_previous_output_;
  extension::SignalCycleAbortReason& last_signal_abort_reason_;
};

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_CYCLE_OUTCOME_RECORDER_H_
