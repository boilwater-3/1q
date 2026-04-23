#ifndef AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_RUNTIME_HOOKS_H_
#define AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_RUNTIME_HOOKS_H_

#include "1q/airborne_radar/extension/ControlReducerTypes.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "airborne_radar/runtime/ControlCommandMapper.h"
#include "airborne_radar/runtime/RadarCycleOrchestrator.h"
#include "airborne_radar/runtime/components/AirborneRuntimeInput.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace airborne_radar {
namespace runtime {
namespace components {

class RadarRuntimeHooks {
 public:
  RadarRuntimeHooks(
      extension::RadarCycleOrchestrator& cycle_orchestrator,
      extension::ControlCommandMapper& command_mapper,
      extension::control::RadarControlProfile& control_profile,
      oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                                 session::ValidationIssueList>& runtime_state,
      bool& last_cycle_executed, bool& last_cycle_reused_previous_output,
      extension::SignalCycleAbortReason& last_signal_abort_reason);

  oneq::internal::runtime::RuntimeValidationResult<session::ValidationIssueList> Validate(
      const AirborneRuntimeInput& input) const;

  void FreezeEnvironment(const AirborneRuntimeInput& input,
                         const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  output::TrackOutputFrame Execute(
      const AirborneRuntimeInput& input,
      const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  output::TrackOutputFrame BuildErrorOutput(
      const AirborneRuntimeInput& input,
      const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

 private:
  extension::RadarCycleOrchestrator& cycle_orchestrator_;
  extension::ControlCommandMapper& command_mapper_;
  extension::control::RadarControlProfile& control_profile_;
  oneq::internal::runtime::RuntimeCycleState<output::TrackOutputFrame,
                                             session::ValidationIssueList>& runtime_state_;
  bool& last_cycle_executed_;
  bool& last_cycle_reused_previous_output_;
  extension::SignalCycleAbortReason& last_signal_abort_reason_;
};

}  // namespace components
}  // namespace runtime
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RUNTIME_COMPONENTS_RADAR_RUNTIME_HOOKS_H_
