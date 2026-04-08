#include "airborne_radar/runtime/RadarCycleOrchestrator.h"

#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/signal/assembly/IDataOutputManager.h"

namespace airborne_radar {
namespace extension {

RadarCycleOrchestrator::RadarCycleOrchestrator(
    extension::ISignalPipeline& signal_pipeline,
    extension::ITacticalDecisionEngine* decision_engine,
    extension::TacticalStateStore* tactical_state_store,
    extension::IEnvironmentService& environment_service,
    signal::assembly::IDataOutputManager& output_manager)
    : signal_pipeline_(signal_pipeline),
      decision_engine_(decision_engine),
      tactical_state_store_(tactical_state_store),
      environment_service_(environment_service),
      output_manager_(output_manager) {}

void RadarCycleOrchestrator::FreezeEnvironment(
    float cycle_dt_sec, const oneq::internal::runtime::RuntimeCycleStamp& stamp) {
  environment::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = stamp.cycle_index;
  environment_cycle_context.dt_sec = cycle_dt_sec;
  environment_service_.BeginCycle(environment_cycle_context);
}

CycleExecutionResult RadarCycleOrchestrator::Execute(
    const model::TargetFeatureList* target_features,
    const model::PlatformAttitudeDeg& platform_attitude,
    const extension::control::RadarControlProfile& current_profile,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) {
  const model::TargetFeatureList kEmptyTargets;
  const model::TargetFeatureList& features =
      target_features != nullptr ? *target_features : kEmptyTargets;

  signal_pipeline_.SetControlProfile(current_profile);
  signal_pipeline_.UpdatePlatformAttitude(platform_attitude);

  CycleExecutionResult result;
  result.signal_result = signal_pipeline_.RunCycle(features, environment_service_);

  model::DecisionInputFrame decision_frame = result.signal_result.decision_frame;
  decision_frame.cycle_index = stamp.cycle_index;
  decision_frame.batch_id = stamp.batch_id;

  result.track_output_frame = output_manager_.BuildTrackOutputFrame(
      stamp.cycle_index, stamp.batch_id, decision_frame.tracks);

  if (decision_engine_ != nullptr && tactical_state_store_ != nullptr) {
    result.decision_result =
        decision_engine_->Evaluate(decision_frame, *tactical_state_store_);
  }

  result.signal_result.decision_frame = decision_frame;
  return result;
}

}  // namespace extension
}  // namespace airborne_radar
