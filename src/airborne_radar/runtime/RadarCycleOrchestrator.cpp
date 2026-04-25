#include "airborne_radar/runtime/RadarCycleOrchestrator.h"

#include <algorithm>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/signal/assembly/IDataOutputManager.h"

namespace airborne_radar {
namespace extension {

RadarCycleOrchestrator::RadarCycleOrchestrator(
    extension::ISignalPipeline& signal_pipeline,
    extension::ITacticalDecisionEngine* decision_engine,
    extension::TacticalStateStore* tactical_state_store,
    environment::IEnvironmentService& environment_service,
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
    const session::RadarSceneTargetList* scene_targets,
    const model::PlatformAttitudeDeg& platform_attitude,
    const extension::control::RadarControlProfile& current_profile,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) {
  const session::RadarSceneTargetList kEmptyTargets;
  const session::RadarSceneTargetList& targets =
      scene_targets != nullptr ? *scene_targets : kEmptyTargets;

  signal_pipeline_.SetControlProfile(current_profile);
  signal_pipeline_.UpdatePlatformAttitude(platform_attitude);

  CycleExecutionResult result;
  result.signal_result = signal_pipeline_.RunCycle(targets, environment_service_);
  if (!result.signal_result.executed_this_cycle) {
    return result;
  }

  model::DecisionInputFrame decision_frame = result.signal_result.decision_frame;
  decision_frame.cycle_index = stamp.cycle_index;
  decision_frame.batch_id = stamp.batch_id;

  result.track_output_frame = output_manager_.BuildTrackOutputFrame(
      stamp.cycle_index, stamp.batch_id, decision_frame.tracks);

  if (decision_engine_ != nullptr && tactical_state_store_ != nullptr) {
    result.decision_result =
        decision_engine_->Evaluate(decision_frame, *tactical_state_store_);

    // 将目标分类结果回填到轨迹输出帧
    const auto& classifications = result.decision_result.target_classification_result;
    auto& output_tracks = result.track_output_frame.tracks;
    const std::size_t count = std::min(classifications.size(), output_tracks.size());
    for (std::size_t i = 0; i < count; ++i) {
      output_tracks[i].target_type = classifications[i].target_type;
      output_tracks[i].target_probability = classifications[i].probability;
    }
  }

  result.signal_result.decision_frame = decision_frame;
  return result;
}

}  // namespace extension
}  // namespace airborne_radar
