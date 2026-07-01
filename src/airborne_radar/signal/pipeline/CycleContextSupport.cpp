#include "airborne_radar/signal/pipeline/CycleContextSupport.h"

#include <algorithm>

#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

void ResetCycleExecutionScratch(const session::ArSceneTargetList& input_state,
                                CycleExecutionScratch& scratch) {
  const std::size_t target_count = input_state.size();
  scratch.output_state.resize(target_count);
  scratch.decision_frame = session::DecisionInputFrame();
  scratch.association_quality_metrics = AssociationQualityMetrics();
  scratch.track_measurements.clear();
  scratch.signal_term_db.resize(target_count);
  scratch.speed_penalty_db.resize(target_count);
  scratch.detection_margin_db.resize(target_count);
  scratch.detection_succeeded.resize(target_count);
  scratch.association_keys.resize(target_count);
  scratch.measurement_slots.assign(target_count, -1);
  scratch.target_geometry.resize(target_count);
  scratch.association_result = association::AssociationResult();
  scratch.dominant_jamming_semantic = config::JammingSemantic::kNone;
  scratch.jamming_severity = 0.0f;
}

void RefreshMeasurementCovariances(
    std::size_t target_count, float kalman_measurement_noise_std,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances) {
  if (measurement_covariances == nullptr) {
    return;
  }
  const float variance =
      std::max(0.0f, kalman_measurement_noise_std * kalman_measurement_noise_std);
  measurement_covariances->assign(target_count,
                                  tracking::MeasurementCovariance::Identity() * variance);
}

bool SyncAssociationAndTrackFilterConfigs(
    const ExecutionConfig& runtime_config, association::DataAssociationEngine* association_engine,
    tracking::TrackFilter* track_filter, tracking::ITrackLifecycleManager* auto_lifecycle_manager) {
  if (auto_lifecycle_manager != nullptr) {
    if (!SyncAutoLifecycleManagerForRuntimeConfig(runtime_config, auto_lifecycle_manager)) {
      return false;
    }
  }
  if (association_engine != nullptr) {
    association_engine->UpdateConfig(
        SignalComponentFactory::BuildAssociationConfig(runtime_config));
  }
  if (track_filter != nullptr) {
    track_filter->UpdateConfig(SignalComponentFactory::BuildTrackFilterConfig(runtime_config));
  }
  return true;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
