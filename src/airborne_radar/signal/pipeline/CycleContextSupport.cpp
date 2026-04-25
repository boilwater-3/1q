#include "airborne_radar/signal/pipeline/CycleContextSupport.h"

#include <algorithm>

#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

bool HasValidCycleWorkspace(const CycleWorkspace& workspace) {
  return workspace.output_state != nullptr && workspace.decision_frame != nullptr &&
         workspace.association_quality_metrics != nullptr &&
         workspace.track_measurements != nullptr && workspace.signal_term_db != nullptr &&
         workspace.speed_penalty_db != nullptr && workspace.detection_margin_db != nullptr &&
         workspace.detection_succeeded != nullptr && workspace.association_keys != nullptr &&
         workspace.measurement_slots != nullptr && workspace.target_geometry != nullptr &&
         workspace.measurement_covariances != nullptr && workspace.association_result != nullptr;
}

}  // namespace

void ResetCycleWorkspace(const session::RadarSceneTargetList& input_state,
                         const ExecutionConfig& runtime_config, CycleWorkspace* workspace) {
  (void)runtime_config;
  if (workspace == nullptr || !HasValidCycleWorkspace(*workspace)) {
    return;
  }

  const std::size_t target_count = input_state.size();
  workspace->output_state->resize(target_count);
  *workspace->decision_frame = model::DecisionInputFrame();
  *workspace->association_quality_metrics = AssociationQualityMetrics();
  workspace->track_measurements->clear();
  workspace->signal_term_db->resize(target_count);
  workspace->speed_penalty_db->resize(target_count);
  workspace->detection_margin_db->resize(target_count);
  workspace->detection_succeeded->resize(target_count);
  workspace->association_keys->resize(target_count);
  workspace->measurement_slots->assign(target_count, -1);
  workspace->target_geometry->resize(target_count);
  RefreshMeasurementCovariances(target_count, 10.0f, workspace->measurement_covariances);
  *workspace->association_result = association::AssociationResult();
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
    if (!assembly::internal::SyncAutoLifecycleManagerForRuntimeConfig(runtime_config,
                                                                      auto_lifecycle_manager)) {
      return false;
    }
  }
  if (association_engine != nullptr) {
    association_engine->UpdateConfig(
        assembly::internal::SignalComponentFactory::BuildAssociationConfig(runtime_config));
  }
  if (track_filter != nullptr) {
    track_filter->UpdateConfig(
        assembly::internal::SignalComponentFactory::BuildTrackFilterConfig(runtime_config));
  }
  return true;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
