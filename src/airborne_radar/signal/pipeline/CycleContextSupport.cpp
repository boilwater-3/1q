#include "airborne_radar/signal/pipeline/CycleContextSupport.h"

#include <algorithm>

#include "airborne_radar/signal/runtime/SignalComponentFactory.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

/**
 * @brief 检测 CycleWorkspace 中所有必需指针是否均已挂载
 * @param[in] workspace 待检查的周期工作空间
 * @return 所有指针均非空时返回 true，否则返回 false
 */
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

void ResetCycleWorkspace(const common::TargetFeatureList& input_state,
                         const SignalPipelineConfig& runtime_config, CycleWorkspace* workspace) {
  if (workspace == nullptr || !HasValidCycleWorkspace(*workspace)) {
    return;
  }

  *workspace->output_state = input_state;
  *workspace->decision_frame = common::DecisionInputFrame();
  *workspace->association_quality_metrics = AssociationQualityMetrics();

  const std::size_t target_count = input_state.size();
  workspace->track_measurements->clear();
  workspace->signal_term_db->assign(target_count, 0.0f);
  workspace->speed_penalty_db->assign(target_count, 0.0f);
  workspace->detection_margin_db->assign(target_count, 0.0f);
  workspace->detection_succeeded->assign(target_count, 0U);
  workspace->association_keys->assign(target_count, 0U);
  workspace->measurement_slots->assign(target_count, -1);
  workspace->target_geometry->resize(target_count);
  RefreshMeasurementCovariances(target_count, runtime_config.tracking.kalman_measurement_noise_std,
                                workspace->measurement_covariances);
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

void SyncAssociationAndTrackFilterConfigs(const SignalPipelineConfig& runtime_config,
                                          association::DataAssociationEngine* association_engine,
                                          tracking::TrackFilter* track_filter) {
  if (association_engine != nullptr) {
    association_engine->UpdateConfig(
        runtime::internal::SignalComponentFactory::BuildAssociationConfig(runtime_config));
  }
  if (track_filter != nullptr) {
    track_filter->UpdateConfig(
        runtime::internal::SignalComponentFactory::BuildTrackFilterConfig(runtime_config));
  }
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
