/**
 * @file CycleContextSupport.h
 * @brief 定义 SignalPipeline 周期缓存初始化与配置同步的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/config/InternalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

struct CycleWorkspace {
  model::TargetFeatureList* output_state{nullptr};
  model::DecisionInputFrame* decision_frame{nullptr};
  AssociationQualityMetrics* association_quality_metrics{nullptr};
  std::vector<tracking::TrackMeasurement>* track_measurements{nullptr};
  std::vector<float>* signal_term_db{nullptr};
  std::vector<float>* speed_penalty_db{nullptr};
  std::vector<float>* detection_margin_db{nullptr};
  std::vector<std::uint8_t>* detection_succeeded{nullptr};
  std::vector<std::uint64_t>* association_keys{nullptr};
  std::vector<int>* measurement_slots{nullptr};
  std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr};
  std::vector<tracking::MeasurementCovariance>* measurement_covariances{nullptr};
  association::AssociationResult* association_result{nullptr};
};

void ResetCycleWorkspace(const model::TargetFeatureList& input_state,
                         const ExecutionConfig& runtime_config, CycleWorkspace* workspace);

void RefreshMeasurementCovariances(
    std::size_t target_count, float kalman_measurement_noise_std,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances);

bool SyncAssociationAndTrackFilterConfigs(
    const ExecutionConfig& runtime_config, association::DataAssociationEngine* association_engine,
    tracking::TrackFilter* track_filter,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager = nullptr);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_
