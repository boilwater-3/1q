/**
 * @file CycleExecutor.h
 * @brief 定义 SignalPipeline 单周期执行器。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/config/InternalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

struct CycleExecutionScratch {
  model::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;
  model::DecisionInputFrame decision_frame{};
  AssociationQualityMetrics association_quality_metrics{};

  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
  association::AssociationResult association_result;
  std::vector<std::uint64_t> association_keys;
  std::vector<tracking::MeasurementCovariance> measurement_covariances;
  std::vector<int> measurement_slots;
  std::vector<detection::ResolvedTargetGeometry> target_geometry;
};

struct CycleExecutionRuntime {
  CycleExecutionRuntime(const ExecutionConfig& base_config,
                        const extension::control::RadarControlProfile& control_profile,
                        association::DataAssociationEngine& association_engine,
                        tracking::TrackFilter& track_filter,
                        tracking::ITrackLifecycleManager& auto_lifecycle_manager,
                        detection::SignalDetector* signal_detector,
                        const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds,
                        bool has_manual_association_seeds)
      : base_config(base_config),
        control_profile(control_profile),
        association_engine(association_engine),
        track_filter(track_filter),
        signal_detector(signal_detector),
        auto_lifecycle_manager(auto_lifecycle_manager),
        manual_association_seeds(manual_association_seeds),
        has_manual_association_seeds(has_manual_association_seeds) {}

  const ExecutionConfig& base_config;
  const extension::control::RadarControlProfile& control_profile;
  association::DataAssociationEngine& association_engine;
  tracking::TrackFilter& track_filter;
  detection::SignalDetector* signal_detector;
  tracking::ITrackLifecycleManager& auto_lifecycle_manager;
  const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds;
  bool has_manual_association_seeds{false};
};

struct CycleExecutionContract {
  CycleExecutionContract(const model::TargetFeatureList& input_state,
                         const environment::EnvironmentSnapshot& environment_snapshot,
                         std::uint32_t cycle_index, std::uint64_t batch_id,
                         ExecutionConfig runtime_config)
      : input_state(input_state),
        environment_snapshot(environment_snapshot),
        cycle_index(cycle_index),
        batch_id(batch_id),
        runtime_config(std::move(runtime_config)) {}

  const model::TargetFeatureList& input_state;
  const environment::EnvironmentSnapshot& environment_snapshot;
  std::uint32_t cycle_index{0U};
  std::uint64_t batch_id{0U};
  ExecutionConfig runtime_config{};
};

struct EnvironmentPhaseOutput {
  model::JammingSemantic dominant_jamming_semantic{model::JammingSemantic::kNone};
  float jamming_severity{0.0f};
};

struct DetectionPhaseOutput {
  DetectionPhaseOutput(const std::vector<std::uint8_t>& detection_succeeded,
                       const std::vector<float>& detection_margin_db,
                       const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
                       const std::vector<detection::ResolvedTargetGeometry>& target_geometry)
      : detection_succeeded(detection_succeeded),
        detection_margin_db(detection_margin_db),
        measurement_covariances(measurement_covariances),
        target_geometry(target_geometry) {}

  const std::vector<std::uint8_t>& detection_succeeded;
  const std::vector<float>& detection_margin_db;
  const std::vector<tracking::MeasurementCovariance>& measurement_covariances;
  const std::vector<detection::ResolvedTargetGeometry>& target_geometry;
};

struct AssociationPhaseOutput {
  AssociationPhaseOutput(const association::AssociationResult& association_result,
                         const std::vector<std::uint64_t>& association_keys)
      : association_result(association_result), association_keys(association_keys) {}

  const association::AssociationResult& association_result;
  const std::vector<std::uint64_t>& association_keys;
};

struct MeasurementBuildPhaseOutput {
  MeasurementBuildPhaseOutput(const std::vector<int>& measurement_slots,
                              const std::vector<tracking::TrackMeasurement>& track_measurements)
      : measurement_slots(measurement_slots), track_measurements(track_measurements) {}

  const std::vector<int>& measurement_slots;
  const std::vector<tracking::TrackMeasurement>& track_measurements;
};

bool ExecuteCycle(const model::TargetFeatureList& input_state,
                  const environment::EnvironmentSnapshot& environment_snapshot,
                  std::uint32_t cycle_index, std::uint64_t batch_id,
                  const CycleExecutionRuntime& runtime, CycleExecutionScratch& cycle_scratch);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_EXECUTOR_H_
