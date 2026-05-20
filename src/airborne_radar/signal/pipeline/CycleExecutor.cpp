#include "airborne_radar/signal/pipeline/CycleExecutor.h"

#include <algorithm>
#include <vector>

#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"
#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

// ---------------------------------------------------------------------------
// 关联种子
// ---------------------------------------------------------------------------

void PrepareAssociationSeeds(const CycleExecutionRuntime& runtime) {
  if (runtime.has_manual_association_seeds) {
    runtime.association_engine.SetAssociationSeeds(runtime.manual_association_seeds);
    return;
  }
  runtime.association_engine.SetAssociationSeeds(
      runtime.auto_lifecycle_manager.BuildAssociationSeeds());
}

// ---------------------------------------------------------------------------
// 环境阶段：写入 scratch.dominant_jamming_semantic / jamming_severity
// ---------------------------------------------------------------------------

bool RunEnvironmentPhase(CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                         CycleExecutionScratch& scratch) {
  ApplyEnvironmentJammingFactsToRuntimeConfig(
      runtime.control_profile, context.environment_snapshot, &context.runtime_config);
  RefreshMeasurementCovariances(
      scratch.target_geometry.size(),
      context.runtime_config.tracking.engineering.kalman_measurement_noise_std,
      &scratch.measurement_covariances);
  if (!SyncAssociationAndTrackFilterConfigs(context.runtime_config, &runtime.association_engine,
                                            &runtime.track_filter,
                                            &runtime.auto_lifecycle_manager)) {
    PROJECT_LOG_ERROR(
        "[SignalPipeline] environment phase aborted because runtime config sync failed.");
    return false;
  }

  scratch.dominant_jamming_semantic =
      ResolveDominantJammingSemantic(runtime.control_profile, context.environment_snapshot);
  scratch.jamming_severity =
      ComputeTrackLevelJammingSeverity(runtime.control_profile, context.environment_snapshot);
  return true;
}

// ---------------------------------------------------------------------------
// 检测阶段：写入 scratch 的 detection_* / target_geometry / measurement_covariances
// ---------------------------------------------------------------------------

void RunDetectionPhase(const CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                       CycleExecutionScratch& scratch) {
  DetectionExecutionBuffers detection_buffers;
  detection_buffers.target_geometry = &scratch.target_geometry;
  detection_buffers.signal_term_db = &scratch.signal_term_db;
  detection_buffers.speed_penalty_db = &scratch.speed_penalty_db;
  detection_buffers.detection_margin_db = &scratch.detection_margin_db;
  detection_buffers.detection_succeeded = &scratch.detection_succeeded;
  detection_buffers.measurement_covariances = &scratch.measurement_covariances;

  if (context.runtime_config.detection.engineering.enable_physics_detection &&
      runtime.signal_detector != nullptr) {
    RunPhysicalDetectionPass(context.input_state, context.runtime_config, runtime.control_profile,
                             context.environment_snapshot, context.platform_altitude_m,
                             runtime.signal_detector, &detection_buffers);
  } else {
    RunHeuristicDetectionPass(context.input_state, context.runtime_config,
                              runtime.control_profile, context.environment_snapshot,
                              &detection_buffers);
  }
}

// ---------------------------------------------------------------------------
// 关联阶段：写入 scratch.association_result / association_keys
// ---------------------------------------------------------------------------

void RunAssociationPhase(const CycleExecutionContext& context,
                         const CycleExecutionRuntime& runtime, CycleExecutionScratch& scratch) {
  scratch.association_result = runtime.association_engine.AssociateDetections(
      context.input_state, scratch.detection_succeeded, scratch.measurement_covariances,
      context.environment_snapshot.cycle_dt_sec);
  scratch.association_keys = scratch.association_result.target_keys;
}

// ---------------------------------------------------------------------------
// 量测构建阶段：写入 scratch.measurement_slots / track_measurements
// ---------------------------------------------------------------------------

void RunMeasurementBuildPhase(const CycleExecutionContext& context,
                              CycleExecutionScratch& scratch) {
  BuildTrackMeasurementsPass(context.input_state, context.environment_snapshot.jamming_detected,
                             scratch);
}

// ---------------------------------------------------------------------------
// 滤波阶段：写入 scratch.output_state / track_measurements.filtered_feature
// ---------------------------------------------------------------------------

void RunTrackFilterPhase(const CycleExecutionContext& context,
                         const CycleExecutionRuntime& runtime, CycleExecutionScratch& scratch) {
  ApplyTrackFilterPass(context.input_state, context.environment_snapshot.jamming_detected,
                       runtime.track_filter, scratch);
}

// ---------------------------------------------------------------------------
// 输出收尾
// ---------------------------------------------------------------------------

namespace {

float ResolveAssociationFragilityWeight(model::JammingSemantic semantic) {
  switch (semantic) {
    case model::JammingSemantic::kDeception:
      return 1.00f;
    case model::JammingSemantic::kRepeater:
      return 0.88f;
    case model::JammingSemantic::kMixed:
      return 0.94f;
    case model::JammingSemantic::kNoiseSuppression:
      return 0.60f;
    case model::JammingSemantic::kNone:
    default:
      return 0.0f;
  }
}

AssociationQualityMetrics ToPipelineAssociationQualityMetrics(
    const association::AssociationQualityMetrics& source,
    model::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    float association_unassigned_cost) {
  AssociationQualityMetrics metrics;
  metrics.prior_track_count = source.prior_track_count;
  metrics.detection_count = source.detection_count;
  metrics.matched_count = source.matched_count;
  metrics.new_track_count = source.new_track_count;
  metrics.missed_track_count = source.missed_track_count;
  metrics.match_rate = source.match_rate;
  metrics.new_track_rate = source.new_track_rate;
  metrics.missed_track_rate = source.missed_track_rate;
  metrics.mean_match_cost = source.mean_match_cost;
  metrics.p95_match_cost = source.p95_match_cost;
  metrics.dominant_jamming_semantic = dominant_jamming_semantic;
  metrics.jamming_severity = std::max(0.0f, std::min(1.0f, jamming_severity));
  const float normalized_cost_pressure =
      association_unassigned_cost > 1e-6f
          ? std::max(0.0f, std::min(1.0f, source.mean_match_cost / association_unassigned_cost))
          : 0.0f;
  const float operational_pressure =
      0.20f + 0.30f * std::max(0.0f, std::min(1.0f, 1.0f - source.match_rate)) +
      0.20f * source.new_track_rate + 0.15f * source.missed_track_rate +
      0.15f * normalized_cost_pressure;
  metrics.association_stress = std::max(
      0.0f,
      std::min(1.0f, metrics.jamming_severity *
                         ResolveAssociationFragilityWeight(metrics.dominant_jamming_semantic) *
                         operational_pressure));
  return metrics;
}

std::uint32_t ResolveLifecycleExtraMissTolerance(
    const extension::control::RadarControlProfile& control_profile) {
  std::uint32_t extra_miss_tolerance = 0U;
  if (control_profile.enable_sidelobe_canceller || control_profile.enable_agility_frequency ||
      control_profile.enable_eccm_rejitter) {
    extra_miss_tolerance += 1U;
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    extra_miss_tolerance += 1U;
  }
  return extra_miss_tolerance;
}

}  // namespace

void CollectCycleOutputs(const extension::control::RadarControlProfile& control_profile,
                         std::uint32_t cycle_index, std::uint64_t batch_id,
                         const ExecutionConfig& runtime_config,
                         const environment::EnvironmentSnapshot& environment_snapshot,
                         const session::RadarSceneTargetList& input_state,
                         tracking::ITrackLifecycleManager* auto_lifecycle_manager,
                         CycleExecutionScratch& scratch) {
  if (auto_lifecycle_manager == nullptr) {
    PROJECT_LOG_ERROR(
        "[CycleExecutor] auto_lifecycle_manager is null; decision frame assembly aborted.");
    return;
  }

  const model::JammingSemantic dominant_jamming_semantic =
      ResolveDominantJammingSemantic(control_profile, environment_snapshot);
  const float jamming_severity =
      ComputeTrackLevelJammingSeverity(control_profile, environment_snapshot);
  scratch.association_quality_metrics = ToPipelineAssociationQualityMetrics(
      scratch.association_result.quality_metrics, dominant_jamming_semantic, jamming_severity,
      runtime_config.association.unassigned_cost);

  const model::EccmSourceInfo eccm_source_info = BuildEccmSourceInfo(environment_snapshot);
  const model::AssociationQualityInfo association_quality_info =
      BuildAssociationQualityInfo(scratch.association_quality_metrics);
  const model::PerceptionQualityInfo perception_quality_info =
      BuildPerceptionQualityInfo(input_state.size(), scratch.association_quality_metrics);

  tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = environment_snapshot.cycle_dt_sec;
  cycle.extra_miss_tolerance = ResolveLifecycleExtraMissTolerance(control_profile);
  auto_lifecycle_manager->Update(cycle, scratch.track_measurements);
  scratch.decision_frame =
      model::DecisionInputFrame(auto_lifecycle_manager->BuildTrackStateSnapshots());
  scratch.decision_frame.cycle_index = cycle_index;
  scratch.decision_frame.batch_id = batch_id;
  scratch.decision_frame.environment_jamming_detected = eccm_source_info.has_jamming_signal;
  scratch.decision_frame.eccm_source_info = eccm_source_info;
  scratch.decision_frame.association_quality_info = association_quality_info;
  scratch.decision_frame.perception_quality_info = perception_quality_info;
}

void AssembleOutputs(std::uint32_t cycle_index, std::uint64_t batch_id,
                     const CycleExecutionContext& context, const CycleExecutionRuntime& runtime,
                     CycleExecutionScratch& scratch) {
  CollectCycleOutputs(runtime.control_profile, cycle_index, batch_id, context.runtime_config,
                      context.environment_snapshot, context.input_state,
                      &runtime.auto_lifecycle_manager, scratch);
}

}  // namespace

// ---------------------------------------------------------------------------
// 顶层入口
// ---------------------------------------------------------------------------

bool ExecuteCycle(CycleExecutionContext& context,
                  const CycleExecutionRuntime& runtime, CycleExecutionScratch& cycle_scratch) {
  ResetCycleExecutionScratch(context.input_state, cycle_scratch);

  if (!RunEnvironmentPhase(context, runtime, cycle_scratch)) {
    return false;
  }
  PrepareAssociationSeeds(runtime);
  RunDetectionPhase(context, runtime, cycle_scratch);
  RunAssociationPhase(context, runtime, cycle_scratch);
  RunMeasurementBuildPhase(context, cycle_scratch);
  RunTrackFilterPhase(context, runtime, cycle_scratch);
  AssembleOutputs(context.cycle_index, context.batch_id, context, runtime, cycle_scratch);
  return true;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
