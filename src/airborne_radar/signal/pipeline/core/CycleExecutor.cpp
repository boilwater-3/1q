#include "airborne_radar/signal/pipeline/core/CycleExecutor.h"

#include <algorithm>
#include <vector>

#include "airborne_radar/signal/pipeline/assembly/DecisionFrameBuilders.h"
#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/assembly/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/pipeline/core/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/core/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/core/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/effects/JammingEffects.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

CycleExecutionContract BuildCycleExecutionContract(
    const session::RadarSceneTargetList& input_state,
    const environment::EnvironmentSnapshot& environment_snapshot, std::uint32_t cycle_index,
    std::uint64_t batch_id, const CycleExecutionRuntime& runtime) {
  const assembly::internal::ResolvedRuntimePipelineConfig resolved =
      assembly::internal::ResolveRuntimePipelineConfig(runtime.base_config,
                                                       runtime.control_profile);
  ExecutionConfig runtime_config = resolved.config;
  core::internal::ApplyScanScheduleToRuntimeConfig(cycle_index, &runtime_config);
  CycleExecutionContract output(input_state, environment_snapshot, cycle_index, batch_id,
                                std::move(runtime_config));
  return output;
}

// ---------------------------------------------------------------------------
// 准备执行状态
// ---------------------------------------------------------------------------

bool PrepareExecutionState(const CycleExecutionContract& contract, CycleExecutionScratch* scratch) {
  CycleWorkspace cycle_workspace;
  cycle_workspace.output_state = &scratch->output_state;
  cycle_workspace.decision_frame = &scratch->decision_frame;
  cycle_workspace.association_quality_metrics = &scratch->association_quality_metrics;
  cycle_workspace.track_measurements = &scratch->track_measurements;
  cycle_workspace.signal_term_db = &scratch->signal_term_db;
  cycle_workspace.speed_penalty_db = &scratch->speed_penalty_db;
  cycle_workspace.detection_margin_db = &scratch->detection_margin_db;
  cycle_workspace.detection_succeeded = &scratch->detection_succeeded;
  cycle_workspace.association_keys = &scratch->association_keys;
  cycle_workspace.measurement_slots = &scratch->measurement_slots;
  cycle_workspace.target_geometry = &scratch->target_geometry;
  cycle_workspace.measurement_covariances = &scratch->measurement_covariances;
  cycle_workspace.association_result = &scratch->association_result;
  ResetCycleWorkspace(contract.input_state, contract.runtime_config, &cycle_workspace);
  return true;
}

// ---------------------------------------------------------------------------
// 准备关联种子
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
// 环境阶段
// ---------------------------------------------------------------------------

bool RunEnvironmentPhase(CycleExecutionContract* contract, const CycleExecutionRuntime& runtime,
                         CycleExecutionScratch* scratch, EnvironmentPhaseOutput* output) {
  ApplyEnvironmentJammingFactsToRuntimeConfig(
      runtime.control_profile, contract->environment_snapshot, &contract->runtime_config);
  RefreshMeasurementCovariances(
      scratch->target_geometry.size(),
      contract->runtime_config.tracking_engineering.kalman_measurement_noise_std,
      &scratch->measurement_covariances);
  if (!SyncAssociationAndTrackFilterConfigs(contract->runtime_config, &runtime.association_engine,
                                            &runtime.track_filter,
                                            &runtime.auto_lifecycle_manager)) {
    PROJECT_LOG_ERROR(
        "[SignalPipeline] environment phase aborted because runtime config sync failed.");
    return false;
  }

  output->dominant_jamming_semantic =
      ResolveDominantJammingSemantic(runtime.control_profile, contract->environment_snapshot);
  output->jamming_severity =
      ComputeTrackLevelJammingSeverity(runtime.control_profile, contract->environment_snapshot);
  return true;
}

// ---------------------------------------------------------------------------
// 检测阶段
// ---------------------------------------------------------------------------

DetectionPhaseOutput RunDetectionPhase(const CycleExecutionContract& contract,
                                       const CycleExecutionRuntime& runtime,
                                       const EnvironmentPhaseOutput&,
                                       CycleExecutionScratch* scratch) {
  DetectionExecutionBuffers detection_buffers;
  detection_buffers.target_geometry = &scratch->target_geometry;
  detection_buffers.signal_term_db = &scratch->signal_term_db;
  detection_buffers.speed_penalty_db = &scratch->speed_penalty_db;
  detection_buffers.detection_margin_db = &scratch->detection_margin_db;
  detection_buffers.detection_succeeded = &scratch->detection_succeeded;
  detection_buffers.measurement_covariances = &scratch->measurement_covariances;

  if (contract.runtime_config.detection_engineering.enable_physics_detection &&
      runtime.signal_detector != nullptr) {
    RunPhysicalDetectionPass(contract.input_state, contract.runtime_config, runtime.control_profile,
                             contract.environment_snapshot, runtime.signal_detector,
                             &detection_buffers);
  } else {
    RunHeuristicDetectionPass(contract.input_state, contract.runtime_config,
                              runtime.control_profile, contract.environment_snapshot,
                              &detection_buffers);
  }

  return DetectionPhaseOutput(scratch->detection_succeeded, scratch->detection_margin_db,
                              scratch->measurement_covariances, scratch->target_geometry);
}

// ---------------------------------------------------------------------------
// 关联阶段
// ---------------------------------------------------------------------------

AssociationPhaseOutput RunAssociationPhase(const CycleExecutionContract& contract,
                                           const CycleExecutionRuntime& runtime,
                                           const EnvironmentPhaseOutput&,
                                           const DetectionPhaseOutput& detection_phase,
                                           CycleExecutionScratch* scratch) {
  scratch->association_result = runtime.association_engine.AssociateDetections(
      contract.input_state, detection_phase.detection_succeeded,
      detection_phase.measurement_covariances,
      contract.environment_snapshot.cycle_dt_sec);
  scratch->association_keys = scratch->association_result.target_keys;

  return AssociationPhaseOutput(scratch->association_result, scratch->association_keys);
}

// ---------------------------------------------------------------------------
// 量测构建阶段
// ---------------------------------------------------------------------------

MeasurementBuildPhaseOutput RunMeasurementBuildPhase(
    const CycleExecutionContract& contract, const EnvironmentPhaseOutput& environment_phase,
    const DetectionPhaseOutput& detection_phase, const AssociationPhaseOutput& association_phase,
    CycleExecutionScratch* scratch) {
  TrackMeasurementBuildContext build_context(
      contract.input_state, association_phase.association_result,
      detection_phase.detection_succeeded, association_phase.association_keys,
      detection_phase.detection_margin_db, detection_phase.target_geometry,
      detection_phase.measurement_covariances, contract.environment_snapshot.jamming_detected,
      environment_phase.dominant_jamming_semantic, environment_phase.jamming_severity,
      scratch->measurement_slots, scratch->track_measurements);
  BuildTrackMeasurementsPass(build_context);

  return MeasurementBuildPhaseOutput(scratch->measurement_slots, scratch->track_measurements);
}

// ---------------------------------------------------------------------------
// 滤波阶段
// ---------------------------------------------------------------------------

void RunTrackFilterPhase(const CycleExecutionContract& contract,
                         const CycleExecutionRuntime& runtime,
                         const EnvironmentPhaseOutput& environment_phase,
                         const DetectionPhaseOutput& detection_phase,
                         const MeasurementBuildPhaseOutput& measurement_phase,
                         CycleExecutionScratch* scratch) {
  TrackFilterApplyContext filter_context(
      contract.input_state, scratch->output_state, detection_phase.detection_succeeded,
      detection_phase.detection_margin_db, contract.environment_snapshot.jamming_detected,
      environment_phase.dominant_jamming_semantic, environment_phase.jamming_severity,
      runtime.track_filter, measurement_phase.measurement_slots, scratch->track_measurements);
  ApplyTrackFilterPass(filter_context);
}

// ---------------------------------------------------------------------------
// 输出收尾（原 OutputAssemblySupport）
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
                         const association::AssociationResult& association_result,
                         const std::vector<tracking::TrackMeasurement>& track_measurements,
                         tracking::ITrackLifecycleManager* auto_lifecycle_manager,
                         AssociationQualityMetrics* association_quality_metrics,
                         model::DecisionInputFrame* decision_frame) {
  if (association_quality_metrics == nullptr || decision_frame == nullptr) {
    return;
  }

  const model::JammingSemantic dominant_jamming_semantic =
      ResolveDominantJammingSemantic(control_profile, environment_snapshot);
  const float jamming_severity =
      ComputeTrackLevelJammingSeverity(control_profile, environment_snapshot);
  *association_quality_metrics = ToPipelineAssociationQualityMetrics(
      association_result.quality_metrics, dominant_jamming_semantic, jamming_severity,
      runtime_config.association_unassigned_cost);

  const model::EccmSourceInfo eccm_source_info = BuildEccmSourceInfo(environment_snapshot);
  const model::AssociationQualityInfo association_quality_info =
      BuildAssociationQualityInfo(*association_quality_metrics);
  const model::PerceptionQualityInfo perception_quality_info =
      BuildPerceptionQualityInfo(input_state.size(), *association_quality_metrics);

  if (auto_lifecycle_manager == nullptr) {
    PROJECT_LOG_ERROR(
        "[CycleExecutor] auto_lifecycle_manager is null; decision frame assembly aborted.");
    return;
  }

  tracking::CycleContext cycle;
  cycle.cycle_index = cycle_index;
  cycle.batch_id = batch_id;
  cycle.dt_sec = environment_snapshot.cycle_dt_sec;
  cycle.extra_miss_tolerance = ResolveLifecycleExtraMissTolerance(control_profile);
  auto_lifecycle_manager->Update(cycle, track_measurements);
  *decision_frame = auto_lifecycle_manager->BuildDecisionFrame(cycle_index, batch_id,
                                                               eccm_source_info.has_jamming_signal);
  decision_frame->environment_jamming_detected = eccm_source_info.has_jamming_signal;
  decision_frame->eccm_source_info = eccm_source_info;
  decision_frame->association_quality_info = association_quality_info;
  decision_frame->perception_quality_info = perception_quality_info;
}

// ---------------------------------------------------------------------------
// 输出装配
// ---------------------------------------------------------------------------

void AssembleOutputs(std::uint32_t cycle_index, std::uint64_t batch_id,
                     const CycleExecutionContract& contract, const EnvironmentPhaseOutput&,
                     const CycleExecutionRuntime& runtime,
                     const AssociationPhaseOutput& association_phase,
                     const MeasurementBuildPhaseOutput& measurement_phase,
                     CycleExecutionScratch* scratch) {
  CollectCycleOutputs(runtime.control_profile, cycle_index, batch_id, contract.runtime_config,
                      contract.environment_snapshot, contract.input_state,
                      association_phase.association_result, measurement_phase.track_measurements,
                      &runtime.auto_lifecycle_manager, &scratch->association_quality_metrics,
                      &scratch->decision_frame);
}

}  // namespace

bool ExecuteCycle(const session::RadarSceneTargetList& input_state,
                  const environment::EnvironmentSnapshot& environment_snapshot,
                  std::uint32_t cycle_index, std::uint64_t batch_id,
                  const CycleExecutionRuntime& runtime, CycleExecutionScratch& cycle_scratch) {
  CycleExecutionContract contract = BuildCycleExecutionContract(input_state, environment_snapshot,
                                                                cycle_index, batch_id, runtime);
  if (!PrepareExecutionState(contract, &cycle_scratch)) {
    return false;
  }

  EnvironmentPhaseOutput environment_phase;
  if (!RunEnvironmentPhase(&contract, runtime, &cycle_scratch, &environment_phase)) {
    return false;
  }
  PrepareAssociationSeeds(runtime);
  const DetectionPhaseOutput detection_phase =
      RunDetectionPhase(contract, runtime, environment_phase, &cycle_scratch);
  const AssociationPhaseOutput association_phase =
      RunAssociationPhase(contract, runtime, environment_phase, detection_phase, &cycle_scratch);
  const MeasurementBuildPhaseOutput measurement_phase = RunMeasurementBuildPhase(
      contract, environment_phase, detection_phase, association_phase, &cycle_scratch);
  RunTrackFilterPhase(contract, runtime, environment_phase, detection_phase, measurement_phase,
                      &cycle_scratch);
  AssembleOutputs(contract.cycle_index, contract.batch_id, contract, environment_phase, runtime,
                  association_phase, measurement_phase, &cycle_scratch);
  return true;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
