#include "airborne_radar/signal/pipeline/CycleExecutor.h"

#include <vector>

#include "airborne_radar/signal/pipeline/AssociationExecutionSupport.h"
#include "airborne_radar/signal/pipeline/ContextBindingSupport.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/OutputAssemblySupport.h"
#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/runtime/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/runtime/ScanScheduleResolver.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

bool HasValidRuntime(const CycleExecutionRuntime& runtime) {
  return runtime.base_config != nullptr && runtime.base_internal_config != nullptr &&
         runtime.control_profile != nullptr &&
         runtime.association_engine != nullptr && runtime.track_filter != nullptr &&
         runtime.output_manager != nullptr;
}

void BindContextAndResolveConfig(const common::model::TargetFeatureList& input_state,
                                 const environment::IEnvironmentService& environment,
                                 std::uint32_t cycle_index, const CycleExecutionRuntime& runtime,
                                 CycleExecutionContext* ctx) {
  ctx->input_state = &input_state;
  ctx->environment = &environment;
  const runtime::internal::ResolvedRuntimeSignalPipelineConfig resolved =
      runtime::internal::BuildRuntimeConfigFromControlProfile(
          *runtime.base_config, *runtime.base_internal_config, *runtime.control_profile);
  ctx->runtime_config = resolved.public_config;
  ctx->internal_runtime_config = resolved.internal_config;
  runtime::internal::ApplyScanScheduleToRuntimeConfig(cycle_index, &ctx->runtime_config);
}

void InitializeWorkspace(const CycleExecutionRuntime& runtime, CycleExecutionContext* ctx) {
  CycleWorkspace cycle_workspace = BuildCycleWorkspaceBindings(
      &ctx->output_state, &ctx->decision_frame,
      &ctx->association_quality_metrics, &ctx->track_measurements,
      &ctx->signal_term_db, &ctx->speed_penalty_db,
      &ctx->detection_margin_db, &ctx->detection_succeeded,
      &ctx->association_keys, &ctx->measurement_slots,
      &ctx->target_geometry, &ctx->measurement_covariances,
      &ctx->association_result);
  ResetCycleWorkspace(*ctx->input_state, ctx->runtime_config, &cycle_workspace);
  SyncAssociationAndTrackFilterConfigs(ctx->runtime_config, ctx->internal_runtime_config,
                                       runtime.association_engine, runtime.track_filter);
}

void PrepareAssociationSeeds(const CycleExecutionRuntime& runtime, CycleExecutionContext*) {
  const std::vector<tracking::AssociationTrackSeed> kEmptyAssociationSeeds;
  const std::vector<tracking::AssociationTrackSeed>& association_seeds =
      runtime.manual_association_seeds != nullptr ? *runtime.manual_association_seeds
                                                  : kEmptyAssociationSeeds;
  PrepareAssociationSeedsForCycle(runtime.has_manual_association_seeds, association_seeds,
                                  runtime.auto_lifecycle_manager, runtime.association_engine);
}

void SampleEnvironmentAndResolveJamming(const CycleExecutionRuntime& runtime,
                                        CycleExecutionContext* ctx) {
  ctx->environment_snapshot = ctx->environment->SampleEnvironment();
  ApplyEnvironmentJammingFactsToRuntimeConfig(*runtime.control_profile,
                                              ctx->environment_snapshot,
                                              &ctx->internal_runtime_config, &ctx->runtime_config);
  RefreshMeasurementCovariances(ctx->target_geometry.size(),
                                ctx->runtime_config.tracking.kalman_measurement_noise_std,
                                &ctx->measurement_covariances);
  SyncAssociationAndTrackFilterConfigs(ctx->runtime_config, ctx->internal_runtime_config,
                                       runtime.association_engine, runtime.track_filter);
  ctx->dominant_jamming_semantic =
      ResolveDominantJammingSemantic(*runtime.control_profile, ctx->environment_snapshot);
  ctx->jamming_severity =
      ComputeTrackLevelJammingSeverity(*runtime.control_profile, ctx->environment_snapshot);
}

void RunDetectionPhase(const CycleExecutionRuntime& runtime, CycleExecutionContext* ctx) {
  DetectionExecutionBuffers detection_buffers = BuildDetectionExecutionBuffers(
      &ctx->target_geometry, &ctx->signal_term_db, &ctx->speed_penalty_db,
      &ctx->detection_margin_db, &ctx->detection_succeeded, &ctx->measurement_covariances);
  if (ctx->runtime_config.detection.enable_physics_detection &&
      runtime.signal_detector != nullptr) {
    RunPhysicalDetectionPass(*ctx->input_state, ctx->runtime_config, ctx->internal_runtime_config,
                             *runtime.control_profile, ctx->environment_snapshot,
                             runtime.signal_detector, &detection_buffers);
  } else {
    RunHeuristicDetectionPass(*ctx->input_state, ctx->runtime_config, ctx->internal_runtime_config,
                              *runtime.control_profile, ctx->environment_snapshot,
                              &detection_buffers);
  }
}

void RunAssociationPhase(const CycleExecutionRuntime& runtime, CycleExecutionContext* ctx) {
  RunAssociationPass(*ctx->input_state, ctx->detection_succeeded, ctx->measurement_covariances,
                     ctx->environment_snapshot.cycle_dt_sec, runtime.association_engine,
                     &ctx->association_result, &ctx->association_keys);
}

void BuildTrackMeasurementsPhase(CycleExecutionContext* ctx) {
  TrackMeasurementBuildContext build_context = BuildTrackMeasurementBuildContextBindings(
      ctx->input_state, &ctx->association_result, &ctx->detection_succeeded,
      &ctx->association_keys, &ctx->detection_margin_db, &ctx->target_geometry,
      &ctx->measurement_covariances, ctx->environment_snapshot.jamming_detected,
      ctx->dominant_jamming_semantic, ctx->jamming_severity, &ctx->measurement_slots,
      &ctx->track_measurements);
  BuildTrackMeasurementsPass(build_context);
}

void ApplyTrackFilterPhase(const CycleExecutionRuntime& runtime, CycleExecutionContext* ctx) {
  TrackFilterApplyContext filter_context = BuildTrackFilterApplyContextBindings(
      ctx->input_state, &ctx->output_state, &ctx->detection_succeeded, &ctx->detection_margin_db,
      ctx->environment_snapshot.jamming_detected, ctx->dominant_jamming_semantic,
      ctx->jamming_severity, runtime.track_filter, &ctx->measurement_slots,
      &ctx->track_measurements);
  ApplyTrackFilterPass(filter_context);
}

void AssembleOutputs(std::uint32_t cycle_index, std::uint64_t batch_id,
                     const CycleExecutionRuntime& runtime, CycleExecutionContext* ctx) {
  CollectCycleOutputs(*runtime.control_profile, cycle_index, batch_id,
                      ctx->internal_runtime_config, ctx->environment_snapshot, *ctx->input_state,
                      ctx->output_state, ctx->association_result, ctx->track_measurements,
                      runtime.output_manager, runtime.auto_lifecycle_manager,
                      &ctx->association_quality_metrics, &ctx->decision_frame);
}

}  // namespace

void ExecuteCycle(const common::model::TargetFeatureList& input_state,
                  const environment::IEnvironmentService& environment,
                  std::uint32_t cycle_index, std::uint64_t batch_id,
                  const CycleExecutionRuntime& runtime, CycleExecutionContext* cycle_context) {
  if (cycle_context == nullptr) {
    PROJECT_LOG_ERROR("[CycleExecutor] ExecuteCycle called with null cycle_context.");
    return;
  }
  if (!HasValidRuntime(runtime)) {
    PROJECT_LOG_ERROR(
        "[CycleExecutor] ExecuteCycle received invalid runtime: base_config={} "
        "base_internal_config={} control_profile={} association_engine={} track_filter={} "
        "output_manager={}",
        runtime.base_config != nullptr, runtime.base_internal_config != nullptr,
        runtime.control_profile != nullptr, runtime.association_engine != nullptr,
        runtime.track_filter != nullptr, runtime.output_manager != nullptr);
    return;
  }
  BindContextAndResolveConfig(input_state, environment, cycle_index, runtime, cycle_context);
  InitializeWorkspace(runtime, cycle_context);
  PrepareAssociationSeeds(runtime, cycle_context);
  SampleEnvironmentAndResolveJamming(runtime, cycle_context);
  RunDetectionPhase(runtime, cycle_context);
  RunAssociationPhase(runtime, cycle_context);
  BuildTrackMeasurementsPhase(cycle_context);
  ApplyTrackFilterPhase(runtime, cycle_context);
  AssembleOutputs(cycle_index, batch_id, runtime, cycle_context);
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
