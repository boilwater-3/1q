#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

#include <utility>

#include "common/logging/ProjectLog.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"
#include "electronic_surveillance_radar/pipeline/PipelineRuntimeSnapshot.h"

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

/**
 * @brief 从 EsrInternalExecutionConfig 的 intercept.cluster 构建特征尺度。
 */
ObservationFeatureScales BuildFeatureScales(const InterceptClusterConfig& cluster) {
  ObservationFeatureScales scales;
  scales.rf_scale_hz = cluster.rf_scale_hz;
  scales.pw_scale_sec = cluster.pw_scale_sec;
  scales.az_scale_deg = cluster.az_scale_deg;
  scales.el_scale_deg = cluster.el_scale_deg;
  scales.snr_scale_db = cluster.snr_scale_db;
  return scales;
}

/**
 * @brief 从 EsrInternalExecutionConfig 的 runtime.track 构建关联配置。
 */
extension::InterceptAssociationConfig BuildAssociationConfig(const RuntimeTrackConfig& track) {
  extension::InterceptAssociationConfig assoc;
  assoc.gate_distance = track.gate_distance;
  assoc.confirm_hits = track.confirm_hits;
  assoc.max_missed_cycles = track.max_missed_cycles;
  assoc.confidence_alpha = track.confidence_alpha;
  assoc.output_tentative = track.output_tentative;
  return assoc;
}

}  // namespace

InterceptPipeline::InterceptPipeline(EsrInternalExecutionConfig config)
    : config_(std::move(config)),
      associator_(BuildAssociationConfig(config_.runtime.track)),
      rng_(config_.intercept.algorithm.random_seed == 0U
               ? 1U
               : config_.intercept.algorithm.random_seed) {
  feature_scales_ = BuildFeatureScales(config_.intercept.cluster);
}

void InterceptPipeline::UpdateConfig(const EsrInternalExecutionConfig& config) {
  config_ = config;
  // 重建依赖 config_ 的派生状态（feature_scales、associator）。
  feature_scales_ = BuildFeatureScales(config_.intercept.cluster);
  associator_.UpdateConfig(BuildAssociationConfig(config_.runtime.track));
}

extension::InterceptPipelineResult InterceptPipeline::RunCycle(
    const session::EsrCycleInput& input_state,
    const environment::IEsrEnvironmentService& environment_service) {
  extension::InterceptPipelineResult result;
  if (!config_.mission.power_on) {
    PROJECT_LOG_DEBUG("[InterceptPipeline] sensor disabled, cycle_index={} skipped.",
                      input_state.cycle_index);
    return result;
  }

  const session::EsrEnvironmentSnapshot environment_snapshot =
      environment_service.SampleEnvironment();

  MutableEsrContext ctx;
  const extension::InterceptPipelineConfig pipeline_config =
      BuildPipelineConfig(config_);
  const extension::InterceptRuntimeConfig runtime_config =
      BuildRuntimeConfig(config_);
  ctx.BeginCycle(input_state, environment_snapshot, pipeline_config, runtime_config);

  const InterceptDetectionOutput detection_output =
      detection_executor_.Execute(ctx, rng_, next_observation_id_);

  result = post_processing_executor_.Execute(detection_output.raw_records, ctx, preprocessor_,
                                             clusterer_, associator_, feature_scales_,
                                             next_hypothesis_id_);

  PROJECT_LOG_INFO("[InterceptPipeline] cycle_index={} raw_records={} clusters={} hypotheses={}",
                   input_state.cycle_index, detection_output.raw_records.size(),
                   result.observation_output.cluster_count,
                   result.emitter_output.hypotheses.size());

  return result;
}

extension::InterceptPipelineRuntimeState InterceptPipeline::CaptureRuntimeState() const {
  auto snapshot = std::make_shared<PipelineRuntimeSnapshot>();
  snapshot->rng = rng_;
  snapshot->next_observation_id = next_observation_id_;
  snapshot->next_hypothesis_id = next_hypothesis_id_;
  snapshot->tracks = associator_.CaptureTracks();

  extension::InterceptPipelineRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 1U;
  CapturePipelineSnapshot(state, snapshot);
  return state;
}

bool InterceptPipeline::RestoreRuntimeState(
    const extension::InterceptPipelineRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 1U) {
    return false;
  }
  const auto* snapshot = RestorePipelineSnapshot(state);
  if (snapshot == nullptr) {
    return false;
  }
  rng_ = snapshot->rng;
  next_observation_id_ = snapshot->next_observation_id;
  next_hypothesis_id_ = snapshot->next_hypothesis_id;
  associator_.RestoreTracks(snapshot->tracks);
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
