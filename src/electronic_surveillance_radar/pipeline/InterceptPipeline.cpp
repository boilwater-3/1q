#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

#include <utility>

#include "1q/electronic_surveillance_radar/extension/IEsrContext.h"
#include "common/logging/ProjectLog.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"
#include "electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h"

namespace electronic_surveillance_radar {
namespace pipeline {

InterceptPipeline::InterceptPipeline(extension::InterceptPipelineConfig config,
                                     extension::InterceptRuntimeConfig runtime_config)
    : config_(std::move(config)),
      runtime_config_(std::move(runtime_config)),
      associator_(config_.association),
      rng_(config_.algorithm.random_seed == 0U ? 1U : config_.algorithm.random_seed) {
  feature_scales_.rf_scale_hz = config_.cluster.rf_scale_hz;
  feature_scales_.pw_scale_sec = config_.cluster.pw_scale_sec;
  feature_scales_.az_scale_deg = config_.cluster.az_scale_deg;
  feature_scales_.el_scale_deg = config_.cluster.el_scale_deg;
  feature_scales_.snr_scale_db = config_.cluster.snr_scale_db;
}

void InterceptPipeline::UpdateConfig(extension::InterceptPipelineConfig config) {
  config_ = std::move(config);
  feature_scales_.rf_scale_hz = config_.cluster.rf_scale_hz;
  feature_scales_.pw_scale_sec = config_.cluster.pw_scale_sec;
  feature_scales_.az_scale_deg = config_.cluster.az_scale_deg;
  feature_scales_.el_scale_deg = config_.cluster.el_scale_deg;
  feature_scales_.snr_scale_db = config_.cluster.snr_scale_db;
  associator_.UpdateConfig(config_.association);
}

void InterceptPipeline::UpdateRuntimeConfig(extension::InterceptRuntimeConfig runtime_config) {
  runtime_config_ = std::move(runtime_config);
}

extension::InterceptPipelineResult InterceptPipeline::RunCycle(
    const session::EsrCycleInput& input_state,
    const environment::IEsrEnvironmentService& environment_service) {
  extension::InterceptPipelineResult result;
  if (!runtime_config_.sensor_enabled) {
    PROJECT_LOG_DEBUG("[InterceptPipeline] sensor disabled, cycle_index={} skipped.",
                      input_state.cycle_index);
    return result;
  }

  const environment::EsrEnvironmentSnapshot environment_snapshot =
      environment_service.SampleEnvironment();

  MutableEsrContext ctx;
  ctx.BeginCycle(input_state, environment_snapshot, config_, runtime_config_);

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

namespace {

struct PipelineRuntimeSnapshot {
  std::mt19937 rng;
  std::uint64_t next_observation_id{1U};
  std::uint64_t next_hypothesis_id{1U};
  std::vector<HypothesisAssociator::TrackState> tracks;
};

}  // namespace

extension::InterceptPipelineRuntimeState InterceptPipeline::CaptureRuntimeState() const {
  auto snapshot = std::make_shared<PipelineRuntimeSnapshot>();
  snapshot->rng = rng_;
  snapshot->next_observation_id = next_observation_id_;
  snapshot->next_hypothesis_id = next_hypothesis_id_;
  snapshot->tracks = associator_.CaptureTracks();

  extension::InterceptPipelineRuntimeState state;
  state.owner_identity = this;
  state.schema_version = 1U;
  state.snapshot = snapshot;
  return state;
}

bool InterceptPipeline::RestoreRuntimeState(
    const extension::InterceptPipelineRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != 1U) {
    return false;
  }
  if (state.snapshot == nullptr) {
    return false;
  }
  const auto* snapshot = static_cast<const PipelineRuntimeSnapshot*>(state.snapshot.get());
  rng_ = snapshot->rng;
  next_observation_id_ = snapshot->next_observation_id;
  next_hypothesis_id_ = snapshot->next_hypothesis_id;
  associator_.RestoreTracks(snapshot->tracks);
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
