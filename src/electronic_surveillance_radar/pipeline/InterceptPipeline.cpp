#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

#include <utility>

#include "1q/electronic_surveillance_radar/extension/IEsrContext.h"
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

void InterceptPipeline::UpdateConfig(extension::InterceptPipelineConfig config) {
  config_.detection.min_detect_snr_db = config.detection.min_detect_snr_db;
  config_.detection.pfa = config.statistical_detection.pfa;
  config_.detection.pulse_count = config.statistical_detection.pulse_count;
  config_.detection.threshold_scale = config.statistical_detection.threshold_scale;
  config_.detection.enable_statistical_detection =
      config.statistical_detection.enable_statistical_detection;

  config_.intercept.detection.max_detect_range_m = config.detection.max_detect_range_m;
  config_.intercept.detection.min_dynamic_range_margin_db = config.detection.min_dynamic_range_margin_db;
  config_.intercept.detection.boundary_resolution_m = config.detection.boundary_resolution_m;
  config_.intercept.detection.boundary_max_iterations = config.detection.boundary_max_iterations;

  config_.intercept.algorithm.random_seed = config.algorithm.random_seed;
  config_.intercept.algorithm.angle_error_coefficient = config.algorithm.angle_error_coefficient;

  config_.intercept.preprocess = InterceptPreprocessConfig();
  config_.intercept.preprocess.dedup_time_window_sec = config.preprocess.dedup_time_window_sec;
  config_.intercept.preprocess.dedup_rf_window_hz = config.preprocess.dedup_rf_window_hz;
  config_.intercept.preprocess.dedup_pw_window_sec = config.preprocess.dedup_pw_window_sec;
  config_.intercept.preprocess.dedup_az_window_deg = config.preprocess.dedup_az_window_deg;
  config_.intercept.preprocess.dedup_el_window_deg = config.preprocess.dedup_el_window_deg;
  config_.intercept.preprocess.normalize_quality = config.preprocess.normalize_quality;

  config_.intercept.cluster.radius = config.cluster.radius;
  config_.intercept.cluster.min_points = config.cluster.min_points;
  config_.intercept.cluster.rf_scale_hz = config.cluster.rf_scale_hz;
  config_.intercept.cluster.pw_scale_sec = config.cluster.pw_scale_sec;
  config_.intercept.cluster.az_scale_deg = config.cluster.az_scale_deg;
  config_.intercept.cluster.el_scale_deg = config.cluster.el_scale_deg;
  config_.intercept.cluster.snr_scale_db = config.cluster.snr_scale_db;

  config_.intercept.spectral_analysis.enable = config.spectral_analysis.enable;
  config_.intercept.spectral_analysis.min_sequence_length =
      config.spectral_analysis.min_sequence_length;
  config_.intercept.spectral_analysis.fft_length = config.spectral_analysis.fft_length;
  config_.intercept.spectral_analysis.broadband_occupancy_threshold =
      config.spectral_analysis.broadband_occupancy_threshold;
  config_.intercept.spectral_analysis.agile_stability_threshold_hz =
      config.spectral_analysis.agile_stability_threshold_hz;
  config_.intercept.spectral_analysis.agile_peak_sparsity_threshold =
      config.spectral_analysis.agile_peak_sparsity_threshold;
  config_.intercept.spectral_analysis.occupancy_peak_floor_ratio =
      config.spectral_analysis.occupancy_peak_floor_ratio;

  config_.intercept.suppression.suppression_noise_scale =
      config.suppression_model.suppression_noise_scale;
  config_.intercept.suppression.suppression_mark_threshold_w =
      config.suppression_model.suppression_mark_threshold_w;

  config_.intercept.deception.false_alarm_probability_scale =
      config.deception_model.false_alarm_probability_scale;
  config_.intercept.deception.confusion_probability_scale =
      config.deception_model.confusion_probability_scale;
  config_.intercept.deception.max_false_observations_per_emitter =
      config.deception_model.max_false_observations_per_emitter;
  config_.intercept.deception.aoa_confusion_std_deg =
      config.deception_model.aoa_confusion_std_deg;
  config_.intercept.deception.rf_confusion_ratio = config.deception_model.rf_confusion_ratio;
  config_.intercept.deception.pw_confusion_ratio = config.deception_model.pw_confusion_ratio;
  config_.intercept.deception.cluster_confidence_penalty_scale =
      config.deception_model.cluster_confidence_penalty_scale;

  config_.runtime.track.gate_distance = config.association.gate_distance;
  config_.runtime.track.confirm_hits = config.association.confirm_hits;
  config_.runtime.track.max_missed_cycles = config.association.max_missed_cycles;
  config_.runtime.track.confidence_alpha = config.association.confidence_alpha;
  config_.runtime.track.output_tentative = config.association.output_tentative;

  config_.runtime.integrator.integration_mode = config.statistical_detection.integration_mode;

  config_.resolved_scan = std::move(config.scan);

  feature_scales_ = BuildFeatureScales(config_.intercept.cluster);
  associator_.UpdateConfig(BuildAssociationConfig(config_.runtime.track));
}

void InterceptPipeline::UpdateRuntimeConfig(extension::InterceptRuntimeConfig runtime_config) {
  config_.mission.power_on = runtime_config.sensor_enabled;
  config_.mission.scan.scan_rate_hz = runtime_config.scan_rate_hz;
  config_.hardware.antenna_mount_az_deg = runtime_config.antenna_mount_az_deg;
  config_.hardware.antenna_mount_el_deg = runtime_config.antenna_mount_el_deg;
  config_.hardware.integrated_receive_loss_db = runtime_config.integrated_receive_loss_db;
  if (runtime_config.use_fixed_receiver_window) {
    config_.hardware.receiver_band_lower_hz = runtime_config.receiver_lower_hz;
    config_.hardware.receiver_band_upper_hz = runtime_config.receiver_upper_hz;
  }
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

  const environment::EsrEnvironmentSnapshot environment_snapshot =
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
