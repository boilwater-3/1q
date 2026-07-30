#include "airborne_radar/signal/pipeline/SignalPipeline.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/ScanScheduleResolver.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

void ResetCycleScratch(CycleExecutionScratch* scratch) {
  if (scratch == nullptr) {
    return;
  }
  *scratch = CycleExecutionScratch();
}

bool HasValidEnvironmentCycle(const session::EnvironmentSnapshot& snapshot) {
  return std::isfinite(snapshot.cycle_dt_sec) != 0 && snapshot.cycle_dt_sec > 0.0f;
}

struct PipelineRuntimeConfig {
  explicit PipelineRuntimeConfig(ExecutionConfig initial_config)
      : base_config(std::move(initial_config)) {}

  ExecutionConfig base_config{};
  float platform_altitude_m{0.0f};
  session::ArControlProfile control_profile_{};
};

struct RuntimeOwnedState {
  explicit RuntimeOwnedState(const PipelineRuntimeConfig& config_state)
      : association_engine(
            SignalComponentFactory::BuildAssociationConfig(config_state.base_config)),
        track_filter(SignalComponentFactory::BuildTrackFilterConfig(config_state.base_config)) {}

  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
  std::unique_ptr<tracking::ITrackLifecycleManager> auto_lifecycle_manager;
};

struct AssociationSeedState {
  std::vector<tracking::AssociationTrackSeed> manual_association_seeds;
  bool has_manual_association_seeds{false};
};

struct SignalPipelineSnapshot {
  ExecutionConfig base_config{};
  float platform_altitude_m{0.0f};
  session::ArControlProfile control_profile{};
  AssociationSeedState association_seeds{};
  std::vector<tracking::TrackMeasurement> track_measurements{};
  session::AssociationQualityMetrics association_quality_metrics{};
  std::uint32_t cycle_index{1U};
  std::uint64_t batch_id{1U};
  association::DataAssociationRuntimeState association_runtime{};
  tracking::TrackLifecycleRuntimeState lifecycle_runtime{};
  bool has_pending_rf_v2_detection_context{false};
  RfV2DetectionContext pending_rf_v2_detection_context{};
  session::ArInterferenceObservationList pending_interference_observations{};
  detection::ArDeceptionClusterList pending_deception_clusters{};
};

bool IsValidIdentity(const oneq::electromagnetics::RfEmissionIdentity& identity) {
  return identity.platform_id != 0U && identity.equipment_id != 0U && identity.emission_id != 0U;
}

bool IsValidRfV2DetectionContext(const RfV2DetectionContext& context) {
  if (!IsValidIdentity(context.own_emission_identity) ||
      context.own_transmit_waveform.kind !=
          oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain ||
      !std::isfinite(context.receive_window_start_time_s) ||
      !std::isfinite(context.receive_window_duration_s) ||
      !std::isfinite(context.beam_pointing_deg.az_deg) ||
      !std::isfinite(context.beam_pointing_deg.el_deg) ||
      context.beam_pointing_deg.az_deg < -180.0f || context.beam_pointing_deg.az_deg > 180.0f ||
      context.beam_pointing_deg.el_deg < -90.0f || context.beam_pointing_deg.el_deg > 90.0f ||
      context.receive_window_duration_s <= 0.0) {
    return false;
  }
  double first_pulse_start_s = 0.0;
  double last_pulse_start_s = 0.0;
  if (!oneq::electromagnetics::TryResolveRfPulseStartTime(context.own_transmit_waveform, 0U,
                                                          &first_pulse_start_s) ||
      !oneq::electromagnetics::TryResolveRfPulseStartTime(
          context.own_transmit_waveform, context.own_transmit_waveform.pulse_count - 1U,
          &last_pulse_start_s) ||
      first_pulse_start_s < context.receive_window_start_time_s ||
      !std::isfinite(last_pulse_start_s)) {
    return false;
  }
  double unused_total_power_w = 0.0;
  if (!oneq::electromagnetics::TryAggregateRfIncidentPower(context.incident_links,
                                                           &unused_total_power_w)) {
    return false;
  }
  for (const auto& link : context.incident_links) {
    bool unused_active = false;
    double unused_arrival_frequency_hz = 0.0;
    if (!std::isfinite(link.received_power_before_overlap_w) ||
        link.received_power_before_overlap_w < 0.0 ||
        !oneq::electromagnetics::TryEvaluateRfArrivalActivity(
            link.emission_waveform, link.propagation_delay_s, link.doppler_shift_hz,
            context.receive_window_start_time_s, &unused_active, &unused_arrival_frequency_hz)) {
      return false;
    }
  }
  return true;
}

struct RuntimeState {
  explicit RuntimeState(ExecutionConfig initial_config)
      : config(std::move(initial_config)), owned(config) {}

  PipelineRuntimeConfig config;
  RuntimeOwnedState owned;
  AssociationSeedState association_seeds;
};

struct CycleState {
  std::uint32_t cycle_index{1};
  std::uint64_t batch_id{1};
  CycleExecutionScratch scratch{};
};

}  // namespace

struct SignalPipeline::Impl {
  explicit Impl(ExecutionConfig initial_config) : runtime_(std::move(initial_config)) {
    RebuildOwnedComponents();
  }

  CycleExecutionRuntime BuildExecutionRuntimeView() {
    return CycleExecutionRuntime(runtime_.config.base_config, runtime_.config.control_profile_,
                                 runtime_.owned.association_engine, runtime_.owned.track_filter,
                                 *runtime_.owned.auto_lifecycle_manager,
                                 runtime_.owned.signal_detector.get(),
                                 runtime_.association_seeds.manual_association_seeds,
                                 runtime_.association_seeds.has_manual_association_seeds);
  }

  ResolvedRuntimePipelineConfig ResolveRuntimeConfig() const {
    return ResolveRuntimePipelineConfig(runtime_.config.base_config,
                                        runtime_.config.control_profile_);
  }

  session::SignalCycleResult RunCycle(const session::ArSceneTargetList& scene_targets,
                                      const environment::IEnvironmentService& environment) {
    const session::ArSceneTargetList& input_state = scene_targets;

    if (!runtime_.config.base_config.sensor_enabled) {
      ResetCycleScratch(&cycle_.scratch);
      session::SignalCycleResult result;
      result.abort_reason = session::SignalCycleAbortReason::kSensorPoweredOff;
      return result;
    }

    if (runtime_.owned.auto_lifecycle_manager == nullptr) {
      PROJECT_LOG_ERROR(
          "[SignalPipeline] RunCycle aborted because auto_lifecycle_manager is unavailable.");
      ResetCycleScratch(&cycle_.scratch);
      session::SignalCycleResult result;
      result.abort_reason = session::SignalCycleAbortReason::kLifecycleUnavailable;
      return result;
    }

    const session::EnvironmentSnapshot environment_snapshot = environment.SampleEnvironment();
    if (!HasValidEnvironmentCycle(environment_snapshot)) {
      PROJECT_LOG_ERROR(
          "[SignalPipeline] RunCycle aborted because environment cycle is not initialized with a "
          "positive dt_sec.");
      ResetCycleScratch(&cycle_.scratch);
      session::SignalCycleResult result;
      result.abort_reason = session::SignalCycleAbortReason::kInvalidEnvironmentCycle;
      return result;
    }
    const CycleExecutionRuntime runtime_execution = BuildExecutionRuntimeView();

    const ResolvedRuntimePipelineConfig resolved = ResolveRuntimePipelineConfig(
        runtime_execution.base_config, runtime_execution.control_profile);
    ExecutionConfig runtime_config = resolved.config;
    ApplyScanScheduleToRuntimeConfig(environment_snapshot.cycle_index, &runtime_config);
    CycleExecutionContext context(
        input_state, environment_snapshot, environment_snapshot.cycle_index, cycle_.batch_id,
        std::move(runtime_config), runtime_.config.platform_altitude_m,
        has_pending_rf_v2_detection_context ? &pending_rf_v2_detection_context : nullptr,
        pending_interference_observations.empty() ? nullptr : &pending_interference_observations,
        pending_deception_clusters.empty() ? nullptr : &pending_deception_clusters);

    if (!ExecuteCycle(context, runtime_execution, cycle_.scratch)) {
      ResetCycleScratch(&cycle_.scratch);
      pending_interference_observations.clear();
      pending_deception_clusters.clear();
      session::SignalCycleResult result;
      result.abort_reason = session::SignalCycleAbortReason::kRuntimePreparationFailed;
      return result;
    }
    has_pending_rf_v2_detection_context = false;
    pending_rf_v2_detection_context = RfV2DetectionContext{};
    // 干扰观测已在本周期量测构建阶段消费，无论周期是否产生假目标标注都应清空，避免跨周期残留。
    pending_interference_observations.clear();
    pending_deception_clusters.clear();

    session::SignalCycleResult result;
    result.executed_this_cycle = true;
    result.abort_reason = session::SignalCycleAbortReason::kNone;
    result.updated_scene_targets = scene_targets;
    result.decision_frame = cycle_.scratch.decision_frame;
    result.association_quality_metrics = cycle_.scratch.association_quality_metrics;
    cycle_.cycle_index = environment_snapshot.cycle_index + 1U;
    ++cycle_.batch_id;
    return result;
  }

  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const {
    return cycle_.scratch.track_measurements;
  }

  session::AssociationQualityMetrics GetLastAssociationQualityMetrics() const {
    return cycle_.scratch.association_quality_metrics;
  }

  SignalPipelineRuntimeState CaptureRuntimeState() const {
    std::shared_ptr<SignalPipelineSnapshot> snapshot(new SignalPipelineSnapshot());
    snapshot->base_config = runtime_.config.base_config;
    snapshot->platform_altitude_m = runtime_.config.platform_altitude_m;
    snapshot->control_profile = runtime_.config.control_profile_;
    snapshot->association_seeds = runtime_.association_seeds;
    snapshot->track_measurements = cycle_.scratch.track_measurements;
    snapshot->association_quality_metrics = cycle_.scratch.association_quality_metrics;
    snapshot->cycle_index = cycle_.cycle_index;
    snapshot->batch_id = cycle_.batch_id;
    snapshot->association_runtime = runtime_.owned.association_engine.CaptureRuntimeState();
    if (runtime_.owned.auto_lifecycle_manager != nullptr) {
      snapshot->lifecycle_runtime = runtime_.owned.auto_lifecycle_manager->CaptureRuntimeState();
    }
    snapshot->has_pending_rf_v2_detection_context = has_pending_rf_v2_detection_context;
    snapshot->pending_rf_v2_detection_context = pending_rf_v2_detection_context;
    snapshot->pending_interference_observations = pending_interference_observations;
    snapshot->pending_deception_clusters = pending_deception_clusters;

    SignalPipelineRuntimeState state;
    state.owner_identity = this;
    state.schema_version = 3U;
    state.opaque = snapshot;
    return state;
  }

  void RestoreRuntimeState(const SignalPipelineRuntimeState& state) {
    if (state.owner_identity != this || state.schema_version != 3U) {
      PROJECT_LOG_ERROR(
          "[SignalPipeline] runtime state restore rejected because snapshot owner "
          "or schema does not match this instance.");
      return;
    }
    const std::shared_ptr<SignalPipelineSnapshot> snapshot =
        std::static_pointer_cast<SignalPipelineSnapshot>(state.opaque);
    if (snapshot == nullptr) {
      return;
    }

    runtime_.config.base_config = snapshot->base_config;
    runtime_.config.platform_altitude_m = snapshot->platform_altitude_m;
    runtime_.config.control_profile_ = snapshot->control_profile;
    RebuildOwnedComponents();
    runtime_.association_seeds = snapshot->association_seeds;
    runtime_.owned.association_engine.RestoreRuntimeState(snapshot->association_runtime);
    if (runtime_.owned.auto_lifecycle_manager != nullptr) {
      runtime_.owned.auto_lifecycle_manager->RestoreRuntimeState(snapshot->lifecycle_runtime);
    }
    cycle_.scratch = CycleExecutionScratch();
    cycle_.scratch.track_measurements = snapshot->track_measurements;
    cycle_.scratch.association_quality_metrics = snapshot->association_quality_metrics;
    cycle_.cycle_index = snapshot->cycle_index;
    cycle_.batch_id = snapshot->batch_id;
    has_pending_rf_v2_detection_context = snapshot->has_pending_rf_v2_detection_context;
    pending_rf_v2_detection_context = snapshot->pending_rf_v2_detection_context;
    pending_interference_observations = snapshot->pending_interference_observations;
    pending_deception_clusters = snapshot->pending_deception_clusters;
  }

  void SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
    if (seeds.empty()) {
      ClearManualAssociationSeeds();
      return;
    }
    if (!runtime_.owned.association_engine.SetAssociationSeeds(seeds)) {
      ClearManualAssociationSeeds();
      return;
    }
    runtime_.association_seeds.has_manual_association_seeds = true;
    runtime_.association_seeds.manual_association_seeds = seeds;
  }

  void ClearManualAssociationSeeds() {
    runtime_.association_seeds.has_manual_association_seeds = false;
    runtime_.association_seeds.manual_association_seeds.clear();
    runtime_.owned.association_engine.ResetAssociationSeedModeToStateless();
  }

  std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManager() const {
    const ResolvedRuntimePipelineConfig runtime_config = ResolveRuntimeConfig();
    return CreateAutoLifecycleManagerForRuntimeConfig(runtime_config.config);
  }

  bool UpdateConfig(ExecutionConfig new_config) {
    const ExecutionConfig previous_config = runtime_.config.base_config;
    runtime_.config.base_config = std::move(new_config);
    if (!SyncAssociationAndTrackFilterConfigs(
            runtime_.config.base_config, &runtime_.owned.association_engine,
            &runtime_.owned.track_filter, runtime_.owned.auto_lifecycle_manager.get())) {
      runtime_.config.base_config = previous_config;
      PROJECT_LOG_ERROR(
          "[SignalPipeline] UpdateConfig rejected because runtime config sync failed; "
          "keeping previous pipeline config.");
      return false;
    }

    OwnedSignalComponents components =
        SignalComponentFactory::BuildOwnedPipelineComponents(runtime_.config.base_config);
    runtime_.owned.kalman_predictor = std::move(components.kalman_predictor);
    runtime_.owned.kalman_updater = std::move(components.kalman_updater);
    runtime_.owned.signal_detector = std::move(components.signal_detector);
    return true;
  }

  void UpdatePlatformAttitude(const config::PlatformAttitudeDeg& platform_attitude_deg) {
    runtime_.config.base_config.detection.platform_attitude_deg = platform_attitude_deg;
  }

  void UpdatePlatformAltitudeM(float platform_altitude_m) {
    runtime_.config.platform_altitude_m = platform_altitude_m;
  }

  config::PlatformAttitudeDeg GetPlatformAttitude() const {
    return runtime_.config.base_config.detection.platform_attitude_deg;
  }

  float GetPlatformAltitudeM() const { return runtime_.config.platform_altitude_m; }

  void SetControlProfile(const session::ArControlProfile& control_profile) {
    runtime_.config.control_profile_ = control_profile;
  }
  session::ArControlProfile GetControlProfile() const { return runtime_.config.control_profile_; }
  void SetPendingInterferenceObservations(session::ArInterferenceObservationList observations,
                                          detection::ArDeceptionClusterList deception_clusters) {
    pending_interference_observations = std::move(observations);
    pending_deception_clusters = std::move(deception_clusters);
  }

  bool SetNextRfV2DetectionContext(const RfV2DetectionContext& context) {
    if (!IsValidRfV2DetectionContext(context)) {
      return false;
    }
    has_pending_rf_v2_detection_context = true;
    pending_rf_v2_detection_context = context;
    return true;
  }

  void RebuildOwnedComponents() {
    OwnedComponentSlots component_slots;
    component_slots.kalman_predictor = &runtime_.owned.kalman_predictor;
    component_slots.kalman_updater = &runtime_.owned.kalman_updater;
    component_slots.signal_detector = &runtime_.owned.signal_detector;
    component_slots.auto_lifecycle_manager = &runtime_.owned.auto_lifecycle_manager;
    RebuildOwnedComponentsForPipeline(runtime_.config.base_config, runtime_.config.control_profile_,
                                      &component_slots);
  }

  RuntimeState runtime_;
  CycleState cycle_;
  bool has_pending_rf_v2_detection_context{false};
  RfV2DetectionContext pending_rf_v2_detection_context{};
  // 由控制层在 RunCycle 前注入的本周期干扰观测，供航迹起批假目标鉴别；周期内消费后清空。
  session::ArInterferenceObservationList pending_interference_observations{};
  detection::ArDeceptionClusterList pending_deception_clusters{};
};

SignalPipeline::SignalPipeline(const ExecutionConfig& config)
    : impl_(std::unique_ptr<Impl>(new Impl(config))) {}

SignalPipeline::SignalPipeline(const config::ArSessionConfig& config)
    : SignalPipeline(::airborne_radar::config::mapping::MapSessionToExecution(config)) {}

bool SignalPipeline::SetNextRfV2DetectionContext(const RfV2DetectionContext& context) {
  return impl_->SetNextRfV2DetectionContext(context);
}

SignalPipeline::~SignalPipeline() = default;

session::SignalCycleResult SignalPipeline::RunCycle(
    const session::ArSceneTargetList& scene_targets,
    const environment::IEnvironmentService& environment) {
  return impl_->RunCycle(scene_targets, environment);
}

std::vector<tracking::TrackMeasurement> SignalPipeline::GetLastTrackMeasurements() const {
  return impl_->GetLastTrackMeasurements();
}

session::AssociationQualityMetrics SignalPipeline::GetLastAssociationQualityMetrics() const {
  return impl_->GetLastAssociationQualityMetrics();
}

SignalPipelineRuntimeState SignalPipeline::CaptureRuntimeState() const {
  return impl_->CaptureRuntimeState();
}

void SignalPipeline::RestoreRuntimeState(const SignalPipelineRuntimeState& state) {
  impl_->RestoreRuntimeState(state);
}

void SignalPipeline::SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
  impl_->SetAssociationSeeds(seeds);
}

void SignalPipeline::ClearManualAssociationSeeds() { impl_->ClearManualAssociationSeeds(); }

std::unique_ptr<tracking::ITrackLifecycleManager> SignalPipeline::CreateAutoLifecycleManager()
    const {
  return impl_->CreateAutoLifecycleManager();
}

void SignalPipeline::UpdatePlatformAttitude(
    const config::PlatformAttitudeDeg& platform_attitude_deg) {
  impl_->UpdatePlatformAttitude(platform_attitude_deg);
}

void SignalPipeline::UpdatePlatformAltitudeM(float platform_altitude_m) {
  impl_->UpdatePlatformAltitudeM(platform_altitude_m);
}

config::PlatformAttitudeDeg SignalPipeline::GetPlatformAttitude() const {
  return impl_->GetPlatformAttitude();
}

float SignalPipeline::GetPlatformAltitudeM() const { return impl_->GetPlatformAltitudeM(); }

void SignalPipeline::SetControlProfile(const session::ArControlProfile& control_profile) {
  impl_->SetControlProfile(control_profile);
}

session::ArControlProfile SignalPipeline::GetControlProfile() const {
  return impl_->GetControlProfile();
}

void SignalPipeline::SetPendingInterferenceObservations(
    session::ArInterferenceObservationList observations,
    detection::ArDeceptionClusterList deception_clusters) {
  impl_->SetPendingInterferenceObservations(std::move(observations), std::move(deception_clusters));
}

bool SignalPipeline::UpdateConfig(const config::ArSessionConfig& config) {
  return UpdateExecutionConfig(::airborne_radar::config::mapping::MapSessionToExecution(config));
}

bool SignalPipeline::UpdateExecutionConfig(const ExecutionConfig& config) {
  return impl_->UpdateConfig(config);
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
