// Copyright 2026. All Rights Reserved.
//
// Description: RadarController 的核心调度实现。

#include "1q/airborne_radar/core/controller/RadarController.h"

#include <algorithm>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/core/event/IEventBus.h"
#include "1q/airborne_radar/core/event/RadarEvents.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/decision/pipeline/ControlReducer.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/decision/pipeline/TacticalCoordinator.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"

namespace airborne_radar {
namespace core {
namespace controller {

namespace {

common::JammingTechnique ToCommonJammingTechnique(
    environment::JammingTechnique technique) {
  switch (technique) {
    case environment::JammingTechnique::kNoiseSuppression:
      return common::JammingTechnique::kNoiseSuppression;
    case environment::JammingTechnique::kDeception:
      return common::JammingTechnique::kDeception;
    case environment::JammingTechnique::kRepeater:
      return common::JammingTechnique::kRepeater;
    case environment::JammingTechnique::kUnknown:
    default:
      return common::JammingTechnique::kUnknown;
  }
}

common::EccmJammerSourceInfo BuildEccmJammerSourceInfo(
    const environment::JammerSourceFact& environment_source) {
  common::EccmJammerSourceInfo source_info;
  source_info.technique = ToCommonJammingTechnique(environment_source.technique);
  source_info.jammer_power_db = environment_source.power_db;
  source_info.jammer_to_signal_db = environment_source.js_db;
  source_info.frequency_overlap_ratio =
      environment_source.frequency_overlap_ratio;
  source_info.prf_lock_risk = environment_source.prf_lock_risk;
  source_info.azimuth_deg = environment_source.azimuth_deg;
  source_info.elevation_deg = environment_source.elevation_deg;
  source_info.angular_span_deg = environment_source.angular_span_deg;
  source_info.jammer_in_sidelobe = environment_source.in_sidelobe;
  source_info.confidence = environment_source.confidence;
  return source_info;
}

common::EccmSourceInfo BuildEccmSourceInfo(
    const environment::EnvironmentSnapshot& environment_snapshot) {
  common::EccmSourceInfo source_info;
  source_info.has_jamming_signal = environment_snapshot.jamming_detected;
  source_info.jammer_power_db = environment_snapshot.jammer_power_db;
  source_info.frequency_overlap_ratio =
      environment_snapshot.jammer_frequency_overlap_ratio;
  source_info.prf_lock_risk = environment_snapshot.jammer_prf_lock_risk;
  source_info.jammer_in_sidelobe = environment_snapshot.jammer_in_sidelobe;
  source_info.jammer_sources.reserve(environment_snapshot.jammer_sources.size());
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    source_info.jammer_sources.push_back(
        BuildEccmJammerSourceInfo(environment_snapshot.jammer_sources[i]));
  }
  return source_info;
}

const char* JammingSemanticName(common::JammingSemantic semantic) {
  switch (semantic) {
    case common::JammingSemantic::kNoiseSuppression:
      return "noise";
    case common::JammingSemantic::kDeception:
      return "deception";
    case common::JammingSemantic::kRepeater:
      return "repeater";
    case common::JammingSemantic::kMixed:
      return "mixed";
    case common::JammingSemantic::kNone:
    default:
      return "none";
  }
}

common::DecisionTrackSnapshot BuildDecisionTrackSnapshotFromFeature(
    const common::TargetFeature& feature, std::size_t index) {
  const std::uint64_t association_key =
      feature.external_target_id != 0U ? feature.external_target_id
                                       : static_cast<std::uint64_t>(index + 1U);
  common::DecisionTrackSnapshot snapshot(
      feature.current_track_velocity_x, feature.current_track_velocity_y,
      feature.current_track_velocity_z, feature.current_track_rcs,
      feature.current_track_acceleration_x,
      feature.current_track_acceleration_y,
      feature.current_track_acceleration_z, false,
      feature.external_target_id, association_key);
  snapshot.state.status = common::DecisionTrackStatus::kConfirmed;
  snapshot.state.position_x = feature.position_x;
  snapshot.state.position_y = feature.position_y;
  snapshot.state.position_z = feature.position_z;
  snapshot.evidence.has_measurement_evidence = true;
  snapshot.evidence.updated_this_cycle = true;
  snapshot.evidence.predicted_only_this_cycle = false;
  return snapshot;
}

common::DecisionInputFrame BuildDecisionFrameFromFeatures(
    const common::TargetFeatureList& features, std::uint32_t cycle_index,
    std::uint64_t batch_id,
    const common::EccmSourceInfo& eccm_source_info) {
  common::DecisionInputFrame frame;
  frame.cycle_index = cycle_index;
  frame.batch_id = batch_id;
  frame.environment_jamming_detected = eccm_source_info.has_jamming_signal;
  frame.eccm_source_info = eccm_source_info;
  frame.tracks.reserve(features.size());
  for (std::size_t i = 0; i < features.size(); ++i) {
    frame.tracks.push_back(BuildDecisionTrackSnapshotFromFeature(features[i], i));
  }
  return frame;
}

common::AssociationQualityInfo BuildAssociationQualityInfo(
    const signal::pipeline::AssociationQualityMetrics& metrics) {
  common::AssociationQualityInfo info;
  info.match_rate = metrics.match_rate;
  info.new_track_rate = metrics.new_track_rate;
  info.missed_track_rate = metrics.missed_track_rate;
  info.mean_match_cost = metrics.mean_match_cost;
  info.p95_match_cost = metrics.p95_match_cost;
  info.dominant_jamming_semantic = metrics.dominant_jamming_semantic;
  info.jamming_severity = metrics.jamming_severity;
  info.association_stress = metrics.association_stress;
  return info;
}

common::PerceptionQualityInfo BuildPerceptionQualityInfo(
    std::size_t input_target_count,
    const signal::pipeline::AssociationQualityMetrics& metrics) {
  common::PerceptionQualityInfo info;
  info.input_target_count = input_target_count;
  info.detection_count = metrics.detection_count;
  if (input_target_count == 0U) {
    info.detection_rate = 1.0f;
    info.detection_stress = 0.0f;
    return info;
  }

  info.detection_rate = std::min(
      1.0f, static_cast<float>(metrics.detection_count) /
                static_cast<float>(input_target_count));
  info.detection_stress = std::max(0.0f, 1.0f - info.detection_rate);
  return info;
}

common::RadarCommandType ToRadarCommandType(
    common::ControlDirectiveType type) {
  switch (type) {
    case common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return common::RadarCommandType::SET_LPI_POWER;
    case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      return common::RadarCommandType::SET_LPI_BEAMFORMING;
    case common::ControlDirectiveType::REQUEST_LPI_DWELL:
      return common::RadarCommandType::SET_LPI_DWELL;
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return common::RadarCommandType::ENABLE_SIDELOBE_CANCELLER;
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return common::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING;
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return common::RadarCommandType::SET_AGILITY_FREQ;
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return common::RadarCommandType::SET_ECCM_REJITTER;
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return common::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN;
    case common::ControlDirectiveType::NONE:
    default:
      return common::RadarCommandType::NONE;
  }
}

common::RadarCommandSource ToRadarCommandSource(
    common::ControlDirectiveSource source) {
  switch (source) {
    case common::ControlDirectiveSource::THREAT_ASSESSMENT:
      return common::RadarCommandSource::CLASSIFIER;
    case common::ControlDirectiveSource::EMISSION_CONTROL:
      return common::RadarCommandSource::LPI;
    case common::ControlDirectiveSource::SURVIVABILITY:
      return common::RadarCommandSource::ECCM;
    case common::ControlDirectiveSource::UNKNOWN:
    default:
      return common::RadarCommandSource::UNKNOWN;
  }
}

common::RadarCommand ToRadarCommand(const common::ControlDirective& directive) {
  return common::RadarCommand(ToRadarCommandType(directive.type),
                              ToRadarCommandSource(directive.source),
                              directive.info);
}

bool HasProfileChanged(const common::RadarControlProfile& previous_profile,
                       const common::RadarControlProfile& next_profile) {
  return previous_profile.version != next_profile.version;
}

/// @brief 解析当前控制真值对生命周期失配容忍的保护增益。
/// @param control_profile 当前控制真值。
/// @return 额外允许的 miss 周期数。
std::uint32_t ResolveLifecycleExtraMissTolerance(
    const common::RadarControlProfile& control_profile) {
  std::uint32_t extra_miss_tolerance = 0U;
  if (control_profile.enable_sidelobe_canceller ||
      control_profile.enable_agility_frequency ||
      control_profile.enable_eccm_rejitter) {
    extra_miss_tolerance += 1U;
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    extra_miss_tolerance += 1U;
  }
  return extra_miss_tolerance;
}

} // namespace

RadarController::RadarController(
    core::context::IRadarContext& radar_context,
    signal::pipeline::ISignalPipeline& signal_pipeline,
    environment::IEnvironmentService& environment_service)
    : radar_context_(radar_context),
      signal_pipeline_(signal_pipeline),
      decision_engine_(nullptr),
      owned_decision_engine_(new decision::pipeline::TacticalCoordinator()),
      environment_service_(environment_service),
      control_profile_(nullptr),
      owned_control_profile_(new common::RadarControlProfile()),
      tactical_state_store_(new decision::pipeline::TacticalStateStore()),
      control_reducer_(new decision::pipeline::ControlReducer()) {
  decision_engine_ = owned_decision_engine_.get();
  control_profile_ = owned_control_profile_.get();
}

RadarController::RadarController(
    core::context::IRadarContext& radar_context,
    signal::pipeline::ISignalPipeline& signal_pipeline,
    environment::IEnvironmentService& environment_service,
    core::event::IEventBus& event_bus)
    : radar_context_(radar_context),
      signal_pipeline_(signal_pipeline),
      decision_engine_(nullptr),
      owned_decision_engine_(new decision::pipeline::TacticalCoordinator()),
      environment_service_(environment_service),
      event_bus_(&event_bus),
      control_profile_(nullptr),
      owned_control_profile_(new common::RadarControlProfile()),
      tactical_state_store_(new decision::pipeline::TacticalStateStore()),
      control_reducer_(new decision::pipeline::ControlReducer()) {
  decision_engine_ = owned_decision_engine_.get();
  control_profile_ = owned_control_profile_.get();
}

RadarController::RadarController(
    core::context::IRadarContext& radar_context,
    signal::pipeline::ISignalPipeline& signal_pipeline,
    decision::pipeline::ITacticalDecisionEngine& decision_engine,
    environment::IEnvironmentService& environment_service)
    : radar_context_(radar_context),
      signal_pipeline_(signal_pipeline),
      decision_engine_(&decision_engine),
      environment_service_(environment_service),
      control_profile_(nullptr),
      owned_control_profile_(
          new common::RadarControlProfile()),
      tactical_state_store_(new decision::pipeline::TacticalStateStore()),
      control_reducer_(new decision::pipeline::ControlReducer()) {
  control_profile_ = owned_control_profile_.get();
}

RadarController::RadarController(
    core::context::IRadarContext& radar_context,
    signal::pipeline::ISignalPipeline& signal_pipeline,
    decision::pipeline::ITacticalDecisionEngine& decision_engine,
    environment::IEnvironmentService& environment_service,
    core::event::IEventBus& event_bus)
    : radar_context_(radar_context),
      signal_pipeline_(signal_pipeline),
      decision_engine_(&decision_engine),
      environment_service_(environment_service),
      event_bus_(&event_bus),
      control_profile_(nullptr),
      owned_control_profile_(new common::RadarControlProfile()),
      tactical_state_store_(new decision::pipeline::TacticalStateStore()),
      control_reducer_(new decision::pipeline::ControlReducer()) {
  control_profile_ = owned_control_profile_.get();
}

RadarController::~RadarController() = default;

void RadarController::EnsureAutoLifecycleManager() {
  if (track_lifecycle_manager_ != nullptr) {
    return;
  }

  auto_track_lifecycle_manager_ = signal_pipeline_.CreateAutoLifecycleManager();
  if (auto_track_lifecycle_manager_ == nullptr) {
    return;
  }

  track_lifecycle_manager_ = auto_track_lifecycle_manager_.get();
  spdlog::info(
      "[RadarController] auto lifecycle manager assembled from pipeline config");
}

void RadarController::RunOnce() {
  signal_pipeline_.SetControlProfile(*control_profile_);
  EnsureAutoLifecycleManager();

  if (event_bus_ != nullptr) {
    event_bus_->BeginCycle();
    event_bus_->DispatchCurrentCycle();
  }

  const common::TargetFeatureList input_features =
      radar_context_.GetTargetFeatures();
  environment::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = cycle_index_;
  environment_cycle_context.dt_sec = radar_context_.GetCycleDeltaTimeSec();
  environment_service_.BeginCycle(environment_cycle_context);
  signal_pipeline_.UpdatePlatformAttitude(radar_context_.GetPlatformAttitude());

  std::size_t association_seed_count = 0;
  if (track_lifecycle_manager_ != nullptr) {
    const std::vector<signal::tracking::AssociationTrackSeed> seeds =
        track_lifecycle_manager_->BuildAssociationSeeds();
    association_seed_count = seeds.size();
    signal_pipeline_.SetAssociationSeeds(seeds);
  } else {
    signal_pipeline_.ResetAssociationSeedModeToStateless();
  }

  const common::TargetFeatureList updated_features =
      signal_pipeline_.RunCycle(input_features, environment_service_);
  const signal::pipeline::AssociationQualityMetrics association_metrics =
      signal_pipeline_.GetLastAssociationQualityMetrics();

  const environment::EnvironmentSnapshot environment_snapshot =
      environment_service_.SampleEnvironment();
  const common::EccmSourceInfo eccm_source_info =
      BuildEccmSourceInfo(environment_snapshot);
  const common::AssociationQualityInfo association_quality_info =
      BuildAssociationQualityInfo(association_metrics);
  const common::PerceptionQualityInfo perception_quality_info =
      BuildPerceptionQualityInfo(input_features.size(), association_metrics);
  const bool environment_jamming_detected = eccm_source_info.has_jamming_signal;
  common::TargetFeatureList decision_features = updated_features;
  common::DecisionInputFrame decision_frame = BuildDecisionFrameFromFeatures(
      updated_features, cycle_index_, batch_id_, eccm_source_info);
  decision_frame.association_quality_info = association_quality_info;
  decision_frame.perception_quality_info = perception_quality_info;
  std::size_t measurement_count = 0;

  if (track_lifecycle_manager_ != nullptr) {
    const std::vector<signal::tracking::TrackMeasurement> measurements =
        signal_pipeline_.GetLastTrackMeasurements();
    measurement_count = measurements.size();
    signal::tracking::CycleContext cycle;
    cycle.cycle_index = cycle_index_;
    cycle.batch_id = batch_id_;
    cycle.dt_sec = environment_cycle_context.dt_sec;
    cycle.extra_miss_tolerance =
        ResolveLifecycleExtraMissTolerance(*control_profile_);
    track_lifecycle_manager_->Update(cycle, measurements);
    decision_features = track_lifecycle_manager_->BuildFeatureSnapshot();
    decision_frame = track_lifecycle_manager_->BuildDecisionFrame(
        cycle_index_, batch_id_, environment_jamming_detected);
    decision_frame.environment_jamming_detected = environment_jamming_detected;
    decision_frame.eccm_source_info = eccm_source_info;
    decision_frame.association_quality_info = association_quality_info;
    decision_frame.perception_quality_info = perception_quality_info;
  }

  decision::pipeline::TacticalDecisionResult decision_result;
  if (decision_engine_ != nullptr && tactical_state_store_ != nullptr) {
    decision_result = decision_engine_->Evaluate(decision_frame, *tactical_state_store_);
  }

  const common::RadarControlProfile previous_profile = *control_profile_;
  const decision::pipeline::ControlReductionResult reduction_result =
      control_reducer_ != nullptr
          ? control_reducer_->Reduce(*control_profile_, decision_result.proposals)
          : decision::pipeline::ControlReductionResult();
  *control_profile_ = reduction_result.profile;
  radar_context_.UpdateRadarControlProfile(*control_profile_);

  ExecuteCommands(reduction_result.applied_directives);

  if (event_bus_ != nullptr) {
    core::event::TracksUpdatedEvent tracks_event;
    tracks_event.state = decision_features;
    event_bus_->Enqueue(tracks_event);

    if (environment_jamming_detected) {
      core::event::JammingAlertEvent jamming_event;
      jamming_event.detected = true;
      event_bus_->Enqueue(jamming_event);
    }

    core::event::CommandsSubmittedEvent commands_event;
    commands_event.command_count = reduction_result.applied_directives.size();
    event_bus_->Enqueue(commands_event);

    if (HasProfileChanged(previous_profile, *control_profile_)) {
      core::event::ControlProfileUpdatedEvent profile_event;
      profile_event.cycle_index = cycle_index_;
      profile_event.profile_version = control_profile_->version;
      profile_event.applied_directive_count =
          reduction_result.applied_directives.size();
      profile_event.rejected_directive_count =
          reduction_result.rejected_directives.size();
      profile_event.lpi_power_control_enabled =
          control_profile_->enable_lpi_power_control;
      profile_event.agility_frequency_enabled =
          control_profile_->enable_agility_frequency;
      profile_event.sidelobe_canceller_enabled =
          control_profile_->enable_sidelobe_canceller;
      profile_event.adaptive_beamforming_enabled =
          control_profile_->enable_adaptive_beamforming;
      event_bus_->Enqueue(profile_event);
    }

    for (std::size_t i = 0; i < reduction_result.applied_directives.size(); ++i) {
      core::event::DirectiveAppliedEvent directive_event;
      directive_event.cycle_index = cycle_index_;
      directive_event.profile_version = control_profile_->version;
      directive_event.directive = reduction_result.applied_directives[i];
      event_bus_->Enqueue(directive_event);
    }

    for (std::size_t i = 0; i < reduction_result.rejected_directives.size(); ++i) {
      core::event::DirectiveRejectedEvent directive_event;
      directive_event.cycle_index = cycle_index_;
      directive_event.profile_version = control_profile_->version;
      directive_event.directive = reduction_result.rejected_directives[i];
      event_bus_->Enqueue(directive_event);
    }

    core::event::RadarCycleCompletedEvent event;
    event.command_count = reduction_result.applied_directives.size();
    event.jamming_detected = environment_jamming_detected;
    event_bus_->Enqueue(event);
    event_bus_->EndCycle();
  }

  spdlog::debug(
      "[RadarController] cycle summary: cycle_index={} batch_id={} input_targets={} association_seeds={} measurements={} decision_features={} directives={} lifecycle_enabled={} jamming_detected={} profile_version={} detect_rate={:.3f} detect_stress={:.3f} assoc_priors={} assoc_detections={} assoc_matches={} assoc_new_tracks={} assoc_missed_tracks={} assoc_match_rate={:.3f} assoc_new_track_rate={:.3f} assoc_missed_rate={:.3f} assoc_mean_cost={:.3f} assoc_p95_cost={:.3f} assoc_jam_semantic={} assoc_jam_severity={:.3f} assoc_stress={:.3f}",
      cycle_index_, batch_id_, input_features.size(), association_seed_count,
      measurement_count, decision_features.size(),
      reduction_result.applied_directives.size(),
      track_lifecycle_manager_ != nullptr ? "true" : "false",
      environment_jamming_detected ? "true" : "false",
      control_profile_->version, perception_quality_info.detection_rate,
      perception_quality_info.detection_stress,
      association_metrics.prior_track_count,
      association_metrics.detection_count, association_metrics.matched_count,
      association_metrics.new_track_count,
      association_metrics.missed_track_count, association_metrics.match_rate,
      association_metrics.new_track_rate,
      association_metrics.missed_track_rate,
      association_metrics.mean_match_cost,
      association_metrics.p95_match_cost,
      JammingSemanticName(association_metrics.dominant_jamming_semantic),
      association_metrics.jamming_severity,
      association_metrics.association_stress);

  ++cycle_index_;
  ++batch_id_;
}

void RadarController::RunCycles(std::size_t cycles) {
  for (std::size_t i = 0; i < cycles; ++i) {
    RunOnce();
  }
}

void RadarController::ExecuteCommands(
    const std::vector<common::ControlDirective>& directives) {
  for (std::size_t i = 0; i < directives.size(); ++i) {
    const common::RadarCommand command = ToRadarCommand(directives[i]);
    if (command.type == common::RadarCommandType::NONE) {
      continue;
    }
    radar_context_.SubmitControlCommand(command);
  }
}

void RadarController::SetEventBus(core::event::IEventBus* event_bus) {
  event_bus_ = event_bus;
}

void RadarController::SetTrackLifecycleManager(
    signal::tracking::ITrackLifecycleManager* lifecycle_manager) {
  auto_track_lifecycle_manager_.reset();
  track_lifecycle_manager_ = lifecycle_manager;
  spdlog::info(
      "[RadarController] track lifecycle manager {} (association priors: external seeds only)",
      lifecycle_manager != nullptr ? "attached" : "detached");
}

void RadarController::UpdateControlReducerConfig(
    const decision::pipeline::ControlReducerConfig& config) {
  if (control_reducer_ == nullptr) {
    return;
  }
  control_reducer_->UpdateConfig(config);
  spdlog::info(
      "[RadarController] control reducer config updated: lpi_power_scale={} dwell_scale={} burnthrough_gain={} burnthrough_power_floor={} lpi_hold={} eccm_hold={} lpi_cooldown={} eccm_cooldown={} prefer_survivability_power={} prefer_survivability_beam={}",
      config.lpi_power_scale_on_reduction, config.lpi_dwell_scale,
      config.eccm_burnthrough_gain, config.burnthrough_lpi_power_floor,
      config.lpi_hold_cycles_after_request,
      config.eccm_hold_cycles_after_request,
      config.lpi_cooldown_cycles_after_release,
      config.eccm_cooldown_cycles_after_release,
      config.prefer_survivability_in_power_conflict ? "true" : "false",
      config.prefer_survivability_in_beam_conflict ? "true" : "false");
}

} // namespace controller
} // namespace core
} // namespace airborne_radar
