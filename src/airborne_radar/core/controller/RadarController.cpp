#include "1q/airborne_radar/core/controller/RadarController.h"

#include <spdlog/spdlog.h>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/core/output/DataOutputManager.h"
#include "airborne_radar/core/output/IDataOutputManager.h"
#include "airborne_radar/decision/pipeline/ControlReducer.h"
#include "airborne_radar/decision/pipeline/TacticalCoordinator.h"

namespace airborne_radar {
namespace core {
namespace controller {

namespace {

/**
 * @brief 将干扰语义枚举转换为日志可读字符串。
 * @param semantic 当前周期主导干扰语义。
 * @return 供日志输出使用的短字符串。
 */
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

/**
 * @brief 将控制意图类型映射为底座命令类型。
 * @param type 控制意图类型。
 * @return 对应的雷达命令类型。
 */
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

/**
 * @brief 将控制意图来源映射为底座命令来源。
 * @param source 控制意图来源。
 * @return 对应的雷达命令来源。
 */
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

/**
 * @brief 将单条控制意图转换为可提交的雷达命令。
 * @param directive 单条控制意图。
 * @return 与控制意图等价的雷达命令。
 */
common::RadarCommand ToRadarCommand(const common::ControlDirective& directive) {
  return common::RadarCommand(ToRadarCommandType(directive.type),
                              ToRadarCommandSource(directive.source));
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
      control_reducer_(new decision::pipeline::ControlReducer()),
      output_manager_(new output::DataOutputManager()) {
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
      control_reducer_(new decision::pipeline::ControlReducer()),
      output_manager_(new output::DataOutputManager()) {
  control_profile_ = owned_control_profile_.get();
}

RadarController::~RadarController() = default;

void RadarController::RunOnce() {
  signal_pipeline_.SetControlProfile(*control_profile_);

  const common::TargetFeatureList input_features =
      radar_context_.GetTargetFeatures();
  environment::EnvironmentCycleContext environment_cycle_context;
  environment_cycle_context.cycle_index = cycle_index_;
  environment_cycle_context.dt_sec = radar_context_.GetCycleDeltaTimeSec();
  environment_service_.BeginCycle(environment_cycle_context);
  signal_pipeline_.UpdatePlatformAttitude(radar_context_.GetPlatformAttitude());

  const signal::pipeline::SignalCycleResult signal_result =
      signal_pipeline_.RunCycle(input_features, environment_service_);
  const signal::pipeline::AssociationQualityMetrics association_metrics =
      signal_result.association_quality_metrics;
  common::DecisionInputFrame decision_frame = signal_result.decision_frame;
  decision_frame.cycle_index = cycle_index_;
  decision_frame.batch_id = batch_id_;
  common::TrackOutputFrame track_output_frame =
      output_manager_->BuildTrackOutputFrame(cycle_index_, batch_id_,
                                             decision_frame.tracks);
  latest_track_output_frame_ = track_output_frame;
  has_latest_track_output_frame_ = true;

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

  spdlog::debug(
      "[RadarController] cycle summary: cycle_index={} batch_id={} input_targets={} decision_features={} directives={} jamming_detected={} profile_version={} detect_rate={:.3f} detect_stress={:.3f} assoc_priors={} assoc_detections={} assoc_matches={} assoc_new_tracks={} assoc_missed_tracks={} assoc_match_rate={:.3f} assoc_new_track_rate={:.3f} assoc_missed_rate={:.3f} assoc_mean_cost={:.3f} assoc_p95_cost={:.3f} assoc_jam_semantic={} assoc_jam_severity={:.3f} assoc_stress={:.3f}",
      cycle_index_, batch_id_, input_features.size(), decision_frame.tracks.size(),
      reduction_result.applied_directives.size(),
      decision_frame.environment_jamming_detected ? "true" : "false",
      control_profile_->version, decision_frame.perception_quality_info.detection_rate,
      decision_frame.perception_quality_info.detection_stress,
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

bool RadarController::HasLatestTrackOutputFrame() const {
  return has_latest_track_output_frame_;
}

const common::TrackOutputFrame& RadarController::GetLatestTrackOutputFrame() const {
  return latest_track_output_frame_;
}

} // namespace controller
} // namespace core
} // namespace airborne_radar
