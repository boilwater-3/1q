// Copyright 2026. All Rights Reserved.
//
// Description: 旧责任链到新决策引擎接口的适配实现。

#include "airborne_radar/decision/LegacyPipelineDecisionEngine.h"

#include "1q/airborne_radar/core/context/DecisionContext.h"

namespace airborne_radar {
namespace decision {

namespace {

common::ControlDirectiveSource ToDirectiveSource(
    common::RadarCommandSource source) {
  switch (source) {
    case common::RadarCommandSource::CLASSIFIER:
      return common::ControlDirectiveSource::THREAT_ASSESSMENT;
    case common::RadarCommandSource::LPI:
      return common::ControlDirectiveSource::EMISSION_CONTROL;
    case common::RadarCommandSource::ECCM:
      return common::ControlDirectiveSource::SURVIVABILITY;
    case common::RadarCommandSource::UNKNOWN:
    default:
      return common::ControlDirectiveSource::LEGACY_PIPELINE;
  }
}

common::ControlDirectiveType ToDirectiveType(common::RadarCommandType type) {
  switch (type) {
    case common::RadarCommandType::SET_LPI_POWER:
      return common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION;
    case common::RadarCommandType::SET_LPI_BEAMFORMING:
      return common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING;
    case common::RadarCommandType::SET_LPI_DWELL:
      return common::ControlDirectiveType::REQUEST_LPI_DWELL;
    case common::RadarCommandType::ENABLE_SIDELOBE_CANCELLER:
      return common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER;
    case common::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING:
      return common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING;
    case common::RadarCommandType::SET_AGILITY_FREQ:
      return common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY;
    case common::RadarCommandType::SET_ECCM_REJITTER:
      return common::ControlDirectiveType::REQUEST_ECCM_REJITTER;
    case common::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN:
      return common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN;
    case common::RadarCommandType::NONE:
    default:
      return common::ControlDirectiveType::NONE;
  }
}

int ToPriority(common::ControlDirectiveType type) {
  switch (type) {
    case common::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return 90;
    case common::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return 85;
    case common::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return 84;
    case common::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return 83;
    case common::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return 82;
    case common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return 60;
    case common::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      return 55;
    case common::ControlDirectiveType::REQUEST_LPI_DWELL:
      return 50;
    case common::ControlDirectiveType::NONE:
    default:
      return 0;
  }
}

} // namespace

LegacyPipelineDecisionEngine::LegacyPipelineDecisionEngine(
    pipeline::ITacticalProcessor& pipeline_head)
    : pipeline_head_(pipeline_head) {}

TacticalDecisionResult LegacyPipelineDecisionEngine::Evaluate(
    const common::DecisionInputFrame& input_frame,
    TacticalStateStore& state_store) {
  core::context::DecisionContext context(input_frame);
  context.eccm_source_info.has_jamming_signal =
      input_frame.environment_jamming_detected;
  pipeline_head_.ProcessTactics(context);

  TacticalDecisionResult result;
  result.target_classification_result.reserve(
      context.target_classification_result.size());
  for (std::size_t i = 0; i < context.target_classification_result.size(); ++i) {
    result.target_classification_result.push_back(common::TargetCategory(
        context.target_classification_result[i].target_type));
  }
  result.selected_mode = input_frame.environment_jamming_detected
                             ? TacticalMode::kProtectedEmission
                             : (context.lpi_source_info.has_recon_platform
                                    ? TacticalMode::kThreatResponse
                                    : TacticalMode::kBaseline);

  for (std::size_t i = 0; i < context.decision_commands.size(); ++i) {
    const common::RadarCommand& command = context.decision_commands[i];
    result.proposals.push_back(TacticalProposal{
        common::ControlDirective(ToDirectiveType(command.type),
                                 ToDirectiveSource(command.source),
                                 command.info),
        ToPriority(ToDirectiveType(command.type)),
        "legacy pipeline command"});
  }

  state_store.current_mode = result.selected_mode;
  state_store.last_classification_labels.clear();
  state_store.last_classification_labels.reserve(
      result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary = "legacy-pipeline";
  return result;
}

} // namespace decision
} // namespace airborne_radar
