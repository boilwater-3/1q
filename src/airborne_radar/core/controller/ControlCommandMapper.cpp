#include "airborne_radar/core/controller/ControlCommandMapper.h"

#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "airborne_radar/decision/pipeline/ControlReducer.h"

namespace airborne_radar {
namespace core {
namespace controller {

namespace {

common::control::RadarCommandType ToRadarCommandType(
    common::control::ControlDirectiveType type) {
  switch (type) {
    case common::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return common::control::RadarCommandType::SET_LPI_POWER;
    case common::control::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      return common::control::RadarCommandType::SET_LPI_BEAMFORMING;
    case common::control::ControlDirectiveType::REQUEST_LPI_DWELL:
      return common::control::RadarCommandType::SET_LPI_DWELL;
    case common::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return common::control::RadarCommandType::ENABLE_SIDELOBE_CANCELLER;
    case common::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return common::control::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING;
    case common::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return common::control::RadarCommandType::SET_AGILITY_FREQ;
    case common::control::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return common::control::RadarCommandType::SET_ECCM_REJITTER;
    case common::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return common::control::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN;
    case common::control::ControlDirectiveType::NONE:
    default:
      return common::control::RadarCommandType::NONE;
  }
}

common::control::RadarCommandSource ToRadarCommandSource(
    common::control::ControlDirectiveSource source) {
  switch (source) {
    case common::control::ControlDirectiveSource::THREAT_ASSESSMENT:
      return common::control::RadarCommandSource::CLASSIFIER;
    case common::control::ControlDirectiveSource::EMISSION_CONTROL:
      return common::control::RadarCommandSource::LPI;
    case common::control::ControlDirectiveSource::SURVIVABILITY:
      return common::control::RadarCommandSource::ECCM;
    case common::control::ControlDirectiveSource::UNKNOWN:
    default:
      return common::control::RadarCommandSource::UNKNOWN;
  }
}

common::control::RadarCommand ToRadarCommand(
    const common::control::ControlDirective& directive) {
  return common::control::RadarCommand(ToRadarCommandType(directive.type),
                                       ToRadarCommandSource(directive.source));
}

}  // namespace

ControlCommandMapper::ControlCommandMapper(
    decision::pipeline::ControlReducer& control_reducer,
    decision::pipeline::TacticalStateStore* tactical_state_store,
    core::context::IRadarContext& radar_context)
    : control_reducer_(control_reducer),
      tactical_state_store_(tactical_state_store),
      radar_context_(radar_context) {}

decision::pipeline::ControlReductionResult ControlCommandMapper::Apply(
    common::control::RadarControlProfile* current_profile,
    const std::vector<decision::pipeline::TacticalProposal>& proposals) {
  const decision::pipeline::ControlReductionResult reduction_result =
      control_reducer_.Reduce(*current_profile, proposals);

  if (tactical_state_store_ != nullptr) {
    const decision::pipeline::ControlReducerRuntimeState reducer_state =
        control_reducer_.GetRuntimeState();
    tactical_state_store_->lpi_hold_cycles_remaining =
        reducer_state.lpi_hold_cycles_remaining;
    tactical_state_store_->eccm_hold_cycles_remaining =
        reducer_state.eccm_hold_cycles_remaining;
  }

  *current_profile = reduction_result.profile;
  radar_context_.UpdateRadarControlProfile(*current_profile);

  for (std::size_t i = 0; i < reduction_result.applied_directives.size(); ++i) {
    const common::control::RadarCommand command =
        ToRadarCommand(reduction_result.applied_directives[i]);
    if (command.type == common::control::RadarCommandType::NONE) {
      continue;
    }
    radar_context_.SubmitControlCommand(command);
  }

  return reduction_result;
}

}  // namespace controller
}  // namespace core
}  // namespace airborne_radar
