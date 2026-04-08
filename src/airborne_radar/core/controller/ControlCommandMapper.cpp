#include "airborne_radar/core/controller/ControlCommandMapper.h"

#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "airborne_radar/decision/pipeline/ControlReducer.h"

namespace airborne_radar {
namespace extension {

namespace {

extension::control::RadarCommandType ToRadarCommandType(
    extension::control::ControlDirectiveType type) {
  switch (type) {
    case extension::control::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return extension::control::RadarCommandType::SET_LPI_POWER;
    case extension::control::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      return extension::control::RadarCommandType::SET_LPI_BEAMFORMING;
    case extension::control::ControlDirectiveType::REQUEST_LPI_DWELL:
      return extension::control::RadarCommandType::SET_LPI_DWELL;
    case extension::control::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return extension::control::RadarCommandType::ENABLE_SIDELOBE_CANCELLER;
    case extension::control::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return extension::control::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING;
    case extension::control::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return extension::control::RadarCommandType::SET_AGILITY_FREQ;
    case extension::control::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return extension::control::RadarCommandType::SET_ECCM_REJITTER;
    case extension::control::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return extension::control::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN;
    case extension::control::ControlDirectiveType::NONE:
    default:
      return extension::control::RadarCommandType::NONE;
  }
}

extension::control::RadarCommandSource ToRadarCommandSource(
    extension::control::ControlDirectiveSource source) {
  switch (source) {
    case extension::control::ControlDirectiveSource::THREAT_ASSESSMENT:
      return extension::control::RadarCommandSource::CLASSIFIER;
    case extension::control::ControlDirectiveSource::EMISSION_CONTROL:
      return extension::control::RadarCommandSource::LPI;
    case extension::control::ControlDirectiveSource::SURVIVABILITY:
      return extension::control::RadarCommandSource::ECCM;
    case extension::control::ControlDirectiveSource::UNKNOWN:
    default:
      return extension::control::RadarCommandSource::UNKNOWN;
  }
}

extension::control::RadarCommand ToRadarCommand(
    const extension::control::ControlDirective& directive) {
  return extension::control::RadarCommand(ToRadarCommandType(directive.type),
                                       ToRadarCommandSource(directive.source));
}

}  // namespace

ControlCommandMapper::ControlCommandMapper(
    decision::pipeline::ControlReducer& control_reducer,
    extension::TacticalStateStore* tactical_state_store,
    extension::IRadarContext& radar_context)
    : control_reducer_(control_reducer),
      tactical_state_store_(tactical_state_store),
      radar_context_(radar_context) {}

extension::ControlReductionResult ControlCommandMapper::Apply(
    extension::control::RadarControlProfile* current_profile,
    const std::vector<extension::TacticalProposal>& proposals) {
  const extension::ControlReductionResult reduction_result =
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
    const extension::control::RadarCommand command =
        ToRadarCommand(reduction_result.applied_directives[i]);
    if (command.type == extension::control::RadarCommandType::NONE) {
      continue;
    }
    radar_context_.SubmitControlCommand(command);
  }

  return reduction_result;
}

}  // namespace extension
}  // namespace airborne_radar
