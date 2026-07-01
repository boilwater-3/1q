#include "airborne_radar/runtime/ControlCommandMapper.h"

#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/session/MutableRadarContext.h"

namespace airborne_radar {
namespace extension {

namespace {

session::ArCommandType ToArCommandType(
    session::ControlDirectiveType type) {
  switch (type) {
    case session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION:
      return session::ArCommandType::SET_LPI_POWER;
    case session::ControlDirectiveType::REQUEST_LPI_BEAMFORMING:
      return session::ArCommandType::SET_LPI_BEAMFORMING;
    case session::ControlDirectiveType::REQUEST_LPI_DWELL:
      return session::ArCommandType::SET_LPI_DWELL;
    case session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER:
      return session::ArCommandType::ENABLE_SIDELOBE_CANCELLER;
    case session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING:
      return session::ArCommandType::ENABLE_ADAPTIVE_BEAMFORMING;
    case session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY:
      return session::ArCommandType::SET_AGILITY_FREQ;
    case session::ControlDirectiveType::REQUEST_ECCM_REJITTER:
      return session::ArCommandType::SET_ECCM_REJITTER;
    case session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN:
      return session::ArCommandType::SET_ECCM_BURNTHROUGH_GAIN;
    case session::ControlDirectiveType::NONE:
    default:
      return session::ArCommandType::NONE;
  }
}

session::ArCommandSource ToArCommandSource(
    session::ControlDirectiveSource source) {
  switch (source) {
    case session::ControlDirectiveSource::THREAT_ASSESSMENT:
      return session::ArCommandSource::CLASSIFIER;
    case session::ControlDirectiveSource::EMISSION_CONTROL:
      return session::ArCommandSource::LPI;
    case session::ControlDirectiveSource::SURVIVABILITY:
      return session::ArCommandSource::ECCM;
    case session::ControlDirectiveSource::UNKNOWN:
    default:
      return session::ArCommandSource::UNKNOWN;
  }
}

session::ArCommand ToArCommand(
    const session::ControlDirective& directive) {
  return session::ArCommand(ToArCommandType(directive.type),
                                       ToArCommandSource(directive.source));
}

}  // namespace

ControlCommandMapper::ControlCommandMapper(
    decision::ControlReducer& control_reducer, session::MutableRadarContext& radar_context)
    : control_reducer_(control_reducer),
      radar_context_(radar_context) {}

extension::ControlReductionResult ControlCommandMapper::Apply(
    session::ArControlProfile* current_profile,
    const std::vector<session::TacticalProposal>& proposals) {
  const extension::ControlReductionResult reduction_result =
      control_reducer_.Reduce(*current_profile, proposals);

  *current_profile = reduction_result.profile;
  radar_context_.UpdateRadarControlProfile(*current_profile);

  for (std::size_t i = 0; i < reduction_result.applied_directives.size(); ++i) {
    const session::ArCommand command =
        ToArCommand(reduction_result.applied_directives[i]);
    if (command.type == session::ArCommandType::NONE) {
      continue;
    }
    radar_context_.SubmitControlCommand(command);
  }

  return reduction_result;
}

}  // namespace extension
}  // namespace airborne_radar
