#include "airborne_radar/runtime/ControlCommandMapper.h"

#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/decision/ControlReducer.h"
#include "airborne_radar/session/MutableArContext.h"

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
    case session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE:
      return session::ArCommandType::ENABLE_ANTI_RGPO_LEADING_EDGE;
    case session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND:
      return session::ArCommandType::ENABLE_ANTI_VGPO_ACCELERATION_BOUND;
    case session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION:
      return session::ArCommandType::ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION;
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
    decision::ControlReducer& control_reducer, session::MutableArContext& radar_context)
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

session::ArCommand ControlCommandMapper::DirectiveToCommand(
    const session::ControlDirective& directive) {
  return ToArCommand(directive);
}

std::vector<session::ControlDirective> ControlCommandMapper::DiffProfiles(
    const session::ArControlProfile& baseline,
    const session::ArControlProfile& target) {
  std::vector<session::ControlDirective> diffs;

  // LPI domain
  if (baseline.enable_lpi_power_control != target.enable_lpi_power_control ||
      baseline.lpi_power_scale != target.lpi_power_scale) {
    if (target.enable_lpi_power_control) {
      diffs.emplace_back(session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
                          session::ControlDirectiveSource::UNKNOWN, target.lpi_power_scale);
    }
  }
  if (baseline.enable_lpi_beamforming != target.enable_lpi_beamforming) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_LPI_BEAMFORMING,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.lpi_dwell_scale != target.lpi_dwell_scale && target.lpi_dwell_scale != 1.0f) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_LPI_DWELL,
                        session::ControlDirectiveSource::UNKNOWN, target.lpi_dwell_scale);
  }

  // ECCM domain
  if (baseline.enable_agility_frequency != target.enable_agility_frequency) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.enable_sidelobe_canceller != target.enable_sidelobe_canceller) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.enable_adaptive_beamforming != target.enable_adaptive_beamforming) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.enable_eccm_rejitter != target.enable_eccm_rejitter) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ECCM_REJITTER,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.eccm_burnthrough_gain != target.eccm_burnthrough_gain &&
      target.eccm_burnthrough_gain > 1.0f) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN,
                        session::ControlDirectiveSource::UNKNOWN, target.eccm_burnthrough_gain);
  }
  if (baseline.enable_anti_rgpo_leading_edge != target.enable_anti_rgpo_leading_edge) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.enable_anti_vgpo_acceleration_bound != target.enable_anti_vgpo_acceleration_bound) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND,
                        session::ControlDirectiveSource::UNKNOWN);
  }
  if (baseline.enable_anti_false_target_discrimination !=
      target.enable_anti_false_target_discrimination) {
    diffs.emplace_back(session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION,
                        session::ControlDirectiveSource::UNKNOWN);
  }

  return diffs;
}

}  // namespace extension
}  // namespace airborne_radar
