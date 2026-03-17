// Copyright 2026. All Rights Reserved.

#include "1q/airborne_radar/decision/eccm/EccmController.h"
#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace eccm {

namespace {

/// @brief 生成 ECCM 旁瓣对消指令。
static common::RadarCommand BuildSidelobeCancellerCommand() {
  return common::RadarCommand(
      common::RadarCommandType::ENABLE_SIDELOBE_CANCELLER,
      common::RadarCommandSource::ECCM);
}

/// @brief 生成 ECCM 频率捷变指令。
static common::RadarCommand BuildAgilityFrequencyCommand() {
  return common::RadarCommand(common::RadarCommandType::SET_AGILITY_FREQ,
                              common::RadarCommandSource::ECCM);
}

/// @brief 生成 ECCM 自适应波束形成指令。
static common::RadarCommand BuildAdaptiveBeamformingCommand() {
  return common::RadarCommand(
      common::RadarCommandType::ENABLE_ADAPTIVE_BEAMFORMING,
      common::RadarCommandSource::ECCM);
}

/// @brief 生成 ECCM 重频抖动指令。
static common::RadarCommand BuildEccmRejitterCommand() {
  return common::RadarCommand(common::RadarCommandType::SET_ECCM_REJITTER,
                              common::RadarCommandSource::ECCM);
}

/// @brief 生成 ECCM 烧穿增益指令。
static common::RadarCommand BuildEccmBurnthroughGainCommand() {
  return common::RadarCommand(
      common::RadarCommandType::SET_ECCM_BURNTHROUGH_GAIN,
      common::RadarCommandSource::ECCM);
}

} // namespace

core::context::EccmControllerView EccmController::CreateView(
    core::context::DecisionContext &context) const {
  return core::context::CreateEccmControllerView(context);
}

void EccmController::ProcessView(core::context::EccmControllerView &view) {
  if (view.eccm_source_info.has_jamming_signal) {
    // 识别到干扰后，生成本周期 ECCM 控制指令。
    view.decision_commands.push_back(BuildSidelobeCancellerCommand());
    view.decision_commands.push_back(BuildAdaptiveBeamformingCommand());
    view.decision_commands.push_back(BuildAgilityFrequencyCommand());
    view.decision_commands.push_back(BuildEccmRejitterCommand());
    view.decision_commands.push_back(BuildEccmBurnthroughGainCommand());
    spdlog::info(
        "[EccmController] Active jamming detected in environment! Appending "
        "commands: ENABLE_SIDELOBE_CANCELLER, "
        "ENABLE_ADAPTIVE_BEAMFORMING, SET_AGILITY_FREQ, "
        "SET_ECCM_REJITTER, SET_ECCM_BURNTHROUGH_GAIN.");
  } else {
    spdlog::info(
        "[EccmController] Environment is clear. Continuing nominal operation.");
  }
}

} // namespace eccm
} // namespace decision
} // namespace airborne_radar
