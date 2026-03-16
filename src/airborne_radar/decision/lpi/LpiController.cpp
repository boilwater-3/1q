// Copyright 2026. All Rights Reserved.

#include "airborne_radar/decision/lpi/LpiController.h"

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace lpi {

namespace {

/// @brief 生成 LPI 功率控制指令。
static common::RadarCommand BuildLpiPowerCommand() {
  return common::RadarCommand(common::RadarCommandType::SET_LPI_POWER,
                              common::RadarCommandSource::LPI);
}

} // namespace

core::context::LpiControllerView LpiController::CreateView(
    core::context::DecisionContext &context) const {
  return core::context::CreateLpiControllerView(context);
}

void LpiController::ProcessView(core::context::LpiControllerView &view) {
  if (view.lpi_source_info.has_recon_platform) {
    // 识别到侦察类平台后，生成本周期 LPI 控制指令（当前仅启用功率控制，后续可根据需求启用更多控制维度）。
    view.decision_commands.push_back(BuildLpiPowerCommand());
    spdlog::info("[LpiController] High threat detected. Appending command: "
                 "SET_LPI_POWER");
  } else {
    spdlog::info(
        "[LpiController] No severe threat. LPI control remains inactive.");
  }
}

} // namespace lpi
} // namespace decision
} // namespace airborne_radar
