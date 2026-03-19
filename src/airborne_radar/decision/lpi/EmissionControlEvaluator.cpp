// Copyright 2026. All Rights Reserved.
//
// @file EmissionControlEvaluator.cpp
// @brief 实现 EmissionControlEvaluator 的 LPI 发射控制评估逻辑。

#include "airborne_radar/decision/lpi/EmissionControlEvaluator.h"

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace lpi {

namespace {

const std::uint32_t kLpiHoldCycles = 2;

/// @brief 构造一条 LPI 降功率控制意图。
/// @return 来源固定为 EMISSION_CONTROL 的控制意图。
common::ControlDirective BuildLpiPowerDirective() {
  return common::ControlDirective(
      common::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
      common::ControlDirectiveSource::EMISSION_CONTROL);
}

} // namespace

void EmissionControlEvaluator::Evaluate(
    const common::DecisionInputFrame& input_frame, pipeline::TacticalStateStore& state_store,
    pipeline::TacticalEvaluationState& evaluation_state) const {
  (void)input_frame;

  bool should_reduce_power = evaluation_state.should_reduce_power;
  if (!should_reduce_power && state_store.lpi_hold_cycles_remaining > 0U) {
    should_reduce_power = true;
    --state_store.lpi_hold_cycles_remaining;
  }

  evaluation_state.should_reduce_power = should_reduce_power;
  if (!should_reduce_power) {
    spdlog::info("[EmissionControlEvaluator] No severe threat. LPI remains inactive.");
    return;
  }

  state_store.lpi_hold_cycles_remaining = kLpiHoldCycles;
  evaluation_state.proposals.push_back(pipeline::TacticalProposal{
      BuildLpiPowerDirective(),
      60,
      "high-confidence threat requires reduced emission"});
  spdlog::info(
      "[EmissionControlEvaluator] High threat detected. Appending proposal: REQUEST_LPI_POWER_REDUCTION");
}

} // namespace lpi
} // namespace decision
} // namespace airborne_radar
