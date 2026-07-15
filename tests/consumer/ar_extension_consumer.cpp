/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达步间外部决策接口可被外部工程使用。
 *
 * 本 consumer 演示 StepWithResult -> 外部 Evaluate -> SubmitExternalDecision。
 */

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"

namespace airborne_radar {
namespace {

session::ExternalDecisionResponse EvaluateExternal(
    const session::DecisionObservation& observation) {
  session::ExternalDecisionResponse response;
  response.source_cycle_index = observation.input_frame.cycle_index;
  response.source_batch_id = observation.input_frame.batch_id;
  return response;
}

}  // namespace
}  // namespace airborne_radar

int main() {
  airborne_radar::session::ArSession session =
      airborne_radar::session::ArSession::Create(airborne_radar::config::ArSessionConfig{});

  airborne_radar::session::ArCycleInput input;
  const airborne_radar::session::ArCycleResult first = session.StepWithResult(input);
  if (!first.executed_this_cycle || !first.has_decision_observation) {
    return 1;
  }
  const airborne_radar::session::ExternalDecisionResponse response =
      airborne_radar::EvaluateExternal(first.decision_observation);
  if (session.SubmitExternalDecision(response) !=
      airborne_radar::session::ExternalDecisionSubmitStatus::kAccepted) {
    return 2;
  }
  ++input.cycle_index;
  const airborne_radar::session::ArCycleResult second = session.StepWithResult(input);
  return second.executed_this_cycle &&
                 second.applied_decision_source ==
                     airborne_radar::session::DecisionControlSource::kExternal
             ? 0
             : 3;
}
