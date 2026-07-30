/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达步间外部 profile 覆盖接口可被外部工程使用。
 *
 * 本 consumer 演示 StepWithResult -> 构建 ExternalDecisionOverride -> SubmitExternalDecision。
 */

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"

int main() {
  airborne_radar::session::ArSession session =
      airborne_radar::session::ArSession::Create(airborne_radar::config::ArSessionConfig{});

  // 第一步：执行周期，确认会话正常运转。
  airborne_radar::session::ArCycleInput input;
  const airborne_radar::session::ArCycleResult first = session.StepWithResult(input);
  if (!first.executed_this_cycle) {
    return 1;
  }

  // 第二步：构建基于回调的 profile 覆盖并提交。
  airborne_radar::session::ExternalDecisionOverride override_decision;
  override_decision.apply = [](const airborne_radar::session::ArControlProfile& current) {
    airborne_radar::session::ArControlProfile modified = current;
    modified.enable_agility_frequency = true;
    return modified;
  };
  if (session.SubmitExternalDecision(std::move(override_decision)) !=
      airborne_radar::session::ExternalDecisionSubmitStatus::kAccepted) {
    return 2;
  }

  // 第三步：执行下一周期，验证覆盖已生效。
  ++input.cycle_index;
  const airborne_radar::session::ArCycleResult second = session.StepWithResult(input);
  return second.executed_this_cycle &&
                 second.applied_decision_source ==
                     airborne_radar::session::DecisionControlSource::kExternal
             ? 0
             : 3;
}
