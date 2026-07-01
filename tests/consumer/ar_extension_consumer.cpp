/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达扩展接口可被外部工程实现并接入会话。
 *
 * 本 consumer 演示 public API 的唯一自定义扩展点:外部实现 ITacticalDecisionEngine
 * 替换 AR 决策逻辑,其余组件(context / pipeline / environment service)由
 * ArSession::Create 内部默认装配,不对外暴露。
 */

#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArSession.h"

namespace airborne_radar {
namespace {

class DummyDecisionEngine : public session::ITacticalDecisionEngine {
 public:
  session::TacticalDecisionResult Evaluate(const session::DecisionInputFrame& input_frame,
                                             session::TacticalStateStore& state_store) override {
    (void)input_frame;
    (void)state_store;
    return {};
  }
};

}  // namespace
}  // namespace airborne_radar

int main() {
  airborne_radar::DummyDecisionEngine decision_engine;

  // 注入自定义决策引擎创建会话;context/pipeline/environment 由工厂内部装配。
  airborne_radar::session::ArSession session =
      airborne_radar::session::ArSession::CreateWithDecisionEngine(
          airborne_radar::config::ArSessionConfig{}, decision_engine);

  airborne_radar::session::ArCycleInput input;
  const airborne_radar::session::ArCycleResult result = session.StepWithResult(input);
  return result.executed_this_cycle ? 0 : 1;
}
