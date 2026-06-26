/**
 * @file RadarSessionFactory.h
 * @brief 定义 RadarSession 的公共创建入口。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SESSION_FACTORY_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SESSION_FACTORY_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {
class ITacticalDecisionEngine;
class RadarController;
class ISignalPipeline;
}  // namespace extension
namespace environment {
class IEnvironmentService;
}
}  // namespace airborne_radar

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSessionFactory 负责会话装配与创建。
 */
class ONEQ_API RadarSessionFactory {
 public:
  static RadarSession Create(const config::RadarSessionConfig& config = {});

  /**
   * @brief 注入自定义决策引擎创建会话。
   *
   * 这是 public API 的唯一自定义扩展点：外部实现 ITacticalDecisionEngine 即可替换
   * AR 决策逻辑，而 context / pipeline / environment service 由工厂内部默认装配，
   * 不对外暴露。
   * @param config 会话配置。
   * @param decision_engine 外部决策引擎，生命周期须长于所创建的会话。
   */
  static RadarSession CreateWithDecisionEngine(
      const config::RadarSessionConfig& config,
      extension::ITacticalDecisionEngine& decision_engine);

  /**
   * @brief 使用外部 signal pipeline 创建会话。
   * @param config 会话配置。
   * @param signal_pipeline 外部提供的 signal pipeline。
   * @note 会话在下游执行失败时会回滚已提交的运行态，因此外部 pipeline 必须正确实现
   *       `CaptureRuntimeState()` / `RestoreRuntimeState()`，并保证恢复后不遗留内部副作用。
   */
  static RadarSession CreateWithSignalPipeline(
      const config::RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline);

  static RadarSession CreateWithEnvironmentService(
      const config::RadarSessionConfig& config, environment::IEnvironmentService& environment_service);

  static RadarSession CreateWithController(const config::RadarSessionConfig& config,
                                           extension::RadarController& controller);

  static RadarSession CreateWithAll(
      const config::RadarSessionConfig& config, extension::IRadarContext& radar_context,
      extension::ISignalPipeline& signal_pipeline,
      environment::IEnvironmentService& environment_service,
      extension::RadarController& controller);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_SESSION_FACTORY_H_
