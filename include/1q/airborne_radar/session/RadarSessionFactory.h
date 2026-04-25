/**
 * @file RadarSessionFactory.h
 * @brief 定义 RadarSession 的公共创建入口。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_

#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace extension {
class RadarController;
class IOverrideControlStrategy;
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
  static RadarSession Create(const RadarSessionConfig& config = {});

  /**
   * @brief 使用外部 signal pipeline 创建会话。
   * @param config 会话配置。
   * @param signal_pipeline 外部提供的 signal pipeline。
   * @note 会话在下游执行失败时会回滚已提交的运行态，因此外部 pipeline 必须正确实现
   *       `CaptureRuntimeState()` / `RestoreRuntimeState()`，并保证恢复后不遗留内部副作用。
   */
  static RadarSession CreateWithSignalPipeline(
      const RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline);

  static RadarSession CreateWithEnvironmentService(
      const RadarSessionConfig& config, environment::IEnvironmentService& environment_service);

  static RadarSession CreateWithController(const RadarSessionConfig& config,
                                           extension::RadarController& controller);

  /**
   * @brief 使用外部覆盖策略创建会话。
   *
   * 内部会自动构建默认为 context/pipeline/environment/controller，
   * 并将 override_strategy 注入到 TacticalCoordinator。
   * 注入后内部 LPI/ECCM evaluator 将被跳过，由外部策略全权决策。
   * @note override_strategy 生命周期必须长于所在会话。
   */
  static RadarSession CreateWithOverrideStrategy(
      const RadarSessionConfig& config,
      extension::IOverrideControlStrategy& override_strategy);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_
