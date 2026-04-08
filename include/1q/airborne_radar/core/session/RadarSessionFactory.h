/**
 * @file RadarSessionFactory.h
 * @brief 定义 RadarSession 的公共创建入口。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_

#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace core {
namespace context {
class IRadarContext;
}
namespace controller {
class RadarController;
}
}  // namespace core
namespace signal {
namespace pipeline {
class ISignalPipeline;
}
}  // namespace signal
namespace environment {
class IEnvironmentService;
}
}  // namespace airborne_radar

namespace airborne_radar {
namespace core {
namespace session {

/**
 * @brief RadarSessionFactory 负责会话装配与创建。
 */
class ONEQ_API RadarSessionFactory {
 public:
  static RadarSession Create(const RadarSessionConfig& config = {});

  static RadarSession CreateWithSignalPipeline(
      const RadarSessionConfig& config, signal::pipeline::ISignalPipeline& signal_pipeline);

  static RadarSession CreateWithEnvironmentService(
      const RadarSessionConfig& config, environment::IEnvironmentService& environment_service);

  static RadarSession CreateWithController(const RadarSessionConfig& config,
                                           controller::RadarController& controller);

  static RadarSession CreateWithExternalChain(
      const RadarSessionConfig& config, context::IRadarContext& radar_context,
      signal::pipeline::ISignalPipeline& signal_pipeline,
      environment::IEnvironmentService& environment_service,
      controller::RadarController& controller);
};

}  // namespace session
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_
