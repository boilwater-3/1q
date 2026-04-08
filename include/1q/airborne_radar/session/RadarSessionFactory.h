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
class ISignalPipeline;
class IEnvironmentService;
}  // namespace extension
}  // namespace airborne_radar

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSessionFactory 负责会话装配与创建。
 */
class ONEQ_API RadarSessionFactory {
 public:
  static RadarSession Create(const RadarSessionConfig& config = {});

  static RadarSession CreateWithSignalPipeline(
      const RadarSessionConfig& config, extension::ISignalPipeline& signal_pipeline);

  static RadarSession CreateWithEnvironmentService(
      const RadarSessionConfig& config, extension::IEnvironmentService& environment_service);

  static RadarSession CreateWithController(const RadarSessionConfig& config,
                                           extension::RadarController& controller);
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_FACTORY_H_
