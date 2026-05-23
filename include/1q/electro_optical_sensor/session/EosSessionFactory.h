/**
 * @file EosSessionFactory.h
 * @brief 定义 EosSession 的公共创建入口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_FACTORY_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_FACTORY_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace extension {
class IEosPipeline;
class EosController;
}
namespace environment {
class IEosEnvironmentService;
}
}

namespace electro_optical_sensor {
namespace session {

class ONEQ_API EosSessionFactory {
 public:
  static EosSession Create(const config::EosSessionConfig& config = {});

  static EosSession CreateWithPipeline(const config::EosSessionConfig& config,
                                        extension::IEosPipeline& pipeline);

  static EosSession CreateWithEnvironmentService(
      const config::EosSessionConfig& config,
      environment::IEosEnvironmentService& environment_service);

  static EosSession CreateWithController(const config::EosSessionConfig& config,
                                          extension::EosController& controller);

  static EosSession CreateWithAll(const config::EosSessionConfig& config,
                                   extension::IEosPipeline& pipeline,
                                   extension::EosController& controller);
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_FACTORY_H_
