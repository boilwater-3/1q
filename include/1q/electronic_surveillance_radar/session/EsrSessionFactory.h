/**
 * @file EsrSessionFactory.h
 * @brief 定义 EsrSession 的公共创建入口。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_FACTORY_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_FACTORY_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace extension {
class IInterceptPipeline;
}
namespace environment {
class IEsrEnvironmentService;
}
namespace extension {
class EsrController;
}
}  // namespace electronic_surveillance_radar

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrSessionFactory 负责电子侦察会话装配与创建。
 */
class ONEQ_API EsrSessionFactory {
 public:
  /**
   * @brief 使用默认配置创建会话。
   * @param config 会话配置。
   * @return 新创建的 EsrSession。
   */
  static EsrSession Create(const config::EsrSessionConfig& config = {});

  /**
   * @brief 使用外部截获流水线创建会话。
   * @param config 会话配置。
   * @param pipeline 外部提供的截获流水线。
   * @return 新创建的 EsrSession。
   */
  static EsrSession CreateWithPipeline(const config::EsrSessionConfig& config,
                                       extension::IInterceptPipeline& pipeline);

  /**
   * @brief 使用外部环境服务创建会话。
   * @param config 会话配置。
   * @param environment_service 外部提供的环境服务。
   * @return 新创建的 EsrSession。
   */
  static EsrSession CreateWithEnvironmentService(
      const config::EsrSessionConfig& config, environment::IEsrEnvironmentService& environment_service);

  /**
   * @brief 使用外部控制器创建会话。
   * @param config 会话配置。
   * @param controller 外部提供的控制器。
   * @return 新创建的 EsrSession。
   */
  static EsrSession CreateWithController(const config::EsrSessionConfig& config,
                                         extension::EsrController& controller);

  /**
   * @brief 使用全外部组件创建会话。
   * @param config 会话配置。
   * @param pipeline 外部流水线。
   * @param environment_service 外部环境服务。
   * @param controller 外部控制器。
   * @return 新创建的 EsrSession。
   */
  static EsrSession CreateWithAll(const config::EsrSessionConfig& config, extension::IInterceptPipeline& pipeline,
                                  environment::IEsrEnvironmentService& environment_service,
                                  extension::EsrController& controller);
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SESSION_FACTORY_H_
