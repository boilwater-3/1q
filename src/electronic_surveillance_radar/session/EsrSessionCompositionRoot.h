/**
 * @file EsrSessionCompositionRoot.h
 * @brief 定义电子侦察会话组合根。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "electronic_surveillance_radar/config/EsrInternalExecutionConfig.h"

namespace electronic_surveillance_radar {
namespace pipeline {
class InterceptPipeline;
}
namespace session {

/**
 * @brief EsrSessionComposition 描述会话装配后的组件集合。
 */
struct EsrSessionComposition {
  EsrInternalExecutionConfig execution_config{};

  std::unique_ptr<pipeline::InterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::EsrController> owned_controller;

  pipeline::InterceptPipeline* pipeline{nullptr};
  environment::IEsrEnvironmentService* environment_service{nullptr};
  extension::EsrController* controller{nullptr};
};

/**
 * @brief EsrSessionCompositionRoot 负责会话对象图装配。
 */
class EsrSessionCompositionRoot {
 public:
  static EsrSessionComposition ComposeDefault(const config::EsrSessionConfig& config);

  static EsrSessionComposition ComposeWithEnvironmentService(
      const config::EsrSessionConfig& config, environment::IEsrEnvironmentService& environment_service);
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_
