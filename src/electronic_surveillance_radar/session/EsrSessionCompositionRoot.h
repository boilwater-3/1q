/**
 * @file EsrSessionCompositionRoot.h
 * @brief 定义电子侦察会话组合根。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrSessionComposition 描述会话装配后的组件集合。
 */
struct EsrSessionComposition {
  extension::InterceptPipelineConfig runtime_pipeline_config{};
  extension::InterceptRuntimeConfig runtime_config{};
  environment::EsrEnvironmentModelConfig runtime_environment_model_config{};

  std::unique_ptr<extension::IInterceptPipeline> owned_pipeline;
  std::unique_ptr<environment::IEsrEnvironmentService> owned_environment_service;
  std::unique_ptr<extension::EsrController> owned_controller;

  extension::IInterceptPipeline* pipeline{nullptr};
  environment::IEsrEnvironmentService* environment_service{nullptr};
  extension::EsrController* controller{nullptr};
};

/**
 * @brief EsrSessionCompositionRoot 负责会话对象图装配。
 */
class EsrSessionCompositionRoot {
 public:
  static EsrSessionComposition ComposeDefault(const EsrSessionConfig& config);

  static EsrSessionComposition ComposeWithPipeline(const EsrSessionConfig& config,
                                                   extension::IInterceptPipeline& pipeline);

  static EsrSessionComposition ComposeWithEnvironmentService(
      const EsrSessionConfig& config, environment::IEsrEnvironmentService& environment_service);

  static EsrSessionComposition ComposeWithController(const EsrSessionConfig& config,
                                                     extension::EsrController& controller);

  static EsrSessionComposition ComposeAllExternal(
      const EsrSessionConfig& config, extension::IInterceptPipeline& pipeline,
      environment::IEsrEnvironmentService& environment_service,
      extension::EsrController& controller);
};

}  // namespace session

}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_SESSION_ESR_SESSION_COMPOSITION_ROOT_H_
