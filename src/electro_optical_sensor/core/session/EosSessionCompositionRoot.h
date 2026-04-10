#ifndef ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/electro_optical_sensor/core/session/EosSession.h"

namespace electro_optical_sensor {
namespace core {
namespace session {
namespace internal {

struct EosSessionComposition {
  std::unique_ptr<::electro_optical_sensor::pipeline::IEosPipeline> owned_pipeline;
  std::unique_ptr<controller::EosController> owned_controller;

  ::electro_optical_sensor::pipeline::IEosPipeline* pipeline{nullptr};
  controller::EosController* controller{nullptr};
};

class EosSessionCompositionRoot {
 public:
  static EosSessionComposition ComposeDefault();

  static EosSessionComposition ComposeWithPipeline(
      ::electro_optical_sensor::pipeline::IEosPipeline& pipeline);

  static EosSessionComposition ComposeWithEnvironmentService(
      environment::IEosEnvironmentService& environment_service);

  static EosSessionComposition ComposeWithController(
      controller::EosController& controller);
};

}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
