#ifndef ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

struct EosSessionComposition {
  std::unique_ptr<::electro_optical_sensor::extension::IEosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;

  ::electro_optical_sensor::extension::IEosPipeline* pipeline{nullptr};
  extension::EosController* controller{nullptr};
};

class EosSessionCompositionRoot {
 public:
  static EosSessionComposition ComposeDefault();

  static EosSessionComposition ComposeWithPipeline(
      ::electro_optical_sensor::extension::IEosPipeline& pipeline);

  static EosSessionComposition ComposeWithEnvironmentService(
      extension::IEosEnvironmentService& environment_service);

  static EosSessionComposition ComposeWithController(
      extension::EosController& controller);
};

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
