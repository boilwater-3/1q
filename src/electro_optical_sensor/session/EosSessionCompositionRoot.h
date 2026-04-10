/**
 * @file EosSessionCompositionRoot.h
 * @brief 定义 EOS 会话组合根及其装配结果。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

/**
 * @brief EosSessionComposition 描述会话所需依赖的组合结果。
 * @note 组合结果需保证 `pipeline` 与 `controller` 始终非空。
 */
struct EosSessionComposition {
  std::unique_ptr<::electro_optical_sensor::extension::IEosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;

  ::electro_optical_sensor::extension::IEosPipeline* pipeline{nullptr};
  extension::EosController* controller{nullptr};
};

/**
 * @brief EosSessionCompositionRoot 负责 EOS 会话依赖装配。
 */
class EosSessionCompositionRoot {
 public:
  /**
   * @brief 以默认依赖装配会话。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeDefault();

  /**
   * @brief 使用外部注入 pipeline 装配会话。
   * @param[in] pipeline 外部注入 pipeline。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeWithPipeline(
      ::electro_optical_sensor::extension::IEosPipeline& pipeline);

  /**
   * @brief 使用外部注入环境服务装配会话。
   * @param[in] environment_service 外部注入环境服务。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeWithEnvironmentService(
      extension::IEosEnvironmentService& environment_service);

  /**
   * @brief 使用外部注入控制器装配会话。
   * @param[in] controller 外部注入控制器。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeWithController(
      extension::EosController& controller);
};

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
