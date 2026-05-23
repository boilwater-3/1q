/**
 * @file EosSessionCompositionRoot.h
 * @brief 定义 EOS 会话组合根及其装配结果。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosSessionComposition 描述会话所需依赖的组合结果。
 * @note 组合结果需保证 `pipeline` 与 `controller` 始终非空。
 * @note 与 AR/ESR 不同，EO 传感器环境为纯观测型（无状态），环境因子以值类型嵌入
 *       CycleInput，ResolveFactors() 为无状态纯函数，因此环境服务托管在 Pipeline 内部
 *       而非在 Composition 层独立管理。
 */
struct EosSessionComposition {
  config::EosSessionConfig runtime_config{};
  extension::EosPipelineConfig pipeline_config{};
  bool initial_reset_scan_phase{true};

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
    * @param[in] config 会话初始化配置。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeDefault(const config::EosSessionConfig& config);

  /**
   * @brief 使用外部注入 pipeline 装配会话。
    * @param[in] config 会话初始化配置。
   * @param[in] pipeline 外部注入 pipeline。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeWithPipeline(
      const config::EosSessionConfig& config,
      ::electro_optical_sensor::extension::IEosPipeline& pipeline);

  /**
   * @brief 使用外部注入环境服务装配会话。
    * @param[in] config 会话初始化配置。
   * @param[in] environment_service 外部注入环境服务。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeWithEnvironmentService(
      const config::EosSessionConfig& config,
      environment::IEosEnvironmentService& environment_service);

  /**
   * @brief 使用外部注入控制器装配会话。
    * @param[in] config 会话初始化配置。
   * @param[in] controller 外部注入控制器。
   * @return 完整的会话组合结果。
   */
  static EosSessionComposition ComposeWithController(
      const config::EosSessionConfig& config,
      extension::EosController& controller);

  static EosSessionComposition ComposeAllExternal(
      const config::EosSessionConfig& config,
      ::electro_optical_sensor::extension::IEosPipeline& pipeline,
      extension::EosController& controller);
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SESSION_COMPOSITION_ROOT_H_
