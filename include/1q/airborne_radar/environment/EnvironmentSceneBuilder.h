/**
 * @file EnvironmentSceneBuilder.h
 * @brief 定义面向外部调用方的环境场景构造器。
 */

#ifndef ONEQ_AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SCENE_BUILDER_H_
#define ONEQ_AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SCENE_BUILDER_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace environment {

/**
 * @brief EnvironmentSceneBuilder 用于构造待生效的环境场景状态。
 */
class ONEQ_API EnvironmentSceneBuilder {
 public:
  /** @brief 默认构造函数，使用 EnvironmentSceneState 默认值初始化 */
  EnvironmentSceneBuilder() = default;

  /**
   * @brief 设置可选大气物理传播参数。
   * @param[in] config 大气物理传播参数。
   */
  EnvironmentSceneBuilder& SetAtmosphericPhysicsConfig(
      const AtmosphericPhysicsConfig& config);

  /**
   * @brief 设置可选植被散射物理参数。
   * @param[in] config 植被散射物理参数。
   */
  EnvironmentSceneBuilder& SetVegetationScatterPhysicsConfig(
      const VegetationScatterPhysicsConfig& config);

  /**
   * @brief 追加一个完整的干扰源状态。
   * @param[in] emitter 干扰源状态。
   */
  EnvironmentSceneBuilder& AddJammer(const JammerEmitterState& emitter);

  /**
   * @brief 追加一个压制式/噪声式干扰源。
   * @param[in] emitter 干扰源状态。
   * @note 无论 `emitter.technique` 的原始值如何，均会被强制覆盖为 `kNoiseSuppression`。
   */
  EnvironmentSceneBuilder& AddNoiseJammer(const JammerEmitterState& emitter);

  /**
   * @brief 追加一个欺骗式干扰源。
   * @param[in] emitter 干扰源状态。
   * @note 无论 `emitter.technique` 的原始值如何，均会被强制覆盖为 `kDeception`。
   */
  EnvironmentSceneBuilder& AddDeceptionJammer(const JammerEmitterState& emitter);

  /**
   * @brief 追加一个转发式/重复器式干扰源。
   * @param[in] emitter 干扰源状态。
   * @note 无论 `emitter.technique` 的原始值如何，均会被强制覆盖为 `kRepeater`。
   */
  EnvironmentSceneBuilder& AddRepeaterJammer(const JammerEmitterState& emitter);

  /**
   * @brief 生成当前构造结果。
   * @return 已构造的环境场景状态。
   */
  EnvironmentSceneState Build() const;

 private:
  EnvironmentSceneState scene_state_{};
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SCENE_BUILDER_H_
