/**
 * @file ecm_sensor_component.h
 * @brief 自定义实体-组件示例：ECM（电子对抗）组件。
 *
 * 封装 EcmSession：每周期读同实体 ESR 组件上一成功周期的去真值化假设，
 * 调度压制/欺骗干扰并将 emission_frame 发布到 RF-WORLD（供 AR/ESR/RIR 消费）。
 * 须挂载在 ESR 之后、AR 之前（挂载序 = 步进序）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ECM_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ECM_SENSOR_COMPONENT_H_

#include <cstdint>

#include "1q/electronic_countermeasure/EcmSession.h"
#include "core/component.h"

namespace component_attachment {

/// 演示层 ECM 发射 equipment_id（与 AR 发射链 1 / 接收链 2 区分）。
// C++14：namespace 作用域 constexpr 自带内部链接（inline 变量为 C++17 特性）。
constexpr std::uint64_t kEcmTransmitterEquipmentId = 101U;
/// 演示层平台 RF platform_id（与 AR/ESR platform_entity_id 一致）。
constexpr std::uint64_t kPlatformEntityId = 1U;

/**
 * @brief ECM 组件：ESR 驱动干扰调度 + RF-WORLD 发布。
 *
 * 会话经 EcmSession::Create(config) 构造后移动进组件。Find<EsrSensorComponent>()
 * 读取 ESR 缓存假设；成功执行时将 emission_frame 追加到 AppSceneState::rf_world。
 */
class EcmSensorComponent : public Component {
 public:
  explicit EcmSensorComponent(electronic_countermeasure::session::EcmSession session);
  ~EcmSensorComponent() override = default;

  EcmSensorComponent(const EcmSensorComponent&) = delete;
  EcmSensorComponent& operator=(const EcmSensorComponent&) = delete;

  const char* Name() const override { return "EcmSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前电源状态（关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 累计成功发射周期数（冒烟断言）。 */
  std::uint32_t executed_cycle_count() const { return executed_cycle_count_; }

 private:
  electronic_countermeasure::session::EcmSession session_;
  Entity* host_{nullptr};
  bool powered_on_{true};
  std::uint32_t executed_cycle_count_{0U};
  std::uint64_t last_submitted_esr_batch_id_{0U};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ECM_SENSOR_COMPONENT_H_
