/**
 * @file sar_sensor_component.h
 * @brief 自定义实体-组件示例：SAR（合成孔径雷达）产品组件。
 *
 * 组件封装 sar 模块会话：每周期从 FlightComponent 读取平台 LLA/NED 状态
 * （航向分解 + 零姿态，示例简化）、从共享场景状态读取 SAR 点目标真值，
 * 构建 SarCycleInput 驱动 SarSession 逐周期积累孔径。图像产品生命周期
 * 事件（产出/持续/丢失/失败）由库内 SarProductLifecycleRecorder 承担
 * （Attach 后 StepWithResult 内部自动驱动），组件转发为 World 信号
 * SarProductEvent。SAR 无探测输出（SarOutputFrame 仅为图像质量元数据，
 * 契约见 docs/review/Bahavior.md），不作为融合输入。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SAR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SAR_SENSOR_COMPONENT_H_

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "1q/sar/session/SarSession.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief SAR 产品组件：孔径积累会话驱动 + 图像产品生命周期事件转发。
 *
 * 会话经 SarSession::Create(config) 构造后移动进组件（PImpl 可移动）。
 * 产品（图像）经积累完成后产出，经 on_sar_product 信号发布；组件不产生
 * 融合探测记录（SAR 无探测通道）。
 */
class SarSensorComponent : public Component {
 public:
  explicit SarSensorComponent(sar::session::SarSession session);
  ~SarSensorComponent() override = default;

  SarSensorComponent(const SarSensorComponent&) = delete;
  SarSensorComponent& operator=(const SarSensorComponent&) = delete;

  const char* Name() const override { return "SarSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /**
   * @brief 运行时修改入口：包装 SarSession::TryApplyRuntimeConfig。
   *
   * SAR 为立即提交：补丁经校验一次生效（调用即生效，session 层无回滚）。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 补丁无效或无变更（现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const sar::config::SarRuntimeConfigPatch& patch);

 private:
  // lifecycle_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  sar::session::SarProductLifecycleRecorder lifecycle_{}; /**< 产品生命周期事件源 */
  sar::session::SarSession session_;
  Entity* host_{nullptr};
  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时组件不驱动会话） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SAR_SENSOR_COMPONENT_H_
