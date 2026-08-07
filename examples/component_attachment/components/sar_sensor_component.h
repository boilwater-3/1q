/**
 * @file sar_sensor_component.h
 * @brief 自定义实体-组件示例：SAR（合成孔径雷达）产品组件。
 *
 * 组件封装 sar 模块会话：驱动 SarSession 逐周期积累孔径产出图像产品；产品
 * 生命周期事件（产出/持续/丢失/失败）由库内 recorder 差分产出，组件转发为
 * World 信号。SAR 无探测输出（SarOutputFrame 仅为图像质量元数据），不作为
 * 融合输入。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SAR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SAR_SENSOR_COMPONENT_H_

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/session/SarProductDebugView.h"
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
   * @brief 最近周期产品调试视图快照（规则 12 落盘示范；阶段型视图）。
   *
   * Step 每周期经 SarProductDebugViewBuilder::Build 回填（执行状态、完成阶段、
   * L1/L3 成像标志、SNR、点目标与问题列表），供调用方结构化持久化到自己的
   * 日志/事件系统；本示例每周期直写中文人读摘要行到集成端日志。
   * @return 最近周期调试视图；关机周期清零（无有效周期），拒绝周期为对应快照。
   */
  const sar::session::SarProductDebugView& LastDebugView() const {
    return last_debug_view_;
  }

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
  sar::session::SarProductDebugView last_debug_view_{}; /**< 最近周期产品调试视图快照（规则 12 落盘） */

  /// 调试视图中文人读摘要行写入（阶段型视图：单行摘要，见 .cpp）。
  void LogDebugView(const sar::session::SarProductDebugView& view);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SAR_SENSOR_COMPONENT_H_
