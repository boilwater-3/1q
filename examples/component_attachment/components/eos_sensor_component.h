/**
 * @file eos_sensor_component.h
 * @brief 自定义实体-组件示例：EOS（光电）传感器组件。
 *
 * 组件封装 electro_optical_sensor 模块会话：每周期经 EosCycleInputAdapter
 * 一步构建周期输入（平台 ECEF 运动学 + 光学目标），驱动 EosSession，
 * 探测记录适配为融合探测记录（无身份仅方位通道）存自身状态；探测生命
 * 周期事件（首发现/更新/丢失）由库内 EosDetectionLifecycleRecorder 承担
 * （Attach 后 StepWithResult 内部自动驱动），组件转发为 World 信号
 * EosDetectionEvent（target_id 经归属映射）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_

#include <vector>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/fusion/DetectionRecord.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief EOS 传感器组件：光电会话驱动 + 探测生命周期事件转发。
 *
 * 会话经 EosSession::Create(config) 构造后移动进组件（PImpl 可移动）。
 * detections() 为本周期适配后的泛型探测记录（key=0，仅方位），
 * FusionComponent 按挂载序在其后 Step 时聚合。
 */
class EosSensorComponent : public Component {
 public:
  explicit EosSensorComponent(electro_optical_sensor::session::EosSession session);
  ~EosSensorComponent() override = default;

  EosSensorComponent(const EosSensorComponent&) = delete;
  EosSensorComponent& operator=(const EosSensorComponent&) = delete;

  const char* Name() const override { return "EosSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读）。 */
  const std::vector<fusion::DetectionRecord>& detections() const { return detections_; }

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 最近周期波束中心方位角（单位：deg，平台系；仅开机且 kCompleted 周期有效，其余为 0）。 */
  float scan_azimuth_deg() const { return scan_azimuth_deg_; }

  /**
   * @brief 最近周期调试视图快照（规则 12 落盘示范）。
   *
   * Step 每周期经 EosOutputDebugViewBuilder::Build 回填（含按目标状态与
   * 规则 13b kInfo 排除诊断），调用方序列化为 JSON 写入自己的日志/事件系统
   * （参考 examples/common/EosDebugViewToJson.h）。
   * @return 最近周期调试视图；关机周期清零（无有效周期），拒绝周期为
   *         kCycleNotExecuted 快照。
   */
  const electro_optical_sensor::session::EosOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

  /**
   * @brief 运行时修改入口：包装 EosSession::TryApplyRuntimeConfig。
   *
   * EOS 为立即提交：补丁经 resolver 原子校验后一次生效（调用即生效，
   * session 层无回滚）；扫描相位是否重置由 resolver 显式决定。frame_rate_hz
   * 热更新经 resolver 校验有限且 > 0（非法值整补丁原子拒绝）。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 补丁无效或无变更（现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const electro_optical_sensor::config::EosRuntimeConfigPatch& patch);

 private:
  // lifecycle_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  electro_optical_sensor::session::EosDetectionLifecycleRecorder lifecycle_{}; /**< 探测生命周期事件源 */
  electro_optical_sensor::session::EosSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
  bool powered_on_{true};     /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时组件不驱动会话） */
  float scan_azimuth_deg_{0.0f}; /**< 最近周期波束中心方位角（deg，随周期结果刷新） */
  electro_optical_sensor::session::EosOutputDebugView last_debug_view_{}; /**< 最近周期调试视图快照（规则 12 落盘） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
