/**
 * @file sbirs_sensor_component.h
 * @brief 自定义实体-组件示例：SBIRS（天基红外）传感器组件。
 *
 * 组件封装 sbirs_sensor 模块会话：每周期从共享场景状态读取卫星 ECEF 位置与
 * 红外目标真值驱动会话；探测生命周期事件（首发现/更新/coasting/丢失）由库内
 * recorder 差分产出，组件转发为 World 信号。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/fusion/DetectionRecord.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "core/component.h"
#include "logger/logger_modes.h"

namespace component_attachment {

/**
 * @brief SBIRS 传感器组件：天基红外会话驱动 + 探测生命周期事件转发。
 *
 * 会话经 SbirsSession::Create(config) 构造后移动进组件（PImpl 可移动）。
 * detections() 为本周期适配后的泛型探测记录（key=0，仅方位，与 EOS 同构），
 * FusionComponent 按挂载序在其后 Step 时聚合。
 */
class SbirsSensorComponent : public Component {
 public:
  explicit SbirsSensorComponent(sbirs_sensor::session::SbirsSession session);
  ~SbirsSensorComponent() override = default;

  SbirsSensorComponent(const SbirsSensorComponent&) = delete;
  SbirsSensorComponent& operator=(const SbirsSensorComponent&) = delete;

  const char* Name() const override { return "SbirsSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读）。 */
  const std::vector<fusion::DetectionRecord>& detections() const { return detections_; }

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 最近周期波束中心方位角（deg，ECI 极坐标参考——库内为弧度，组件转度显示；仅开机且最近周期 kCompleted 为有效扫描方位，关机时组件清零）。 */
  float scan_azimuth_deg() const { return scan_azimuth_deg_; }

  /**
   * @brief 最近周期调试视图快照（规则 12 落盘示范）。
   *
   * Step 每周期经 SbirsOutputDebugViewBuilder::Build 回填（含按目标状态与
   * 规则 13b kInfo 排除诊断），供调用方结构化持久化到自己的日志/事件系统；
   * 本示例每周期直写人读摘要行到集成端日志（logger/logger.h 的 CA_LOG_VIEW）。
   * @return 最近周期调试视图；关机周期清零（无有效周期），拒绝周期为
   *         kCycleNotExecuted 快照。
   */
  const sbirs_sensor::session::SbirsOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

  /**
   * @brief 运行时修改入口：包装 SbirsSession::TryApplyRuntimeConfig。
   *
   * SBIRS 为立即提交：补丁经校验一次生效（调用即生效，session 层无回滚）。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 补丁无效或无变更（现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const sbirs_sensor::config::SbirsRuntimeConfigPatch& patch);

 private:
  // lifecycle_/exclusion_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder lifecycle_{}; /**< 探测生命周期事件源 */
  sbirs_sensor::session::SbirsExclusionCauseRecorder exclusion_{}; /**< 排除原因跨周期差分事件源 */
  sbirs_sensor::session::SbirsSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
  bool powered_on_{true};     /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时组件不驱动会话） */
  float scan_azimuth_deg_{0.0f}; /**< 最近周期波束中心方位角（deg，随周期结果刷新） */
  sbirs_sensor::session::SbirsOutputDebugView last_debug_view_{}; /**< 最近周期调试视图快照（规则 12 落盘） */

  /// 调试视图中文人读行写入（三模式分支见 .cpp；宏选择见 logger/logger.h）。
  void LogDebugView(const sbirs_sensor::session::SbirsOutputDebugView& view);
#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 模式二（跨周期状态增量）用：上一周期状态表（target_id → status）。
  std::unordered_map<std::uint64_t, sbirs_sensor::session::SbirsDebugTargetStatus>
      prev_target_status_{};
#endif
  bool step_timing_logged_{false}; /**< 单步执行时间是否已写入示例日志 */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
