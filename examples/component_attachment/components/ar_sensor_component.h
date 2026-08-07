/**
 * @file ar_sensor_component.h
 * @brief 自定义实体-组件示例：AR（机载雷达）传感器组件。
 *
 * 组件封装 airborne_radar 模块会话：驱动 ArSession 产出探测记录；轨迹生命周期
 * 事件（首确认/失跟）由库内 recorder 差分产出，组件仅转发为 World 信号。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/fusion/DetectionRecord.h"
#include "core/component.h"
#include "demo_log_modes.h"

namespace component_attachment {

/**
 * @brief AR 传感器组件：雷达会话驱动 + 轨迹生命周期事件转发。
 *
 * 会话经 ArSession::Create(config) 构造后移动进组件（PImpl 可移动），
 * 构造内把库内 ArTrackLifecycleRecorder 挂到会话上（首确认/失跟事件源，
 * 替代集成侧自研状态判定）。detections() 为本周期适配后的泛型探测记录
 * （已发布轨迹快照，跳过 kLost），FusionComponent 按挂载序在其后 Step 时聚合。
 */
class ArSensorComponent : public Component {
 public:
  explicit ArSensorComponent(airborne_radar::session::ArSession session);
  ~ArSensorComponent() override = default;

  ArSensorComponent(const ArSensorComponent&) = delete;
  ArSensorComponent& operator=(const ArSensorComponent&) = delete;

  const char* Name() const override { return "ArSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读）。 */
  const std::vector<fusion::DetectionRecord>& detections() const { return detections_; }

  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /**
   * @brief 最近周期调试视图快照（规则 12 落盘示范）。
   *
   * Step 每周期经 ArTrackOutputDebugViewBuilder::Build 回填（含按目标状态与
   * 规则 13b kInfo 排除诊断），供调用方结构化持久化到自己的日志/事件系统；
   * 本示例每周期直写人读摘要行到集成端日志（components/demo_log.h 的 CA_LOG_VIEW）。
   * @return 最近周期调试视图；关机周期清零（无有效周期），拒绝周期为
   *         kCycleNotCompleted 快照。
   */
  const airborne_radar::session::ArTrackOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

  /**
   * @brief 运行时修改入口：包装 ArSession::TryApplyRuntimeConfig。
   *
   * AR 为事务性提交：补丁先暂存，下次成功周期边界统一生效（提交失败
   * 由库内快照完整回滚）；与现有配置冲突的非法补丁在入口即原子拒绝。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已接受并暂存；false 补丁非法（原子拒绝，现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const airborne_radar::config::ArRuntimeConfigPatch& patch);

 private:
  // lifecycle_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  airborne_radar::session::ArTrackLifecycleRecorder lifecycle_{}; /**< 轨迹生命周期事件源 */
  airborne_radar::session::ArSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时组件不驱动会话） */
  airborne_radar::session::ArTrackOutputDebugView last_debug_view_{}; /**< 最近周期调试视图快照（规则 12 落盘） */

  /// 调试视图中文人读行写入（三模式分支见 .cpp；宏选择见 components/demo_log.h）。
  void LogDebugView(const airborne_radar::session::ArTrackOutputDebugView& view);
#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 模式二（跨周期状态增量）用：上一周期状态表（external_target_id → status）。
  std::unordered_map<std::uint64_t, airborne_radar::session::ArDebugTrackStatus>
      prev_track_status_{};
#endif
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
