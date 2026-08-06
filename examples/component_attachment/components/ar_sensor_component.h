/**
 * @file ar_sensor_component.h
 * @brief 自定义实体-组件示例：AR（机载雷达）传感器组件。
 *
 * 组件封装 airborne_radar 模块会话：每周期经 host_ 类型化读取同实体
 * FlightComponent 的平台位姿（周期内同步数据通路），驱动 ArSession，
 * 输出适配为 fusion::DetectionRecord 存自身状态（供 FusionComponent
 * 聚合）；轨迹生命周期事件（首确认/失跟）由库内 ArTrackLifecycleRecorder
 * 承担（Attach 后 StepWithResult 内部自动驱动），组件仅把差分事件转发
 * 为 World 信号（跨周期通知）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_

#include <vector>

#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/fusion/DetectionRecord.h"
#include "core/component.h"

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
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
