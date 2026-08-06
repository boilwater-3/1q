/**
 * @file ar_sensor_component.h
 * @brief 自定义实体-组件示例：AR（机载雷达）传感器组件。
 *
 * 组件封装 airborne_radar 模块会话：每周期经 host_ 类型化读取同实体
 * FlightComponent 的平台位姿（周期内同步数据通路），驱动 ArSession，
 * 输出适配为 fusion::DetectionRecord 存自身状态（供 FusionComponent
 * 聚合）；目标首次确认/失跟经 World 信号发布事件（跨周期通知）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/fusion/DetectionRecord.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief AR 传感器组件：雷达会话驱动 + 轨迹事件判定。
 *
 * 会话经 ArSession::Create(config) 构造后移动进组件（PImpl 可移动）。
 * detections() 为本周期适配后的泛型探测记录（已发布轨迹快照，跳过
 * kLost），FusionComponent 按挂载序在其后 Step 时聚合。
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

 private:
  airborne_radar::session::ArSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
  std::vector<std::uint64_t> confirmed_keys_{}; /**< 已确认关联键（首次确认判定） */
  std::vector<std::uint64_t> lost_keys_{};      /**< 已报失跟键（防重复报告） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_AR_SENSOR_COMPONENT_H_
