/**
 * @file eos_sensor_component.h
 * @brief 自定义实体-组件示例：EOS（光电）传感器组件。
 *
 * 组件封装 electro_optical_sensor 模块会话：每周期经 EosCycleInputAdapter
 * 一步构建周期输入（平台 ECEF 运动学 + 光学目标），驱动 EosSession，
 * 探测记录适配为融合探测记录（无身份仅方位通道）存自身状态；通过门限
 * 的探测经 World 信号发布 EosDetectionEvent（target_id 经归属映射）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_

#include <vector>

#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/fusion/DetectionRecord.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief EOS 传感器组件：光电会话驱动 + 探测事件发布。
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

 private:
  electro_optical_sensor::session::EosSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
