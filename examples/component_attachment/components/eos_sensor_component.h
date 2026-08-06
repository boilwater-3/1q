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
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_EOS_SENSOR_COMPONENT_H_
