/**
 * @file sbirs_sensor_component.h
 * @brief 自定义实体-组件示例：SBIRS（天基红外）传感器组件。
 *
 * 组件封装 sbirs_sensor 模块会话：每周期从共享场景状态读取天基平台
 * （卫星）ECEF 位置与红外目标真值，构建 SbirsCycleInput 驱动 SbirsSession，
 * 探测记录适配为融合探测记录（无身份仅方位通道）存自身状态；探测生命
 * 周期事件（首发现/更新/coasting/丢失）由库内 SbirsDetectionLifecycleRecorder
 * 承担（Attach 后 StepWithResult 内部自动驱动），组件转发为 World 信号
 * SbirsDetectionEvent（target_id 经归属映射）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_

#include <vector>

#include "1q/fusion/DetectionRecord.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "core/component.h"

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

  /**
   * @brief 运行时修改入口：包装 SbirsSession::TryApplyRuntimeConfig。
   *
   * SBIRS 为立即提交：补丁经校验一次生效（调用即生效，session 层无回滚）。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 补丁无效或无变更（现有配置不变）。
   */
  bool TryApplyRuntimeConfig(const sbirs_sensor::config::SbirsRuntimeConfigPatch& patch);

 private:
  // lifecycle_ 声明在 session_ 之前：析构逆序时 session_ 先析构（其析构不
  // 触达 recorder 指针），满足"recorder 生命周期长于 Session 注册期"约束。
  sbirs_sensor::session::SbirsDetectionLifecycleRecorder lifecycle_{}; /**< 探测生命周期事件源 */
  sbirs_sensor::session::SbirsSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
