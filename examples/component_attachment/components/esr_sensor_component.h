/**
 * @file esr_sensor_component.h
 * @brief 自定义实体-组件示例：ESR（电子侦察）传感器组件。
 *
 * 组件封装 electronic_surveillance_radar 模块会话：每周期驱动 EsrSession，
 * 辐射源假设适配为融合探测记录（方位 + 射频特征）存自身状态；每条新
 * 假设经 World 信号发布 EmitterHypothesisEvent（跨周期通知）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ESR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ESR_SENSOR_COMPONENT_H_

#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/fusion/DetectionRecord.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief ESR 传感器组件：侦察会话驱动 + 假设事件发布。
 *
 * 会话经 EsrSession::Create(config) 构造后移动进组件（PImpl 可移动）。
 * detections() 为本周期适配后的泛型探测记录（假设键 ≠ 0），
 * FusionComponent 按挂载序在其后 Step 时聚合。
 */
class EsrSensorComponent : public Component {
 public:
  explicit EsrSensorComponent(electronic_surveillance_radar::session::EsrSession session);
  ~EsrSensorComponent() override = default;

  EsrSensorComponent(const EsrSensorComponent&) = delete;
  EsrSensorComponent& operator=(const EsrSensorComponent&) = delete;

  const char* Name() const override { return "EsrSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读）。 */
  const std::vector<fusion::DetectionRecord>& detections() const { return detections_; }

  /**
   * @brief 运行时修改入口：包装 EsrSession::TryApplyRuntimeConfig。
   *
   * ESR 为立即提交：调用即生效、单向落定（session 层无回滚），扫描相位
   * 重置语义由库内 resolver 决定。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已应用；false 未请求更新或补丁被拒绝（整补丁原子拒绝）。
   */
  bool TryApplyRuntimeConfig(
      const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch);

  /**
   * @brief 运行时修改入口（结构化结果）：包装
   *        EsrSession::ApplyRuntimeConfigWithResult。
   *
   * 与 TryApplyRuntimeConfig 同语义，另返回结构化状态码（含拒绝原因枚举），
   * 供外部决策/诊断使用。
   * @param[in] patch 运行期可变参数补丁。
   * @return 结构化应用结果（status 枚举 + applied/has_requested_update 位）。
   */
  electronic_surveillance_radar::session::EsrRuntimeConfigApplyResult ApplyRuntimeConfigWithResult(
      const electronic_surveillance_radar::config::EsrRuntimeConfigPatch& patch);

 private:
  electronic_surveillance_radar::session::EsrSession session_;
  Entity* host_{nullptr};
  std::vector<fusion::DetectionRecord> detections_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_ESR_SENSOR_COMPONENT_H_
