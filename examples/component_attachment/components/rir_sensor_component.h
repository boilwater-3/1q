/**
 * @file rir_sensor_component.h
 * @brief 自定义实体-组件示例：RIR（远程识别雷达）地基站点组件。
 *
 * 组件封装 remote_identification_radar 会话：挂载在独立的地基站点实体上
 * （非机载平台——S 波段识别雷达的物理摆放；先于平台实体创建保证融合读同周期
 * 量测）。每周期从共享场景状态读取站点局部 ENU 场景目标（含识别特征真值）
 * 驱动会话；识别结论状态迁移、指定任务生命周期经集成端事件日志输出，特征
 * 量测适配为泛型探测记录（fusion 源通道 kRirSourceId=5）供融合聚合。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_RIR_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_RIR_SENSOR_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "core/component.h"

namespace component_attachment {

/// RIR 地基站点实体名（main 创建实体与 FusionComponent 跨实体聚合共用）。
// C++14：namespace 作用域 constexpr 自带内部链接（inline 变量为 C++17 特性）。
constexpr char kRirSiteEntityName[] = "rir_ground_site";

/**
 * @brief RIR 地基站点传感器组件：识别会话驱动 + 识别/指定任务事件 + 融合量测。
 *
 * 会话经 RirSession::Create(config) 构造后移动进组件（PImpl 可移动）。站点为
 * 固定 LLA（构造时解析 ECEF，逐周期作为 platform_position 提供特征量测
 * sensor_origin）；场景目标真值（站点局部 ENU + 识别特征）由消费方每周期注入
 * DemoSceneState::rir_targets。
 */
class RirSensorComponent : public Component {
 public:
  RirSensorComponent(remote_identification_radar::session::RirSession session,
                     const oneq::coordinate::LlaPositionDegM& site_origin,
                     std::uint64_t designated_target_id, std::uint32_t designation_duration_cycles,
                     std::uint64_t sensor_platform_id, float recognition_dwell_sec);
  ~RirSensorComponent() override = default;

  RirSensorComponent(const RirSensorComponent&) = delete;
  RirSensorComponent& operator=(const RirSensorComponent&) = delete;

  const char* Name() const override { return "RirSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读；源通道 kRirSourceId）。 */
  const std::vector<fusion::DetectionRecord>& detections() const { return detections_; }

  /** @brief 当前电源状态（关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 累计识别结论输出数（冒烟断言：确认态结论按周期计数）。 */
  std::uint32_t confirmed_recognition_outputs() const { return confirmed_recognition_outputs_; }

 private:
  remote_identification_radar::session::RirSession session_;
  Entity* host_{nullptr};
  oneq::coordinate::LlaPositionDegM site_origin_{};   /**< 站点 LLA（ENU 原点） */
  oneq::coordinate::EcefPositionM site_ecef_{};       /**< 站点 ECEF（构造时解析一次） */
  std::uint64_t designated_target_id_{0U};            /**< 指定任务目标 ID（0 = 无任务） */
  std::uint32_t designation_duration_cycles_{0U};     /**< 指定任务窗口（周期；0 = 无限期） */
  std::uint64_t sensor_platform_id_{0U};              /**< RF scene 平台身份（排除自身发射） */
  float recognition_dwell_sec_{0.05f};                /**< 识别驻留窗口（rf_scene 时间对齐） */
  bool designation_applied_{false};                   /**< 指定任务补丁是否已下发 */
  std::vector<fusion::DetectionRecord> detections_{};
  bool powered_on_{true};
  std::uint32_t confirmed_recognition_outputs_{0U};
  /// 上一周期逐航迹识别状态（状态迁移事件判定：accumulating → confirmed）。
  std::unordered_map<std::uint64_t, remote_identification_radar::session::RirRecognitionState>
      prev_recognition_states_{};
  bool prev_designation_assigned_{false}; /**< 上一周期指定任务是否在案（终态沿事件判定） */
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_RIR_SENSOR_COMPONENT_H_
