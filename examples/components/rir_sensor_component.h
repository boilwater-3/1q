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
#include "1q/remote_identification_radar/session/RirExclusionCauseRecorder.h"
#include "1q/remote_identification_radar/session/RirOutputDebugView.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "1q/remote_identification_radar/session/RirTrackLifecycleRecorder.h"
#include "core/component.h"
#include "logger/logger_modes.h"

namespace component_attachment {

struct AppSceneState;  // core/scene_types.h（头文件仅引用，实现文件含入）

/// RIR 地基站点实体名（main 创建实体与 FusionComponent 跨实体聚合共用）。
// C++14：namespace 作用域 constexpr 自带内部链接（inline 变量为 C++17 特性）。
constexpr char kRirSiteEntityName[] = "rir_ground_site";

/**
 * @brief RIR 地基站点传感器组件：识别会话驱动 + 识别/指定任务事件 + 融合量测。
 *
 * 会话经 RirSession::Create(config) 构造后移动进组件（PImpl 可移动）。站点为
 * 固定 LLA（构造时解析 ECEF，逐周期作为 platform_position 提供特征量测
 * sensor_origin）；场景目标真值（站点局部 ENU + 识别特征）由消费方每周期注入
 * AppSceneState::rir_targets。
 */
class RirSensorComponent : public Component {
 public:
  RirSensorComponent(remote_identification_radar::session::RirSession session,
                     const oneq::coordinate::LlaPositionDegM& site_origin,
                     std::uint64_t sensor_platform_id, float recognition_dwell_sec);
  ~RirSensorComponent() override = default;

  RirSensorComponent(const RirSensorComponent&) = delete;
  RirSensorComponent& operator=(const RirSensorComponent&) = delete;

  const char* Name() const override { return "RirSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读；源通道 kRirSourceId）。 */
  const std::vector<fusion::DetectionRecord>& detections() const { return detections_; }

  /**
   * @brief 运行时修改入口：包装 RirSession::TryApplyRuntimeConfig。
   *
   * 指定识别任务（designated_external_target_id + 限时窗口）经此运行期下发
   * （外部指令事件 → CommandRouter → 本入口），不再走任务开始构造注入；
   * 补丁在会话下次成功周期边界统一生效，非法补丁原子拒绝。
   * @param[in] patch 运行期可变参数补丁（has_* 位标志选择字段）。
   * @return true 已接受并暂存；false 补丁非法（原子拒绝，现有配置不变）。
   */
  bool TryApplyRuntimeConfig(
      const remote_identification_radar::config::RirRuntimeConfigPatch& patch);

  /** @brief 当前电源状态（关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 累计识别结论输出数（冒烟断言：确认态结论按周期计数）。 */
  std::uint32_t confirmed_recognition_outputs() const { return confirmed_recognition_outputs_; }

  /** @brief 最近周期目标输出调试视图（规则 12；关机周期重置为空视图）。 */
  const remote_identification_radar::session::RirOutputDebugView& LastDebugView() const {
    return last_debug_view_;
  }

 private:
  /// 视图行三模式落盘（密度由编译期宏门控；纯观测）。
  void LogDebugView(World& world,
                    const remote_identification_radar::session::RirOutputDebugView& view);
  /// 周期输入组装：站点 ECEF + 共享场景目标 + RF 世界投影（供 StepWithResult）。
  remote_identification_radar::session::RirCycleInput BuildCycleInput(
      const AppSceneState& scene, double dt_sec) const;
  /// 识别结论迁移事件（进入确认态沿）+ 确认态周期计数（冒烟下限）。
  void PublishRecognitionEvents(
      World& world, const remote_identification_radar::session::RirCycleResult& result);
  /// 指定任务终态沿事件（designated_target_id 归零沿区分识别达成/窗口耗尽）。
  void PublishDesignationEvent(
      World& world, const remote_identification_radar::session::RirCycleResult& result);
  /// 航迹生命周期事件转发（首确认/丢失/指定作废沿；kUpdated/kNotTracked 不落盘）。
  void PublishTrackLifecycleEvents(
      World& world, const remote_identification_radar::session::RirCycleResult& result);
  /// 排除原因跨周期差分事件（纯诊断观测，仅落事件日志）。
  void PublishExclusionEvents(
      World& world, const remote_identification_radar::session::RirCycleResult& result);
  /// 特征量测 → 泛型探测记录 + 归属表键重写（库内键 → 外部目标 ID 并键融合）。
  void AdaptDetections(const remote_identification_radar::session::RirCycleResult& result);

  // 观测投影记录器（规则 10/11）：声明在 session_ 之前——析构顺序保证
  // "recorder 生命周期长于 Session 注册期"（Session 持非拥有裸指针）。
  remote_identification_radar::session::RirTrackLifecycleRecorder lifecycle_{};
  remote_identification_radar::session::RirExclusionCauseRecorder exclusion_{};
  remote_identification_radar::session::RirSession session_;
  Entity* host_{nullptr};
  oneq::coordinate::LlaPositionDegM site_origin_{};   /**< 站点 LLA（ENU 原点） */
  oneq::coordinate::EcefPositionM site_ecef_{};       /**< 站点 ECEF（构造时解析一次） */
  std::uint64_t sensor_platform_id_{0U};              /**< RF scene 平台身份（排除自身发射） */
  float recognition_dwell_sec_{0.05f};                /**< 识别驻留窗口（rf_scene 时间对齐） */
  std::vector<fusion::DetectionRecord> detections_{};
  bool powered_on_{true};
  std::uint32_t confirmed_recognition_outputs_{0U};
  /// 上一周期逐航迹识别状态（状态迁移事件判定：accumulating → confirmed）。
  std::unordered_map<std::uint64_t, remote_identification_radar::session::RirRecognitionState>
      prev_recognition_states_{};
  std::uint64_t prev_designated_target_id_{0U}; /**< 上一周期指定目标 ID（终态沿事件判定；0 = 无任务） */
  bool step_timing_logged_{false};        /**< 单步执行时间是否已写入示例日志 */
  remote_identification_radar::session::RirOutputDebugView last_debug_view_{}; /**< 最近周期视图 */
#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 视图模式二（跨周期增量）：上一周期逐目标调试状态（首次出现视为变化；
  /// 表只增不减，目标集长期收缩时调用方可按需清理——示例保持简单，不清理）。
  std::unordered_map<std::uint64_t, remote_identification_radar::session::RirDebugTargetStatus>
      prev_target_status_{};
#endif
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_RIR_SENSOR_COMPONENT_H_
