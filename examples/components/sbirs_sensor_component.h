/**
 * @file sbirs_sensor_component.h
 * @brief 自定义实体-组件示例：SBIRS（天基红外）传感器组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "core/component.h"
#include "logger/logger_modes.h"

namespace component_attachment {

struct AppSceneState;  // core/scene_types.h（头文件仅引用，实现文件含入）

/** @brief 卫星探测帧投递方式（ground_station_source_id 非 0 时生效）。 */
enum class SbirsGroundDeliveryMode {
  kSharedBlackboard = 0, /**< 写 AppSceneState.sbirs_ground_station_inbox */
  kMessage = 1           /**< 发 on_sbirs_frame_submitted */
};

/**
 * @brief SBIRS 传感器组件：天基红外会话驱动 + 探测生命周期事件转发。
 */
class SbirsSensorComponent : public Component {
 public:
  explicit SbirsSensorComponent(sbirs_sensor::session::SbirsSession session);
  /**
   * @brief 卫星实体：自持星历，探测帧投递地面站（不写探测池）。
   * @param[in] ground_station_source_id 融合源通道（每星互异）
   */
  SbirsSensorComponent(sbirs_sensor::session::SbirsSession session,
                       std::uint32_t ground_station_source_id,
                       sbirs_sensor::session::SbirsVector3M position_ecef_m,
                       sbirs_sensor::session::SbirsVector3M velocity_ecef_m_per_s,
                       sbirs_sensor::session::SbirsEulerAnglesDeg attitude_eci_body_deg,
                       SbirsGroundDeliveryMode ground_delivery =
                           SbirsGroundDeliveryMode::kSharedBlackboard);
  ~SbirsSensorComponent() override = default;

  SbirsSensorComponent(const SbirsSensorComponent&) = delete;
  SbirsSensorComponent& operator=(const SbirsSensorComponent&) = delete;

  const char* Name() const override { return "SbirsSensor"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 本周期适配后的泛型探测记录（融合聚合读）。 */
  /** @brief 当前电源状态（由 sensor_enabled 补丁唯一维护；未步进前默认 true，关机时组件不驱动会话）。 */
  bool powered_on() const { return powered_on_; }

  /** @brief 最近周期波束中心方位角（deg，ECI 极坐标参考——库内为弧度，组件转度显示；仅开机且最近周期 kCompleted 为有效扫描方位，关机时组件清零）。 */
  float scan_azimuth_deg() const { return scan_azimuth_deg_; }

  /**
   * @brief 最近周期调试视图快照（关机周期清零，拒绝周期为 kCycleNotExecuted）。
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

  float scan_azimuth_deg_{0.0f}; /**< 最近周期波束中心方位角（deg，ECI 极坐标——库内弧度，组件转度显示） */

  sbirs_sensor::session::SbirsOutputDebugView last_debug_view_{};  /**< 本周期调试视图快照（视图行数据源） */

#if defined(CA_VIEW_LOG_MODE_DELTA)
  /// 视图模式二：上一周期逐目标调试状态（首次出现视为变化；表只增不减，示例不清理）。
  std::unordered_map<std::uint64_t, sbirs_sensor::session::SbirsDebugTargetStatus>
      prev_target_status_{};
#endif

  bool powered_on_{true}; /**< 电源状态（由 sensor_enabled 补丁唯一维护；关机时不驱动会话） */
  bool step_timing_logged_{false}; /**< 单步执行时间是否已写入示例日志（只写首个周期） */
  std::uint32_t ground_station_source_id_{0U}; /**< 非 0：探测帧投递地面站，不写探测池 */
  SbirsGroundDeliveryMode ground_delivery_{SbirsGroundDeliveryMode::kSharedBlackboard};
  bool use_own_satellite_pose_{false}; /**< 真：用本实体星历，不要求同实体 Flight */
  sbirs_sensor::session::SbirsVector3M own_position_ecef_m_{}; /**< 本实体卫星 ECEF 位置（m） */
  sbirs_sensor::session::SbirsVector3M own_velocity_ecef_m_per_s_{}; /**< 本实体卫星 ECEF 速度（m/s） */
  sbirs_sensor::session::SbirsEulerAnglesDeg own_attitude_eci_body_deg_{}; /**< 本实体卫星姿态（Body→ECI，deg） */

  /// 调试视图中文人读行写入（三模式分支见 .cpp；宏选择见 logger/logger_modes.h）。
  void LogDebugView(const sbirs_sensor::session::SbirsOutputDebugView& view);
  /// 周期输入组装：天基平台状态 + ECI 场景目标（消费方每周期注入共享场景状态）。
  sbirs_sensor::session::SbirsCycleInput BuildCycleInput(const AppSceneState& scene,
                                                  double dt_sec) const;
  /// 探测生命周期事件发布（首发现/更新/coasting/丢失 → World 信号 + 事件日志）。
  void PublishDetectionEvents(World& world, const AppSceneState& scene,
                             const sbirs_sensor::session::SbirsCycleResult& result);
  /// 排除原因跨周期差分事件（纯诊断观测，仅落事件日志）。
  void PublishExclusionEvents(World& world);
  /// 探测记录 → 泛型探测记录（源通道 kSbirsSourceId，无身份键 0）。
  void AdaptDetections(AppSceneState& scene,
                       const sbirs_sensor::session::SbirsCycleResult& result);
  /// 探测帧 → 地面站（黑板或 on_sbirs_frame_submitted 消息）。
  void PublishToGroundStation(World& world, AppSceneState& scene,
                              const sbirs_sensor::session::SbirsCycleResult& result);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_SBIRS_SENSOR_COMPONENT_H_
