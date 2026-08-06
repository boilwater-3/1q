/**
 * @file ar_sensor_component.cpp
 * @brief AR 传感器组件实现（会话驱动 + 生命周期事件转发）。
 *
 * 驱动模式与行为层 recon_system::DriveArSession 同构：平台位姿经
 * host_ 读取 FlightComponent 状态计算 ECEF（零姿态：共享平台局部系），
 * ArCycleOutputAdapter 把雷达局部坐标轨迹帧转到外部 ECEF，再适配为
 * 泛型探测记录（examples/common/sensor_adapt.h）。轨迹生命周期事件
 * （首确认/失跟）由库内 ArTrackLifecycleRecorder 承担（Attach 后
 * StepWithResult 内部自动驱动，GetLastEvents 取本周期差分事件）：
 * kFirstConfirmed → TargetConfirmedEvent；kLost → TargetLostEvent。
 * 事件目标 ID 用外部原始目标标识（external_target_id），无外部标识时
 * 回退内部关联键；recorder 事件不含目标位置，位置从本周期外部帧按
 * 关联键回查。
 */

#include "ar_sensor_component.h"

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/coordinate/position_transform.h"
#include "core/events.h"
#include "flight_component.h"
#include "core/world.h"
#include "scene_types.h"
#include "sensor_adapt.h"
#include "sensor_utils.h"

namespace component_attachment {

ArSensorComponent::ArSensorComponent(airborne_radar::session::ArSession session)
    : session_(std::move(session)) {
  // 生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachTrackLifecycleRecorder(&lifecycle_);
}

void ArSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  // 周期输入：世界目标事实由消费方每周期写入共享场景状态。
  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());
  airborne_radar::session::ArCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.cycle_start_time_s = scene.t_sec;
  input.dt_sec = dt_sec;
  airborne_radar::session::ArExternalPoseInput pose;
  pose.platform_entity_id = 1U;  // 平台实体标识（本示例单平台）
  ResolvePlatformEcef(flight->position(), flight->heading_deg(), flight->speed_mps(),
                      &pose.platform_position_ecef_m, &pose.platform_velocity_mps);
  input.platform = pose;
  input.targets = scene.ar_targets;

  const airborne_radar::session::ArCycleResult result = session_.StepWithResult(input);
  if (result.status != airborne_radar::session::ArCycleStatus::kCompleted) {
    return;  // 周期被拒绝/电源关闭：本周期无探测
  }

  // 事件转发与探测适配统一用外部轨迹帧（雷达局部坐标 → ECEF 已在
  // ArCycleOutputAdapter 边界转换；内部帧为 TrackStateSnapshot，无 ECEF）。
  airborne_radar::session::ArExternalTrackOutputFrame external_frame;
  if (!airborne_radar::session::ArCycleOutputAdapter::Build(input.platform,
                                                            result.track_output_frame,
                                                            &external_frame)) {
    return;  // 坐标适配失败：本周期无探测
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（首确认/失跟），组件
  // 仅做事件映射转发（kUpdated 不转发——示例 AR 信号只关心边界事件）；
  // 掉轨后重捕自动重新产生 kFirstConfirmed。
  for (const auto& event : lifecycle_.GetLastEvents()) {
    // 事件目标 ID：优先外部原始目标标识（1001/1002），无则回退内部关联键。
    const std::uint64_t event_target_id =
        event.external_target_id != 0U ? event.external_target_id : event.association_key;

    if (event.kind == airborne_radar::session::ArTrackLifecycleEventKind::kFirstConfirmed) {
      TargetConfirmedEvent confirmed;
      confirmed.cycle = scene.cycle;
      confirmed.target_id = event_target_id;
      // recorder 事件无位置字段：从本周期帧内同关联键轨迹取回（事件来自
      // 本周期帧，键必命中）。
      for (const auto& track : external_frame.tracks) {
        if (track.association_key == event.association_key) {
          oneq::coordinate::LlaPositionDegM lla;
          if (oneq::coordinate::TryEcefToLla(track.target_position_ecef_m, &lla)) {
            confirmed.position = lla;
          }
          break;
        }
      }
      world.signals().on_target_confirmed(confirmed);
    } else if (event.kind == airborne_radar::session::ArTrackLifecycleEventKind::kLost) {
      TargetLostEvent lost;
      lost.cycle = scene.cycle;
      lost.target_id = event_target_id;
      lost.reason = "track_lost";  // recorder 的 kLost 事件 reason 恒为 kNone
      world.signals().on_target_lost(lost);
    }
  }

  detections_ = examples::sensor_adapt::AdaptTracksToDetections(
      examples::sensor_adapt::kArSourceId, external_frame);
}

}  // namespace component_attachment
