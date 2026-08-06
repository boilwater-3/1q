/**
 * @file ar_sensor_component.cpp
 * @brief AR 传感器组件实现（会话驱动 + 事件判定）。
 *
 * 驱动模式与行为层 recon_system::DriveArSession 同构：平台位姿经
 * host_ 读取 FlightComponent 状态计算 ECEF（零姿态：共享平台局部系），
 * ArCycleOutputAdapter 把雷达局部坐标轨迹帧转到外部 ECEF，再适配为
 * 泛型探测记录（examples/common/sensor_adapt.h）。事件判定基于轨迹状态
 * 迁移：kConfirmed 首次出现 → TargetConfirmedEvent；kLost 且此前已确认
 * → TargetLostEvent。事件目标 ID 用外部原始目标标识（external_target_id），
 * 无外部标识时回退内部关联键。
 */

#include "ar_sensor_component.h"

#include <algorithm>

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
    : session_(std::move(session)) {}

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

  // 事件判定与探测适配统一用外部轨迹帧（雷达局部坐标 → ECEF 已在
  // ArCycleOutputAdapter 边界转换；内部帧为 TrackStateSnapshot，无 ECEF）。
  airborne_radar::session::ArExternalTrackOutputFrame external_frame;
  if (!airborne_radar::session::ArCycleOutputAdapter::Build(input.platform,
                                                            result.track_output_frame,
                                                            &external_frame)) {
    return;  // 坐标适配失败：本周期无探测
  }

  // 本周期帧内见过的关联键（帧结束后用于裁剪事件判定集合）。
  std::vector<std::uint64_t> frame_keys;
  frame_keys.reserve(external_frame.tracks.size());
  for (const auto& track : external_frame.tracks) {
    frame_keys.push_back(track.association_key);
    const bool was_confirmed =
        std::find(confirmed_keys_.begin(), confirmed_keys_.end(), track.association_key) !=
        confirmed_keys_.end();
    const bool was_reported_lost =
        std::find(lost_keys_.begin(), lost_keys_.end(), track.association_key) !=
        lost_keys_.end();
    // 事件目标 ID：优先外部原始目标标识（1001/1002），无则回退内部关联键。
    const std::uint64_t event_target_id =
        track.external_target_id != 0U ? track.external_target_id : track.association_key;

    if (track.status == airborne_radar::session::TrackStatus::kConfirmed && !was_confirmed) {
      TargetConfirmedEvent event;
      event.cycle = scene.cycle;
      event.target_id = event_target_id;
      oneq::coordinate::LlaPositionDegM lla;
      if (oneq::coordinate::TryEcefToLla(track.target_position_ecef_m, &lla)) {
        event.position = lla;
      }
      confirmed_keys_.push_back(track.association_key);
      lost_keys_.erase(std::remove(lost_keys_.begin(), lost_keys_.end(), track.association_key),
                       lost_keys_.end());  // 重捕后允许再次失跟报告
      world.signals().on_target_confirmed(event);
    } else if (track.status == airborne_radar::session::TrackStatus::kLost && was_confirmed &&
               !was_reported_lost) {
      TargetLostEvent event;
      event.cycle = scene.cycle;
      event.target_id = event_target_id;
      event.reason = "track_lost";
      confirmed_keys_.erase(
          std::remove(confirmed_keys_.begin(), confirmed_keys_.end(), track.association_key),
          confirmed_keys_.end());
      lost_keys_.push_back(track.association_key);
      world.signals().on_target_lost(event);
    }
  }

  // 帧结束：按本周期见过的键裁剪判定集合——静默掉轨（无 kLost 输出）的键
  // 不滞留，避免长时运行集合无界增长与掉轨后重捕的漏报。
  const auto prune_keys = [&frame_keys](std::vector<std::uint64_t>& keys) {
    keys.erase(std::remove_if(keys.begin(), keys.end(),
                              [&frame_keys](std::uint64_t key) {
                                return std::find(frame_keys.begin(), frame_keys.end(), key) ==
                                       frame_keys.end();
                              }),
               keys.end());
  };
  prune_keys(confirmed_keys_);
  prune_keys(lost_keys_);

  detections_ = examples::sensor_adapt::AdaptTracksToDetections(
      examples::sensor_adapt::kArSourceId, external_frame);
}

}  // namespace component_attachment
