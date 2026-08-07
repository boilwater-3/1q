/**
 * @file ar_sensor_component.cpp
 * @brief AR 传感器组件实现（会话驱动 + 轨迹生命周期事件转发）。
 *
 * 1. 平台位姿经 host_ 读 FlightComponent（零姿态共享局部系），构建 ArCycleInput
 *    驱动 ArSession，输出适配为泛型探测记录（examples/common/sensor_adapt.h）；
 * 2. 轨迹事件（首确认/失跟）由库内 ArTrackLifecycleRecorder 差分产出，组件仅
 *    映射转发为 World 信号（事件位置从本周期外部帧按关联键回查）；
 * 3. 事件与调试视图直写集成端日志（CA_LOG_EVENT / CA_LOG_VIEW，中文人读行）。
 */

#include "ar_sensor_component.h"

#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/coordinate/position_transform.h"
#include "core/events.h"
#include "demo_log.h"
#include "flight_component.h"
#include "core/world.h"
#include "scene_types.h"
#include "sensor_adapt.h"
#include "sensor_utils.h"

namespace component_attachment {

namespace {

/// 调试轨迹状态 → 中文名（人读日志）。
const char* ArTrackStatusName(airborne_radar::session::ArDebugTrackStatus status) {
  switch (status) {
    case airborne_radar::session::ArDebugTrackStatus::kConfirmed:
      return "已确认";
    case airborne_radar::session::ArDebugTrackStatus::kTentative:
      return "候选";
    case airborne_radar::session::ArDebugTrackStatus::kLost:
      return "丢失";
    case airborne_radar::session::ArDebugTrackStatus::kNotInOutput:
      return "不在输出";
    case airborne_radar::session::ArDebugTrackStatus::kCycleNotCompleted:
      return "周期未完成";
  }
  return "未知";
}

}  // namespace

ArSensorComponent::ArSensorComponent(airborne_radar::session::ArSession session)
    : session_(std::move(session)) {
  // 轨迹生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachTrackLifecycleRecorder(&lifecycle_);
}

bool ArSensorComponent::TryApplyRuntimeConfig(
    const airborne_radar::config::ArRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    // 电源状态由补丁唯一维护（组件层电源门控）：AR 补丁为事务性暂存（会话下个
    // 成功周期边界生效），组件在指令接受时即反映开关机；关机期间组件不驱动
    // Step，暂存补丁不提交，标志与会话状态无失步。
    powered_on_ = patch.sensor_enabled;
  }
  return applied;
}

// 视图行写入（三模式，宏门控——未选中的模式不参与编译，见 demo_log.h 模式选择
// 区）。DebugView 每周期都构建，落多少、怎么落由集成方按需求选择。
void ArSensorComponent::LogDebugView(
    const airborne_radar::session::ArTrackOutputDebugView& view) {
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一（只落非标称行）：跳过已确认（标称）目标，日志量 ∝ 异常数。
  std::size_t non_nominal = 0U;
  for (const auto& track : view.tracks) {
    if (track.status == airborne_radar::session::ArDebugTrackStatus::kConfirmed) {
      continue;
    }
    ++non_nominal;
    CA_LOG_VIEW("ar", "周期={} 目标={} 状态={} 位置=({:.1f},{:.1f},{:.1f})m 速度={:.1f}m/s",
                view.world_cycle_index, track.external_target_id,
                ArTrackStatusName(track.status), track.position_x, track.position_y,
                track.position_z, track.speed);
  }
  if (non_nominal == 0U) {
    CA_LOG_VIEW("ar", "周期={} 全部正常（{} 个目标均已确认）", view.world_cycle_index,
                view.tracks.size());
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二（跨周期状态增量）：上一周期状态表由组件持有（external_target_id →
  // status），首次出现视为变化；表只增不减，目标集长期收缩时调用方可按需清理。
  std::size_t changed = 0U;
  for (const auto& track : view.tracks) {
    const auto it = prev_track_status_.find(track.external_target_id);
    if (it == prev_track_status_.end() || it->second != track.status) {
      ++changed;
      CA_LOG_VIEW("ar", "周期={} 目标={} 状态={}",
                  view.world_cycle_index, track.external_target_id,
                  ArTrackStatusName(track.status));
    }
    prev_track_status_[track.external_target_id] = track.status;
  }
  if (changed == 0U) {
    CA_LOG_VIEW("ar", "周期={} 无状态变化", view.world_cycle_index);
  }
#else  // CA_VIEW_LOG_MODE_SUMMARY（默认）
  // 模式三（每周期摘要行）：目标状态明细 + 问题 code 列表，一眼可读。
  std::string tracks_text;
  for (const auto& track : view.tracks) {
    if (!tracks_text.empty()) {
      tracks_text += ", ";
    }
    tracks_text += spdlog::fmt_lib::format("{} {}", track.external_target_id,
                                           ArTrackStatusName(track.status));
  }
  std::string issues_text;
  for (const auto& issue : view.issues) {
    if (!issues_text.empty()) {
      issues_text += ", ";
    }
    issues_text += issue.code;
  }
  CA_LOG_VIEW("ar", "周期={} 完成={} 目标=[{}] 问题=[{}]",
              view.world_cycle_index, view.completed_this_cycle ? "是" : "否",
              tracks_text.empty() ? "无" : tracks_text.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif  // CA_VIEW_LOG_MODE_*
}

void ArSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    last_debug_view_ = airborne_radar::session::ArTrackOutputDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_);
    return;  // 关机：组件不驱动会话（设备不工作），本周期无探测
  }

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
  // 规则 12 落盘示范：每周期构建调试视图快照（拒绝周期为 kCycleNotCompleted，
  // 含规则 13b kInfo 排除诊断），供调用方结构化持久化；本示例经 LogDebugView
  // 直写中文人读行（三模式由集成方按需选择）。
  last_debug_view_ = airborne_radar::session::ArTrackOutputDebugViewBuilder::Build(input.targets, result);
  LogDebugView(last_debug_view_);
  if (result.status != airborne_radar::session::ArCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }

  // 事件转发与探测适配统一用外部轨迹帧（雷达局部坐标 → ECEF 已在
  // ArCycleOutputAdapter 边界转换；内部帧为 TrackStateSnapshot，无 ECEF）。
  airborne_radar::session::ArExternalTrackOutputFrame external_frame;
  if (!airborne_radar::session::ArCycleOutputAdapter::Build(input.platform,
                                                            result.track_output_frame,
                                                            &external_frame)) {
    return;  // 坐标适配失败：本周期无探测
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（首确认/失跟），组件仅做
  // 映射转发（kUpdated 不转发——示例 AR 信号只关心边界事件）；掉轨后重捕自动
  // 重新产生 kFirstConfirmed。事件目标 ID 优先外部原始目标标识（1001/1002），
  // 无则回退内部关联键。
  for (const auto& event : lifecycle_.GetLastEvents()) {
    const std::uint64_t event_target_id =
        event.external_target_id != 0U ? event.external_target_id : event.association_key;

    if (event.kind == airborne_radar::session::ArTrackLifecycleEventKind::kFirstConfirmed) {
      TargetConfirmedEvent confirmed;
      confirmed.cycle = scene.cycle;
      confirmed.target_id = event_target_id;
      // recorder 事件无位置字段：从本周期帧内同关联键轨迹取回（事件来自本周期
      // 帧，键必命中）。
      for (const auto& track : external_frame.tracks) {
        if (track.association_key == event.association_key) {
          oneq::coordinate::LlaPositionDegM lla;
          if (oneq::coordinate::TryEcefToLla(track.target_position_ecef_m, &lla)) {
            confirmed.position = lla;
          }
          break;
        }
      }
      CA_LOG_EVENT(world, "target_confirmed", "目标={} 位置=({:.5f},{:.5f})",
                   static_cast<unsigned long long>(confirmed.target_id),
                   confirmed.position.latitude_deg, confirmed.position.longitude_deg);
      world.signals().on_target_confirmed(confirmed);
    } else if (event.kind == airborne_radar::session::ArTrackLifecycleEventKind::kLost) {
      TargetLostEvent lost;
      lost.cycle = scene.cycle;
      lost.target_id = event_target_id;
      lost.reason = "track_lost";  // recorder 的 kLost 事件 reason 恒为 kNone
      CA_LOG_EVENT(world, "target_lost", "目标={} 原因=失跟",
                   static_cast<unsigned long long>(lost.target_id));
      world.signals().on_target_lost(lost);
    }
  }

  detections_ = examples::sensor_adapt::AdaptTracksToDetections(
      examples::sensor_adapt::kArSourceId, external_frame);
}

}  // namespace component_attachment
