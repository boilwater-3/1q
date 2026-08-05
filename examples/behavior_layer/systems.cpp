/**
 * @file systems.cpp
 * @brief 行为层四系统实现。
 *
 * 每个系统为纯函数式推进：从组件/上下文读取输入，写入组件输出；
 * 会话与引擎经 BehaviorContext 访问，不直接持有会话状态。
 */

#include "systems.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "assembly.h"
#include "components.h"

namespace behavior_layer {

namespace {

/// 单周期时长（s），与 AR 会话周期语义一致。
constexpr double kDtSec = 1.0;

/// 决策门限：融合置信度达到该值视为高置信威胁（示例业务策略，非库内标准）。
/// 单源单周期最高贡献 1.0（verdict=1 × quality=1 × weight=1），窗口 10 → 上限 10.0。
constexpr double kHighThreatConfidence = 3.0;

/// 探测质量基准：无识别置信度（target_probability == 0）时按轨迹状态取基准值。
double BaseQualityForStatus(airborne_radar::session::TrackStatus status) {
  return status == airborne_radar::session::TrackStatus::kConfirmed ? 1.0 : 0.5;
}

/// 把 AR 外部轨迹帧适配为融合探测记录（key = association_key，跳过失落轨迹）。
std::vector<fusion::DetectionRecord> AdaptTracksToDetections(
    std::uint32_t source_id,
    const airborne_radar::session::ArExternalTrackOutputFrame& external_frame) {
  std::vector<fusion::DetectionRecord> detections;
  detections.reserve(external_frame.tracks.size());
  for (const auto& track : external_frame.tracks) {
    if (track.status == airborne_radar::session::TrackStatus::kLost) {
      continue;  // 失跟轨迹不入融合（避免以旧位置续命航迹）
    }
    fusion::DetectionRecord record;
    record.key = track.association_key;
    record.source_id = source_id;
    record.has_position = true;
    oneq::coordinate::LlaPositionDegM lla;
    if (oneq::coordinate::TryEcefToLla(track.target_position_ecef_m, &lla)) {
      record.position = lla;
    } else {
      record.has_position = false;  // 转换失败则退化为仅身份键记录
    }
    record.verdict = 1.0;  // 已发布的轨迹快照视为有效探测
    record.quality = track.target_probability > 0.0f
                         ? static_cast<double>(track.target_probability)
                         : BaseQualityForStatus(track.status);
    detections.push_back(record);
  }
  return detections;
}

/// 构建平台位姿输入：LLA → ECEF 位置；航向/速度 → ENU → ECEF 速度。
airborne_radar::session::ArExternalPoseInput MakePlatformPose(
    const FleetStatusComponent& fleet) {
  airborne_radar::session::ArExternalPoseInput pose;
  pose.platform_entity_id = fleet.platform_entity_id;
  oneq::coordinate::EcefPositionM ecef;
  if (oneq::coordinate::TryLlaToEcef(fleet.position, &ecef)) {
    pose.platform_position_ecef_m = ecef;
  }
  const double heading_rad = fleet.heading_deg * 3.14159265358979323846 / 180.0;
  oneq::coordinate::EnuVelocityMps enu_velocity;
  enu_velocity.east_mps = fleet.speed_mps * std::sin(heading_rad);
  enu_velocity.north_mps = fleet.speed_mps * std::cos(heading_rad);
  enu_velocity.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps ecef_velocity;
  if (oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, fleet.position, &ecef_velocity)) {
    pose.platform_velocity_mps = ecef_velocity;
  }
  return pose;
}

}  // namespace

void recon_system(entt::registry& registry) {
  auto& context = registry.ctx().get<BehaviorContext>();

  const auto view = registry.view<SensorObservationComponent, FleetStatusComponent>();
  for (const auto entity : view) {
    auto& sensor = view.get<SensorObservationComponent>(entity);
    const auto& fleet = view.get<FleetStatusComponent>(entity);

    airborne_radar::session::ArCycleInput input;
    input.cycle_index = static_cast<std::uint32_t>(context.cycle);
    input.cycle_start_time_s = static_cast<double>(context.cycle);
    input.dt_sec = kDtSec;
    input.platform = MakePlatformPose(fleet);
    input.targets = context.world_targets;  // 世界目标事实由消费方脚本注入

    context.last_ar_result = context.ar_session->StepWithResult(input);

    // 轨迹输出适配：雷达局部 → ECEF 由库内 ArCycleOutputAdapter 完成，再转度制 LLA。
    sensor.detections.clear();
    if (context.last_ar_result.status == airborne_radar::session::ArCycleStatus::kCompleted) {
      airborne_radar::session::ArExternalTrackOutputFrame external_frame;
      if (airborne_radar::session::ArCycleOutputAdapter::Build(
              input.platform, context.last_ar_result.track_output_frame, &external_frame)) {
        sensor.detections = AdaptTracksToDetections(sensor.source_id, external_frame);
      }
    }
  }

  // 融合：聚合全部传感器实体本周期探测，更新融合态势（含新/消失事件计数）。
  std::vector<fusion::DetectionRecord> all_detections;
  for (const auto entity : registry.view<SensorObservationComponent>()) {
    const auto& sensor = registry.get<SensorObservationComponent>(entity);
    all_detections.insert(all_detections.end(), sensor.detections.begin(), sensor.detections.end());
  }
  const std::vector<fusion::FusedTarget> fused =
      context.fusion_engine->Update(all_detections, context.cycle);

  for (const auto entity : registry.view<FusedSituationComponent>()) {
    auto situation = registry.get<FusedSituationComponent>(entity);
    // 新/消失目标按键集合差分（N 很小，O(N²) 即可；FusedTarget 按 key 升序）。
    std::size_t new_count = 0U;
    for (const auto& target : fused) {
      const auto it = std::find_if(situation.targets.begin(), situation.targets.end(),
                                   [&target](const fusion::FusedTarget& prev) {
                                     return prev.key == target.key;
                                   });
      if (it == situation.targets.end()) {
        ++new_count;
      }
    }
    std::size_t lost_count = 0U;
    for (const auto& prev : situation.targets) {
      const auto it = std::find_if(fused.begin(), fused.end(),
                                   [&prev](const fusion::FusedTarget& target) {
                                     return target.key == prev.key;
                                   });
      if (it == fused.end()) {
        ++lost_count;
      }
    }
    situation.targets = fused;
    situation.new_target_count = new_count;
    situation.lost_target_count = lost_count;
    registry.replace<FusedSituationComponent>(entity, situation);  // 触发观察者
  }
}

void maneuver_system(entt::registry& registry) {
  auto& context = registry.ctx().get<BehaviorContext>();

  const auto view = registry.view<TaskingComponent, RoutePlanComponent>();
  for (const auto entity : view) {
    const auto& tasking = view.get<TaskingComponent>(entity);
    if (tasking.role == Role::kWingman) {
      continue;  // 有上级：被动零计算，航路由长机写入
    }
    auto& route = view.get<RoutePlanComponent>(entity);
    if (!route.route.empty()) {
      continue;  // 已有计划：重规划触发（区域任务变化检测）属业务层策略，首期仅规划一次
    }
    const navigation::RoutePlan plan =
        context.planner.Plan(tasking.region, tasking.region_config);
    if (plan.empty()) {
      continue;  // 任务不合法（区域/参数非法）：保持空计划，由消费方告警
    }
    route.route = plan;
    ++route.version;
    // 长机：下发同一计划到各僚机（编队偏移与航段驱动属消费方职责）。
    if (tasking.role == Role::kLead) {
      for (const auto sub : tasking.subordinates) {
        if (registry.valid(sub) && registry.all_of<RoutePlanComponent>(sub)) {
          auto& sub_route = registry.get<RoutePlanComponent>(sub);
          sub_route.route = plan;
          ++sub_route.version;
        }
      }
    }
  }
}

void jam_system(entt::registry& registry) {
  auto& context = registry.ctx().get<BehaviorContext>();

  const auto view =
      registry.view<FleetStatusComponent, FusedSituationComponent, CommandFrameComponent>();
  for (const auto entity : view) {
    const auto& fleet = view.get<FleetStatusComponent>(entity);
    auto& command = view.get<CommandFrameComponent>(entity);

    // ECM 周期输入骨架：仅走 ECM 既有公共面（EcmCycleInput），无逐威胁 tasking SPI
    // （冻结契约 §5/§7）。ESR 会话接入后由 recon 适配其输出填充
    // sensor_observation_frame；无观测帧时 ECM 以默认技术调度。
    electronic_countermeasure::session::EcmCycleInput input;
    input.cycle_index = static_cast<std::uint32_t>(context.cycle);
    input.cycle_start_time_s = static_cast<double>(context.cycle);
    input.dt_sec = kDtSec;
    input.input_mode = electronic_countermeasure::EcmInputMode::kSensorDriven;
    input.platform_entity_id = fleet.platform_entity_id;
    oneq::coordinate::EcefPositionM ecef;
    if (oneq::coordinate::TryLlaToEcef(fleet.position, &ecef)) {
      input.platform_position_ecef_m = ecef;
    }
    // 编队状态只提供航向/速度标量；ECEF 速度向量与 ES 观测帧适配均属
    // 接入对应源的业务层职责（首期 AR 单域不填），保持帧结构完整。
    command.ecm_inputs.assign(1U, input);
  }
}

void decision_system(entt::registry& registry) {
  const auto view = registry.view<FusedSituationComponent, CommandFrameComponent>();
  for (const auto entity : view) {
    const auto& situation = view.get<FusedSituationComponent>(entity);
    auto& command = view.get<CommandFrameComponent>(entity);

    command.ar_commands.clear();
    command.has_external_decision = false;

    // 高置信威胁判定：融合置信度超过门限即下发 ECCM 反制指令（示例业务策略）。
    double max_confidence = 0.0;
    for (const auto& target : situation.targets) {
      max_confidence = std::max(max_confidence, target.confidence);
    }
    if (max_confidence >= kHighThreatConfidence) {
      command.ar_commands.push_back(airborne_radar::session::ArCommand(
          airborne_radar::session::ArCommandType::ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION,
          airborne_radar::session::ArCommandSource::ECCM));
      command.ar_commands.push_back(airborne_radar::session::ArCommand(
          airborne_radar::session::ArCommandType::SET_AGILITY_FREQ,
          airborne_radar::session::ArCommandSource::ECCM));
    }
    // external_decision 为预留接线位：示例策略不覆盖 AR 原生决策
    // （覆盖合法性由执行面 SubmitExternalDecision 校验，见 ArSession.h）。
  }
}

}  // namespace behavior_layer
