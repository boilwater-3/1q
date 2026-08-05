/**
 * @file systems.cpp
 * @brief 行为层四系统实现。
 *
 * 每个系统为纯函数式推进：从组件/上下文读取输入，写入组件输出；
 * 会话与引擎经 BehaviorContext 访问，不直接持有会话状态。
 *
 * 多传感器接入：recon_system 按实体 source_id 分派三会话（同周期同时间戳，
 * 多源时间对齐即业务层职责）；各会话输出在边界适配为泛型探测记录
 * （去真值化 → fusion::DetectionRecord），一次 Update 进入融合引擎。
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
#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/session/EmitterObservation.h"
#include "assembly.h"
#include "components.h"

namespace behavior_layer {

namespace {

/// 单周期时长（s），与三会话周期语义一致。
constexpr double kDtSec = 1.0;

/// 决策门限：融合置信度达到该值视为高置信威胁（示例业务策略，非库内标准）。
/// 单源单周期最高贡献 = 源权重（verdict=1 × quality=1），窗口 10。
constexpr double kHighThreatConfidence = 3.0;

/// 探测质量基准：无识别置信度（target_probability == 0）时按轨迹状态取基准值。
double BaseQualityForStatus(airborne_radar::session::TrackStatus status) {
  return status == airborne_radar::session::TrackStatus::kConfirmed ? 1.0 : 0.5;
}

/// ESR 观测质量枚举 → 威胁分数（业务层映射，供 ECM 输入帧）。
double QualityToThreatScore(electronic_surveillance_radar::session::EsrObservationQuality quality) {
  switch (quality) {
    case electronic_surveillance_radar::session::EsrObservationQuality::kHigh:
      return 0.9;
    case electronic_surveillance_radar::session::EsrObservationQuality::kMedium:
      return 0.6;
    default:
      return 0.3;  // kLow
  }
}

/// 平台 LLA/航向/速度 → ECEF 位置与速度（三会话共用；零姿态保持共享局部系）。
void ResolvePlatformEcef(const FleetStatusComponent& fleet,
                         oneq::coordinate::EcefPositionM* ecef_position,
                         oneq::coordinate::EcefVelocityMps* ecef_velocity) {
  oneq::coordinate::EcefPositionM ecef;
  if (oneq::coordinate::TryLlaToEcef(fleet.position, &ecef)) {
    *ecef_position = ecef;
  }
  const double heading_rad = fleet.heading_deg * 3.14159265358979323846 / 180.0;
  oneq::coordinate::EnuVelocityMps enu_velocity;
  enu_velocity.east_mps = fleet.speed_mps * std::sin(heading_rad);
  enu_velocity.north_mps = fleet.speed_mps * std::cos(heading_rad);
  enu_velocity.up_mps = 0.0;
  oneq::coordinate::EcefVelocityMps ecef_vel;
  if (oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, fleet.position, &ecef_vel)) {
    *ecef_velocity = ecef_vel;
  }
}

/// 构建 AR 平台位姿输入（零姿态：三会话共享同一平台局部坐标系）。
airborne_radar::session::ArExternalPoseInput MakePlatformPose(
    const FleetStatusComponent& fleet) {
  airborne_radar::session::ArExternalPoseInput pose;
  pose.platform_entity_id = fleet.platform_entity_id;
  oneq::coordinate::EcefPositionM ecef_position;
  oneq::coordinate::EcefVelocityMps ecef_velocity;
  ResolvePlatformEcef(fleet, &ecef_position, &ecef_velocity);
  pose.platform_position_ecef_m = ecef_position;
  pose.platform_velocity_mps = ecef_velocity;
  return pose;
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

/// 把 ESR 辐射源假设适配为融合探测记录（key=hypothesis_id，方位 + 归一化射频特征）。
std::vector<fusion::DetectionRecord> AdaptHypothesesToDetections(
    std::uint32_t source_id,
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses) {
  std::vector<fusion::DetectionRecord> detections;
  detections.reserve(hypotheses.size());
  for (const auto& hypothesis : hypotheses) {
    if (hypothesis.hypothesis_id == 0U) {
      continue;  // 库内键 0 = 无身份，不适用身份直挂
    }
    fusion::DetectionRecord record;
    record.key = hypothesis.hypothesis_id;
    record.source_id = source_id;
    record.has_bearing = true;
    record.bearing_az_deg = hypothesis.bearing_az_deg;
    record.bearing_el_deg = hypothesis.bearing_el_deg;
    // 射频特征归一化到可比尺度（GHz/MHz/ms/µs），供特征门限未来启用。
    record.feature = {hypothesis.estimated_center_frequency_hz / 1.0e9,
                      hypothesis.estimated_bandwidth_hz / 1.0e6,
                      hypothesis.estimated_pri_s * 1.0e3,
                      hypothesis.estimated_pulse_width_s * 1.0e6};
    record.verdict = 1.0;  // 已发布的辐射源假设视为有效探测
    record.quality = hypothesis.confidence;
    detections.push_back(record);
  }
  return detections;
}

/// 把 EOS 探测记录适配为融合探测记录（key=0 无身份，首期仅方位通道）。
std::vector<fusion::DetectionRecord> AdaptEosDetectionsToDetections(
    std::uint32_t source_id,
    const electro_optical_sensor::output::EosDetectionRecordList& records) {
  std::vector<fusion::DetectionRecord> detections;
  detections.reserve(records.size());
  for (const auto& record : records) {
    if (!record.detected) {
      continue;  // 未过探测门限不产生探测
    }
    fusion::DetectionRecord detection;
    detection.key = 0U;  // 无外部身份通道：走方位相干关联（去真值化纪律）
    detection.source_id = source_id;
    detection.has_bearing = true;
    detection.bearing_az_deg = record.azimuth_deg;
    detection.bearing_el_deg = record.elevation_deg;
    // range 通道首期不使用（保留方位相干关联路径）；
    // 质量 = 融合 SNR 归一化（10 dB → 1.0，业务层映射）。
    detection.verdict = 1.0;
    detection.quality =
        std::min(1.0, std::max(0.0, static_cast<double>(record.fused_snr_db) / 10.0));
    detections.push_back(detection);
  }
  return detections;
}

/// 驱动 AR 会话并适配轨迹输出为探测记录（source_id = kArSourceId）。
void DriveArSession(BehaviorContext& context, const FleetStatusComponent& fleet,
                    SensorObservationComponent* sensor) {
  airborne_radar::session::ArCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(context.cycle);
  input.cycle_start_time_s = static_cast<double>(context.cycle);
  input.dt_sec = kDtSec;
  input.platform = MakePlatformPose(fleet);
  input.targets = context.world_targets;  // 世界目标事实由消费方脚本注入

  context.last_ar_result = context.ar_session->StepWithResult(input);

  sensor->detections.clear();
  if (context.last_ar_result.status == airborne_radar::session::ArCycleStatus::kCompleted) {
    airborne_radar::session::ArExternalTrackOutputFrame external_frame;
    if (airborne_radar::session::ArCycleOutputAdapter::Build(
            input.platform, context.last_ar_result.track_output_frame, &external_frame)) {
      sensor->detections = AdaptTracksToDetections(sensor->source_id, external_frame);
    }
  }
}

/// 驱动 ESR 会话并适配辐射源假设为探测记录（source_id = kEsrSourceId）。
void DriveEsrSession(BehaviorContext& context, const FleetStatusComponent& fleet,
                     SensorObservationComponent* sensor) {
  electronic_surveillance_radar::session::EsrCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(context.cycle);
  input.cycle_start_time_s = static_cast<double>(context.cycle);
  input.dt_sec = static_cast<float>(kDtSec);
  input.platform_entity_id = fleet.platform_entity_id;
  input.has_platform_ecef_kinematics = true;
  oneq::coordinate::EcefPositionM ecef_position;
  oneq::coordinate::EcefVelocityMps ecef_velocity;
  ResolvePlatformEcef(fleet, &ecef_position, &ecef_velocity);
  input.platform_position_ecef_m = ecef_position;
  input.platform_velocity_ecef_mps = ecef_velocity;
  // RF 场景包络必须与本周期权威时间一致（含空帧亦须填齐）。
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  input.rf_emissions.emissions = context.emitter_truths;  // 辐射源真值由消费方脚本注入

  context.esr_last_result = context.esr_session->StepWithResult(input);

  sensor->detections.clear();
  if (context.esr_last_result.status ==
      electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
    sensor->detections = AdaptHypothesesToDetections(
        sensor->source_id, context.esr_last_result.output_frame.emitter_output.hypotheses);
  }
}

/// 驱动 EOS 会话并适配探测记录（source_id = kEosSourceId；首期仅方位通道）。
void DriveEosSession(BehaviorContext& context, const FleetStatusComponent& fleet,
                     SensorObservationComponent* sensor) {
  electro_optical_sensor::session::EosExternalPoseInput pose;
  oneq::coordinate::EcefPositionM ecef_position;
  oneq::coordinate::EcefVelocityMps ecef_velocity;
  ResolvePlatformEcef(fleet, &ecef_position, &ecef_velocity);
  pose.platform_position_ecef_m = ecef_position;
  pose.platform_velocity_mps = ecef_velocity;
  // 姿态零值：三会话共享同一平台局部坐标系，方位可直接相干比较。

  electro_optical_sensor::session::EosCycleInput input;
  electro_optical_sensor::session::EosCoordinateStatus status;
  if (!electro_optical_sensor::session::EosCycleInputAdapter::Build(
          pose, context.optical_targets, static_cast<float>(kDtSec), &input, &status)) {
    sensor->detections.clear();  // 坐标适配失败：本周期不产生探测
    return;
  }
  input.cycle_index = static_cast<std::uint32_t>(context.cycle);

  context.eos_last_result = context.eos_session->StepWithResult(input);

  sensor->detections.clear();
  if (context.eos_last_result.status ==
      electro_optical_sensor::session::EosCycleStatus::kCompleted) {
    sensor->detections = AdaptEosDetectionsToDetections(
        sensor->source_id, context.eos_last_result.output_frame.detections);
  }
}

}  // namespace

void recon_system(entt::registry& registry) {
  auto& context = registry.ctx().get<BehaviorContext>();

  // 分派：每传感器实体按其 source_id 驱动对应会话（同周期时间对齐）。
  const auto view = registry.view<SensorObservationComponent, FleetStatusComponent>();
  for (const auto entity : view) {
    auto& sensor = view.get<SensorObservationComponent>(entity);
    const auto& fleet = view.get<FleetStatusComponent>(entity);
    switch (sensor.source_id) {
      case kArSourceId:
        DriveArSession(context, fleet, &sensor);
        break;
      case kEsrSourceId:
        DriveEsrSession(context, fleet, &sensor);
        break;
      case kEosSourceId:
        DriveEosSession(context, fleet, &sensor);
        break;
      default:
        sensor.detections.clear();  // 未知源通道：不产生探测
        break;
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

    // ECM 周期输入：仅走 ECM 既有公共面（EcmCycleInput），无逐威胁 tasking SPI
    // （冻结契约 §5/§7）。ESR 成功周期发布去真值化观测帧时填充
    // sensor_observation_frame；无观测帧时 ECM 以默认技术调度。
    electronic_countermeasure::session::EcmCycleInput input;
    input.cycle_index = static_cast<std::uint32_t>(context.cycle);
    input.cycle_start_time_s = static_cast<double>(context.cycle);
    input.dt_sec = kDtSec;
    input.input_mode = electronic_countermeasure::EcmInputMode::kSensorDriven;
    input.platform_entity_id = fleet.platform_entity_id;
    oneq::coordinate::EcefPositionM ecef_position;
    oneq::coordinate::EcefVelocityMps ecef_velocity;
    ResolvePlatformEcef(fleet, &ecef_position, &ecef_velocity);
    input.platform_position_ecef_m = ecef_position;
    input.platform_velocity_ecef_mps = ecef_velocity;

    if (context.esr_last_result.status ==
        electronic_surveillance_radar::session::EsrCycleExecutionStatus::kCompleted) {
      const auto& frame = context.esr_last_result.output_frame;
      input.has_sensor_observation_frame = true;
      input.sensor_observation_frame.source_esr_batch_id = frame.batch_id;
      for (const auto& observation : frame.observation_output.observations) {
        electronic_countermeasure::session::EcmSensorObservation ecm_observation;
        ecm_observation.source_hypothesis_id = observation.observation_id;
        ecm_observation.estimated_center_frequency_hz = observation.rf_hz;
        ecm_observation.estimated_bandwidth_hz = observation.bandwidth_hz;
        ecm_observation.estimated_pri_s = observation.pri_s;
        ecm_observation.estimated_pulse_width_s = observation.pulse_width_s;
        ecm_observation.center_frequency_std_hz = observation.rf_std_hz;
        ecm_observation.bandwidth_std_hz = observation.bandwidth_std_hz;
        ecm_observation.bearing_az_deg = observation.aoa_az_deg;
        ecm_observation.bearing_el_deg = observation.aoa_el_deg;
        // 业务层映射：观测质量 → 威胁分数、SNR → 置信度（非库内标准）。
        ecm_observation.threat_score =
            static_cast<float>(QualityToThreatScore(observation.quality));
        ecm_observation.confidence =
            static_cast<float>(std::min(1.0, std::max(0.0, observation.snr_db / 20.0)));
        input.sensor_observation_frame.observations.push_back(ecm_observation);
      }
    }
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
