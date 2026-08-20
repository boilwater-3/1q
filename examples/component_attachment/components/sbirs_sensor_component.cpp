/**
 * @file sbirs_sensor_component.cpp
 * @brief SBIRS 传感器组件实现（会话驱动 + 探测生命周期事件转发）。
 *
 * 1. 从共享场景状态读取卫星 ECEF 位置与红外目标真值，构建 SbirsCycleInput
 *    驱动 SbirsSession（天基平台方位参考系与机载通道不同，见 README 简化声明）；
 * 2. 探测事件（首发现/更新/coasting/丢失）由库内 SbirsDetectionLifecycleRecorder
 *    差分产出，组件经归属映射关联目标 ID 后发布 SbirsDetectionEvent；
 * 3. 事件与调试视图直写集成端日志（CA_LOG_EVENT / CA_LOG_VIEW，中文人读行）。
 */

#include "sbirs_sensor_component.h"

#include "1q/fusion/SensorAdapters.h"
#include "core/events.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "flight_component.h"
#include "scene_types.h"

namespace component_attachment {

namespace {

// 库内角度换算工具（common/numerics/Constants.h）随 src/ PRIVATE 包含，示例层不可达；
// 以本地命名常量替代内联魔法数（库内 rad 输出 → 示例 deg 显示的唯一换算点）。
constexpr float kRadToDeg = 57.29577951308232f;

/// 生命周期事件类型 → 示例事件类型（kNotDetected 诊断事件不转发）。
SbirsDetectionEventKind ToDemoKind(
    sbirs_sensor::session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return SbirsDetectionEventKind::kFirstDetected;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kUpdated:
      return SbirsDetectionEventKind::kUpdated;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kCoasting:
      return SbirsDetectionEventKind::kCoasting;
    case sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost:
      return SbirsDetectionEventKind::kLost;
    default:
      break;  // kNotDetected：组件未开启诊断事件，调用方显式跳过，不会到达
  }
  return SbirsDetectionEventKind::kUpdated;
}

/// 示例事件类型 → 中文名（人读日志）。
const char* SbirsEventKindName(SbirsDetectionEventKind kind) {
  switch (kind) {
    case SbirsDetectionEventKind::kFirstDetected:
      return "首发现";
    case SbirsDetectionEventKind::kUpdated:
      return "更新";
    case SbirsDetectionEventKind::kCoasting:
      return "跟踪锁定";
    case SbirsDetectionEventKind::kLost:
      return "丢失";
  }
  return "未知";
}

/// 库内丢失原因 → 示例事件原因（细分透出：区分目标真消失/扫描间隙/调度跳过/
/// 门失败，避免消费方把扫描间隙误读为目标丢失）。
SbirsDetectionLossReason ToDemoLossReason(
    sbirs_sensor::session::SbirsDetectionLifecycleReason reason) {
  switch (reason) {
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kOutOfFieldOfView:
      return SbirsDetectionLossReason::kOutOfFieldOfView;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kBelowSnrThreshold:
      return SbirsDetectionLossReason::kBelowSnrThreshold;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kTargetMissingFromInput:
      return SbirsDetectionLossReason::kTargetMissingFromInput;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovAcquisitionFailed:
      return SbirsDetectionLossReason::kAcquisitionFailed;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kSchedulerSkipped:
      return SbirsDetectionLossReason::kSchedulerSkipped;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kEstimationNisGateLost:
      return SbirsDetectionLossReason::kNisGateLost;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovPointingTimeout:
      return SbirsDetectionLossReason::kPointingTimeout;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNfovTrackingGateLost:
      return SbirsDetectionLossReason::kTrackingGateLost;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kUnknown:
      return SbirsDetectionLossReason::kUnknown;
    case sbirs_sensor::session::SbirsDetectionLifecycleReason::kNone:
      break;
  }
  return SbirsDetectionLossReason::kNone;
}

/// 丢失原因 → 中文名（人读日志）。
const char* LossReasonName(SbirsDetectionLossReason reason) {
  switch (reason) {
    case SbirsDetectionLossReason::kOutOfFieldOfView:
      return "视场外";
    case SbirsDetectionLossReason::kBelowSnrThreshold:
      return "低于信噪比门限";
    case SbirsDetectionLossReason::kTargetMissingFromInput:
      return "目标消失";
    case SbirsDetectionLossReason::kAcquisitionFailed:
      return "捕获失败";
    case SbirsDetectionLossReason::kSchedulerSkipped:
      return "调度跳过";
    case SbirsDetectionLossReason::kNisGateLost:
      return "NIS 超限丢锁";
    case SbirsDetectionLossReason::kPointingTimeout:
      return "指向超时";
    case SbirsDetectionLossReason::kTrackingGateLost:
      return "跟踪门连续失败";
    case SbirsDetectionLossReason::kUnknown:
      return "未知";
    case SbirsDetectionLossReason::kNone:
      break;
  }
  return "无";
}

/// 检测记录 ID → 仿真目标 ID（无归属返回 0）。
std::uint64_t AttributionTargetId(
    std::uint64_t detection_id,
    const sbirs_sensor::attribution::SbirsDetectionAttributionRecordList& attributions) {
  for (const auto& attribution : attributions) {
    if (attribution.detection_id == detection_id) {
      return attribution.target_id;
    }
  }
  return 0U;  // 无归属（调试视图缺失）：未知目标
}

/// 调试目标状态 → 中文名（人读日志）。
const char* SbirsTargetStatusName(sbirs_sensor::session::SbirsDebugTargetStatus status) {
  switch (status) {
    case sbirs_sensor::session::SbirsDebugTargetStatus::kDetected:
      return "已检测";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kObservedBelowThreshold:
      return "低于门限";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCoasting:
      return "跟踪锁定";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kNotInOutput:
      return "不在输出";
    case sbirs_sensor::session::SbirsDebugTargetStatus::kCycleNotExecuted:
      return "周期未执行";
  }
  return "未知";
}

/// 排除原因门内归因 → 中文名（人读日志，规则 13b 门内归因条款）。
const char* SbirsExclusionCauseName(sbirs_sensor::session::SbirsIssueCause cause) {
  switch (cause) {
    case sbirs_sensor::session::SbirsIssueCause::kNone:
      return "无归因";
    case sbirs_sensor::session::SbirsIssueCause::kAzOutside:
      return "方位越界";
    case sbirs_sensor::session::SbirsIssueCause::kElOutside:
      return "俯仰越界";
    case sbirs_sensor::session::SbirsIssueCause::kBothAxesOutside:
      return "双轴越界";
    case sbirs_sensor::session::SbirsIssueCause::kDistanceLimited:
      return "距离受限";
    case sbirs_sensor::session::SbirsIssueCause::kAttenuationLimited:
      return "大气衰减";
    case sbirs_sensor::session::SbirsIssueCause::kSignatureLimited:
      return "签名受限";
    case sbirs_sensor::session::SbirsIssueCause::kNoiseLimited:
      return "噪声底受限";
    case sbirs_sensor::session::SbirsIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

}  // namespace

SbirsSensorComponent::SbirsSensorComponent(sbirs_sensor::session::SbirsSession session)
    : session_(std::move(session)) {
  // 探测生命周期事件由库内 recorder 承担（StepWithResult 内部自动喂）。
  session_.AttachDetectionLifecycleRecorder(&lifecycle_);
  // 排除原因跨周期差分事件由库内 recorder 承担（与 lifecycle recorder 独立并列）。
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

bool SbirsSensorComponent::TryApplyRuntimeConfig(
    const sbirs_sensor::config::SbirsRuntimeConfigPatch& patch) {
  const bool applied = session_.TryApplyRuntimeConfig(patch);
  if (applied && patch.has_sensor_enabled) {
    powered_on_ = patch.sensor_enabled;  // 电源状态由补丁唯一维护（组件层电源门控）
  }
  return applied;
}

// 视图行写入（三模式，宏门控——未选中的模式不参与编译，见 logger/logger.h 模式选择
// 区）。DebugView 每周期都构建，落多少、怎么落由集成方按需求选择。
void SbirsSensorComponent::LogDebugView(
    const sbirs_sensor::session::SbirsOutputDebugView& view) {
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一（只落非标称行）：跳过已检测（标称）目标，日志量 ∝ 异常数。
  std::size_t non_nominal = 0U;
  for (const auto& target : view.targets) {
    if (target.status == sbirs_sensor::session::SbirsDebugTargetStatus::kDetected) {
      continue;
    }
    ++non_nominal;
    // 2026-08 正式变更：库内方位/俯仰输出为 ECI 极坐标弧度；示例按可读性转
    // 度显示。距离仅存在于库内诊断字段（estimated_range_m，仅归属目标有值），
    // 非标称行不再展示距离（被动红外测距无物理依据，见 docs/common/contract.md）。
    CA_LOG_VIEW("sbirs", "周期={} 目标={} 状态={} 方位(ECI)={:.1f}° 仰角(ECI)={:.1f}°",
                view.input_cycle_index, target.target_id,
                SbirsTargetStatusName(target.status), target.azimuth_rad * kRadToDeg,
                target.elevation_rad * kRadToDeg);
  }
  if (non_nominal == 0U) {
    CA_LOG_VIEW("sbirs", "周期={} 全部正常（{} 个目标均已检测）", view.input_cycle_index,
                view.targets.size());
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二（跨周期状态增量）：上一周期状态表由组件持有（target_id → status），
  // 首次出现视为变化；表只增不减，目标集长期收缩时调用方可按需清理。
  std::size_t changed = 0U;
  for (const auto& target : view.targets) {
    const auto it = prev_target_status_.find(target.target_id);
    if (it == prev_target_status_.end() || it->second != target.status) {
      ++changed;
      CA_LOG_VIEW("sbirs", "周期={} 目标={} 状态={}",
                  view.input_cycle_index, target.target_id,
                  SbirsTargetStatusName(target.status));
    }
    prev_target_status_[target.target_id] = target.status;
  }
  if (changed == 0U) {
    CA_LOG_VIEW("sbirs", "周期={} 无状态变化", view.input_cycle_index);
  }
#else  // CA_VIEW_LOG_MODE_SUMMARY（默认）
  // 模式三（每周期摘要行）：目标状态明细带结构化量值（input 回填，未检测也
  // 可见目标角度）+ 问题中文名，一眼可读。
  std::string targets_text;
  for (const auto& target : view.targets) {
    if (!targets_text.empty()) {
      targets_text += ", ";
    }
    targets_text += CA_FMT_FORMAT("{} {}(方位{:.1f}° 俯仰{:.1f}°)",
                                  target.target_id, SbirsTargetStatusName(target.status),
                                  target.azimuth_rad * kRadToDeg,
                                  target.elevation_rad * kRadToDeg);
  }
  const std::string issues_text = demo::FormatIssueText(view.issues);
  CA_LOG_VIEW("sbirs", "周期={} 执行={} 目标=[{}] 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              targets_text.empty() ? "无" : targets_text.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif  // CA_VIEW_LOG_MODE_*
}

void SbirsSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    scan_azimuth_deg_ = 0.0f;  // 关机：不驱动会话，角度无有效值（清零）
    last_debug_view_ = sbirs_sensor::session::SbirsOutputDebugView{};  // 关机：调试视图清零（无有效周期）
    LogDebugView(last_debug_view_);
    return;
  }

  const FlightComponent* flight = host_ != nullptr ? host_->Find<FlightComponent>() : nullptr;
  if (flight == nullptr) {
    return;  // 无平台动力学（空场景）：不产生探测
  }

  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());

  // 天基平台（卫星）位置/速度由消费方每周期注入共享场景状态（世界模型驱动）。
  sbirs_sensor::session::SbirsCycleInput input;
  input.cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.dt_sec = static_cast<float>(dt_sec);
  input.has_satellite_position = true;
  input.satellite_position_ecef_m = scene.sbirs_satellite_position_ecef_m;
  input.has_satellite_velocity_ecef_m_per_s = true;
  input.satellite_velocity_ecef_m_per_s = scene.sbirs_satellite_velocity_ecef_m_per_s;
  input.has_satellite_attitude = true;
  input.satellite_attitude_eci_body_deg = scene.sbirs_satellite_attitude_eci_body_deg;
  input.utc_julian_day = scene.sbirs_utc_julian_day;  // ECI 输出参考系（UTC 儒略日）
  input.scene = scene.sbirs_targets;

  const sbirs_sensor::session::SbirsCycleResult result = session_.StepWithResult(input);
  // 扫描方位随周期结果刷新（拒绝周期为空帧 → 0）；库内为 ECI 弧度，组件转度显示。
  scan_azimuth_deg_ = result.output_frame.scan_azimuth_rad * kRadToDeg;
  // 规则 12 落盘示范：每周期构建调试视图快照（拒绝周期为 kCycleNotExecuted，
  // 含规则 13b kInfo 排除诊断），供调用方结构化持久化；本示例经 LogDebugView
  // 直写中文人读行（三模式由集成方按需选择）。
  last_debug_view_ = sbirs_sensor::session::SbirsOutputDebugViewBuilder::Build(input, result);
  LogDebugView(last_debug_view_);
  if (result.status != sbirs_sensor::session::SbirsCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无探测
  }

  // 库内 recorder 已按跨周期状态差分产出本周期事件（首发现/更新/coasting/丢失）。
  // recorder 事件无方位/探测 ID 字段：非丢失事件从本周期输出帧按归属目标回查；
  // 丢失事件目标不在帧内，字段留默认。kNotDetected 诊断事件未开启，显式跳过。
  const auto& records = result.output_frame.detections;
  for (const auto& event : lifecycle_.GetLastEvents()) {
    if (event.kind == sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kNotDetected) {
      continue;  // 诊断事件（未开启 emit_not_detected_events）：不发布
    }
    SbirsDetectionEvent sbirs_event;
    sbirs_event.cycle = scene.cycle;
    sbirs_event.kind = ToDemoKind(event.kind);
    sbirs_event.reason = ToDemoLossReason(event.reason);
    sbirs_event.target_id = event.target_id;
    sbirs_event.infrared_snr_linear = event.infrared_snr_linear;
    if (event.kind != sbirs_sensor::session::SbirsDetectionLifecycleEventKind::kLost) {
      for (const auto& record : records) {
        if (!record.detected) {
          continue;  // 未过探测门限不参与回查
        }
        if (AttributionTargetId(record.detection_id, result.detection_attributions) ==
            event.target_id) {
          sbirs_event.detection_id = record.detection_id;
          sbirs_event.az_deg = record.azimuth_rad * kRadToDeg;  // ECI rad → 度（显示用）
          break;
        }
      }
    }
    if (sbirs_event.kind == SbirsDetectionEventKind::kUpdated) {
      // 更新类事件每周期重复：事件模式一下不落盘（信号照常发布）。
      CA_LOG_EVENT_DUP(world, "sbirs_detection",
                       "类型={} 探测ID={} 目标={} 信噪比(线性)={:.1f} 方位={:.1f}°",
                       SbirsEventKindName(sbirs_event.kind),
                       static_cast<unsigned long long>(sbirs_event.detection_id),
                       static_cast<unsigned long long>(sbirs_event.target_id),
                       sbirs_event.infrared_snr_linear, sbirs_event.az_deg);
    } else if (sbirs_event.kind == SbirsDetectionEventKind::kLost) {
      // 丢失事件携带细分原因（视场外/调度跳过/门失败…），区分扫描间隙与真丢失。
      CA_LOG_EVENT(world, "sbirs_detection",
                   "类型=丢失({}) 探测ID={} 目标={} 信噪比(线性)={:.1f} 方位={:.1f}°",
                   LossReasonName(sbirs_event.reason),
                   static_cast<unsigned long long>(sbirs_event.detection_id),
                   static_cast<unsigned long long>(sbirs_event.target_id),
                   sbirs_event.infrared_snr_linear, sbirs_event.az_deg);
    } else {
      CA_LOG_EVENT(world, "sbirs_detection",
                   "类型={} 探测ID={} 目标={} 信噪比(线性)={:.1f} 方位={:.1f}°",
                   SbirsEventKindName(sbirs_event.kind),
                   static_cast<unsigned long long>(sbirs_event.detection_id),
                   static_cast<unsigned long long>(sbirs_event.target_id),
                   sbirs_event.infrared_snr_linear, sbirs_event.az_deg);
    }
    world.signals().on_sbirs_detection(sbirs_event);
  }

  // 排除原因跨周期差分事件（规则 13e）：纯诊断观测，仅落事件日志（不发 World
  // 信号——不驱动融合/威胁等下游组件）。SBIRS 排除涵盖遮挡/距离带/视场/SNR 四门，
  // 差分键为 (code,cause) 组合对——遮挡↔距离带切换（同为 kNone、code 不同）亦产事件。
  for (const auto& event : exclusion_.GetLastEvents()) {
    const std::uint64_t event_target_id = event.target_id;
    if (event.kind == sbirs_sensor::session::SbirsExclusionCauseEventKind::kEntered) {
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=进入排除 排除码={} 主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.current_code, SbirsExclusionCauseName(event.current_cause));
    } else if (event.kind == sbirs_sensor::session::SbirsExclusionCauseEventKind::kChanged) {
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, SbirsExclusionCauseName(event.previous_cause),
                   event.current_code, SbirsExclusionCauseName(event.current_cause));
    } else if (event.kind == sbirs_sensor::session::SbirsExclusionCauseEventKind::kExited) {
      CA_LOG_EVENT(world, "exclusion_cause",
                   "目标={} 类型=退出排除 旧码={} 旧主因={}",
                   static_cast<unsigned long long>(event_target_id),
                   event.previous_code, SbirsExclusionCauseName(event.previous_cause));
    }
  }

  detections_ = fusion::AdaptSbirsDetectionsToDetectionRecords(
      fusion::kSbirsSourceId, records);
}

}  // namespace component_attachment
