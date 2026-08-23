/**
 * @file rir_sensor_component.cpp
 * @brief RIR 地基站点传感器组件实现（会话驱动 + 识别/指定任务事件 + 融合量测）。
 *
 * 1. RirCycleInput 直接构造（固定站点海拔/ECEF + 消费方注入的站点局部 ENU
 *    场景目标帧），驱动 RirSession；指定识别任务经运行期补丁接口下发
 *    （外部指令事件 → CommandRouter → TryApplyRuntimeConfig；识别完成或
 *    窗口耗尽由会话状态机自动回扫）；
 * 2. 识别结论进入确认态时发关键事件（逐航迹状态迁移判定，避免每周期重复）；
 *    指定任务回扫沿发事件；每周期视图行汇总归属航迹/驻留中心；
 * 3. 特征量测（出口①）适配为泛型探测记录（fusion::AdaptRirFeatureMeasurements-
 *    ToDetectionRecords，源通道 kRirSourceId=5），FusionComponent 跨实体聚合。
 */

#include "rir_sensor_component.h"

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "1q/coordinate/position_transform.h"
#include "1q/fusion/SensorAdapters.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "logger/acceptance_timing.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/logger_i18n.h"
#include "core/rf_world_broker.h"
#include "core/scene_types.h"
#include "components/sensor_utils.h"

namespace component_attachment {

namespace {

namespace rir = remote_identification_radar::session;

/// 识别状态 → 中文名（人读事件行）。
const char* RecognitionStateName(rir::RirRecognitionState state) {
  switch (state) {
    case rir::RirRecognitionState::kDisabled:
      return "未启用";
    case rir::RirRecognitionState::kAccumulating:
      return "积累中";
    case rir::RirRecognitionState::kCategoryConfirmed:
      return "大类确认";
    case rir::RirRecognitionState::kModelConfirmed:
      return "型号确认";
    case rir::RirRecognitionState::kUnknown:
      return "无候选";
    case rir::RirRecognitionState::kStale:
      return "结论过期";
  }
  return "未知";
}

/// 识别大类 → 中文名（人读事件行）。
const char* RecognitionCategoryName(rir::RirRecognitionCategory category) {
  switch (category) {
    case rir::RirRecognitionCategory::kBallistic:
      return "弹道目标";
    case rir::RirRecognitionCategory::kNearSpace:
      return "临近空间";
    case rir::RirRecognitionCategory::kOther:
      return "其它";
    case rir::RirRecognitionCategory::kUnknown:
      return "未知";
    case rir::RirRecognitionCategory::kFighter:
      return "战斗机";
    case rir::RirRecognitionCategory::kBomber:
      return "轰炸机";
    case rir::RirRecognitionCategory::kMissile:
      return "导弹";
  }
  return "未知";
}

/// 指定任务回退成因 → 中文名（人读事件行）。
const char* DesignationRevertReasonName(
    remote_identification_radar::session::RirDesignationRevertReason reason) {
  switch (reason) {
    case remote_identification_radar::session::RirDesignationRevertReason::kNone:
      return "无";
    case remote_identification_radar::session::RirDesignationRevertReason::kNotRecognized:
      return "目标缺席";
    case remote_identification_radar::session::RirDesignationRevertReason::kAcquisitionTimeout:
      return "窗口超时";
    case remote_identification_radar::session::RirDesignationRevertReason::kOutsideSteerableVolume:
      return "超出可扫描体积";
  }
  return "未知";
}

/// 调试目标状态 → 中文名（人读视图行）。
const char* DebugTargetStatusName(rir::RirDebugTargetStatus status) {
  switch (status) {
    case rir::RirDebugTargetStatus::kConfirmed:
      return "已确认";
    case rir::RirDebugTargetStatus::kTentative:
      return "候选";
    case rir::RirDebugTargetStatus::kLost:
      return "丢失";
    case rir::RirDebugTargetStatus::kNotInOutput:
      return "无航迹";
    case rir::RirDebugTargetStatus::kCycleNotCompleted:
      return "周期未完成";
  }
  return "未知";
}

/// 排除主因 → 中文名（人读事件行，规则 13b/13e）。
const char* ExclusionCauseName(rir::RirIssueCause cause) {
  switch (cause) {
    case rir::RirIssueCause::kNone:
      return "无归因";
    case rir::RirIssueCause::kDistanceLimited:
      return "距离受限";
    case rir::RirIssueCause::kBeamLimited:
      return "波束偏轴";
    case rir::RirIssueCause::kNoiseLimited:
      return "噪声底受限";
    case rir::RirIssueCause::kRcsLimited:
      return "RCS受限";
    case rir::RirIssueCause::kUnknown:
      return "未知主因";
  }
  return "未知主因";
}

/// 航迹生命周期状态 → 中文名（人读事件行）。
const char* TrackStatusName(rir::RirTrackLifecycleStatus status) {
  switch (status) {
    case rir::RirTrackLifecycleStatus::kConfirmed:
      return "已确认";
    case rir::RirTrackLifecycleStatus::kTentative:
      return "候选";
    case rir::RirTrackLifecycleStatus::kLost:
      return "丢失";
  }
  return "未知";
}

}  // namespace

RirSensorComponent::RirSensorComponent(
    rir::RirSession session, const oneq::coordinate::LlaPositionDegM& site_origin,
    std::uint64_t sensor_platform_id, float recognition_dwell_sec)
    : session_(std::move(session)),
      site_origin_(site_origin),
      sensor_platform_id_(sensor_platform_id),
      recognition_dwell_sec_(recognition_dwell_sec) {
  // 站点固定：ECEF 解析一次，逐周期作为特征量测 sensor_origin 提供。
  oneq::coordinate::TryLlaToEcef(site_origin_, &site_ecef_);
  // 观测投影记录器（规则 10/11）：StepWithResult 内部自动喂，组件只读事件。
  session_.AttachTrackLifecycleRecorder(&lifecycle_);
  session_.AttachExclusionCauseRecorder(&exclusion_);
}

bool RirSensorComponent::TryApplyRuntimeConfig(
    const remote_identification_radar::config::RirRuntimeConfigPatch& patch) {
  return session_.TryApplyRuntimeConfig(patch);
}

rir::RirCycleInput RirSensorComponent::BuildCycleInput(const AppSceneState& scene,
                                                       double dt_sec) const {
  rir::RirCycleInput input;
  input.input_cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.dt_sec = dt_sec;
  input.sim_time_sec = static_cast<float>(scene.t_sec);
  input.platform_position = site_ecef_;
  input.scene_targets = scene.rir_targets;  // 站点局部 ENU + 识别特征真值（消费方注入）
  input.rf_scene = BuildExternalRfScene(scene.rf_world, sensor_platform_id_, scene.t_sec,
                                        static_cast<double>(recognition_dwell_sec_),
                                        static_cast<std::uint64_t>(scene.cycle));
  return input;
}

void RirSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    // 关机：不驱动会话；视图重置为空快照仍写一行（与 AR 组件同形）。
    last_debug_view_ = rir::RirOutputDebugView{};
    LogDebugView(world, last_debug_view_);
    return;
  }
  const auto& scene = static_cast<const AppSceneState&>(world.scene_state());
  auto& mutable_scene = static_cast<AppSceneState&>(world.scene_state());

  const rir::RirCycleInput input = BuildCycleInput(scene, dt_sec);

  const std::chrono::steady_clock::time_point step_begin = std::chrono::steady_clock::now();
  const rir::RirCycleResult result = session_.StepWithResult(input);
  if (!step_timing_logged_) {
    app::LogAcceptanceMs(scene.cycle, scene.t_sec, "单步执行时间", "RIR",
                          app::SteadyElapsedMs(step_begin));
    step_timing_logged_ = true;
  }
  // 规则 12 落盘示范：每周期构建调试视图快照（拒绝/关机周期为 kCycleNotCompleted，
  // 含规则 13b kInfo 排除诊断），经 LogDebugView 按三模式写视图行。
  last_debug_view_ = rir::RirOutputDebugViewBuilder::Build(input, result);
  LogDebugView(world, last_debug_view_);
  if (result.status != rir::RirCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无量测/结论
  }
  PublishEquipmentEmissions(&mutable_scene, result.emission_frame);
  PublishRecognitionEvents(world, result);
  PublishDesignationEvent(world, result);
  PublishTrackLifecycleEvents(world, result);
  PublishExclusionEvents(world, result);
  AdaptDetections(result);
}

void RirSensorComponent::PublishRecognitionEvents(
    World& world, const rir::RirCycleResult& result) {
  // 识别结论：确认态周期计数（冒烟下限）+ 进入确认态的迁移事件（关键事件，
  // 不逐周期重复）。
  bool any_confirmed = false;
  for (const auto& output : result.output_frame.recognition_outputs) {
    const rir::RirRecognitionState state = output.result.state;
    const bool confirmed =
        state == rir::RirRecognitionState::kCategoryConfirmed ||
        state == rir::RirRecognitionState::kModelConfirmed;
    any_confirmed = any_confirmed || confirmed;
    const auto it = prev_recognition_states_.find(output.association_key);
    const bool was_confirmed =
        it != prev_recognition_states_.end() &&
        (it->second == rir::RirRecognitionState::kCategoryConfirmed ||
         it->second == rir::RirRecognitionState::kModelConfirmed);
    if (confirmed && !was_confirmed) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string rir_recognition_event_log =
          std::string("航迹=") +
          std::to_string(static_cast<unsigned long long>(output.association_key)) +
          " 状态=" +
          (RecognitionStateName(state)) +
          " 大类=" +
          (RecognitionCategoryName(output.result.target_category)) +
          " 型号=" +
          (output.result.target_model.empty() ? "-" : output.result.target_model) +
          " 置信=" +
          std::to_string(output.result.confidence) +
          " 观测数=" +
          std::to_string(output.result.observation_count);
      CA_LOG_EVENT(world, "rir_recognition",
                   "航迹={} 状态={} 大类={} 型号={} 置信={:.2f} 观测数={}",
                   static_cast<unsigned long long>(output.association_key),
                   RecognitionStateName(state),
                   RecognitionCategoryName(output.result.target_category),
                   output.result.target_model.empty() ? "-" : output.result.target_model,
                   output.result.confidence, output.result.observation_count);
    }
    prev_recognition_states_[output.association_key] = state;
  }
  if (any_confirmed) {
    ++confirmed_recognition_outputs_;
  }

}

void RirSensorComponent::PublishDesignationEvent(
    World& world, const rir::RirCycleResult& result) {
  // 指定任务终态沿事件：designated_target_id 从非零归零 = 任务结束。结束语义按
  // 回退成因区分——kAcquisitionTimeout 为窗口耗尽作废；无回退标志为识别达成
  // 完成（成功完成不置 designation_reverted_to_scan，该标志仅缺席/超时使用）。
  const bool designation_assigned = result.designated_target_id != 0U;
  if (!designation_assigned && prev_designated_target_id_ != 0U) {
    const bool timed_out =
        result.designation_revert_reason ==
        remote_identification_radar::session::RirDesignationRevertReason::kAcquisitionTimeout;
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string rir_designation_event_log =
        std::string("指定目标=") +
        std::to_string(static_cast<unsigned long long>(prev_designated_target_id_)) +
        " 类型=" +
        (timed_out ? "任务作废回扫" : "识别达成完成") +
        " 成因=" +
        (DesignationRevertReasonName(result.designation_revert_reason));
    CA_LOG_EVENT(world, "rir_designation",
                 "指定目标={} 类型={} 成因={}",
                 static_cast<unsigned long long>(prev_designated_target_id_),
                 timed_out ? "任务作废回扫" : "识别达成完成",
                 DesignationRevertReasonName(result.designation_revert_reason));
  }
  prev_designated_target_id_ = result.designated_target_id;

}

void RirSensorComponent::PublishTrackLifecycleEvents(
    World& world, const rir::RirCycleResult& result) {
  // 航迹生命周期事件（规则 10）：首确认/丢失/指定任务作废沿写关键事件——纯日志，
  // 不发 World 信号（与 rir_recognition/rir_designation 一致）；kUpdated 为周期性
  // 重复事件、kNotTracked 为默认关闭的诊断事件，均不落盘。
  for (const auto& event : lifecycle_.GetLastEvents()) {
    switch (event.kind) {
      case rir::RirTrackLifecycleEventKind::kFirstConfirmed: {
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string rir_track_confirmed_log =
            std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
            std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
            " 航迹=" +
            std::to_string(static_cast<unsigned long long>(event.association_key)) +
            " 状态=已确认 速度=" + std::to_string(event.speed_m_per_s);
        CA_LOG_EVENT(world, "rir_track_confirmed",
                     "周期={} 目标={} 航迹={} 状态=已确认 速度={:.1f}",
                     event.world_cycle_index,
                     static_cast<unsigned long long>(event.external_target_id),
                     static_cast<unsigned long long>(event.association_key),
                     event.speed_m_per_s);
        break;
      }
      case rir::RirTrackLifecycleEventKind::kLost: {
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string rir_track_lost_log =
            std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
            std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
            " 航迹=" +
            std::to_string(static_cast<unsigned long long>(event.association_key)) +
            " 状态=丢失";
        CA_LOG_EVENT(world, "rir_track_lost", "周期={} 目标={} 航迹={} 状态=丢失",
                     event.world_cycle_index,
                     static_cast<unsigned long long>(event.external_target_id),
                     static_cast<unsigned long long>(event.association_key));
        break;
      }
      case rir::RirTrackLifecycleEventKind::kDesignationDropped: {
        // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
        const std::string rir_designation_dropped_log =
            std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
            std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
            " 航迹=" +
            std::to_string(static_cast<unsigned long long>(event.association_key)) +
            " 状态=" + (TrackStatusName(event.track_status)) + " 作废成因=" +
            (DesignationRevertReasonName(event.designation_revert_reason));
        CA_LOG_EVENT(world, "rir_designation_dropped",
                     "周期={} 目标={} 航迹={} 状态={} 作废成因={}",
                     event.world_cycle_index,
                     static_cast<unsigned long long>(event.external_target_id),
                     static_cast<unsigned long long>(event.association_key),
                     TrackStatusName(event.track_status),
                     DesignationRevertReasonName(event.designation_revert_reason));
        break;
      }
      case rir::RirTrackLifecycleEventKind::kUpdated:
      case rir::RirTrackLifecycleEventKind::kNotTracked:
      default:
        break;
    }
  }

}

void RirSensorComponent::PublishExclusionEvents(
    World& world, const rir::RirCycleResult& result) {
  // 排除原因跨周期差分事件（规则 13e）：纯诊断观测，仅落事件日志（不发 World
  // 信号——不驱动融合/威胁等下游组件）。A1（原因稳定）不产事件，天然适配 KEY
  // 事件模式（边界事件 A2/A3/A4 逐条落盘，无刷屏）。
  for (const auto& event : exclusion_.GetLastEvents()) {
    if (event.kind == rir::RirExclusionCauseEventKind::kEntered) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log =
          std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
          " 类型=进入排除 排除码=" + event.current_code + " 主因=" +
          (ExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "周期={} 目标={} 类型=进入排除 排除码={} 主因={}",
                   event.world_cycle_index,
                   static_cast<unsigned long long>(event.external_target_id),
                   event.current_code, ExclusionCauseName(event.current_cause));
    } else if (event.kind == rir::RirExclusionCauseEventKind::kChanged) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_2 =
          std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
          " 类型=原因变化 旧码=" + event.previous_code + " 旧主因=" +
          (ExclusionCauseName(event.previous_cause)) + " 新码=" + event.current_code +
          " 新主因=" + (ExclusionCauseName(event.current_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "周期={} 目标={} 类型=原因变化 旧码={} 旧主因={} 新码={} 新主因={}",
                   event.world_cycle_index,
                   static_cast<unsigned long long>(event.external_target_id),
                   event.previous_code, ExclusionCauseName(event.previous_cause),
                   event.current_code, ExclusionCauseName(event.current_cause));
    } else if (event.kind == rir::RirExclusionCauseEventKind::kExited) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string exclusion_cause_event_log_3 =
          std::string("周期=") + std::to_string(event.world_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(event.external_target_id)) +
          " 类型=退出排除 旧码=" + event.previous_code + " 旧主因=" +
          (ExclusionCauseName(event.previous_cause));
      CA_LOG_EVENT(world, "exclusion_cause",
                   "周期={} 目标={} 类型=退出排除 旧码={} 旧主因={}",
                   event.world_cycle_index,
                   static_cast<unsigned long long>(event.external_target_id),
                   event.previous_code, ExclusionCauseName(event.previous_cause));
    }
  }

}

void RirSensorComponent::AdaptDetections(const rir::RirCycleResult& result) {
  // 特征量测（出口①）→ 泛型探测记录（源通道 kRirSourceId；含 sensor_origin
  // 时进融合三维方位滤波，East→North 方位换算归适配器）。键空间统一在组件层
  // 完成：出口①只带库内 association_key（去真值化纪律），而其余源用外部目标
  // ID 直挂——按结果层归属视图（track_attributions）把库内键重写为外部 ID，
  // RIR 量测即并入与机载传感器同键的融合航迹；无归属（ID=0）的记录保留库内键。
  rir::RirFeatureMeasurementFrame frame;
  frame.input_cycle_index = result.output_frame.input_cycle_index;
  frame.batch_id = result.output_frame.batch_id;
  frame.records = result.output_frame.feature_measurements;
  detections_ = fusion::AdaptRirFeatureMeasurementsToDetectionRecords(
      fusion::kRirSourceId, frame);
  std::unordered_map<std::uint64_t, std::uint64_t> key_to_external;
  for (const auto& attribution : result.track_attributions) {
    if (attribution.external_target_id != 0U) {
      key_to_external[attribution.association_key] = attribution.external_target_id;
    }
  }
  for (auto& record : detections_) {
    const auto it = key_to_external.find(record.key);
    if (it != key_to_external.end()) {
      record.key = it->second;
    }
  }
}

void RirSensorComponent::LogDebugView(World& world, const rir::RirOutputDebugView& view) {
  // 视图行来自标准投影 DebugView（规则 12）：逐目标状态枚举 + 识别诊断 + 排除
  // 诊断；密度三模式由编译期宏门控（纯观测，不影响会话执行与信号）。
#if defined(CA_VIEW_LOG_MODE_NONNOMINAL)
  // 模式一：只写非标称目标（非已确认），全标称写一行"全部正常"。
  std::uint32_t non_nominal = 0U;
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U ||
        state.status == rir::RirDebugTargetStatus::kConfirmed) {
      continue;
    }
    ++non_nominal;
    oneq::coordinate::LlaPositionDegM lla;
    const bool have_lla =
        state.has_track && TryEnuMetersToLla(state.position_enu_x_m, state.position_enu_y_m,
                                             state.position_enu_z_m, site_origin_, &lla);
    if (have_lla) {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string rir_view_log =
          std::string("周期=") + std::to_string(view.input_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(state.external_target_id)) +
          " 状态=" + (DebugTargetStatusName(state.status)) + " 位置LLA=有" + " 速度=" +
          std::to_string(state.speed_m_per_s);
      CA_LOG_VIEW("rir", "周期={} 目标={} 状态={} 位置LLA=({:.5f},{:.5f},{:.0f}) 速度={:.1f}",
                  view.input_cycle_index,
                  static_cast<unsigned long long>(state.external_target_id),
                  DebugTargetStatusName(state.status), lla.latitude_deg, lla.longitude_deg,
                  lla.altitude_m, state.speed_m_per_s);
    } else {
      // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
      const std::string rir_view_log =
          std::string("周期=") + std::to_string(view.input_cycle_index) + " 目标=" +
          std::to_string(static_cast<unsigned long long>(state.external_target_id)) +
          " 状态=" + (DebugTargetStatusName(state.status)) + " 斜距=" +
          std::to_string(state.slant_range_m);
      CA_LOG_VIEW("rir", "周期={} 目标={} 状态={} 斜距={:.0f}m", view.input_cycle_index,
                  static_cast<unsigned long long>(state.external_target_id),
                  DebugTargetStatusName(state.status), state.slant_range_m);
    }
  }
  if (non_nominal == 0U) {
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string rir_view_log =
        std::string("周期=") + std::to_string(view.input_cycle_index) + " 全部正常（" +
        std::to_string(view.targets.size()) + " 个目标航迹均已确认）";
    CA_LOG_VIEW("rir", "周期={} 全部正常（{} 个目标航迹均已确认）", view.input_cycle_index,
                view.targets.size());
  }
#elif defined(CA_VIEW_LOG_MODE_DELTA)
  // 模式二：只写状态与上一周期不同的目标行，无变化写一行"无状态变化"。
  std::uint32_t changed = 0U;
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U) {
      continue;
    }
    const auto it = prev_target_status_.find(state.external_target_id);
    const bool first_or_changed = it == prev_target_status_.end() || it->second != state.status;
    prev_target_status_[state.external_target_id] = state.status;
    if (!first_or_changed) {
      continue;
    }
    ++changed;
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string rir_view_log =
        std::string("周期=") + std::to_string(view.input_cycle_index) + " 目标=" +
        std::to_string(static_cast<unsigned long long>(state.external_target_id)) + " 状态=" +
        (DebugTargetStatusName(state.status));
    CA_LOG_VIEW("rir", "周期={} 目标={} 状态={}", view.input_cycle_index,
                static_cast<unsigned long long>(state.external_target_id),
                DebugTargetStatusName(state.status));
  }
  if (changed == 0U) {
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string rir_view_log =
        std::string("周期=") + std::to_string(view.input_cycle_index) + " 无状态变化";
    CA_LOG_VIEW("rir", "周期={} 无状态变化", view.input_cycle_index);
  }
#else
  // 模式三：每周期恰好一行完整摘要（含指定任务镜像与排除诊断问题列表）。
  std::string target_parts;
  std::size_t track_count = 0U;
  bool any_confirmed = false;
  for (const auto& state : view.targets) {
    if (state.external_target_id == 0U) {
      continue;
    }
    track_count += state.has_track ? 1U : 0U;
    any_confirmed = any_confirmed || state.status == rir::RirDebugTargetStatus::kConfirmed;
    if (!target_parts.empty()) {
      target_parts += ", ";
    }
    std::string part = CA_FMT_FORMAT(
        "{} {}({})", static_cast<unsigned long long>(state.external_target_id),
        DebugTargetStatusName(state.status),
        state.target_name.empty() ? "-" : state.target_name);
    if (state.has_recognition_output) {
      part += CA_FMT_FORMAT(" {} 置信{:.2f}", RecognitionStateName(state.recognition_state),
                            state.confidence);
    }
    oneq::coordinate::LlaPositionDegM lla;
    if (state.has_track &&
        TryEnuMetersToLla(state.position_enu_x_m, state.position_enu_y_m,
                          state.position_enu_z_m, site_origin_, &lla)) {
      part += CA_FMT_FORMAT(" 位置LLA=({:.5f},{:.5f},{:.0f})", lla.latitude_deg,
                            lla.longitude_deg, lla.altitude_m);
    } else {
      part += CA_FMT_FORMAT(" 斜距={:.0f}m", state.slant_range_m);
    }
    part += CA_FMT_FORMAT(" 速度={:.1f}", state.speed_m_per_s);
    target_parts += part;
  }
  const std::string issues_text = app::FormatIssueText(view.issues);
  // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
  const std::string rir_view_log =
      std::string("周期=") + std::to_string(view.input_cycle_index) + " 完成=" +
      (view.executed_this_cycle ? "是" : "否") + " 航迹=" +
      std::to_string(track_count) + " 确认=" + (any_confirmed ? "是" : "否") +
      " 指定=" + (view.designation_active ? "执行中" : "无") + " 驻留中心=(" +
      std::to_string(view.dwell_center_deg.az_deg) + "°," +
      std::to_string(view.dwell_center_deg.el_deg) + "°) 目标=[" +
      (target_parts.empty() ? "无" : target_parts) + "] 问题=[" +
      (issues_text.empty() ? "无" : issues_text) + "]";
  CA_LOG_VIEW("rir",
              "周期={} 完成={} 航迹={} 确认={} 指定={} 驻留中心=({:.1f}°,{:.1f}°) 目标=[{}] 问题=[{}]",
              view.input_cycle_index, view.executed_this_cycle ? "是" : "否",
              track_count, any_confirmed ? "是" : "否",
              view.designation_active ? "执行中" : "无", view.dwell_center_deg.az_deg,
              view.dwell_center_deg.el_deg, target_parts.empty() ? "无" : target_parts.c_str(),
              issues_text.empty() ? "无" : issues_text.c_str());
#endif
}

}  // namespace component_attachment
