/**
 * @file rir_sensor_component.cpp
 * @brief RIR 地基站点传感器组件实现（会话驱动 + 识别/指定任务事件 + 融合量测）。
 *
 * 1. RirCycleInput 直接构造（固定站点海拔/ECEF + 消费方注入的站点局部 ENU
 *    场景目标帧），驱动 RirSession；指定目标任务经运行期补丁在首周期下发
 *    （识别完成或窗口耗尽由会话状态机自动回扫）；
 * 2. 识别结论进入确认态时发关键事件（逐航迹状态迁移判定，避免每周期重复）；
 *    指定任务回扫沿发事件；每周期视图行汇总归属航迹/驻留中心；
 * 3. 特征量测（出口①）适配为泛型探测记录（fusion::AdaptRirFeatureMeasurements-
 *    ToDetectionRecords，源通道 kRirSourceId=5），FusionComponent 跨实体聚合。
 */

#include "rir_sensor_component.h"

#include <cstdint>
#include <string>
#include <unordered_map>

#include "1q/coordinate/position_transform.h"
#include "1q/fusion/SensorAdapters.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "core/world.h"
#include "logger/logger.h"
#include "scene_types.h"

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
    case rir::RirRecognitionCategory::kUav:
      return "无人机";
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
  }
  return "未知";
}

}  // namespace

RirSensorComponent::RirSensorComponent(
    rir::RirSession session, const oneq::coordinate::LlaPositionDegM& site_origin,
    std::uint64_t designated_target_id, std::uint32_t designation_duration_cycles)
    : session_(std::move(session)),
      site_origin_(site_origin),
      designated_target_id_(designated_target_id),
      designation_duration_cycles_(designation_duration_cycles) {
  // 站点固定：ECEF 解析一次，逐周期作为特征量测 sensor_origin 提供。
  oneq::coordinate::TryLlaToEcef(site_origin_, &site_ecef_);
}

void RirSensorComponent::Step(World& world, double dt_sec) {
  detections_.clear();

  if (!powered_on_) {
    return;  // 关机：不驱动会话
  }
  const auto& scene = static_cast<const DemoSceneState&>(world.scene_state());

  // 指定目标任务首周期下发（识别完成/窗口耗尽的回扫由会话状态机处置）。
  if (!designation_applied_ && designated_target_id_ != 0U) {
    remote_identification_radar::config::RirRuntimeConfigPatch patch;
    patch.has_designated_target_id = true;
    patch.designated_external_target_id = designated_target_id_;
    if (designation_duration_cycles_ > 0U) {
      patch.has_designation_duration_cycles = true;
      patch.designation_duration_cycles = designation_duration_cycles_;
    }
    session_.TryApplyRuntimeConfig(patch);
    designation_applied_ = true;
  }

  rir::RirCycleInput input;
  input.input_cycle_index = static_cast<std::uint32_t>(scene.cycle);
  input.batch_id = 1U;
  input.dt_sec = dt_sec;
  input.sim_time_sec = static_cast<float>(scene.t_sec);
  input.platform_altitude_m = static_cast<float>(site_origin_.altitude_m);
  input.has_platform_position = true;
  input.platform_position.x_m = site_ecef_.x_m;
  input.platform_position.y_m = site_ecef_.y_m;
  input.platform_position.z_m = site_ecef_.z_m;
  input.scene_targets = scene.rir_targets;  // 站点局部 ENU + 识别特征真值（消费方注入）

  const rir::RirCycleResult result = session_.StepWithResult(input);
  if (result.status != rir::RirCycleStatus::kCompleted) {
    return;  // 周期被拒绝：本周期无量测/结论
  }

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

  // 指定任务终态沿事件：designated_target_id 从非零归零 = 任务结束。结束语义按
  // 回退成因区分——kAcquisitionTimeout 为窗口耗尽作废；无回退标志为识别达成
  // 完成（成功完成不置 designation_reverted_to_scan，该标志仅缺席/超时使用）。
  const bool designation_assigned = result.designated_target_id != 0U;
  if (!designation_assigned && prev_designation_assigned_) {
    const bool timed_out =
        result.designation_revert_reason ==
        remote_identification_radar::session::RirDesignationRevertReason::kAcquisitionTimeout;
    CA_LOG_EVENT(world, "rir_designation",
                 "指定目标={} 类型={} 成因={}",
                 static_cast<unsigned long long>(designated_target_id_),
                 timed_out ? "任务作废回扫" : "识别达成完成",
                 DesignationRevertReasonName(result.designation_revert_reason));
  }
  prev_designation_assigned_ = designation_assigned;

  // 每周期视图行：归属航迹（滤波位置/速度）+ 驻留中心 + 指定任务状态。
  std::string attribution_parts;
  for (const auto& attribution : result.track_attributions) {
    if (!attribution_parts.empty()) {
      attribution_parts += ", ";
    }
    attribution_parts += CA_FMT_FORMAT(
        "目标={}({}) 位置ENU=({:.0f},{:.0f},{:.0f}) 速度={:.1f}",
        static_cast<unsigned long long>(attribution.external_target_id),
        attribution.target_name.empty() ? "-" : attribution.target_name,
        attribution.position_enu_x_m, attribution.position_enu_y_m,
        attribution.position_enu_z_m, attribution.speed_m_per_s);
  }
  CA_LOG_VIEW("rir", "航迹={} 确认={} 指定={} 驻留中心=({:.1f}°,{:.1f}°) [{}]",
              result.track_attributions.size(), any_confirmed ? "是" : "否",
              result.designation_active ? "执行中" : "无",
              result.dwell_center_deg.az_deg, result.dwell_center_deg.el_deg,
              attribution_parts.empty() ? "无航迹" : attribution_parts);

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

}  // namespace component_attachment
