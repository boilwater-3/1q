/**
 * @file fusion_component.cpp
 * @brief 融合组件实现（地面站枢纽：收探测 → 适配 → FusionEngine::Update）。
 *
 * 1. 聚合本周期探测池（机载/RIR 等源已写成 DetectionRecord）以及双星收件箱
 *    （卫星实体写下的 SBIRS 周期结果，本组件内 AdaptSbirsResultToDetectionRecords
 *    编成源 4 / 源 104）；一次 FusionEngine::Update。
 * 2. 若挂了评估会话：把两星原始结果 + 融合航迹交给 PrecisionEvaluationSession::Step。
 * 3. 每个融合目标经 World 信号发布 FusionUpdatedEvent。
 */

#include "fusion_component.h"

#include <algorithm>
#include <utility>

#include "1q/coordinate/inertial_transform.h"
#include "1q/precision_evaluation/SbirsBearingAdapter.h"
#include "core/events.h"
#include "core/scene_types.h"
#include "core/world.h"
#include "logger/logger.h"

namespace component_attachment {

namespace {

oneq::coordinate::EcefPositionM ToEcef(
    const sbirs_sensor::session::SbirsVector3M& position) {
  return oneq::coordinate::EcefPositionM(position.x, position.y, position.z);
}

}  // namespace

FusionComponent::FusionComponent(std::unique_ptr<fusion::FusionEngine> engine)
    : engine_(std::move(engine)) {}

FusionComponent::FusionComponent(
    std::unique_ptr<fusion::FusionEngine> engine,
    std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation,
    std::uint32_t satellite_a_source_id, std::uint32_t satellite_b_source_id)
    : engine_(std::move(engine)),
      evaluation_(std::move(evaluation)),
      satellite_a_source_id_(satellite_a_source_id),
      satellite_b_source_id_(satellite_b_source_id) {
  if (satellite_b_source_id_ == satellite_a_source_id_) {
    satellite_b_source_id_ = satellite_a_source_id_ + 100U;
  }
}

void FusionComponent::SetEvaluationInputs(
    const precision_evaluation::DualSatEphemerisInput& ephemeris,
    const std::vector<precision_evaluation::EvaluationTruthTarget>& truth) {
  evaluation_ephemeris_ = ephemeris;
  evaluation_truth_ = truth;
}

precision_evaluation::PrecisionEvaluationReport FusionComponent::SummarizeEvaluation() const {
  if (evaluation_ == nullptr) {
    return precision_evaluation::PrecisionEvaluationReport{};
  }
  return evaluation_->Summarize();
}

void FusionComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  if (engine_ == nullptr) {
    return;  // 无引擎：无融合
  }

  auto& scene = static_cast<AppSceneState&>(world.scene_state());
  std::vector<fusion::DetectionRecord> fusion_records = scene.detection_pool;

  double gmst_rad = 0.0;
  const bool have_gmst =
      oneq::coordinate::TryComputeGmstRad(scene.sbirs_utc_julian_day, &gmst_rad);
  const sbirs_sensor::session::SbirsCycleResult* result_a = nullptr;
  const sbirs_sensor::session::SbirsCycleResult* result_b = nullptr;
  for (const SbirsGroundStationFrame& frame : scene.sbirs_ground_station_inbox) {
    if (have_gmst) {
      const std::vector<fusion::DetectionRecord> adapted =
          precision_evaluation::AdaptSbirsResultToDetectionRecords(
              frame.result, ToEcef(frame.satellite_position_ecef_m), gmst_rad, frame.source_id);
      fusion_records.insert(fusion_records.end(), adapted.begin(), adapted.end());
    }
    if (frame.source_id == satellite_a_source_id_) {
      result_a = &frame.result;
    } else if (frame.source_id == satellite_b_source_id_) {
      result_b = &frame.result;
    }
  }

  const std::uint64_t cycle = world.scene_state().cycle;
  const std::vector<fusion::FusedTarget> fused = engine_->Update(fusion_records, cycle);

  if (evaluation_ != nullptr) {
    sbirs_sensor::session::SbirsCycleResult empty_a;
    sbirs_sensor::session::SbirsCycleResult empty_b;
    last_evaluation_ = evaluation_->Step(
        static_cast<std::uint32_t>(cycle), static_cast<float>(dt_sec), scene.sbirs_utc_julian_day,
        evaluation_ephemeris_, evaluation_truth_, result_a != nullptr ? *result_a : empty_a,
        result_b != nullptr ? *result_b : empty_b, fused);
  }

  // 新/消失目标按键集合差分（对照上一周期态势）。
  std::size_t new_count = 0U;
  for (const auto& target : fused) {
    const auto it = std::find_if(targets_.begin(), targets_.end(),
                                 [&target](const fusion::FusedTarget& prev) {
                                   return prev.key == target.key;
                                 });
    if (it == targets_.end()) {
      ++new_count;
    }
  }
  std::size_t lost_count = 0U;
  for (const auto& prev : targets_) {
    const auto it = std::find_if(fused.begin(), fused.end(),
                                 [&prev](const fusion::FusedTarget& target) {
                                   return target.key == prev.key;
                                 });
    if (it == fused.end()) {
      ++lost_count;
    }
  }
  targets_ = fused;

  // 发布融合态势事件（每融合目标一条，携带通道样本构成、周期差分计数与
  // 运动学估计展平字段——事件为集成契约，消费方本地重建库类型）。
  for (const auto& target : targets_) {
    FusionUpdatedEvent event;
    event.cycle = cycle;
    event.key = target.key;
    event.confidence = target.confidence;
    for (const auto& channel : target.channels) {
      event.channels.emplace_back(channel.source_id, channel.sample_count);
    }
    event.new_targets = new_count;
    event.lost_targets = lost_count;
    event.has_kinematic_estimate = target.has_kinematic_estimate;
    if (target.has_kinematic_estimate) {
      event.latitude_deg = target.kinematic_estimate.position.latitude_deg;
      event.longitude_deg = target.kinematic_estimate.position.longitude_deg;
      event.altitude_m = target.kinematic_estimate.position.altitude_m;
      event.velocity_ecef_m_per_s = target.kinematic_estimate.velocity_ecef_m_per_s;
      event.covariance_ecef = target.kinematic_estimate.covariance_ecef;
    }
    // 融合态势事件每周期重复（目标恒在时）：事件模式一下不落盘（信号照常发布）。
    std::string channels;
    for (const auto& channel : event.channels) {
      if (!channels.empty()) channels += ",";
      channels += std::to_string(channel.first) + ":" + std::to_string(channel.second);
    }
    // 与下方写入行同内容：纯 std::string/std::to_string 拼接的日志字符串，供集成方直接搬入己方日志（示例自身不消费）。
    const std::string fusion_updated_event_log =
        std::string("键=") +
        std::to_string(static_cast<unsigned long long>(event.key)) +
        " 置信=" +
        std::to_string(event.confidence) +
        " 新增=" +
        std::to_string(event.new_targets) +
        " 消失=" +
        std::to_string(event.lost_targets) +
        " 通道=[" +
        (channels.c_str()) +
        "]";
    CA_LOG_EVENT_DUP(world, "fusion_updated", "键={} 置信={:.2f} 新增={} 消失={} 通道=[{}]",
                     static_cast<unsigned long long>(event.key), event.confidence,
                     event.new_targets, event.lost_targets, channels.c_str());
    world.signals().on_fusion_updated(event);
  }
}

}  // namespace component_attachment
