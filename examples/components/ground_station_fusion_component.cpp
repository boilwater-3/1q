/**
 * @file ground_station_fusion_component.cpp
 * @brief 消息驱动地面站融合组件实现。
 *
 * 1. 订阅探测批次 / SBIRS 帧事件，写入成员收件箱（本周期任意数量卫星）；
 * 2. 在本地把展平字段重建为库类型，适配后一次 FusionEngine::Update；
 * 3. 若挂了评估会话：从本周期帧里按评估源通道挑两颗，喂双星交会 API。
 */

#include "ground_station_fusion_component.h"

#include <algorithm>
#include <utility>

#include "1q/coordinate/inertial_transform.h"
#include "1q/coordinate/types.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/precision_evaluation/SbirsBearingAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "core/fusion_detection_bridge.h"
#include "core/scene_types.h"
#include "core/world.h"
#include "logger/logger.h"

namespace component_attachment {

namespace {

sbirs_sensor::session::SbirsCycleResult RebuildSbirsCycleResult(
    const SbirsFrameSubmittedEvent& event) {
  sbirs_sensor::session::SbirsCycleResult result;
  result.input_cycle_index = static_cast<std::uint32_t>(event.cycle);
  result.status = sbirs_sensor::session::SbirsCycleStatus::kCompleted;
  result.output_frame.cycle_index = result.input_cycle_index;
  for (const SbirsBearingSample& sample : event.detections) {
    sbirs_sensor::output::SbirsDetectionRecord detection;
    detection.detection_id = sample.detection_id;
    detection.azimuth_rad = sample.azimuth_rad;
    detection.elevation_rad = sample.elevation_rad;
    detection.infrared_snr_linear = sample.infrared_snr_linear;
    detection.detected = true;
    // 观测阶段回填（消息平铺 → 库枚举；双星定位仅收窄场数据的过滤依据）。
    switch (sample.observation_stage) {
      case 1:
        detection.observation_stage =
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition;
        break;
      case 2:
        detection.observation_stage =
            sbirs_sensor::output::SbirsObservationStage::kNarrowFieldTrack;
        break;
      default:
        detection.observation_stage =
            sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch;
        break;
    }
    result.output_frame.detections.push_back(detection);
    if (sample.target_id == 0U) {
      continue;
    }
    sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
    attribution.detection_id = sample.detection_id;
    attribution.target_id = sample.target_id;
    if (sample.has_focal_plane_offset) {
      attribution.has_focal_plane_offset = true;
      attribution.focal_plane_offset_x_m = sample.focal_plane_offset_x_m;
      attribution.focal_plane_offset_y_m = sample.focal_plane_offset_y_m;
    }
    result.detection_attributions.push_back(attribution);
  }
  return result;
}

oneq::coordinate::EcefPositionM FrameEcef(const SbirsFrameSubmittedEvent& event) {
  return oneq::coordinate::EcefPositionM(event.satellite_ecef_x_m, event.satellite_ecef_y_m,
                                         event.satellite_ecef_z_m);
}

const SbirsFrameSubmittedEvent* FindFrameBySource(
    const std::vector<SbirsFrameSubmittedEvent>& frames, std::uint32_t source_id) {
  for (const SbirsFrameSubmittedEvent& frame : frames) {
    if (frame.source_id == source_id) {
      return &frame;
    }
  }
  return nullptr;
}

}  // namespace

// 本组件不自己 new FusionEngine；main 用 scene.config.fusion 创建后 std::move 移入。
// 注意：sbirs_triple_sat_fix_messages.json 里没有 fusion 字段——config.fusion 是
// FusionConfig 结构体默认值（position_radius_m=1000 等，见 FusionConfig.h）；
// main 仅额外设 enable_track_filtering=true（与默认一致）。
GroundStationFusionComponent::GroundStationFusionComponent(
    std::unique_ptr<fusion::FusionEngine> engine)
    : engine_(std::move(engine)) {}

// 三个参数都在 main 里事先创建好，再 std::move 移入（见 main.cpp ground_station.Attach）：
//   engine — FusionEngine(scene.config.fusion)，来源见上（JSON 无 fusion 字段）；
//   evaluation — PrecisionEvaluationSession(scene.config)，JSON 填 satellites[]/targets[] 等；
//   evaluation_source_ids — {4, 104}，来自 PrecisionEvaluationConfig 默认，JSON 无此字段。
GroundStationFusionComponent::GroundStationFusionComponent(
    std::unique_ptr<fusion::FusionEngine> engine,
    std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation,
    std::vector<std::uint32_t> evaluation_source_ids)
    : engine_(std::move(engine)),
      evaluation_(std::move(evaluation)),
      evaluation_source_ids_(std::move(evaluation_source_ids)) {}

void GroundStationFusionComponent::BeginCycle(World& world, std::uint64_t cycle) {
  EnsureSignalConnections(world);
  inbox_cycle_ = cycle;
  detection_inbox_.clear();
  sbirs_inbox_.clear();
  // cross-cue 冲刷：上一周期各星发的递话在本周期开头广播（各星忽略自己发的，
  // 本周期步进前注入 → 递话内容固定滞后一个周期）。
  for (const SbirsCrossCueEvent& event : cross_cue_relay_buffer_) {
    world.signals().on_sbirs_cross_cue_relay(event);
  }
  cross_cue_relay_buffer_.clear();
}

void GroundStationFusionComponent::SetEvaluationInputs(
    const precision_evaluation::DualSatEphemerisInput& ephemeris,
    const std::vector<precision_evaluation::EvaluationTruthTarget>& truth) {
  evaluation_ephemeris_ = ephemeris;
  evaluation_truth_ = truth;
}

precision_evaluation::PrecisionEvaluationReport GroundStationFusionComponent::SummarizeEvaluation()
    const {
  if (evaluation_ == nullptr) {
    return precision_evaluation::PrecisionEvaluationReport{};
  }
  return evaluation_->Summarize();
}

void GroundStationFusionComponent::EnsureSignalConnections(World& world) {
  if (!detection_batch_connection_.connected()) {
    detection_batch_connection_ = world.signals().on_detection_batch_submitted.connect(
        [this](const DetectionBatchSubmittedEvent& event) { OnDetectionBatchSubmitted(event); });
  }
  if (!sbirs_frame_connection_.connected()) {
    sbirs_frame_connection_ = world.signals().on_sbirs_frame_submitted.connect(
        [this](const SbirsFrameSubmittedEvent& event) { OnSbirsFrameSubmitted(event); });
  }
  if (!cross_cue_connection_.connected()) {
    cross_cue_connection_ = world.signals().on_sbirs_cross_cue.connect(
        [this](const SbirsCrossCueEvent& event) { OnSbirsCrossCue(event); });
  }
}

void GroundStationFusionComponent::OnDetectionBatchSubmitted(
    const DetectionBatchSubmittedEvent& event) {
  if (event.cycle != inbox_cycle_) {
    return;
  }
  detection_inbox_.insert(detection_inbox_.end(), event.records.begin(), event.records.end());
}

void GroundStationFusionComponent::OnSbirsFrameSubmitted(const SbirsFrameSubmittedEvent& event) {
  if (event.cycle != inbox_cycle_) {
    return;
  }
  sbirs_inbox_.push_back(event);
}

void GroundStationFusionComponent::OnSbirsCrossCue(const SbirsCrossCueEvent& event) {
  // 星→地→星（2026-09-01 裁定 1）：地面站只做存储转发，不解析不改写；
  // 转发在下一周期 BeginCycle 冲刷（递话固定迟到一周期，受话星按周期门消费）。
  cross_cue_relay_buffer_.push_back(event);
}

void GroundStationFusionComponent::Step(World& world, double dt_sec) {
  (void)dt_sec;
  if (engine_ == nullptr) {
    return;
  }

  EnsureSignalConnections(world);

  auto& scene = static_cast<AppSceneState&>(world.scene_state());
  std::vector<fusion::DetectionRecord> fusion_records;
  fusion_records.reserve(detection_inbox_.size());
  for (const FusionDetectionSample& sample : detection_inbox_) {
    fusion_records.push_back(ToDetectionRecord(sample));
  }

  double gmst_rad = 0.0;
  const bool have_gmst =
      oneq::coordinate::TryComputeGmstRad(scene.sbirs_utc_julian_day, &gmst_rad);
  std::vector<sbirs_sensor::session::SbirsCycleResult> rebuilt_results;
  rebuilt_results.reserve(sbirs_inbox_.size());
  for (const SbirsFrameSubmittedEvent& frame : sbirs_inbox_) {
    rebuilt_results.push_back(RebuildSbirsCycleResult(frame));
    if (!have_gmst) {
      continue;
    }
    const std::vector<fusion::DetectionRecord> adapted =
        precision_evaluation::AdaptSbirsResultToDetectionRecords(
            rebuilt_results.back(), FrameEcef(frame), gmst_rad, frame.source_id);
    fusion_records.insert(fusion_records.end(), adapted.begin(), adapted.end());
  }

  const std::uint64_t cycle = world.scene_state().cycle;
  const std::vector<fusion::FusedTarget> fused = engine_->Update(fusion_records, cycle);

  if (evaluation_ != nullptr && evaluation_source_ids_.size() >= 2U) {
    const SbirsFrameSubmittedEvent* frame_a =
        FindFrameBySource(sbirs_inbox_, evaluation_source_ids_[0]);
    const SbirsFrameSubmittedEvent* frame_b =
        FindFrameBySource(sbirs_inbox_, evaluation_source_ids_[1]);
    sbirs_sensor::session::SbirsCycleResult empty;
    const sbirs_sensor::session::SbirsCycleResult result_a =
        frame_a != nullptr ? RebuildSbirsCycleResult(*frame_a) : empty;
    const sbirs_sensor::session::SbirsCycleResult result_b =
        frame_b != nullptr ? RebuildSbirsCycleResult(*frame_b) : empty;
    last_evaluation_ = evaluation_->Step(
        static_cast<std::uint32_t>(cycle), static_cast<float>(dt_sec), scene.sbirs_utc_julian_day,
        evaluation_ephemeris_, evaluation_truth_, result_a, result_b, fused);
    // 评审 2026-08-26 条12：落点预报真外发——评估周期里有落点解的估计航迹逐条
    // 以事件发布（订阅方经 on_impact_forecast_published 消费）。
    for (const precision_evaluation::ImpactForecastSample& forecast : last_evaluation_.forecasts) {
      ImpactForecastEvent event;
      event.cycle = cycle;
      event.key = forecast.key;
      event.impact_latitude_deg = forecast.impact_point.latitude_deg;
      event.impact_longitude_deg = forecast.impact_point.longitude_deg;
      event.impact_altitude_m = forecast.impact_point.altitude_m;
      event.impact_position_sigma_m = forecast.impact_position_sigma_m;
      event.has_burnout_sigma = forecast.has_burnout_sigma;
      event.burnout_position_sigma_m = forecast.burnout_position_sigma_m;
      world.signals().on_impact_forecast_published(event);
    }
  }

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
    std::string channels;
    for (const auto& channel : event.channels) {
      if (!channels.empty()) {
        channels += ",";
      }
      channels += std::to_string(channel.first) + ":" + std::to_string(channel.second);
    }
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
