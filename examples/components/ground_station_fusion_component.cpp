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
#include "core/scene_types.h"
#include "core/world.h"
#include "logger/logger.h"

namespace component_attachment {

namespace {

fusion::DetectionRecord ToDetectionRecord(const FusionDetectionSample& sample) {
  fusion::DetectionRecord record;
  record.key = sample.key;
  record.source_id = sample.source_id;
  record.has_position = sample.has_position;
  if (sample.has_position) {
    record.position = oneq::coordinate::LlaPositionDegM(
        sample.latitude_deg, sample.longitude_deg, sample.altitude_m);
  }
  record.has_bearing = sample.has_bearing;
  record.bearing_az_deg = sample.bearing_az_deg;
  record.bearing_el_deg = sample.bearing_el_deg;
  record.has_sensor_origin = sample.has_sensor_origin;
  if (sample.has_sensor_origin) {
    record.sensor_origin = oneq::coordinate::LlaPositionDegM(
        sample.origin_latitude_deg, sample.origin_longitude_deg, sample.origin_altitude_m);
  }
  record.has_bearing_noise = sample.has_bearing_noise;
  record.bearing_noise_sigma_rad = sample.bearing_noise_sigma_rad;
  record.verdict = sample.verdict;
  record.quality = sample.quality;
  return record;
}

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
    result.output_frame.detections.push_back(detection);
    if (sample.target_id == 0U) {
      continue;
    }
    sbirs_sensor::attribution::SbirsDetectionAttributionRecord attribution;
    attribution.detection_id = sample.detection_id;
    attribution.target_id = sample.target_id;
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

GroundStationFusionComponent::GroundStationFusionComponent(
    std::unique_ptr<fusion::FusionEngine> engine)
    : engine_(std::move(engine)) {}

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
