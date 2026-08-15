/**
 * @file SensorAdapters.cpp
 * @brief 传感器输出 → 融合探测记录的官方适配器实现。
 *
 * 语义逐行镜像原 examples/common/sensor_adapt.h（示例层共享边界适配）：
 * 行为不变，仅命名空间从 examples::sensor_adapt 上移为 fusion，函数名加
 * 模块前缀 + ToDetectionRecords 后缀。质量归一化基准为库默认（见头文件）。
 */

#include "1q/fusion/SensorAdapters.h"

#include "1q/coordinate/position_transform.h"
#include "common/numerics/ClampUtils.h"

namespace fusion {

namespace {

/// 探测质量基准：无识别置信度（target_probability == 0）时按轨迹状态取基准值。
double ArBaseQualityForStatus(airborne_radar::session::TrackStatus status) {
  return status == airborne_radar::session::TrackStatus::kConfirmed ? 1.0 : 0.5;
}

}  // namespace

std::vector<DetectionRecord> AdaptArTracksToDetectionRecords(
    std::uint32_t source_id,
    const airborne_radar::session::ArExternalTrackOutputFrame& frame) {
  std::vector<DetectionRecord> detections;
  detections.reserve(frame.tracks.size());
  for (const auto& track : frame.tracks) {
    if (track.status == airborne_radar::session::TrackStatus::kLost) {
      continue;  // 失跟轨迹不入融合（避免以旧位置续命航迹）
    }
    DetectionRecord record;
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
                         : ArBaseQualityForStatus(track.status);
    detections.push_back(record);
  }
  return detections;
}

std::vector<DetectionRecord> AdaptEsrHypothesesToDetectionRecords(
    std::uint32_t source_id,
    const electronic_surveillance_radar::session::EmitterHypothesisList& hypotheses) {
  std::vector<DetectionRecord> detections;
  detections.reserve(hypotheses.size());
  for (const auto& hypothesis : hypotheses) {
    if (hypothesis.hypothesis_id == 0U) {
      continue;  // 库内键 0 = 无身份，不适用身份直挂
    }
    DetectionRecord record;
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

std::vector<DetectionRecord> AdaptEosDetectionsToDetectionRecords(
    std::uint32_t source_id,
    const electro_optical_sensor::output::EosDetectionRecordList& records) {
  std::vector<DetectionRecord> detections;
  detections.reserve(records.size());
  for (const auto& record : records) {
    if (!record.detected) {
      continue;  // 未过探测门限不产生探测
    }
    DetectionRecord detection;
    detection.key = 0U;  // 无外部身份通道：走方位相干关联（去真值化纪律）
    detection.source_id = source_id;
    detection.has_bearing = true;
    detection.bearing_az_deg = record.azimuth_deg;
    detection.bearing_el_deg = record.elevation_deg;
    // range 通道首期不使用（保留方位相干关联路径）；
    // 质量 = 融合 SNR 归一化（10 dB → 1.0，库默认基准）。
    detection.verdict = 1.0;
    detection.quality =
        oneq::common::numerics::Clamp01(static_cast<double>(record.fused_snr_db) / 10.0);
    detections.push_back(detection);
  }
  return detections;
}

std::vector<DetectionRecord> AdaptSbirsDetectionsToDetectionRecords(
    std::uint32_t source_id,
    const sbirs_sensor::output::SbirsDetectionRecordList& records) {
  std::vector<DetectionRecord> detections;
  detections.reserve(records.size());
  for (const auto& record : records) {
    if (!record.detected) {
      continue;  // 未过探测门限不产生探测
    }
    DetectionRecord detection;
    detection.key = 0U;  // 无外部身份通道：走方位相干关联（去真值化纪律）
    detection.source_id = source_id;
    detection.has_bearing = true;
    // SBIRS 输出为 ECI 极坐标弧度（2026-08 正式变更），融合方位通道为 deg（北偏东
    // 平台系语义由调用方对齐）；此处仅做 rad→deg 单位换算，不改变参考系语义。
    detection.bearing_az_deg = record.azimuth_rad * 57.29577951308232f;
    detection.bearing_el_deg = record.elevation_rad * 57.29577951308232f;
    // 质量 = 线性 IR SNR 相对 WFOV 检测门限（4.0 → 1.0）的归一化（库默认基准）。
    detection.verdict = 1.0;
    detection.quality = oneq::common::numerics::Clamp01(
        static_cast<double>(record.infrared_snr_linear) / 4.0);
    detections.push_back(detection);
  }
  return detections;
}

}  // namespace fusion
