/**
 * @file sensor_adapt.h
 * @brief 示例共享工具：传感器输出 → 融合探测记录的边界适配。
 *
 * 四传感器会话输出（AR 轨迹帧 / ESR 辐射源假设 / EOS 探测记录 / SBIRS
 * 探测记录）在业务层边界适配为去真值化的泛型融合探测记录
 * （fusion::DetectionRecord）。本文件供 behavior_layer（EnTT 模式）与
 * component_attachment（自定义实体-组件模式）两个示例共用，消除同构适配
 * 逻辑的双份维护（任何一边修复须同步的问题）。源通道常量与融合配置
 * source_weights 索引一致（索引 0 未用）。
 */

#ifndef EXAMPLES_COMMON_SENSOR_ADAPT_H_
#define EXAMPLES_COMMON_SENSOR_ADAPT_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArCycleOutputAdapter.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"

namespace examples {
namespace sensor_adapt {

/** @brief 演示源通道标识（与融合配置 source_weights 索引一致；索引 0 未用）。 */
constexpr std::uint32_t kArSourceId = 1U;    /**< AR 源通道 */
constexpr std::uint32_t kEsrSourceId = 2U;   /**< ESR 源通道 */
constexpr std::uint32_t kEosSourceId = 3U;   /**< EOS 源通道 */
constexpr std::uint32_t kSbirsSourceId = 4U; /**< SBIRS 源通道 */

/// 探测质量基准：无识别置信度（target_probability == 0）时按轨迹状态取基准值。
inline double BaseQualityForStatus(airborne_radar::session::TrackStatus status) {
  return status == airborne_radar::session::TrackStatus::kConfirmed ? 1.0 : 0.5;
}

/// 把 AR 外部轨迹帧适配为融合探测记录（key = association_key，跳过失落轨迹）。
inline std::vector<fusion::DetectionRecord> AdaptTracksToDetections(
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
inline std::vector<fusion::DetectionRecord> AdaptHypothesesToDetections(
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
inline std::vector<fusion::DetectionRecord> AdaptEosDetectionsToDetections(
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

/// 把 SBIRS 探测记录适配为融合探测记录（key=0 无身份，仅方位通道，与 EOS 同构）。
inline std::vector<fusion::DetectionRecord> AdaptSbirsDetectionsToDetections(
    std::uint32_t source_id,
    const sbirs_sensor::output::SbirsDetectionRecordList& records) {
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
    // 质量 = 线性 IR SNR 相对 WFOV 检测门限（4.0 → 1.0）的归一化（业务层映射）。
    detection.verdict = 1.0;
    detection.quality =
        std::min(1.0, std::max(0.0, static_cast<double>(record.infrared_snr_linear) / 4.0));
    detections.push_back(detection);
  }
  return detections;
}

}  // namespace sensor_adapt
}  // namespace examples

#endif  // EXAMPLES_COMMON_SENSOR_ADAPT_H_
