/**
 * @file SensorAdapters.cpp
 * @brief 传感器输出 → 融合探测记录的官方适配器实现。
 *
 * 语义逐行镜像原 examples/common/sensor_adapt.h（示例层共享边界适配）：
 * 行为不变，仅命名空间从 examples::sensor_adapt 上移为 fusion，函数名加
 * 模块前缀 + ToDetectionRecords 后缀。质量归一化基准为库默认（见头文件）。
 */

#include "1q/fusion/SensorAdapters.h"

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "common/numerics/ClampUtils.h"
#include "common/numerics/Constants.h"

namespace fusion {

namespace {

}  // namespace

std::vector<DetectionRecord> AdaptArDetectionsToDetectionRecords(
    std::uint32_t source_id, const airborne_radar::session::ArExternalPoseInput& platform,
    const airborne_radar::session::ArDetectionOutputFrame& frame) {
  std::vector<DetectionRecord> detections;
  oneq::coordinate::LocalFrameReference reference;
  oneq::foundation::Vector3f radar_local_velocity;
  const oneq::coordinate::EulerAnglesDeg zero_mount{};
  if (!airborne_radar::session::TryMakeArPoseFromExternalKinematics(
          platform, zero_mount, &reference, &radar_local_velocity)) {
    return detections;  // 平台位姿非法：无可靠参考系，整帧不适配（返回空，不部分写回）
  }
  detections.reserve(frame.detections.size());
  for (const auto& detection : frame.detections) {
    DetectionRecord record;
    record.key = 0U;  // 量测无身份键：走空间/方位门关联（去真值化纪律）
    record.source_id = source_id;
    record.has_position = true;
    // 雷达局部 ENU（含平台姿态旋转）→ ENU → ECEF → LLA，换算与决策 SPI 快照
    // 位置语义同帧（TARGET-OQ-1 处置：AR 公开输出保持量测形态）。
    const oneq::coordinate::Vector3d position_enu =
        oneq::coordinate::RotateLocalToEnu(
            static_cast<double>(detection.position_x_m),
            static_cast<double>(detection.position_y_m),
            static_cast<double>(detection.position_z_m), reference.frame_attitude_deg);
    oneq::coordinate::EnuPositionM enu;
    enu.east_m = position_enu.x;
    enu.north_m = position_enu.y;
    enu.up_m = position_enu.z;
    oneq::coordinate::EcefPositionM position_ecef;
    if (oneq::coordinate::TryEnuToEcef(enu, reference.origin_lla, &position_ecef)) {
      oneq::coordinate::LlaPositionDegM lla;
      if (oneq::coordinate::TryEcefToLla(position_ecef, &lla)) {
        record.position = lla;
      } else {
        record.has_position = false;  // ECEF→LLA 失败则退化为无位置记录
      }
    } else {
      record.has_position = false;  // ENU→ECEF 失败则退化为无位置记录
    }
    record.verdict = 1.0;  // 已发布的量测记录视为有效探测
    // 质量 = 检测裕量归一化（10 dB margin → 1.0，库默认基准，与 EOS 10 dB SNR 同口径）。
    record.quality =
        oneq::common::numerics::Clamp01(static_cast<double>(detection.detection_margin_db) / 10.0);
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
    detection.bearing_az_deg = oneq::common::numerics::RadToDeg(record.azimuth_rad);
    detection.bearing_el_deg = oneq::common::numerics::RadToDeg(record.elevation_rad);
    // 质量 = 线性 IR SNR 相对 WFOV 检测门限（4.0 → 1.0）的归一化（库默认基准）。
    detection.verdict = 1.0;
    detection.quality = oneq::common::numerics::Clamp01(
        static_cast<double>(record.infrared_snr_linear) / 4.0);
    detections.push_back(detection);
  }
  return detections;
}

}  // namespace fusion
