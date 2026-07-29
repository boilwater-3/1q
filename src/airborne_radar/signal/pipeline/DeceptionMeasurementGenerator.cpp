#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"

#include <cmath>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

// 合成量测 source_index 的 sentinel：超出任何真实场景列表大小，TagFalseTargetMeasurements
// 与 ApplyTrackFilterPass 的边界检查会自然跳过，确保合成量测不索引 per-target scratch 数组。
constexpr std::size_t kDeceptionSourceIndexSentinel =
    static_cast<std::size_t>(-1);

// association_key 派生基：与场景目标键空间隔离，避免与真实关联键冲突。
// 真实 association_keys 来自关联引擎（基于 track id 空间），不会进入此高位段。
constexpr std::uint64_t kDeceptionKeyBase = 0x8000'0000'0000'0000ULL;

// 距离波门抖动幅度：合成假目标在距离上分散开，避免完全重叠成单点。
constexpr double kRangeGateJitterFraction = 0.001;  // 千分之一视距

double ResolveLocalAzimuthDeg(const session::ArInterferenceObservation& obs) {
  // 优先局部系方位（与目标 look angle 同系）；无 pose 时回退 ECEF 切平面并告警。
  if (obs.estimated_bearing_azimuth_local_deg != 0.0 ||
      obs.estimated_bearing_elevation_local_deg != 0.0) {
    return obs.estimated_bearing_azimuth_local_deg;
  }
  return obs.estimated_bearing_azimuth_deg;
}

double ResolveLocalElevationDeg(const session::ArInterferenceObservation& obs) {
  if (obs.estimated_bearing_azimuth_local_deg != 0.0 ||
      obs.estimated_bearing_elevation_local_deg != 0.0) {
    return obs.estimated_bearing_elevation_local_deg;
  }
  return obs.estimated_bearing_elevation_deg;
}

// 由局部系方位/俯仰 + 视距合成笛卡尔局部坐标（与 TargetLookResolver/ArSceneTarget 同系）。
// 口径：x = r*cos(el)*cos(az), y = r*cos(el)*sin(az), z = r*sin(el)。
Eigen::Vector3f ResolveLocalPosition(double azimuth_deg, double elevation_deg, double range_m) {
  const double az_rad = azimuth_deg * M_PI / 180.0;
  const double el_rad = elevation_deg * M_PI / 180.0;
  const double cos_el = std::cos(el_rad);
  const double x = range_m * cos_el * std::cos(az_rad);
  const double y = range_m * cos_el * std::sin(az_rad);
  const double z = range_m * std::sin(el_rad);
  return Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
}

// 由径向速度沿视线方向投影得速度向量。视线方向取局部系方位/俯仰。
Eigen::Vector3f ResolveVelocityAlongLineOfSight(double azimuth_deg, double elevation_deg,
                                                double range_rate_mps) {
  const double az_rad = azimuth_deg * M_PI / 180.0;
  const double el_rad = elevation_deg * M_PI / 180.0;
  const double cos_el = std::cos(el_rad);
  const double vx = range_rate_mps * cos_el * std::cos(az_rad);
  const double vy = range_rate_mps * cos_el * std::sin(az_rad);
  const double vz = range_rate_mps * std::sin(el_rad);
  return Eigen::Vector3f(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz));
}

}  // namespace

void InjectDeceptionMeasurementsPass(const CycleExecutionContext& context,
                                     const ExecutionConfig& runtime_config,
                                     CycleExecutionScratch& scratch) {
  if (!runtime_config.enable_anti_false_target_discrimination) {
    return;
  }
  if (context.interference_observations == nullptr ||
      context.interference_observations->empty()) {
    return;
  }
  // 仅当局部系方位不可用（pose 缺失）且需回退 ECEF 时告警一次，避免静默使用跨系方位合成。
  bool fell_back_to_ecef = false;

  for (const session::ArInterferenceObservation& obs : *context.interference_observations) {
    if (obs.deception_class != session::DeceptionClass::kLikelyFalseTarget) {
      continue;
    }
    if (obs.coherent_emission_count == 0U) {
      continue;
    }
    if (obs.estimated_bearing_azimuth_local_deg == 0.0 &&
        obs.estimated_bearing_elevation_local_deg == 0.0) {
      fell_back_to_ecef = true;
    }
    const double azimuth_deg = ResolveLocalAzimuthDeg(obs);
    const double elevation_deg = ResolveLocalElevationDeg(obs);
    const double base_range_m =
        obs.estimated_slant_range_m > 0.0 ? obs.estimated_slant_range_m : 50000.0;

    for (std::uint32_t i = 0U; i < obs.coherent_emission_count; ++i) {
      // 第 i 个假目标在距离上按比例抖动分散，避免完全重叠。
      const double range_m = base_range_m * (1.0 + kRangeGateJitterFraction * static_cast<double>(i));
      tracking::TrackMeasurement measurement;
      measurement.raw_measurement.source_index = kDeceptionSourceIndexSentinel;
      measurement.raw_measurement.target_name = "deception";
      measurement.raw_measurement.external_target_id = 0U;
      // 确定性 association_key：同 observation_id 的假目标跨周期映射到同一 key 段，
      // 使 lifecycle 把同源假目标聚到同一假航迹。i 编码区分同一观测的多个假目标。
      measurement.raw_measurement.association_key =
          kDeceptionKeyBase |
          ((obs.observation_id & 0x7FFF'FFFF'FFFF'FFFFULL) << 8) |
          static_cast<std::uint64_t>(i & 0xFFULL);
      measurement.raw_measurement.matched_existing_track = false;
      measurement.raw_measurement.classified_as_false_target = true;
      measurement.raw_measurement.position = ResolveLocalPosition(azimuth_deg, elevation_deg, range_m);
      // 量测噪声协方差：由方位标准差与距离导出对角阵（与真实量测同口径的笛卡尔 R）。
      const double sigma_cross_range = range_m * (obs.bearing_standard_deviation_deg * M_PI / 180.0);
      const double sigma_range = std::max(sigma_cross_range, 50.0);
      measurement.raw_measurement.measurement_covariance =
          Eigen::Matrix3f(static_cast<Eigen::Matrix3f>(Eigen::DiagonalMatrix<float, 3>(
              static_cast<float>(sigma_range * sigma_range),
              static_cast<float>(sigma_cross_range * sigma_cross_range),
              static_cast<float>(sigma_cross_range * sigma_cross_range))));
      // 滤波特征：速度沿视线方向投影，rcs 取 0（假目标无真实 RCS）。
      measurement.filtered_feature.velocity =
          ResolveVelocityAlongLineOfSight(azimuth_deg, elevation_deg, obs.estimated_range_rate_mps);
      measurement.filtered_feature.observed_speed =
          measurement.filtered_feature.velocity.norm();
      measurement.filtered_feature.rcs = 0.0f;
      scratch.track_measurements.push_back(measurement);
    }
  }
  if (fell_back_to_ecef) {
    PROJECT_LOG_WARN(
        "[DeceptionMeasurementGenerator] synthesized measurements using ECEF tangent-plane "
        "bearings; platform frame not available, cross-frame geometry degraded.");
  }
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
