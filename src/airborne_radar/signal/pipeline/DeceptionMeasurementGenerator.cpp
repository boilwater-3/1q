#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

// 合成量测 source_index 的 sentinel：超出任何真实场景列表大小，
// 使下游 per-target 数组边界检查自然跳过，确保合成量测不索引真实 scratch 数据。
constexpr std::size_t kDeceptionSourceIndexSentinel = static_cast<std::size_t>(-1);

// association_key 派生基：与场景目标键空间隔离，避免与真实关联键冲突。
// 真实 association_keys 来自关联引擎（基于 track id 空间），不会进入此高位段。
constexpr std::uint64_t kDeceptionKeyBase = 0x8000000000000000ULL;

    // 距离波门抖动幅度：合成假目标在距离上分散开，避免完全重叠成单点。
    constexpr double kRangeGateJitterFraction = 0.001;  // 千分之一视距

double ResolveLocalAzimuthDeg(const session::ArInterferenceObservation& obs) {
  if (obs.has_local_bearings) {
    return obs.estimated_bearing_azimuth_local_deg;
  }
  return obs.estimated_bearing_azimuth_deg;
}

double ResolveLocalElevationDeg(const session::ArInterferenceObservation& obs) {
  if (obs.has_local_bearings) {
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

std::uint64_t MixAssociationKey(std::uint64_t seed, std::uint32_t member_index) {
  std::uint64_t value =
      seed ^ (static_cast<std::uint64_t>(member_index) + UINT64_C(0x9e3779b97f4a7c15));
  value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
  value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
  value ^= value >> 31U;
  return kDeceptionKeyBase | (value & UINT64_C(0x7fffffffffffffff));
}

}  // namespace

void InjectDeceptionMeasurementsPass(const CycleExecutionContext& context,
                                     CycleExecutionScratch& scratch) {
  // 攻击现象独立于反制开关：合成假目标量测始终从 kLikelyFalseTarget 观测注入。
  // 反制开关（enable_anti_false_target_discrimination）只在下游 PromoteState 控制
  // tentative→confirmed 的抑制策略——量测本身（含 classified_as_false_target=true）
  // 必须存在，否则开关 OFF 时假目标现象完全消失，造成因果反转。
  if (context.interference_observations == nullptr || context.interference_observations->empty() ||
      context.deception_clusters == nullptr || context.deception_clusters->empty()) {
    return;
  }
  // 仅当局部系方位不可用（pose 缺失）且需回退 ECEF 时告警一次，避免静默使用跨系方位合成。
  bool fell_back_to_ecef = false;

  // resolver 是簇身份的唯一所有者；每个内部 cluster 只引用一条公开代表观测并携带稳定
  // association seed。generator 不再按方位/频率重新分组，因此每簇严格生成 N 条而非 N²。
  for (const detection::ArDeceptionCluster& cluster : *context.deception_clusters) {
    const auto observation = std::find_if(
        context.interference_observations->begin(), context.interference_observations->end(),
        [&cluster](const session::ArInterferenceObservation& candidate) {
          return candidate.observation_id == cluster.representative_observation_id;
        });
    if (observation == context.interference_observations->end()) {
      PROJECT_LOG_ERROR("[DeceptionMeasurementGenerator] missing representative observation {}.",
                        cluster.representative_observation_id);
      continue;
    }
    const session::ArInterferenceObservation& obs = *observation;
    if (obs.deception_class != session::DeceptionClass::kLikelyFalseTarget ||
        cluster.emission_count < 2U || obs.coherent_emission_count != cluster.emission_count ||
        cluster.association_key_seed == 0U) {
      PROJECT_LOG_ERROR(
          "[DeceptionMeasurementGenerator] inconsistent deception cluster for observation {}.",
          obs.observation_id);
      continue;
    }
    if (!obs.has_local_bearings) {
      fell_back_to_ecef = true;
    }
    const double azimuth_deg = ResolveLocalAzimuthDeg(obs);
    const double elevation_deg = ResolveLocalElevationDeg(obs);
    const double base_range_m =
        obs.estimated_slant_range_m > 0.0 ? obs.estimated_slant_range_m : 50000.0;
    if (obs.estimated_slant_range_m <= 0.0) {
      PROJECT_LOG_WARN(
          "[DeceptionMeasurementGenerator] estimated_slant_range_m <= 0 for observation {}; "
          "falling back to synthetic range {}m.",
          obs.observation_id, base_range_m);
    }

    for (std::uint32_t i = 0U; i < cluster.emission_count; ++i) {
      // 第 i 个假目标在距离上按比例抖动分散，避免完全重叠。
      const double range_m =
          base_range_m * (1.0 + kRangeGateJitterFraction * static_cast<double>(i));
      tracking::TrackMeasurement measurement;
      measurement.raw_measurement.source_index = kDeceptionSourceIndexSentinel;
      measurement.raw_measurement.target_name = "deception";
      measurement.raw_measurement.external_target_id = 0U;
      // 跨周期稳定的 association_key 由 resolver 的 source-equipment 簇种子派生；
      // member index 经 64 位混合区分同簇假目标，不再受 8-bit 截断限制。
      measurement.raw_measurement.association_key =
          MixAssociationKey(cluster.association_key_seed, i);
      measurement.raw_measurement.matched_existing_track = false;
      measurement.raw_measurement.classified_as_false_target = true;
      measurement.raw_measurement.position =
          ResolveLocalPosition(azimuth_deg, elevation_deg, range_m);
      // 量测噪声协方差：由方位标准差与距离导出对角阵（与真实量测同口径的笛卡尔 R）。
      // 零或负 bearing_standard_deviation 会导致奇异协方差矩阵，使卡尔曼更新器
      // Cholesky/LDLT 分解失败。这里加最小下限并与距离联合钳制。
      const double bearing_sigma_deg = std::max(obs.bearing_standard_deviation_deg, 0.1);
      const double sigma_cross_range = range_m * (bearing_sigma_deg * M_PI / 180.0);
      const double sigma_range = std::max(sigma_cross_range, 50.0);
      measurement.raw_measurement.measurement_covariance =
          Eigen::Matrix3f(static_cast<Eigen::Matrix3f>(Eigen::DiagonalMatrix<float, 3>(
              static_cast<float>(sigma_range * sigma_range),
              static_cast<float>(sigma_cross_range * sigma_cross_range),
              static_cast<float>(sigma_cross_range * sigma_cross_range))));
      // 滤波特征：速度沿视线方向投影，rcs 取 0（假目标无真实 RCS）。
      measurement.filtered_feature.velocity =
          ResolveVelocityAlongLineOfSight(azimuth_deg, elevation_deg, obs.estimated_range_rate_mps);
      measurement.filtered_feature.observed_speed = measurement.filtered_feature.velocity.norm();
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
