#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"

#include <cmath>
#include <cstdint>
#include <set>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

// 合成量测 source_index 的 sentinel：超出任何真实场景列表大小，
  // 使下游 per-target 数组边界检查自然跳过，确保合成量测不索引真实 scratch 数据。
constexpr std::size_t kDeceptionSourceIndexSentinel =
    static_cast<std::size_t>(-1);

// association_key 派生基：与场景目标键空间隔离，避免与真实关联键冲突。
// 真实 association_keys 来自关联引擎（基于 track id 空间），不会进入此高位段。
constexpr std::uint64_t kDeceptionKeyBase = 0x8000'0000'0000'0000ULL;

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

// 相干簇的稳定签名：把局部系方位/俯仰/中心频率量化到粗网格，使同源假目标簇跨周期
// （即使 observation_id 每周期重新编号）映射到同一签名。量化粒度取波束宽度量级，
// 远大于单周期数值噪声，确保同一物理簇稳定聚合。
constexpr double kClusterAzimuthGridDeg = 1.0;     // 方位量化格（度）
constexpr double kClusterElevationGridDeg = 1.0;   // 俯仰量化格（度）
constexpr double kClusterFrequencyGridHz = 1.0e6;  // 中心频率量化格（1 MHz）

std::uint64_t ClusterSignature(const session::ArInterferenceObservation& obs) {
  // 方位归一化到 [0, 360) 后量化，消除跨周期 ±360 抖动导致的签名漂移。
  const double az_deg = ResolveLocalAzimuthDeg(obs);
  double az_normalized = std::fmod(az_deg, 360.0);
  if (az_normalized < 0.0) {
    az_normalized += 360.0;
  }
  const std::int64_t az_bin = static_cast<std::int64_t>(std::round(az_normalized / kClusterAzimuthGridDeg));
  const std::int64_t el_bin =
      static_cast<std::int64_t>(std::round(ResolveLocalElevationDeg(obs) / kClusterElevationGridDeg));
  const std::int64_t freq_bin =
      static_cast<std::int64_t>(std::round(obs.estimated_center_frequency_hz / kClusterFrequencyGridHz));
  // 签名布局：方位 12 bit（0..3599）、俯仰带符号 12 bit、频率 32 bit。
  const std::uint64_t az_part = static_cast<std::uint64_t>(az_bin & 0xFFFULL);
  const std::uint64_t el_part = static_cast<std::uint64_t>(el_bin & 0xFFFULL);
  const std::uint64_t freq_part = static_cast<std::uint64_t>(freq_bin & 0xFFFF'FFFFLL);
  return (az_part << 44U) | (el_part << 32U) | freq_part;
}

}  // namespace

void InjectDeceptionMeasurementsPass(const CycleExecutionContext& context,
                                     CycleExecutionScratch& scratch) {
  // 攻击现象独立于反制开关：合成假目标量测始终从 kLikelyFalseTarget 观测注入。
  // 反制开关（enable_anti_false_target_discrimination）只在下游 PromoteState 控制
  // tentative→confirmed 的抑制策略——量测本身（含 classified_as_false_target=true）
  // 必须存在，否则开关 OFF 时假目标现象完全消失，造成因果反转。
  if (context.interference_observations == nullptr ||
      context.interference_observations->empty()) {
    return;
  }
  // 仅当局部系方位不可用（pose 缺失）且需回退 ECEF 时告警一次，避免静默使用跨系方位合成。
  bool fell_back_to_ecef = false;

  // 簇去重：resolver 把同方向 N 个脉冲列各置 coherent_emission_count=N，若对每条观测都
  // 循环 N 次会生成 N² 条量测。改为按簇签名（量化的局部方位/俯仰/中心频率）去重，每个簇
  // 只由其首个代表生成 N 条量测。签名同时用作跨周期稳定的 association_key（observation_id
  // 每周期重新编号，不能跨周期聚合同源假航迹）。
  std::set<std::uint64_t> emitted_signatures;

  for (const session::ArInterferenceObservation& obs : *context.interference_observations) {
    if (obs.deception_class != session::DeceptionClass::kLikelyFalseTarget) {
      continue;
    }
    if (obs.coherent_emission_count == 0U) {
      continue;
    }
    if (!obs.has_local_bearings) {
      fell_back_to_ecef = true;
    }
    const std::uint64_t signature = ClusterSignature(obs);
    if (!emitted_signatures.insert(signature).second) {
      // 同簇已由先前观测代表生成，跳过以免 N² 膨胀。
      continue;
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

    for (std::uint32_t i = 0U; i < obs.coherent_emission_count; ++i) {
      // 第 i 个假目标在距离上按比例抖动分散，避免完全重叠。
      const double range_m = base_range_m * (1.0 + kRangeGateJitterFraction * static_cast<double>(i));
      tracking::TrackMeasurement measurement;
      measurement.raw_measurement.source_index = kDeceptionSourceIndexSentinel;
      measurement.raw_measurement.target_name = "deception";
      measurement.raw_measurement.external_target_id = 0U;
      // 跨周期稳定的 association_key：用簇签名（而非每周期重新编号的 observation_id）派生，
      // 使 lifecycle 把同源假目标跨周期聚到同一假航迹。i 编码区分同一簇的多个假目标。
      measurement.raw_measurement.association_key =
          kDeceptionKeyBase | ((signature & 0x00FF'FFFF'FFFF'FFFFULL) << 8) |
          static_cast<std::uint64_t>(i & 0xFFULL);
      measurement.raw_measurement.matched_existing_track = false;
      measurement.raw_measurement.classified_as_false_target = true;
      measurement.raw_measurement.position = ResolveLocalPosition(azimuth_deg, elevation_deg, range_m);
      // 量测噪声协方差：由方位标准差与距离导出对角阵（与真实量测同口径的笛卡尔 R）。
      // 零或负 bearing_standard_deviation 会导致奇异协方差矩阵，使卡尔曼更新器
      // Cholesky/LDLT 分解失败。这里加最小下限并与距离联合钳制。
      const double bearing_sigma_deg =
          std::max(obs.bearing_standard_deviation_deg, 0.1);
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
