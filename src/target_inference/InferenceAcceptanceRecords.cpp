/**
 * @file InferenceAcceptanceRecords.cpp
 * @brief 推演验收行：轨迹预报 / 落点 / 发射点 / 发布；关机点按机械能最大点近似。
 */

#include "target_inference/InferenceAcceptanceRecords.h"

#include <cmath>
#include <map>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "common/logging/AcceptanceText.h"
#include "target_inference/InferenceAcceptanceLog.h"

namespace target_inference {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatVec3;

struct LastVel {
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  bool valid{false};
};

std::map<std::uint64_t, LastVel>& LastVelocities() {
  static std::map<std::uint64_t, LastVel> values;
  return values;
}

/**
 * @brief 单航迹比机械能峰值状态（会话内累计，仿 LastVelocities 先例）。
 * @note 甲方 2026-08-22 批注「判断其势能+动能最大的地方」：峰值下降沿 =
 *       关机点近似（无推力模型，方法口径见验收目录条目）。
 */
struct EnergyPeak {
  double energy_j_per_kg{0.0};   /**< 至今比机械能峰值。 */
  double time_sec{0.0};          /**< 峰值采样时刻。 */
  oneq::coordinate::EcefPositionM position{}; /**< 峰值位置。 */
  double radius_m{0.0};          /**< 峰值地心距（阈值与高度换算用）。 */
  double speed_mps{0.0};         /**< 峰值速度模。 */
  bool peak_is_first_sample{false}; /**< 峰值仍为首采样（下降即意味关机早于跟踪起点）。 */
  bool confirmed{false};         /**< 已观测到下降沿。 */
  bool valid{false};             /**< 已收到首个有效采样。 */
};

std::map<std::uint64_t, EnergyPeak>& EnergyPeaks() {
  static std::map<std::uint64_t, EnergyPeak> values;
  return values;
}

void HorizontalEllipseFromCov(const std::array<double, 36U>& cov, double* semi_major,
                              double* semi_minor, double* azimuth_deg) {
  // 位置方差在 [x,vx,y,vy,z,vz] 的 (0,0)/(2,2)/(4,4)；水平取 ECEF x-y 近似。
  const double a = cov[0];
  const double b = cov[2];
  const double c = cov[14];
  const double disc = std::sqrt(std::max(0.0, (a - c) * (a - c) + 4.0 * b * b));
  const double l1 = 0.5 * (a + c + disc);
  const double l2 = 0.5 * (a + c - disc);
  *semi_major = std::sqrt(std::max(0.0, l1));
  *semi_minor = std::sqrt(std::max(0.0, l2));
  *azimuth_deg = 0.5 * std::atan2(2.0 * b, a - c) * 57.29577951308232;
}

}  // namespace

double SpecificMechanicalEnergyJPerKg(const oneq::coordinate::EcefPositionM& position,
                                      const std::array<double, 3U>& velocity_ecef_m_per_s,
                                      double earth_mu_m3_per_s2) {
  const double radius = std::sqrt(position.x_m * position.x_m + position.y_m * position.y_m +
                                  position.z_m * position.z_m);
  if (radius <= 0.0) {
    return 0.0;
  }
  const double speed_sq = velocity_ecef_m_per_s[0] * velocity_ecef_m_per_s[0] +
                          velocity_ecef_m_per_s[1] * velocity_ecef_m_per_s[1] +
                          velocity_ecef_m_per_s[2] * velocity_ecef_m_per_s[2];
  return 0.5 * speed_sq - earth_mu_m3_per_s2 / radius;
}

void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              const std::vector<TargetInferenceResult>& results,
                              const TargetInferenceConfig& config) {
  if (!INFERENCE_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }

  // 关机点预测（2026-08-22 甲方批注「判断其势能+动能最大的地方」）：逐航迹
  // 累计比机械能峰值，下降沿（降幅 > 0.1% 势能尺度）确认为关机点；无下降沿
  // 如实写观测中。行内不带方法标注，公式与阈值见验收目录条目。
  if (tracks.empty()) {
    INFERENCE_ACCEPTANCE_ITEM(0.0f, 0U, "关机点预测", "暂无");
  }
  for (const InferenceTrackState& track : tracks) {
    const float row_sim_time = static_cast<float>(track.sim_time_sec);
    const std::uint32_t row_cycle = track.input_cycle_index;
    const double radius = std::sqrt(track.position.x_m * track.position.x_m +
                                    track.position.y_m * track.position.y_m +
                                    track.position.z_m * track.position.z_m);
    if (radius <= 0.0) {
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "关机点预测",
                                "目标键=" + std::to_string(track.key) + " 位置不可用");
      continue;
    }
    const double speed = std::sqrt(
        track.velocity_ecef_m_per_s[0] * track.velocity_ecef_m_per_s[0] +
        track.velocity_ecef_m_per_s[1] * track.velocity_ecef_m_per_s[1] +
        track.velocity_ecef_m_per_s[2] * track.velocity_ecef_m_per_s[2]);
    const double energy = SpecificMechanicalEnergyJPerKg(track.position, track.velocity_ecef_m_per_s,
                                                        config.earth_mu_m3_per_s2);
    EnergyPeak& peak = EnergyPeaks()[track.key];
    const bool first_sample = !peak.valid;
    if (first_sample || energy > peak.energy_j_per_kg) {
      peak.energy_j_per_kg = energy;
      peak.time_sec = track.sim_time_sec;
      peak.position = track.position;
      peak.radius_m = radius;
      peak.speed_mps = speed;
      peak.peak_is_first_sample = first_sample;
      peak.valid = true;
    }
    if (!peak.confirmed) {
      const double margin = 1.0e-3 * config.earth_mu_m3_per_s2 / peak.radius_m;
      if (energy < peak.energy_j_per_kg - margin) {
        peak.confirmed = true;
      }
    }
    std::string burnout = "目标键=" + std::to_string(track.key);
    if (peak.confirmed) {
      if (peak.peak_is_first_sample) {
        burnout += " 状态=关机点早于跟踪起点 能量自首采样即下降 峰值=" +
                   FormatF(peak.energy_j_per_kg, 0) + "J/kg@" + FormatF(peak.time_sec, 1) + "s";
      } else {
        burnout += " 状态=已确认";
        burnout += " 关机时刻=" + FormatF(peak.time_sec, 1) + "s";
        // 关机点输出经纬高（LLA，库内 TryEcefToLla 换算）：与落点/发射点行同风格，
        // 高度即椭球高（m）。
        oneq::coordinate::LlaPositionDegM peak_lla{};
        if (oneq::coordinate::TryEcefToLla(peak.position, &peak_lla)) {
          burnout += " 关机点经纬高=" + FormatVec3(peak_lla.latitude_deg,
                                                  peak_lla.longitude_deg,
                                                  peak_lla.altitude_m, 3);
        } else {
          burnout += " 关机点经纬高=无";
        }
        burnout += " 当时速度=" + FormatF(peak.speed_mps, 1) + "m/s";
        burnout += " 机械能峰值=" + FormatF(peak.energy_j_per_kg, 0) + "J/kg";
      }
    } else {
      burnout += " 状态=观测中 能量峰值=" + FormatF(peak.energy_j_per_kg, 0) + "J/kg@" +
                 FormatF(peak.time_sec, 1) + "s";
    }
    INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "关机点预测", burnout);
  }

  for (std::size_t i = 0U; i < results.size(); ++i) {
    const TargetInferenceResult& result = results[i];
    const TrajectoryPrediction& traj = result.trajectory;
    // 行前缀时间：与航迹采样同源（调用方未提供时保持 0，与既有行为一致）。
    const float row_sim_time =
        i < tracks.size() ? static_cast<float>(tracks[i].sim_time_sec) : 0.0f;
    const std::uint32_t row_cycle = i < tracks.size() ? tracks[i].input_cycle_index : 0U;
    if (!traj.valid) {
      continue;
    }
    double horizon0 = 0.0;
    double horizon1 = 0.0;
    if (!traj.waypoints.empty()) {
      horizon0 = traj.waypoints.front().time_offset_sec;
      horizon1 = traj.waypoints.back().time_offset_sec;
    }
    const double sigma = traj.impact_position_sigma_m;
    std::string forecast = "航路点点数=" + std::to_string(traj.waypoints.size());
    forecast += " 预报时段=[" + FormatF(horizon0, 1) + "," + FormatF(horizon1, 1) + "]s";
    if (traj.has_impact) {
      forecast += " 落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                              traj.impact_point.longitude_deg,
                                              traj.impact_point.altitude_m, 3);
      forecast += " 误差椭圆半长/半短/方位=(" + FormatF(sigma, 1) + "m," + FormatF(sigma, 1) +
                  "m,0°)";
    } else {
      forecast += " 落点经纬高=无";
      forecast += " 误差椭圆半长/半短/方位=无";
    }
    INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "目标轨迹预报", forecast);

    if (traj.has_impact) {
      std::string impact = "初步落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                                          traj.impact_point.longitude_deg,
                                                          traj.impact_point.altitude_m, 3);
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "落点预测", impact);

      std::string publish = "弹道模型=无推力弹道外推";
      publish += " 预测落点=" + FormatVec3(traj.impact_point.latitude_deg,
                                           traj.impact_point.longitude_deg,
                                           traj.impact_point.altitude_m, 3);
      publish += " 误差1σ=" + FormatF(sigma, 1) + "m";
      publish += " 置信度=0.68";
      // 甲方 2026-08-22 批注「写明文」：库内无标准编解码与发布订阅——封装按
      // 明文报文原样写出（字段自包含），分发状态如实标注明文落盘、未外发。
      publish += " 标准化封装=明文:[落点预报|落点=" +
                 FormatVec3(traj.impact_point.latitude_deg, traj.impact_point.longitude_deg,
                            traj.impact_point.altitude_m, 3) +
                 "|1σ=" + FormatF(sigma, 1) + "m|置信度=0.68]";
      publish += " 分发状态=明文落盘未外发";
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "落点预报与信息发布", publish);
    }

    if (traj.has_launch) {
      double a = 0.0;
      double b = 0.0;
      double az = 0.0;
      HorizontalEllipseFromCov(traj.launch_covariance_ecef, &a, &b, &az);
      std::string launch = "预测发射时刻相对t=" + FormatF(traj.launch_time_offset_sec, 1) + "s";
      launch += " 地理位置=" + FormatVec3(traj.launch_point.latitude_deg, traj.launch_point.longitude_deg,
                                          traj.launch_point.altitude_m, 3);
      launch += " 位置误差1σ=" + FormatF(traj.launch_position_sigma_m, 1) + "m";
      launch += " 椭圆半长/半短/方位=(" + FormatF(a, 1) + "m," + FormatF(b, 1) + "m," +
                FormatF(az, 1) + "°)";
      launch += " 置信度=0.68";
      INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "发射点预测", launch);
    }

    if (i < tracks.size()) {
      const InferenceTrackState& track = tracks[i];
      LastVel& prev = LastVelocities()[track.key];
      const double speed = std::sqrt(track.velocity_ecef_m_per_s[0] * track.velocity_ecef_m_per_s[0] +
                                     track.velocity_ecef_m_per_s[1] * track.velocity_ecef_m_per_s[1] +
                                     track.velocity_ecef_m_per_s[2] * track.velocity_ecef_m_per_s[2]);
      if (prev.valid) {
        const double dv = std::sqrt(
            (track.velocity_ecef_m_per_s[0] - prev.vx) * (track.velocity_ecef_m_per_s[0] - prev.vx) +
            (track.velocity_ecef_m_per_s[1] - prev.vy) * (track.velocity_ecef_m_per_s[1] - prev.vy) +
            (track.velocity_ecef_m_per_s[2] - prev.vz) * (track.velocity_ecef_m_per_s[2] - prev.vz));
        const double rel = dv / std::max(speed, 1.0);
        if (rel > 0.2) {
          std::string event = "等级=3 目标键=" + std::to_string(track.key);
          event += " 事件=轨迹突变 相对Δv=" + FormatF(rel, 3);
          event += " 位置ECEF=" + FormatVec3(track.position.x_m, track.position.y_m, track.position.z_m, 1);
          INFERENCE_ACCEPTANCE_ITEM(row_sim_time, row_cycle, "特殊事件监测与提示", event);
        }
      }
      prev.vx = track.velocity_ecef_m_per_s[0];
      prev.vy = track.velocity_ecef_m_per_s[1];
      prev.vz = track.velocity_ecef_m_per_s[2];
      prev.valid = true;
    }
  }
}

}  // namespace target_inference
