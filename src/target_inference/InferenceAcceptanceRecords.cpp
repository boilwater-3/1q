/**
 * @file InferenceAcceptanceRecords.cpp
 * @brief 推演验收行：轨迹预报 / 落点 / 发射点 / 发布；关机点写暂无。
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

void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              const std::vector<TargetInferenceResult>& results) {
  if (!INFERENCE_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const float sim_time = 0.0f;
  const std::uint32_t cycle = 0U;
  INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, "关机点预测", "暂无");

  for (std::size_t i = 0; i < results.size(); ++i) {
    const TargetInferenceResult& result = results[i];
    const TrajectoryPrediction& traj = result.trajectory;
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
    } else {
      forecast += " 落点经纬高=无";
    }
    forecast += " 误差椭圆半长/半短/方位=(" + FormatF(sigma, 1) + "m," + FormatF(sigma, 1) +
                "m,0°)";
    INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, "目标轨迹预报", forecast);

    if (traj.has_impact) {
      std::string impact = "初步落点经纬高=" + FormatVec3(traj.impact_point.latitude_deg,
                                                          traj.impact_point.longitude_deg,
                                                          traj.impact_point.altitude_m, 3);
      INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, "落点预测", impact);

      std::string publish = "弹道模型=无推力弹道外推";
      publish += " 预测落点=" + FormatVec3(traj.impact_point.latitude_deg,
                                           traj.impact_point.longitude_deg,
                                           traj.impact_point.altitude_m, 3);
      publish += " 误差1σ=" + FormatF(sigma, 1) + "m";
      publish += " 置信度=0.68 标准化封装=无 分发=无";
      INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, "落点预报与信息发布", publish);
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
      INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, "发射点预测", launch);
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
          INFERENCE_ACCEPTANCE_ITEM(sim_time, cycle, "特殊事件监测与提示", event);
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
