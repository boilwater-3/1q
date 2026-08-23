/**
 * @file FusionAcceptanceRecords.cpp
 * @brief 融合验收行：多传感器跟踪 / 接力可写子集 / 协同融合 / UKF。
 */

#include "fusion/FusionAcceptanceRecords.h"

#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>

#include "1q/coordinate/position_transform.h"
#include "common/logging/AcceptanceText.h"
#include "fusion/FusionAcceptanceLog.h"

namespace fusion {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatCov6x6;
using oneq::logging::FormatCovDiag6;
using oneq::logging::FormatVec3;
using oneq::logging::CovarianceTrace6;

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
 * @brief （航迹键, 源）视线角采样状态（会话内累计，仿 LastVelocities 先例）。
 * @note 2026-08-22 甲方批注：剩余覆盖时间以「视场宽度 ÷ 视线角速率」的简易
 *       外推估计（从该源首见该航迹起倒计时）；库内无接力协议。
 */
struct RelaySight {
  double az_deg{0.0};        /**< 最近一次方位角（deg）。 */
  double el_deg{0.0};        /**< 最近一次俯仰角（deg）。 */
  std::uint64_t cycle{0U};   /**< 最近采样周期号。 */
  std::uint64_t first_cycle{0U}; /**< 该（键,源）首见周期号（覆盖倒计时锚点）。 */
  bool valid{false};         /**< 已收到首个方位样本。 */
};

std::map<std::pair<std::uint64_t, std::uint32_t>, RelaySight>& RelaySights() {
  static std::map<std::pair<std::uint64_t, std::uint32_t>, RelaySight> values;
  return values;
}

std::string ChannelNames(const FusedTarget& track) {
  std::set<std::uint32_t> ids;
  for (const ChannelMeasurement& channel : track.channels) {
    ids.insert(channel.source_id);
  }
  std::string text;
  for (std::uint32_t id : ids) {
    if (!text.empty()) {
      text += "+";
    }
    text += "源" + std::to_string(id);
  }
  return text.empty() ? std::string("无") : text;
}

}  // namespace

double AngularSpeedDegPerSec(double az0_deg, double el0_deg, double az1_deg, double el1_deg,
                             double dt_sec) {
  if (dt_sec <= 0.0) {
    return 0.0;
  }
  const double d_az = az1_deg - az0_deg;
  const double d_el = el1_deg - el0_deg;
  return std::sqrt(d_az * d_az + d_el * d_el) / dt_sec;
}

double RelayCoverageSec(double fov_width_deg, double angular_speed_deg_per_s) {
  if (fov_width_deg <= 0.0 || angular_speed_deg_per_s <= 0.0) {
    return -1.0;
  }
  return fov_width_deg / angular_speed_deg_per_s;
}

void WriteFusionAcceptance(std::uint32_t cycle, const std::vector<FusedTarget>& tracks,
                           bool filtering_enabled, const FusionConfig& config) {
  if (!FUSION_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 行前缀仿真时间 = 周期 × 周期时长（此前隐含 dt=1s，按配置单源修正）。
  const float sim_time =
      static_cast<float>(static_cast<double>(cycle) * config.track_cycle_period_sec);
  const std::uint64_t cycle_u64 = static_cast<std::uint64_t>(cycle);
  if (tracks.empty()) {
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器目标跟踪", "暂无");
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "协同探测信息融合", "暂无");
    return;
  }

  std::string multi = "本周期航迹数=" + std::to_string(tracks.size());
  for (const FusedTarget& track : tracks) {
    oneq::coordinate::EcefPositionM ecef{};
    const bool have_ecef = track.has_kinematic_estimate &&
                           oneq::coordinate::TryLlaToEcef(track.kinematic_estimate.position, &ecef);
    std::string content = "航迹键=" + std::to_string(track.key);
    if (have_ecef) {
      content += " ECEF位置m=" + FormatVec3(ecef.x_m, ecef.y_m, ecef.z_m, 1);
    } else {
      content += " ECEF位置m=无";
    }
    if (track.has_kinematic_estimate) {
      content += " 协方差对角=" + FormatCovDiag6(track.kinematic_estimate.covariance_ecef);
    } else {
      content += " 协方差对角=无";
    }
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器目标跟踪", content);

    const auto& vel = track.kinematic_estimate.velocity_ecef_m_per_s;
    LastVel& prev = LastVelocities()[track.key];
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;
    if (prev.valid) {
      ax = vel[0] - prev.vx;
      ay = vel[1] - prev.vy;
      az = vel[2] - prev.vz;
    }
    const double speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
    const double acc = std::sqrt(ax * ax + ay * ay + az * az);
    prev.vx = vel[0];
    prev.vy = vel[1];
    prev.vz = vel[2];
    prev.valid = track.has_kinematic_estimate;

    std::string relay = "目标键=" + std::to_string(track.key);
    relay += " 位置LLA=(";
    relay += FormatF(track.kinematic_estimate.position.latitude_deg, 6) + ",";
    relay += FormatF(track.kinematic_estimate.position.longitude_deg, 6) + ",";
    relay += FormatF(track.kinematic_estimate.position.altitude_m, 1) + ")";
    relay += " 速度模=" + FormatF(speed, 3) + "m/s";
    relay += " 加速度模=" + FormatF(acc, 3) + "m/s²";
    relay += " 预测航路点=无";
    relay += " 融合航迹数=" + std::to_string(tracks.size());
    relay += " 置信度=" + FormatF(track.confidence, 3);

    // 接力三项（2026-08-22 甲方批注）：逐方位源以「视场宽度 ÷ 视线角速率」
    // 外推覆盖时长，自该源首见该航迹起倒计时；取最早离开的源为接力对象。
    std::uint32_t exit_source = 0U;
    double min_remaining_sec = -1.0;
    for (const ChannelMeasurement& channel : track.channels) {
      if (!channel.has_bearing) {
        continue;
      }
      RelaySight& sight = RelaySights()[{track.key, channel.source_id}];
      double remaining = -1.0;
      if (sight.valid && cycle_u64 > sight.cycle) {
        const double dt_sec = static_cast<double>(cycle_u64 - sight.cycle) *
                              config.track_cycle_period_sec;
        const double omega = AngularSpeedDegPerSec(sight.az_deg, sight.el_deg,
                                                   channel.bearing_az_deg,
                                                   channel.bearing_el_deg, dt_sec);
        const double coverage_sec =
            RelayCoverageSec(config.relay_fov_width_deg, omega);
        if (coverage_sec > 0.0) {
          const double elapsed_sec = static_cast<double>(cycle_u64 - sight.first_cycle) *
                                     config.track_cycle_period_sec;
          remaining = coverage_sec - elapsed_sec;
          if (remaining < 0.0) {
            remaining = 0.0;
          }
        }
      }
      if (remaining >= 0.0 && (min_remaining_sec < 0.0 || remaining < min_remaining_sec)) {
        min_remaining_sec = remaining;
        exit_source = channel.source_id;
      }
      if (!sight.valid) {
        sight.first_cycle = cycle_u64;
        sight.valid = true;
      }
      sight.az_deg = channel.bearing_az_deg;
      sight.el_deg = channel.bearing_el_deg;
      sight.cycle = cycle_u64;
    }

    // 交接对象：同航迹上滑窗内仍有量测的其他源（取量测数最多者）；无则如实写无。
    std::uint32_t takeover_source = 0U;
    std::size_t takeover_samples = 0U;
    if (exit_source != 0U) {
      for (const ChannelMeasurement& channel : track.channels) {
        if (channel.source_id == exit_source || channel.sample_count == 0U) {
          continue;
        }
        if (channel.sample_count > takeover_samples) {
          takeover_samples = channel.sample_count;
          takeover_source = channel.source_id;
        }
      }
    }

    if (exit_source != 0U) {
      relay += " 剩余覆盖时间=" + FormatF(min_remaining_sec, 1) + "s(源" +
               std::to_string(exit_source) + ")";
      relay += " 接力计划=源" + std::to_string(exit_source) + "预计" +
               FormatF(min_remaining_sec, 1) + "s离开视场";
      if (takeover_source != 0U) {
        relay += " 交接指令=源" + std::to_string(exit_source) + "→源" +
                 std::to_string(takeover_source) + "@T+" + FormatF(min_remaining_sec, 1) + "s";
      } else {
        relay += " 交接指令=无";
      }
    } else {
      relay += " 剩余覆盖时间=无 接力计划=无 交接指令=无";
    }
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器接力跟踪", relay);

    if (filtering_enabled && track.has_kinematic_estimate) {
      std::string ukf = "位置LLA=(";
      ukf += FormatF(track.kinematic_estimate.position.latitude_deg, 6) + ",";
      ukf += FormatF(track.kinematic_estimate.position.longitude_deg, 6) + ",";
      ukf += FormatF(track.kinematic_estimate.position.altitude_m, 1) + ")";
      if (have_ecef) {
        ukf += " ECEF位置m=" + FormatVec3(ecef.x_m, ecef.y_m, ecef.z_m, 1);
      } else {
        ukf += " ECEF位置m=无";
      }
      ukf += " 速度m/s=" + FormatVec3(vel[0], vel[1], vel[2], 3);
      ukf += " 加速度m/s²=" + FormatVec3(ax, ay, az, 3);
      ukf += " 协方差迹=" + FormatF(CovarianceTrace6(track.kinematic_estimate.covariance_ecef), 2);
      ukf += " 位置估计误差=无";
      ukf += " 完整协方差=" + FormatCov6x6(track.kinematic_estimate.covariance_ecef);
      FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "UKF滤波", ukf);
    }
  }

  std::string collab = "参与通道=" + ChannelNames(tracks.front());
  collab += " 通道数=" + std::to_string(tracks.front().channels.size());
  collab += " 融合置信度=" + FormatF(tracks.front().confidence, 3);
  collab += " 融合目标数=" + std::to_string(tracks.size());
  FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "协同探测信息融合", collab);
}

}  // namespace fusion
