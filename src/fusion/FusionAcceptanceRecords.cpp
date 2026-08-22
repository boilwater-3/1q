/**
 * @file FusionAcceptanceRecords.cpp
 * @brief 融合验收行：多传感器跟踪 / 接力可写子集 / 协同融合 / UKF。
 */

#include "fusion/FusionAcceptanceRecords.h"

#include <cmath>
#include <map>
#include <set>
#include <string>

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

void WriteFusionAcceptance(std::uint32_t cycle, const std::vector<FusedTarget>& tracks,
                           bool filtering_enabled) {
  if (!FUSION_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const float sim_time = static_cast<float>(cycle);
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

    std::string relay = "位置LLA=(";
    relay += FormatF(track.kinematic_estimate.position.latitude_deg, 6) + ",";
    relay += FormatF(track.kinematic_estimate.position.longitude_deg, 6) + ",";
    relay += FormatF(track.kinematic_estimate.position.altitude_m, 1) + ")";
    relay += " 速度模=" + FormatF(speed, 3) + "m/s";
    relay += " 加速度模=" + FormatF(acc, 3) + "m/s²";
    relay += " 预测航路点=无";
    relay += " 融合航迹数=" + std::to_string(tracks.size());
    relay += " 置信度=" + FormatF(track.confidence, 3);
    relay += " 剩余覆盖时间=无 接力计划=无 交接指令=无";
    FUSION_ACCEPTANCE_ITEM(sim_time, cycle, "多传感器接力跟踪", relay);

    if (filtering_enabled && track.has_kinematic_estimate) {
      std::string ukf = "位置ENU=由LLA换算";
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
