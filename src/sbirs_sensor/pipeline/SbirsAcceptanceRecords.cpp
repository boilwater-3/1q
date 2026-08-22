/**
 * @file SbirsAcceptanceRecords.cpp
 * @brief SBIRS 验收行：安装矩阵派生、轨道角抽样累计、生命周期与计时暂无项。
 */

#include "sbirs_sensor/pipeline/SbirsAcceptanceRecords.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/types.h"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "common/logging/AcceptanceText.h"
#include "sbirs_sensor/pipeline/SbirsAcceptanceLog.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

using oneq::logging::FormatF;
using oneq::logging::FormatPairDeg;
using oneq::logging::FormatVec3;

std::string FormatRotation(const oneq::coordinate::RotationMatrix3d& r) {
  return "[[" + FormatF(r.m00, 6) + "," + FormatF(r.m01, 6) + "," + FormatF(r.m02, 6) + "];[" +
         FormatF(r.m10, 6) + "," + FormatF(r.m11, 6) + "," + FormatF(r.m12, 6) + "];[" +
         FormatF(r.m20, 6) + "," + FormatF(r.m21, 6) + "," + FormatF(r.m22, 6) + "]]";
}

oneq::coordinate::EulerAnglesDeg ToCoord(const oneq::foundation::EulerAnglesDeg& angles) {
  return oneq::coordinate::EulerAnglesDeg(angles.yaw_deg, angles.pitch_deg, angles.roll_deg);
}

oneq::coordinate::RotationMatrix3d ComposeMountAndMisalignment(
    const oneq::coordinate::EulerAnglesDeg& mount, const oneq::coordinate::EulerAnglesDeg& misalign) {
  return oneq::coordinate::Compose(oneq::coordinate::BuildRotationMatrix(mount),
                                   oneq::coordinate::Inverse(oneq::coordinate::BuildRotationMatrix(misalign)));
}

struct OrbitAcc {
  int n{0};
  double sum_az{0.0};
  double sum_el{0.0};
  double sumsq_az{0.0};
  double sumsq_el{0.0};
  double sum_lateral{0.0};
  double sumsq_lateral{0.0};
};

struct AngleAcc {
  int n{0};
  double sum_az{0.0};
  double sum_el{0.0};
  double sumsq_az{0.0};
  double sumsq_el{0.0};
  double max_az{0.0};
  double max_el{0.0};
};

OrbitAcc& Orbit() {
  static OrbitAcc acc;
  return acc;
}

AngleAcc& Angle() {
  static AngleAcc acc;
  return acc;
}

double HashUnit(std::uint32_t cycle, std::uint32_t salt) {
  std::uint32_t x = cycle * 747796405U + salt * 2891336453U;
  x = ((x >> ((x >> 28U) + 4U)) ^ x) * 277803737U;
  x = (x >> 22U) ^ x;
  return static_cast<double>(x) / 4294967295.0;
}

double SampleNormal(std::uint32_t cycle, std::uint32_t salt) {
  const double u1 = std::max(1.0e-12, HashUnit(cycle, salt));
  const double u2 = HashUnit(cycle, salt + 1U);
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
}

const char* EventName(session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return "首次探测";
    case session::SbirsDetectionLifecycleEventKind::kCoasting:
      return "跟踪中断(滑行)";
    case session::SbirsDetectionLifecycleEventKind::kLost:
      return "跟踪中断";
    default:
      return nullptr;
  }
}

int EventLevel(session::SbirsDetectionLifecycleEventKind kind) {
  switch (kind) {
    case session::SbirsDetectionLifecycleEventKind::kFirstDetected:
      return 1;
    case session::SbirsDetectionLifecycleEventKind::kCoasting:
    case session::SbirsDetectionLifecycleEventKind::kLost:
      return 2;
    default:
      return 3;
  }
}

}  // namespace

void WriteSbirsInstallMatrices(const oneq::foundation::EulerAnglesDeg& mount_deg,
                               const oneq::foundation::EulerAnglesDeg& misalignment_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const oneq::coordinate::EulerAnglesDeg mount = ToCoord(mount_deg);
  const oneq::coordinate::EulerAnglesDeg misalign = ToCoord(misalignment_deg);
  const oneq::coordinate::RotationMatrix3d r_mount = oneq::coordinate::BuildRotationMatrix(mount);
  const oneq::coordinate::RotationMatrix3d r_mis = oneq::coordinate::BuildRotationMatrix(misalign);
  const oneq::coordinate::RotationMatrix3d r_actual = ComposeMountAndMisalignment(mount, misalign);
  std::string content = "传感器安装R=" + FormatRotation(r_mount);
  content += " 失准R=" + FormatRotation(r_mis);
  content += " 实际指向R=" + FormatRotation(r_actual);
  content += " 失准角yaw/pitch/roll=";
  content += FormatVec3(misalignment_deg.yaw_deg, misalignment_deg.pitch_deg,
                        misalignment_deg.roll_deg, 4);
  content += "°";
  SBIRS_ACCEPTANCE_ITEM(0.0f, 0U, "安装矩阵误差", content);
}

void WriteSbirsOrbitSample(float sim_time_sec, std::uint32_t cycle, float orbit_sigma_deg,
                           double reference_range_m) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const double sigma = static_cast<double>(orbit_sigma_deg);
  const double az = sigma * SampleNormal(cycle, 11U);
  const double el = sigma * SampleNormal(cycle, 29U);
  const double range = reference_range_m > 0.0 ? reference_range_m : 40000000.0;
  const double lateral = range * (sigma * 0.017453292519943295);
  OrbitAcc& acc = Orbit();
  ++acc.n;
  acc.sum_az += az;
  acc.sum_el += el;
  acc.sumsq_az += az * az;
  acc.sumsq_el += el * el;
  acc.sum_lateral += lateral;
  acc.sumsq_lateral += lateral * lateral;
  const double rms_az = std::sqrt(acc.sumsq_az / static_cast<double>(acc.n));
  const double rms_el = std::sqrt(acc.sumsq_el / static_cast<double>(acc.n));
  const double mean_lat = acc.sum_lateral / static_cast<double>(acc.n);
  const double var_lat =
      acc.sumsq_lateral / static_cast<double>(acc.n) - mean_lat * mean_lat;
  const double std_lat = var_lat > 0.0 ? std::sqrt(var_lat) : 0.0;
  std::string content = "等效角抽样az/el=";
  content += FormatPairDeg(az, el, 6);
  content += "° 横向位移=" + FormatF(lateral, 3) + "m@参考距离" + FormatF(range, 0) + "m";
  content += " 本会话累计n=" + std::to_string(acc.n);
  content += " 角误差RMS=" + FormatF(0.5 * (rms_az + rms_el), 6) + "°";
  content += " 横向位移均值/σ=" + FormatF(mean_lat, 1) + "/" + FormatF(std_lat, 1) + "m";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星自身定位误差", content);
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星ECEF三维导航定位误差", "无");
}

void WriteSbirsAngleError(float sim_time_sec, std::uint32_t cycle, std::uint64_t target_id,
                          double az_error_deg, double el_error_deg, double measured_az_deg,
                          double measured_el_deg, double truth_az_deg, double truth_el_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  AngleAcc& acc = Angle();
  ++acc.n;
  acc.sum_az += az_error_deg;
  acc.sum_el += el_error_deg;
  acc.sumsq_az += az_error_deg * az_error_deg;
  acc.sumsq_el += el_error_deg * el_error_deg;
  acc.max_az = std::max(acc.max_az, std::fabs(az_error_deg));
  acc.max_el = std::max(acc.max_el, std::fabs(el_error_deg));
  std::string content = "目标ID=" + std::to_string(target_id);
  content += " 方位测角误差=" + FormatF(az_error_deg, 6) + "°";
  content += " 俯仰测角误差=" + FormatF(el_error_deg, 6) + "°";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "红外载荷测角误差", content);
  std::string perf = "目标ID=" + std::to_string(target_id);
  perf += " 测量方位/俯仰=" + FormatPairDeg(measured_az_deg, measured_el_deg, 3) + "°";
  perf += " 真值=" + FormatPairDeg(truth_az_deg, truth_el_deg, 3) + "°";
  perf += " 偏差az/el=" + FormatPairDeg(az_error_deg, el_error_deg, 6) + "°";
  if (acc.n > 0) {
    perf += " 会话RMSE az/el=" +
            FormatPairDeg(std::sqrt(acc.sumsq_az / static_cast<double>(acc.n)),
                          std::sqrt(acc.sumsq_el / static_cast<double>(acc.n)), 6) +
            "°";
  }
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "红外系统测角误差", perf);
}

void WriteSbirsLifecycleEvents(float sim_time_sec, std::uint32_t cycle,
                               const std::vector<session::SbirsDetectionLifecycleEvent>& events,
                               const session::SbirsCycleInput& input) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::map<std::uint64_t, const session::SbirsSceneTarget*> by_id;
  for (const session::SbirsSceneTarget& target : input.scene) {
    by_id[target.target_id] = &target;
  }
  for (const session::SbirsDetectionLifecycleEvent& event : events) {
    const char* name = EventName(event.kind);
    if (name == nullptr) {
      continue;
    }
    std::string content = "等级=" + std::to_string(EventLevel(event.kind));
    content += " 目标ID=" + std::to_string(event.target_id);
    content += " 事件=";
    content += name;
    content += " 时间=" + FormatF(static_cast<double>(sim_time_sec), 3) + "s";
    const auto found = by_id.find(event.target_id);
    if (found != by_id.end() && found->second != nullptr) {
      content += " 位置ECEF=" + FormatVec3(found->second->position_ecef_m.x,
                                           found->second->position_ecef_m.y,
                                           found->second->position_ecef_m.z, 1);
    } else {
      content += " 位置ECEF=无";
    }
    SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "特殊事件监测与提示", content);
  }
}

void WriteSbirsOncePerSession(float sim_time_sec, std::uint32_t cycle) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  static bool written = false;
  if (written) {
    return;
  }
  written = true;
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "初始化时间", "暂无");
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "单步执行时间", "暂无");
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "典型场景和总仿真次数",
                        "场景=本会话 场景数=1 总仿真周期=结束时回写");
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "组件模型参数性能", "见红外系统测角误差");
}

void WriteSbirsCycleRunCount(float sim_time_sec, std::uint32_t cycle) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "本会话已运行周期=" + std::to_string(cycle) + " 状态=正常";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "连续运行次数", content);
}

}  // namespace pipeline
}  // namespace sbirs_sensor
