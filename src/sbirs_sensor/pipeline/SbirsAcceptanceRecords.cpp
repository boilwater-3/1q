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

oneq::coordinate::RotationMatrix3d ComposeMountAndMisalignment(
    const oneq::coordinate::EulerAnglesDeg& mount, const oneq::coordinate::EulerAnglesDeg& misalign) {
  return oneq::coordinate::Compose(oneq::coordinate::BuildRotationMatrix(mount),
                                   oneq::coordinate::Inverse(oneq::coordinate::BuildRotationMatrix(misalign)));
}

struct OrbitAcc {
  int n{0};
  double sum_err{0.0};
  double sumsq_err{0.0};
  double sum_c0{0.0};  /**< 分量0累计（东；真值ECEF缺失时为X）。 */
  double sum_c1{0.0};  /**< 分量1累计（北；真值ECEF缺失时为Y）。 */
  double sum_c2{0.0};  /**< 分量2累计（天/径向；真值ECEF缺失时为Z）。 */
  double sumsq_c0{0.0};
  double sumsq_c1{0.0};
  double sumsq_c2{0.0};
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

std::map<const void*, OrbitAcc>& OrbitAccs() {
  static std::map<const void*, OrbitAcc> accs;
  return accs;
}

std::map<const void*, AngleAcc>& AngleAccs() {
  static std::map<const void*, AngleAcc> accs;
  return accs;
}

OrbitAcc& Orbit(const void* instance_key) {
  return OrbitAccs()[instance_key];
}

AngleAcc& Angle(const void* instance_key) {
  return AngleAccs()[instance_key];
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

const char* StageName(const session::SbirsDetectionLifecycleEvent& event) {
  switch (event.observation_stage) {
    case output::SbirsObservationStage::kWideFieldSearch:
      return "宽视场搜索";
    case output::SbirsObservationStage::kNarrowFieldAcquisition:
      return "窄视场捕获";
    case output::SbirsObservationStage::kNarrowFieldTrack:
      return "窄视场跟踪";
    default:
      return "未知阶段";
  }
}

}  // namespace

void WriteSbirsInstallMatrices(const oneq::coordinate::EulerAnglesDeg& mount_deg,
                               const oneq::coordinate::EulerAnglesDeg& misalignment_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const oneq::coordinate::RotationMatrix3d r_mount = oneq::coordinate::BuildRotationMatrix(mount_deg);
  const oneq::coordinate::RotationMatrix3d r_mis =
      oneq::coordinate::BuildRotationMatrix(misalignment_deg);
  const oneq::coordinate::RotationMatrix3d r_actual =
      ComposeMountAndMisalignment(mount_deg, misalignment_deg);
  std::string content = "传感器安装R=" + FormatRotation(r_mount);
  content += " 失准R=" + FormatRotation(r_mis);
  content += " 实际指向R=" + FormatRotation(r_actual);
  content += " 失准角yaw/pitch/roll=";
  content += FormatVec3(misalignment_deg.yaw_deg, misalignment_deg.pitch_deg,
                        misalignment_deg.roll_deg, 4);
  content += "°";
  SBIRS_ACCEPTANCE_ITEM(0.0f, 0U, "安装矩阵误差", content);
}

void WriteSbirsOrbitSample(const void* instance_key, float sim_time_sec, std::uint32_t cycle,
                           float orbit_sigma_deg, double reference_range_m,
                           float nav_position_sigma_m,
                           const session::SbirsVector3M& satellite_ecef) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const double sigma = static_cast<double>(orbit_sigma_deg);
  const double nav_sigma = static_cast<double>(nav_position_sigma_m);
  // 卫星位置混入盐值：双星同进程同周期时两星抽样相互独立（否则逐拍同值）。
  const std::uint32_t sat_salt =
      static_cast<std::uint32_t>(std::fabs(satellite_ecef.x) * 1.0e-3) * 31U +
      static_cast<std::uint32_t>(std::fabs(satellite_ecef.y) * 1.0e-3) * 17U +
      static_cast<std::uint32_t>(std::fabs(satellite_ecef.z) * 1.0e-3);
  const double az = sigma * SampleNormal(cycle, 11U + sat_salt);
  const double el = sigma * SampleNormal(cycle, 29U + sat_salt);
  const double range = reference_range_m > 0.0 ? reference_range_m : 40000000.0;

  // 评审 2026-08-26 条1 + 2026-08-27 条4：卫星自身定位误差 = 导航定位误差统计
  // （东/北由轨道角抽样弧长映射 + 径向按导航 σ 独立抽样；轨道角 σ 为 0 的场景
  // 仍有径向分量，不再整行恒 0——旧版只统计轨道角 σ×参考距离的横向位移）。
  // 真值（传入 ECEF）不可用时退化为按导航 σ 三维随机抽样（分量记 X/Y/Z）。
  const double r_norm = std::sqrt(satellite_ecef.x * satellite_ecef.x +
                                  satellite_ecef.y * satellite_ecef.y +
                                  satellite_ecef.z * satellite_ecef.z);
  const double radial = nav_sigma * SampleNormal(cycle, 47U + sat_salt);
  bool have_truth_ecef = r_norm > 0.0;
  double dx;
  double dy;
  double dz;
  if (have_truth_ecef) {
    const double kDegToRad = 0.017453292519943295;
    // ENU 分量（统计行直用）：东/北 = 轨道角抽样弧长，天 = 径向抽样。
    dx = az * kDegToRad * range;
    dy = el * kDegToRad * range;
    dz = radial;
  } else {
    dx = nav_sigma * SampleNormal(cycle, 53U + sat_salt);
    dy = nav_sigma * SampleNormal(cycle, 59U + sat_salt);
    dz = radial;
  }
  const double err_norm = std::sqrt(dx * dx + dy * dy + dz * dz);

  OrbitAcc& acc = Orbit(instance_key);
  ++acc.n;
  acc.sum_err += err_norm;
  acc.sumsq_err += err_norm * err_norm;
  acc.sum_c0 += dx;
  acc.sum_c1 += dy;
  acc.sum_c2 += dz;
  acc.sumsq_c0 += dx * dx;
  acc.sumsq_c1 += dy * dy;
  acc.sumsq_c2 += dz * dz;
  const double mean_err = acc.sum_err / static_cast<double>(acc.n);
  const double rms_err = std::sqrt(acc.sumsq_err / static_cast<double>(acc.n));
  const double var0 = acc.sumsq_c0 / static_cast<double>(acc.n) -
                      (acc.sum_c0 / static_cast<double>(acc.n)) * (acc.sum_c0 / static_cast<double>(acc.n));
  const double var1 = acc.sumsq_c1 / static_cast<double>(acc.n) -
                      (acc.sum_c1 / static_cast<double>(acc.n)) * (acc.sum_c1 / static_cast<double>(acc.n));
  const double var2 = acc.sumsq_c2 / static_cast<double>(acc.n) -
                      (acc.sum_c2 / static_cast<double>(acc.n)) * (acc.sum_c2 / static_cast<double>(acc.n));
  const char* comp_label = have_truth_ecef ? "东/北/天" : "X/Y/Z";
  std::string content = "本拍导航误差(";
  content += comp_label;
  content += ")=(" + FormatF(dx, 1) + "," + FormatF(dy, 1) + "," + FormatF(dz, 1) + ")m";
  content += " |误差|=" + FormatF(err_norm, 1) + "m";
  content += " 本会话累计n=" + std::to_string(acc.n);
  content += " |误差|均值=" + FormatF(mean_err, 1) + "m |误差|RMS=" + FormatF(rms_err, 1) + "m";
  content += " 分量σ(";
  content += comp_label;
  content += ")=(" + FormatF(std::sqrt(std::max(var0, 0.0)), 1) + "," +
             FormatF(std::sqrt(std::max(var1, 0.0)), 1) + "," +
             FormatF(std::sqrt(std::max(var2, 0.0)), 1) + ")m";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星自身定位误差", content);

  if (have_truth_ecef) {
    // ECEF 误差向量 = ENU 分量经本地东北天基旋转合成。
    const double up_x = satellite_ecef.x / r_norm;
    const double up_y = satellite_ecef.y / r_norm;
    const double up_z = satellite_ecef.z / r_norm;
    // 东向单位矢 = ẑ×r̂ = (-ry, rx, 0)/|·|（极轨卫星不在极点，叉积非退化；退化时
    // 回退到经度 0 的本地东北天基）；北向单位矢 = r̂×east。
    const double east_norm = std::sqrt(satellite_ecef.x * satellite_ecef.x +
                                       satellite_ecef.y * satellite_ecef.y);
    double east_x;
    double east_y;
    if (east_norm > 1.0e-6) {
      east_x = -satellite_ecef.y / east_norm;
      east_y = satellite_ecef.x / east_norm;
    } else {
      east_x = 0.0;
      east_y = 1.0;
    }
    const double north_x = -up_z * east_y;
    const double north_y = up_z * east_x;
    const double north_z = up_x * east_y - up_y * east_x;
    const double vec_x = dx * east_x + dy * north_x + dz * up_x;
    const double vec_y = dx * east_y + dy * north_y + dz * up_y;
    const double vec_z = dy * north_z + dz * up_z;
    std::string nav = "真值ECEF=";
    nav += FormatVec3(satellite_ecef.x, satellite_ecef.y, satellite_ecef.z, 1);
    nav += "m 错误三维ECEF=";
    nav += FormatVec3(satellite_ecef.x + vec_x, satellite_ecef.y + vec_y,
                      satellite_ecef.z + vec_z, 1);
    nav += "m 误差向量=(" + FormatF(vec_x, 1) + "," + FormatF(vec_y, 1) + "," +
           FormatF(vec_z, 1) + ")m |误差|=" + FormatF(err_norm, 1);
    nav += "m 口径=抽样角经参考距离弧长映射至本地水平面(东/北)+径向按导航σ" +
           FormatF(nav_sigma, 1) + "m抽样";
    SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星ECEF三维导航定位误差", nav);
  } else {
    // 传入 ECEF 不可用：仅给出按导航 σ 三维抽样的误差向量（评审条1「假设找不到
    // 则使用传入的ECEF进行随机数」——此处连传入 ECEF 都缺失，位置字段如实记不可用）。
    std::string nav = "真值ECEF=不可用 错误三维ECEF=不可用";
    nav += " 误差向量=(" + FormatF(dx, 1) + "," + FormatF(dy, 1) + "," + FormatF(dz, 1) +
           ")m |误差|=" + FormatF(err_norm, 1);
    nav += "m 口径=卫星位置不可用,按导航σ" + FormatF(nav_sigma, 1) + "m三维随机抽样";
    SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星ECEF三维导航定位误差", nav);
  }
}

void WriteSbirsAngleError(const void* instance_key, float sim_time_sec, std::uint32_t cycle,
                          std::uint64_t target_id, double az_error_deg, double el_error_deg,
                          double measured_az_deg, double measured_el_deg, double truth_az_deg,
                          double truth_el_deg, float sigma_orbit_deg, float sigma_attitude_deg,
                          float sigma_fov_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  // 评审 2026-08-26 条24：统计按管线实例（每星）分离，双星同进程不再混计；
  // 行内标注 σ 三分项配置来源，便于评审核对量级。
  AngleAcc& acc = Angle(instance_key);
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
    perf += " 本星会话RMSE az/el=" +
            FormatPairDeg(std::sqrt(acc.sumsq_az / static_cast<double>(acc.n)),
                          std::sqrt(acc.sumsq_el / static_cast<double>(acc.n)), 6) +
            "°";
  }
  // σ 标注 6 位小数：甲方 2026-08-27 指标（红外系统测角误差 ≤3 μrad =
  // 0.000172°）下默认 σ 在 1e-4° 量级，3 位小数会全舍成 0.000 造成「参数为零
  // 但误差非零」的误读。
  perf += " σ来源(轨道/姿态/视场)=" + FormatF(sigma_orbit_deg, 6) + "/" +
          FormatF(sigma_attitude_deg, 6) + "/" + FormatF(sigma_fov_deg, 6) + "°";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "红外系统测角误差", perf);
}

void WriteSbirsAngleStateEstimate(float sim_time_sec, std::uint32_t cycle, std::uint64_t target_id,
                                  double azimuth_deg, double elevation_deg,
                                  double azimuth_rate_deg_per_s, double elevation_rate_deg_per_s) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  std::string content = "目标ID=" + std::to_string(target_id);
  content += " 滤波方位=" + FormatF(azimuth_deg, 6) + "°";
  content += " 滤波俯仰=" + FormatF(elevation_deg, 6) + "°";
  content += " 方位变化率=" + FormatF(azimuth_rate_deg_per_s, 8) + "°/s";
  content += " 俯仰变化率=" + FormatF(elevation_rate_deg_per_s, 8) + "°/s";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "目标角度状态估计", content);
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
    // 评审 2026-08-26 条11：事件行标注观测阶段，区分宽场首次探测与窄场（捕获/
    // 跟踪）首次探测。
    content += " 阶段=";
    content += StageName(event);
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
  // 评审 2026-08-26 条22（方案B）：库内不做墙钟计时，真实初始化耗时在示例层
  // integration_events.log 的同名验收项（模块=SBIRS）。
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "初始化时间",
                        "见integration_events.log[验收项：初始化时间]（模块=SBIRS）");
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
