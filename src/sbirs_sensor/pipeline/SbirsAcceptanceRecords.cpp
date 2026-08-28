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

struct AngleAcc {
  int n{0};
  double sum_az{0.0};
  double sum_el{0.0};
  double sumsq_az{0.0};
  double sumsq_el{0.0};
  double max_az{0.0};
  double max_el{0.0};
};

std::map<const void*, AngleAcc>& AngleAccs() {
  static std::map<const void*, AngleAcc> accs;
  return accs;
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

void WriteSbirsInstallMatrices(std::uint32_t satellite_id,
                               const oneq::coordinate::EulerAnglesDeg& mount_deg,
                               const oneq::coordinate::EulerAnglesDeg& misalignment_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  const oneq::coordinate::RotationMatrix3d r_mount = oneq::coordinate::BuildRotationMatrix(mount_deg);
  const oneq::coordinate::RotationMatrix3d r_mis =
      oneq::coordinate::BuildRotationMatrix(misalignment_deg);
  const oneq::coordinate::RotationMatrix3d r_actual =
      ComposeMountAndMisalignment(mount_deg, misalignment_deg);
  // 规范口径（验收判定标准 第10项）：只写理想指向R/实际指向R/安装误差矩阵R + 卫星ID，
  // 不输出失准角欧拉角字段。
  std::string content = "卫星ID=" + std::to_string(satellite_id);
  content += " 理想指向R=" + FormatRotation(r_mount);
  content += " 实际指向R=" + FormatRotation(r_actual);
  content += " 安装误差矩阵R=" + FormatRotation(r_mis);
  SBIRS_ACCEPTANCE_ITEM(0.0f, 0U, "安装矩阵误差功能测试", content);
}

void WriteSbirsOrbitSample(std::uint32_t satellite_id, float sim_time_sec, std::uint32_t cycle,
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

  // 定位误差抽样口径（评审 2026-08-26 条1 + 2026-08-27 条4）：东/北 = 轨道角抽样
  // 弧长映射（轨道角 σ 为 0 的场景仍有径向分量），天 = 径向按导航 σ 独立抽样；
  // 真值（传入 ECEF）不可用时退化为按导航 σ 三维随机抽样（分量记 X/Y/Z）。
  const double r_norm = std::sqrt(satellite_ecef.x * satellite_ecef.x +
                                  satellite_ecef.y * satellite_ecef.y +
                                  satellite_ecef.z * satellite_ecef.z);
  const double radial = nav_sigma * SampleNormal(cycle, 47U + sat_salt);
  const bool have_truth_ecef = r_norm > 0.0;
  double dx;
  double dy;
  double dz;
  if (have_truth_ecef) {
    const double kDegToRad = 0.017453292519943295;
    dx = az * kDegToRad * range;
    dy = el * kDegToRad * range;
    dz = radial;
  } else {
    dx = nav_sigma * SampleNormal(cycle, 53U + sat_salt);
    dy = nav_sigma * SampleNormal(cycle, 59U + sat_salt);
    dz = radial;
  }
  const double err_norm = std::sqrt(dx * dx + dy * dy + dz * dz);

  // 规范口径（验收判定标准 第8项）：只写真值ECEF/实际ECEF/定位误差ECEF/模长 + 卫星ID；
  // 不输出东/北/天统计、会话 n、均值/RMS、分量 σ 与口径说明。
  std::string nav = "卫星ID=" + std::to_string(satellite_id);
  if (have_truth_ecef) {
    // ECEF 误差向量 = ENU 分量经本地东北天基旋转合成；实际 ECEF = 真值 + 误差向量。
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
    nav += " 真值ECEF=";
    nav += FormatVec3(satellite_ecef.x, satellite_ecef.y, satellite_ecef.z, 1);
    nav += "m 实际ECEF=";
    nav += FormatVec3(satellite_ecef.x + vec_x, satellite_ecef.y + vec_y,
                      satellite_ecef.z + vec_z, 1);
    nav += "m 定位误差ECEF=(" + FormatF(vec_x, 1) + "," + FormatF(vec_y, 1) + "," +
           FormatF(vec_z, 1) + ")m";
  } else {
    // 传入 ECEF 不可用：位置字段如实记不可用，误差向量按导航 σ 三维抽样。
    nav += " 真值ECEF=不可用 实际ECEF=不可用";
    nav += " 定位误差ECEF=(" + FormatF(dx, 1) + "," + FormatF(dy, 1) + "," + FormatF(dz, 1) +
           ")m";
  }
  nav += " |定位误差|=" + FormatF(err_norm, 1) + "m";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "卫星自身定位误差功能测试", nav);
}

void WriteSbirsAngleError(const void* instance_key, std::uint32_t satellite_id,
                          float sim_time_sec, std::uint32_t cycle,
                          std::uint64_t target_id, double az_error_deg, double el_error_deg,
                          double measured_az_deg, double measured_el_deg, double truth_az_deg,
                          double truth_el_deg, float sigma_orbit_deg, float sigma_attitude_deg,
                          float sigma_fov_deg) {
  if (!SBIRS_ACCEPTANCE_LOG_ENABLED()) {
    return;
  }
  constexpr double kDegToRad = 0.017453292519943295;
  // 规范口径（验收判定标准 第9项）：只写 ECI 测角残差（量测−真值，rad）+
  // 相对卫星ID/目标ID；测量/真值角本身、会话 RMSE、σ 来源不写进本项。
  std::string content = "相对卫星ID=" + std::to_string(satellite_id);
  content += " 目标ID=" + std::to_string(target_id);
  content += " 测角残差方位/俯仰(ECI)=(" + FormatF(az_error_deg * kDegToRad, 8) + "," +
             FormatF(el_error_deg * kDegToRad, 8) + ")rad";
  SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "红外载荷角误差功能测试", content);
  // 评审 2026-08-26 条24：统计按管线实例（每星）分离，双星同进程不再混计；
  // 行内标注 σ 三分项配置来源，便于评审核对量级。（红外系统测角误差性能测试行，
  // 验收判定标准 第55项。）
  AngleAcc& acc = Angle(instance_key);
  ++acc.n;
  acc.sum_az += az_error_deg;
  acc.sum_el += el_error_deg;
  acc.sumsq_az += az_error_deg * az_error_deg;
  acc.sumsq_el += el_error_deg * el_error_deg;
  acc.max_az = std::max(acc.max_az, std::fabs(az_error_deg));
  acc.max_el = std::max(acc.max_el, std::fabs(el_error_deg));
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

void WriteSbirsLifecycleEvents(std::uint32_t satellite_id, float sim_time_sec,
                               std::uint32_t cycle,
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
    // 规范口径（验收判定标准 第24项）：「验收内容：」后首字段为 事件=，随后卫星ID/
    // 等级/目标ID/阶段/时间/位置（评审 2026-08-26 条11：事件行标注观测阶段，区分
    // 宽场首次探测与窄场（捕获/跟踪）首次探测）。
    std::string content = "事件=";
    content += name;
    content += " 卫星ID=" + std::to_string(satellite_id);
    content += " 等级=" + std::to_string(EventLevel(event.kind));
    content += " 目标ID=" + std::to_string(event.target_id);
    content += " 阶段=";
    content += StageName(event);
    content += " 时间=" + FormatF(static_cast<double>(sim_time_sec), 3) + "s";
    const auto found = by_id.find(event.target_id);
    if (found != by_id.end() && found->second != nullptr) {
      content += " 位置ECEF=" + FormatVec3(found->second->position_ecef_m.x,
                                           found->second->position_ecef_m.y,
                                           found->second->position_ecef_m.z, 1);
    }
    SBIRS_ACCEPTANCE_ITEM(sim_time_sec, cycle, "特殊事件监测与提示功能测试", content);
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
