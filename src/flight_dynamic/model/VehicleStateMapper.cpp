/**
 * @file VehicleStateMapper.cpp
 * @brief JSBSim 积分状态 → 1Q FlightDynamicOutput 坐标/单位转换实现。
 */

#include "flight_dynamic/model/VehicleStateMapper.h"

#include <cmath>
#include <stdexcept>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/coordinate/attitude_transform.h"

// JSBSim
#include "FGFDMExec.h"
#include "models/FGPropagate.h"
#include "models/FGAccelerations.h"
#include "math/FGLocation.h"
#include "math/FGColumnVector3.h"
#include "math/FGQuaternion.h"

namespace flight_dynamic {
namespace model {

namespace {

// ---- 单位换算常数 ----
constexpr double kFtToM = 0.3048;
constexpr double kFtpsToMps = kFtToM;          // ft/s → m/s
constexpr double kKnotsToMps = 0.514444;
constexpr double kFtps2ToMps2 = kFtToM;        // ft/s² → m/s²
// 角速度 rad/s — JSBSim 内部已是 rad/s，无需换算

/**
 * @brief 从 JSBSim FGLocation（ECEF，ft）提取 1Q EcefPositionM（m）。
 */
oneq::coordinate::EcefPositionM LocationToEcef(const JSBSim::FGLocation& loc) {
  oneq::coordinate::EcefPositionM ecef{};
  ecef.x_m = loc(1) * kFtToM;  // FGLocation 使用 1-indexed，单位 ft
  ecef.y_m = loc(2) * kFtToM;
  ecef.z_m = loc(3) * kFtToM;
  return ecef;
}

/**
 * @brief 从 JSBSim FGLocation 提取 LLA（deg, m）。
 */
oneq::coordinate::LlaPositionDegM LocationToLla(const JSBSim::FGLocation& loc) {
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = loc.GetGeodLatitudeDeg();
  lla.longitude_deg = loc.GetLongitudeDeg();
  lla.altitude_m = loc.GetGeodAltitude() * kFtToM;
  return lla;
}

}  // namespace

FlightDynamicOutput VehicleStateMapper::Map(
    const JSBSim::FGPropagate& propagate,
    const JSBSim::FGAccelerations& accelerations,
    JSBSim::FGFDMExec& fdm_exec) const {
  FlightDynamicOutput out{};
  out.ok = true;

  // ---- C2a: ECEF 位置（直接映射）----
  MapPosition(propagate, out);

  // ---- C2b/C2c: 速度（NED→ENU→ECEF）+ 姿态（NED四元数→ENU欧拉角）----
  // 先提取 LLA（MapVelocity 和 MapAttitude 均需要）
  const auto& loc = propagate.GetLocation();
  const auto lla = LocationToLla(loc);
  out.state.altitude_msl_m = lla.altitude_m;

  MapVelocity(propagate, lla, out);
  MapAttitude(propagate, out);
  MapAngularVelocity(propagate, out);

  // ---- C2d: 加速度（ft/s² → m/s²）----
  MapAcceleration(accelerations, out);

  // ---- 气动参考量 ----
  MapAeroParams(fdm_exec, out);

  // ---- ExternalKinematics 填充（供传感器模块直接消费）----
  out.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  out.kinematics.position_ecef_m.x_m = out.state.position_ecef_x_m;
  out.kinematics.position_ecef_m.y_m = out.state.position_ecef_y_m;
  out.kinematics.position_ecef_m.z_m = out.state.position_ecef_z_m;
  out.kinematics.velocity_mps.x_mps = out.state.velocity_ecef_x_mps;
  out.kinematics.velocity_mps.y_mps = out.state.velocity_ecef_y_mps;
  out.kinematics.velocity_mps.z_mps = out.state.velocity_ecef_z_mps;
  out.kinematics.attitude_deg.yaw_deg = out.state.yaw_deg;
  out.kinematics.attitude_deg.pitch_deg = out.state.pitch_deg;
  out.kinematics.attitude_deg.roll_deg = out.state.roll_deg;

  return out;
}

// ---- 私有方法实现 ----

void VehicleStateMapper::MapPosition(const JSBSim::FGPropagate& propagate,
                                      FlightDynamicOutput& out) const {
  // C2a: JSBSim vLocation (ECEF, ft) → 1Q EcefPositionM (m)
  const auto& loc = propagate.GetLocation();
  const auto ecef = LocationToEcef(loc);
  out.state.position_ecef_x_m = ecef.x_m;
  out.state.position_ecef_y_m = ecef.y_m;
  out.state.position_ecef_z_m = ecef.z_m;
}

void VehicleStateMapper::MapVelocity(const JSBSim::FGPropagate& propagate,
                                      const oneq::coordinate::LlaPositionDegM& lla,
                                      FlightDynamicOutput& out) const {
  // JSBSim vUVW 是 Body 系相对 ECEF 的速度（ft/s）
  // 需要将 Body 速度转换为 NED 速度，再转 ENU，再转 ECEF
  // 实际上 JSBSim 也提供 vVel（NED 系速度），直接使用更简单
  const JSBSim::FGColumnVector3& vel_ned_fps = propagate.GetVel();  // NED, ft/s
  oneq::coordinate::NedVelocityMps ned_vel{};
  ned_vel.north_mps = vel_ned_fps(1) * kFtpsToMps;
  ned_vel.east_mps  = vel_ned_fps(2) * kFtpsToMps;
  ned_vel.down_mps  = vel_ned_fps(3) * kFtpsToMps;

  // C2b: NED → ENU 轴重排
  const auto enu_vel = oneq::coordinate::ToEnuVelocity(ned_vel);

  // ENU → ECEF
  oneq::coordinate::EcefVelocityMps ecef_vel{};
  if (oneq::coordinate::TryEnuToEcefVelocity(enu_vel, lla, &ecef_vel)) {
    out.state.velocity_ecef_x_mps = ecef_vel.x_mps;
    out.state.velocity_ecef_y_mps = ecef_vel.y_mps;
    out.state.velocity_ecef_z_mps = ecef_vel.z_mps;
  }

  // 地速（NED 合速度的水平分量）
  out.state.ground_speed_mps = std::sqrt(
      ned_vel.north_mps * ned_vel.north_mps +
      ned_vel.east_mps  * ned_vel.east_mps);
}

void VehicleStateMapper::MapAttitude(const JSBSim::FGPropagate& propagate,
                                      FlightDynamicOutput& out) const {
  // C2c: JSBSim qAttitudeLocal（Body→NED 四元数）→ 1Q Body→ENU 欧拉角
  // 步骤：四元数 → NED 欧拉角 → ToEnuAttitude → ENU 欧拉角
  const JSBSim::FGQuaternion& q = propagate.GetQuaternion();

  // JSBSim 四元数到欧拉角（已是 NED 系，deg）
  // FGQuaternion::GetEuler() 返回 (phi=roll, theta=pitch, psi=yaw) in radians
  const JSBSim::FGColumnVector3 euler_rad = q.GetEuler();
  constexpr double kRadToDeg = 180.0 / M_PI;
  oneq::coordinate::EulerAnglesDeg ned_att{};
  ned_att.yaw_deg   = euler_rad(3) * kRadToDeg;  // psi
  ned_att.pitch_deg = euler_rad(2) * kRadToDeg;  // theta
  ned_att.roll_deg  = euler_rad(1) * kRadToDeg;  // phi

  // 规范化 yaw 到 [0, 360)
  if (ned_att.yaw_deg < 0.0) {
    ned_att.yaw_deg += 360.0;
  }

  // NED → ENU 姿态转换
  const auto enu_att = oneq::coordinate::ToEnuAttitude(ned_att);
  out.state.yaw_deg   = enu_att.yaw_deg;
  out.state.pitch_deg = enu_att.pitch_deg;
  out.state.roll_deg  = enu_att.roll_deg;
}

void VehicleStateMapper::MapAngularVelocity(const JSBSim::FGPropagate& propagate,
                                             FlightDynamicOutput& out) const {
  // JSBSim vPQR（Body 系，rad/s）— 直接映射，无需坐标转换
  const JSBSim::FGColumnVector3& pqr = propagate.GetPQR();
  out.state.roll_rate_radps  = pqr(1);  // p
  out.state.pitch_rate_radps = pqr(2);  // q
  out.state.yaw_rate_radps   = pqr(3);  // r
}

void VehicleStateMapper::MapAcceleration(const JSBSim::FGAccelerations& accelerations,
                                          FlightDynamicOutput& out) const {
  // C2d: Body 系加速度（ft/s²）→ m/s²
  const JSBSim::FGColumnVector3& acc = accelerations.GetBodyAccel();
  out.state.acc_x_mps2 = acc(1) * kFtps2ToMps2;
  out.state.acc_y_mps2 = acc(2) * kFtps2ToMps2;
  out.state.acc_z_mps2 = acc(3) * kFtps2ToMps2;
}

void VehicleStateMapper::MapAeroParams(JSBSim::FGFDMExec& fdm_exec,
                                        FlightDynamicOutput& out) const {
  // 空速（kts → m/s）
  out.state.airspeed_mps =
      fdm_exec.GetPropertyValue("velocities/vc-kts") * kKnotsToMps;

  // 马赫数（无单位）
  out.state.mach = fdm_exec.GetPropertyValue("velocities/mach");

  // 迎角 / 侧滑角（deg）
  out.state.alpha_deg = fdm_exec.GetPropertyValue("aero/alpha-deg");
  out.state.beta_deg  = fdm_exec.GetPropertyValue("aero/beta-deg");
}

}  // namespace model
}  // namespace flight_dynamic
