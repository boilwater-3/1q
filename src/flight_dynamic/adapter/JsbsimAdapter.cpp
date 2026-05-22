/**
 * @file JsbsimAdapter.cpp
 * @brief JSBSim FGFDMExec 生命周期管理实现。
 */

#include "flight_dynamic/adapter/JsbsimAdapter.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/coordinate/attitude_transform.h"

// JSBSim
#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropagate.h"
#include "models/FGAccelerations.h"
#include "models/propulsion/FGEngine.h"
#include "models/FGExternalReactions.h"
#include "models/FGExternalForce.h"
#include "math/FGLocation.h"
#include "math/FGColumnVector3.h"
#include "math/FGQuaternion.h"

namespace flight_dynamic {
namespace adapter {

namespace {

// ---- 单位换算常数 ----
constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 1.0 / kFtToM;
constexpr double kKnotsToMps = 0.514444;
constexpr double kMpsToKnots = 1.0 / kKnotsToMps;
constexpr double kNToLbf = 0.224809;
constexpr double kNmToLbfFt = 0.737562;

}  // namespace

JsbsimAdapter::JsbsimAdapter(const config::FlightDynamicConfig& config)
    : fdm_exec_(new JSBSim::FGFDMExec()) {
  // 静默模式：重定向 JSBSim 输出
  if (config.silent) {
    fdm_exec_->SetDebugLevel(0);
  }

  // 设置数据文件根目录
  if (!config.aircraft.root_dir.empty()) {
    fdm_exec_->SetRootDir(SGPath(config.aircraft.root_dir));
    fdm_exec_->SetAircraftPath(SGPath(config.aircraft.root_dir + "/aircraft"));
    fdm_exec_->SetEnginePath(SGPath(config.aircraft.root_dir + "/engine"));
    fdm_exec_->SetSystemsPath(SGPath(config.aircraft.root_dir + "/systems"));
  }

  // 加载飞行器模型
  if (!fdm_exec_->LoadModel(config.aircraft.model_name)) {
    throw std::runtime_error(
        "JsbsimAdapter: Failed to load aircraft model: " + config.aircraft.model_name);
  }

  // 应用初始条件
  ApplyInitialConditions(config.initial_kinematics);

  if (!fdm_exec_->RunIC()) {
    throw std::runtime_error("JsbsimAdapter: RunIC() failed");
  }
}

JsbsimAdapter::~JsbsimAdapter() = default;

bool JsbsimAdapter::Run(const model::FlightDynamicInput& input) {
  // 设置仿真步长
  fdm_exec_->Setdt(static_cast<double>(input.dt_sec));

  // 应用控制面输入
  ApplyControlInputs(input);

  // 应用外部力/力矩
  ApplyExternalForces(input);

  // 推进一步积分
  return fdm_exec_->Run();
}

void JsbsimAdapter::Reset(const oneq::coordinate::ExternalKinematics& kinematics) {
  ApplyInitialConditions(kinematics);
  if (!fdm_exec_->RunIC()) {
    throw std::runtime_error("JsbsimAdapter: RunIC() failed during Reset");
  }
}

double JsbsimAdapter::GetProperty(const std::string& property_name) const {
  return fdm_exec_->GetPropertyValue(property_name);
}

void JsbsimAdapter::SetProperty(const std::string& property_name, double value) {
  fdm_exec_->SetPropertyValue(property_name, value);
}

const JSBSim::FGPropagate& JsbsimAdapter::GetPropagate() const {
  return *fdm_exec_->GetPropagate();
}

const JSBSim::FGAccelerations& JsbsimAdapter::GetAccelerations() const {
  return *fdm_exec_->GetAccelerations();
}

// ---- 私有方法 ----

void JsbsimAdapter::ApplyInitialConditions(
    const oneq::coordinate::ExternalKinematics& kinematics) {
  auto* ic = fdm_exec_->GetIC();

  // 位置：JSBSim 初始条件使用经纬高
  oneq::coordinate::LlaPositionDegM lla{};
  if (kinematics.position_frame == oneq::coordinate::PositionFrame::kLla) {
    lla = kinematics.position_lla_deg_m;
  } else {
    // ECEF → LLA
    if (!oneq::coordinate::TryEcefToLla(kinematics.position_ecef_m, &lla)) {
      throw std::runtime_error(
          "JsbsimAdapter: TryEcefToLla failed in ApplyInitialConditions");
    }
  }

  // JSBSim 初始条件：经纬度(deg)、高度(ft)
  ic->SetGeodLatitudeDegIC(lla.latitude_deg);
  ic->SetLongitudeDegIC(lla.longitude_deg);
  ic->SetAltitudeASLFtIC(lla.altitude_m * kMToFt);

  // 速度：ECEF → ENU → NED，再转换为空速(kts)注入
  // 注意：JSBSim 初始条件最简单的注入方式是 NED 速度
  oneq::coordinate::EnuVelocityMps enu_vel{};
  if (oneq::coordinate::TryEcefToEnuVelocity(kinematics.velocity_mps, lla, &enu_vel)) {
    const auto ned_vel = oneq::coordinate::ToNedVelocity(enu_vel);
    // JSBSim 使用 vn/ve/vd（ft/s）
    ic->SetVNorthFpsIC(ned_vel.north_mps * kMToFt);
    ic->SetVEastFpsIC(ned_vel.east_mps * kMToFt);
    ic->SetVDownFpsIC(ned_vel.down_mps * kMToFt);
  }

  // 姿态：ENU 欧拉角 → NED 欧拉角（JSBSim 使用 NED 系）
  const auto ned_att = oneq::coordinate::ToNedAttitude(kinematics.attitude_deg);
  ic->SetPsiDegIC(ned_att.yaw_deg);    // 航向
  ic->SetThetaDegIC(ned_att.pitch_deg); // 俯仰
  ic->SetPhiDegIC(ned_att.roll_deg);   // 滚转
}

void JsbsimAdapter::ApplyControlInputs(const model::FlightDynamicInput& input) {
  const auto& ctrl = input.control;
  fdm_exec_->SetPropertyValue("fcs/throttle-cmd-norm[0]", ctrl.throttle);
  fdm_exec_->SetPropertyValue("fcs/aileron-cmd-norm", ctrl.aileron);
  fdm_exec_->SetPropertyValue("fcs/elevator-cmd-norm", ctrl.elevator);
  fdm_exec_->SetPropertyValue("fcs/rudder-cmd-norm", ctrl.rudder);

  // AP 指令（仅当设置值 >= 0 时激活，哨兵值 -1 表示不使用）
  if (ctrl.heading_setpoint_deg >= 0.0) {
    fdm_exec_->SetPropertyValue("ap/heading_setpoint", ctrl.heading_setpoint_deg);
  }
  fdm_exec_->SetPropertyValue("ap/heading_hold", ctrl.heading_hold ? 1.0 : 0.0);
  if (ctrl.altitude_setpoint_m >= 0.0) {
    // JSBSim AP 内部使用英制，需将米转换为英尺
    fdm_exec_->SetPropertyValue("ap/altitude_setpoint", ctrl.altitude_setpoint_m * kMToFt);
  }
  fdm_exec_->SetPropertyValue("ap/altitude_hold", ctrl.altitude_hold ? 1.0 : 0.0);
}

void JsbsimAdapter::ApplyExternalForces(const model::FlightDynamicInput& input) {
  const auto& ef = input.ext_force;
  // 仅当有非零外部力时才注入，避免不必要的 Property 写入开销
  const bool has_force = (ef.force_x_n != 0.0 || ef.force_y_n != 0.0 ||
                          ef.force_z_n != 0.0 || ef.moment_x_nm != 0.0 ||
                          ef.moment_y_nm != 0.0 || ef.moment_z_nm != 0.0);
  if (!has_force) {
    return;
  }
  // JSBSim 外部力使用英制 lbf/lbf·ft，Body 系
  fdm_exec_->SetPropertyValue("external_reactions/external/x",
                              ef.force_x_n * kNToLbf);
  fdm_exec_->SetPropertyValue("external_reactions/external/y",
                              ef.force_y_n * kNToLbf);
  fdm_exec_->SetPropertyValue("external_reactions/external/z",
                              ef.force_z_n * kNToLbf);
  fdm_exec_->SetPropertyValue("external_reactions/external/l",
                              ef.moment_x_nm * kNmToLbfFt);
  fdm_exec_->SetPropertyValue("external_reactions/external/m",
                              ef.moment_y_nm * kNmToLbfFt);
  fdm_exec_->SetPropertyValue("external_reactions/external/n",
                              ef.moment_z_nm * kNmToLbfFt);
}

}  // namespace adapter
}  // namespace flight_dynamic
