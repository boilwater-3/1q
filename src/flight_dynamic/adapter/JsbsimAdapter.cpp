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
    : fdm_exec_(new JSBSim::FGFDMExec()), do_trim_(config.do_trim) {
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

  // 4. 空中初始配平（避免由于舵面不在平衡位置而产生的初始巨大不平衡力矩导致 NaNs）
  if (config.do_trim) {
    fdm_exec_->SetPropertyValue("propulsion/set-running", -1); // 启动所有引擎
    try {
      fdm_exec_->DoTrim(0); 
    } catch (const std::exception& e) {
      std::cerr << "JsbsimAdapter: DoTrim exception: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "JsbsimAdapter: DoTrim unknown exception" << std::endl;
    }
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

  if (do_trim_) {
    fdm_exec_->SetPropertyValue("propulsion/set-running", -1);
    try {
      fdm_exec_->DoTrim(0);
    } catch (...) {
      std::cerr << "JsbsimAdapter: Reset DoTrim unknown exception" << std::endl;
    }
  }

  airspeed_integral_ = 0.0;
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

  // 姿态：1Q ENU 欧拉角 → JSBSim NED 初始条件
  // 1Q 的 ENU yaw 约定：0°=北, 90°=东（与 NED heading 相同）。
  // ToNedAttitude 通过旋转矩阵复合转换，对纯水平飞行会产生 roll=180° 的错误结果。
  // 对于飞机初始条件，yaw 直接传递即可；pitch 和 roll 取反（ENU 天向朝上 vs NED 天向朝下）。
  // 注意：必须在速度之前设置姿态，JSBSim 用 heading 来分解空速为 NED 分量
  ic->SetPsiDegIC(kinematics.attitude_deg.yaw_deg);     // 航向（ENU yaw = NED heading）
  ic->SetThetaDegIC(-kinematics.attitude_deg.pitch_deg); // 俯仰（ENU 正仰角 → NED 负俯仰）
  ic->SetPhiDegIC(-kinematics.attitude_deg.roll_deg);    // 滚转（方向取反）

  // 速度：使用 JSBSim 原生的校准空速接口（SetVcalibratedKtsIC）。
  // JSBSim 内部会根据 heading (psi) 和 alpha 将空速分解为 body/NED 速度分量。
  // 这种方式能正确初始化 alpha，使 DoTrim 可以正常收敛。
  // 若使用 SetVNorthFpsIC/SetVEastFpsIC，在某些情况下会产生退化的 alpha=90° 问题。
  const auto& vel = kinematics.velocity_mps;
  const double speed_mps = std::sqrt(
      vel.x_mps * vel.x_mps + vel.y_mps * vel.y_mps + vel.z_mps * vel.z_mps);
  if (speed_mps > 1.0) {
    ic->SetVcalibratedKtsIC(speed_mps * kMpsToKnots);
  }
}

void JsbsimAdapter::ApplyControlInputs(const model::FlightDynamicInput& input) {
  const auto& ctrl = input.control;

  // ---- 基础控制面 ----
  fdm_exec_->SetPropertyValue("fcs/aileron-cmd-norm", ctrl.aileron);
  fdm_exec_->SetPropertyValue("fcs/elevator-cmd-norm", ctrl.elevator);
  fdm_exec_->SetPropertyValue("fcs/rudder-cmd-norm", ctrl.rudder);

  // ---- AP 航向（通过 c172ap.xml → ap/aileron_cmd → FCS Roll 通道） ----
  // 1Q ENU yaw 约定与 JSBSim NED heading 相同：0°=北, 90°=东，直接传递。
  if (ctrl.heading_setpoint_deg >= 0.0) {
    fdm_exec_->SetPropertyValue("ap/heading_setpoint",
                                ctrl.heading_setpoint_deg);
  }
  fdm_exec_->SetPropertyValue("ap/heading_hold",
                              ctrl.heading_hold ? 1.0 : 0.0);

  // ---- AP 高度（通过 c172ap.xml → ap/elevator_cmd → FCS Pitch 通道） ----
  if (ctrl.altitude_setpoint_m >= 0.0) {
    fdm_exec_->SetPropertyValue("ap/altitude_setpoint",
                                ctrl.altitude_setpoint_m * kMToFt);
  }
  fdm_exec_->SetPropertyValue("ap/altitude_hold",
                              ctrl.altitude_hold ? 1.0 : 0.0);

  // ---- AP 速度（C++ 侧 PI 控制器） ----
  // c172ap.xml 原版无 auto-throttle 通道，此处在 C++ 侧实现空速保持。
  if (ctrl.airspeed_hold && ctrl.airspeed_setpoint_mps > 0.0) {
    const double target_kts = ctrl.airspeed_setpoint_mps * kMpsToKnots;
    const double current_kts = fdm_exec_->GetPropertyValue("velocities/vc-kts");
    double error = target_kts - current_kts;
    error = std::max(-50.0, std::min(50.0, error));

    const double dt = static_cast<double>(input.dt_sec);
    airspeed_integral_ += error * dt;
    airspeed_integral_ = std::max(-50.0, std::min(50.0, airspeed_integral_));

    double throttle = 0.05 * error + 0.01 * airspeed_integral_;
    throttle = std::max(0.0, std::min(1.0, throttle));
    fdm_exec_->SetPropertyValue("fcs/throttle-cmd-norm[0]", throttle);
  } else {
    fdm_exec_->SetPropertyValue("fcs/throttle-cmd-norm[0]", ctrl.throttle);
  }
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
