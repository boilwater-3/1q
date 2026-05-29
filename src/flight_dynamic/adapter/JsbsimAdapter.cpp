#include "flight_dynamic/adapter/JsbsimAdapter.h"

#include <iostream>
#include <stdexcept>

#include "1q/coordinate/position_transform.h"
#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"
#include "flight_dynamic/adapter/PropertyNames.h"
#include "flight_dynamic/model/VehicleStateMapper.h"

namespace oneq {
namespace flight_dynamic {
namespace adapter {

namespace {

void SetPropertyIfPresent(JSBSim::FGFDMExec& fdm_exec, const std::string& name, double value) {
  if (fdm_exec.GetPropertyManager()->GetNode(name) != nullptr) {
    fdm_exec.SetPropertyValue(name, value);
  }
}

void RetractLandingGearIfModeled(JSBSim::FGFDMExec& fdm_exec) {
  if (fdm_exec.GetPropertyManager()->GetNode(property::kGearCmdNorm) == nullptr) {
    return;
  }
  fdm_exec.SetPropertyValue(property::kGearCmdNorm, 0.0);
  SetPropertyIfPresent(fdm_exec, property::kGearPosNorm, 0.0);
}

void DisableXmlOutput(JSBSim::FGFDMExec& fdm_exec) {
  auto* pm = fdm_exec.GetPropertyManager().get();
  auto* output_node = pm->GetNode("simulation/output");
  if (output_node == nullptr) return;

  auto* count_node = pm->GetNode("simulation/output/num-files");
  if (count_node == nullptr) return;
  int count = static_cast<int>(count_node->getDoubleValue());
  for (int i = 0; i < count; ++i) {
    std::string enable_prop = "simulation/output/" + std::to_string(i) + "/enabled";
    auto* node = pm->GetNode(enable_prop);
    if (node != nullptr) {
      node->setDoubleValue(0.0);
    }
  }
}

void ResetControlStateAfterTrimFailure(JSBSim::FGFDMExec& fdm_exec) {
  const char* properties[] = {
      property::kAileronCmd,
      property::kElevatorCmd,
      property::kRudderCmd,
      property::kPitchTrimCmd,
      property::kRollTrimCmd,
      property::kYawTrimCmd,
      property::kPitchRateIntegrator,
      property::kRollRateIntegrator,
      property::kYawRateIntegrator,
      property::kElevatorPosNorm,
      property::kElevatorPosRad,
      property::kLeftAileronPosRad,
      property::kRightAileronPosRad,
      property::kRudderPosRad,
  };
  for (const char* property : properties) {
    SetPropertyIfPresent(fdm_exec, property, 0.0);
  }
}

double InitialAltitudeM(const coordinate::ExternalKinematics& kinematics) {
  if (kinematics.position_frame == coordinate::PositionFrame::kLla) {
    return kinematics.position_lla_deg_m.altitude_m;
  }
  coordinate::LlaPositionDegM lla;
  if (coordinate::TryEcefToLla(kinematics.position_ecef_m, &lla)) {
    return lla.altitude_m;
  }
  return 0.0;
}

}  // namespace

JsbsimAdapter::JsbsimAdapter(const config::FlightDynamicConfig& config) {
  fdm_exec_.reset(new JSBSim::FGFDMExec());
  if (config.silent_mode) {
    fdm_exec_->SetDebugLevel(0);
  }

  SetDeltaT(config.dt_sec);

  if (!LoadAircraft(config)) {
    throw std::runtime_error("JsbsimAdapter: failed to load aircraft: " +
                             config.aircraft_model);
  }
  init_diag_.model_loaded = true;

  DisableXmlOutput(*fdm_exec_);

  ConfigureIntegrators(config);

  model::VehicleStateMapper::ApplyInitialConditions(*fdm_exec_, config.initial_kinematics,
                                                    config.initial_velocity_frame);
  init_diag_.ic_applied = true;

  if (!RunIC()) {
    throw std::runtime_error("JsbsimAdapter: RunIC() failed");
  }
  init_diag_.run_ic_ok = true;

  // Start all engines so that JSBSim can trim longitudinal velocity (udot).
  fdm_exec_->GetPropulsion()->InitRunning(-1);
  init_diag_.engines_started = true;
  if (InitialAltitudeM(config.initial_kinematics) > 10.0) {
    RetractLandingGearIfModeled(*fdm_exec_);
    init_diag_.gear_retracted = true;
  }

  if (config.do_trim) {
    init_diag_.trim_attempted = true;
    try {
      fdm_exec_->DoTrim(0);
      init_diag_.trim_succeeded = true;
    } catch (...) {
      init_diag_.trim_succeeded = false;
      std::cerr << "JsbsimAdapter: DoTrim(0) threw an exception, proceeding anyway." << std::endl;
      model::VehicleStateMapper::ApplyInitialConditions(*fdm_exec_, config.initial_kinematics,
                                                        config.initial_velocity_frame);
      if (!RunIC()) {
        throw std::runtime_error("JsbsimAdapter: RunIC() failed after trim recovery");
      }
      fdm_exec_->GetPropulsion()->InitRunning(-1);
      if (InitialAltitudeM(config.initial_kinematics) > 10.0) {
        RetractLandingGearIfModeled(*fdm_exec_);
      }
      ResetControlStateAfterTrimFailure(*fdm_exec_);
      init_diag_.trim_recovery_applied = true;
    }
  }
}

JsbsimAdapter::~JsbsimAdapter() = default;

bool JsbsimAdapter::Run() {
  return fdm_exec_->Run();
}

bool JsbsimAdapter::RunIC() {
  return fdm_exec_->RunIC();
}

void JsbsimAdapter::SetDeltaT(double dt_sec) {
  fdm_exec_->Setdt(dt_sec);
}

double JsbsimAdapter::GetDeltaT() const {
  return fdm_exec_->GetDeltaT();
}

double JsbsimAdapter::GetProperty(const std::string& name) const {
  return fdm_exec_->GetPropertyValue(name);
}

void JsbsimAdapter::SetProperty(const std::string& name, double value) {
  fdm_exec_->SetPropertyValue(name, value);
}

bool JsbsimAdapter::HasProperty(const std::string& name) const {
  return fdm_exec_->GetPropertyManager()->GetNode(name) != nullptr;
}

JSBSim::FGPropagate& JsbsimAdapter::GetPropagate() {
  return *fdm_exec_->GetPropagate();
}

const JSBSim::FGPropagate& JsbsimAdapter::GetPropagate() const {
  return *fdm_exec_->GetPropagate();
}

JSBSim::FGAccelerations& JsbsimAdapter::GetAccelerations() {
  return *fdm_exec_->GetAccelerations();
}

const JSBSim::FGAccelerations& JsbsimAdapter::GetAccelerations() const {
  return *fdm_exec_->GetAccelerations();
}

JSBSim::FGFDMExec& JsbsimAdapter::GetFdmExec() {
  return *fdm_exec_;
}

const JSBSim::FGFDMExec& JsbsimAdapter::GetFdmExec() const {
  return *fdm_exec_;
}

bool JsbsimAdapter::LoadAircraft(const config::FlightDynamicConfig& config) {
  if (!config.aircraft_root_dir.empty()) {
    SGPath root(config.aircraft_root_dir);
    fdm_exec_->SetRootDir(root);
    fdm_exec_->SetAircraftPath(SGPath("aircraft"));
    fdm_exec_->SetEnginePath(SGPath("engine"));
    fdm_exec_->SetSystemsPath(SGPath("systems"));
  }
  return fdm_exec_->LoadModel(config.aircraft_model, true);
}

void JsbsimAdapter::ConfigureIntegrators(
    const config::FlightDynamicConfig& config) {
  SetProperty(property::kIntegratorRateRot,
              static_cast<double>(config.integrator_rate_rotational));
  SetProperty(property::kIntegratorRateTrans,
              static_cast<double>(config.integrator_rate_translational));
  SetProperty(property::kIntegratorPosRot,
              static_cast<double>(config.integrator_pos_rotational));
  SetProperty(property::kIntegratorPosTrans,
              static_cast<double>(config.integrator_pos_translational));
  SetProperty(property::kGravityModel,
              static_cast<double>(config.gravity_model));
  if (fdm_exec_->GetPropertyManager()->GetNode(property::kGuidanceRollAngleLimit) != nullptr) {
    SetProperty(property::kGuidanceRollAngleLimit, 0.785);  // 45 deg
    SetProperty(property::kGuidanceRollRateLimit, 1.5);     // ~86 deg/s
  }
}

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq
