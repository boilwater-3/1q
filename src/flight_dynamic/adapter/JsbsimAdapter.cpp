#include "flight_dynamic/adapter/JsbsimAdapter.h"

#include <iostream>
#include <stdexcept>

#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"
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
  if (fdm_exec.GetPropertyManager()->GetNode("gear/gear-cmd-norm") == nullptr) {
    return;
  }
  fdm_exec.SetPropertyValue("gear/gear-cmd-norm", 0.0);
  SetPropertyIfPresent(fdm_exec, "gear/gear-pos-norm", 0.0);
}

void ResetControlStateAfterTrimFailure(JSBSim::FGFDMExec& fdm_exec) {
  const char* properties[] = {
      "fcs/aileron-cmd-norm",
      "fcs/elevator-cmd-norm",
      "fcs/rudder-cmd-norm",
      "fcs/pitch-trim-cmd-norm",
      "fcs/roll-trim-cmd-norm",
      "fcs/yaw-trim-cmd-norm",
      "fcs/pitch-rate-integrator",
      "fcs/roll-rate-integrator",
      "fcs/yaw-rate-integrator",
      "fcs/elevator-pos-norm",
      "fcs/elevator-pos-rad",
      "fcs/left-aileron-pos-rad",
      "fcs/right-aileron-pos-rad",
      "fcs/rudder-pos-rad",
  };
  for (const char* property : properties) {
    SetPropertyIfPresent(fdm_exec, property, 0.0);
  }
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

  ConfigureIntegrators(config);

  model::VehicleStateMapper::ApplyInitialConditions(*fdm_exec_,
                                                     config.initial_kinematics);

  if (!RunIC()) {
    throw std::runtime_error("JsbsimAdapter: RunIC() failed");
  }

  // Start all engines so that JSBSim can trim longitudinal velocity (udot).
  fdm_exec_->GetPropulsion()->InitRunning(-1);
  if (config.initial_kinematics.position_lla_deg_m.altitude_m > 10.0) {
    RetractLandingGearIfModeled(*fdm_exec_);
  }

  if (config.do_trim) {
    trim_attempted_ = true;
    try {
      fdm_exec_->DoTrim(0);
      trim_succeeded_ = true;
    } catch (...) {
      trim_succeeded_ = false;
      std::cerr << "JsbsimAdapter: DoTrim(0) threw an exception, proceeding anyway." << std::endl;
      model::VehicleStateMapper::ApplyInitialConditions(*fdm_exec_, config.initial_kinematics);
      if (!RunIC()) {
        throw std::runtime_error("JsbsimAdapter: RunIC() failed after trim recovery");
      }
      fdm_exec_->GetPropulsion()->InitRunning(-1);
      if (config.initial_kinematics.position_lla_deg_m.altitude_m > 10.0) {
        RetractLandingGearIfModeled(*fdm_exec_);
      }
      ResetControlStateAfterTrimFailure(*fdm_exec_);
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
  SetProperty("simulation/integrator/rate/rotational",
              static_cast<double>(config.integrator_rate_rotational));
  SetProperty("simulation/integrator/rate/translational",
              static_cast<double>(config.integrator_rate_translational));
  SetProperty("simulation/integrator/position/rotational",
              static_cast<double>(config.integrator_pos_rotational));
  SetProperty("simulation/integrator/position/translational",
              static_cast<double>(config.integrator_pos_translational));
  SetProperty("simulation/gravity-model",
              static_cast<double>(config.gravity_model));
  if (fdm_exec_->GetPropertyManager()->GetNode("guidance/roll-angle-limit") != nullptr) {
    SetProperty("guidance/roll-angle-limit", 0.785);  // 45°
    SetProperty("guidance/roll-rate-limit", 1.5);     // ~86°/s
  }
}

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq
