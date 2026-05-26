#include "flight_dynamic/adapter/JsbsimAdapter.h"

#include <stdexcept>
#include <iostream>

#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"
#include "flight_dynamic/model/VehicleStateMapper.h"

namespace oneq {
namespace flight_dynamic {
namespace adapter {

JsbsimAdapter::JsbsimAdapter(const config::FlightDynamicConfig& config) {
  fdm_exec_.reset(new JSBSim::FGFDMExec());

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

  if (config.do_trim) {
    try {
      fdm_exec_->DoTrim(0);
    } catch (...) {
      std::cerr << "JsbsimAdapter: DoTrim(0) threw an exception, proceeding anyway." << std::endl;
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

    // 优先使用机型特定的系统文件路径，不存在则使用全局路径
    std::string model_systems_path = config.aircraft_root_dir + "/aircraft/" + config.aircraft_model + "/Systems";
    if (SGPath(model_systems_path).exists()) {
      fdm_exec_->SetSystemsPath(SGPath("aircraft/" + config.aircraft_model + "/Systems"));
    } else {
      fdm_exec_->SetSystemsPath(SGPath("systems"));
    }
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
}

}  // namespace adapter
}  // namespace flight_dynamic
}  // namespace oneq
