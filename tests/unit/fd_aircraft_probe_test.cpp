#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "1q/flight_dynamic/guidance/Maneuver.h"
#include "FGFDMExec.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/model/VehicleStateMapper.h"
#include "initialization/FGInitialCondition.h"
#include "models/FGPropulsion.h"
#include "simgear/misc/sg_path.hxx"

namespace oneq {
namespace flight_dynamic {
namespace {

constexpr double kProbeRunSec = 10.0;
constexpr double kDt = 0.005;
constexpr double kMToFt = 1.0 / 0.3048;

struct AircraftProbeParam {
  std::string model;
  double altitude_m;
  double speed_mps;
};

struct ProbeSnapshot {
  bool run_ok = false;
  bool stable = false;
  double sim_time_sec = 0.0;
  double alt_start_m = 0.0;
  double alt_end_m = 0.0;
  double vt_start_mps = 0.0;
  double vt_end_mps = 0.0;
};

struct AileronProbe {
  double roll_delta_rad = 0.0;
  double left_aileron_delta_rad = 0.0;
  double right_aileron_delta_rad = 0.0;
};

struct F22CommandProbeResult {
  std::string scenario;
  bool do_trim = false;
  double command = 0.0;
  bool trim_attempted = false;
  bool trim_succeeded = false;
  bool run_ok = false;
  bool stable = false;
  double sim_time_sec = 0.0;
  double alt_start_m = 0.0;
  double alt_end_m = 0.0;
  double speed_start_mps = 0.0;
  double speed_end_mps = 0.0;
  double roll_start_rad = 0.0;
  double roll_end_rad = 0.0;
  double roll_rate_start_rad_sec = 0.0;
  double roll_rate_end_rad_sec = 0.0;
  double aileron_cmd_end = 0.0;
  double roll_rate_cmd_end = 0.0;
  double roll_rate_error_end = 0.0;
  double roll_rate_integrator_end = 0.0;
  double roll_cmd_end = 0.0;
  double aileron_act_end = 0.0;
  double left_aileron_pos_end_rad = 0.0;
  double right_aileron_pos_end_rad = 0.0;
};

struct F22ThrottleProbeResult {
  bool do_trim = false;
  double command = 0.0;
  bool trim_attempted = false;
  bool trim_succeeded = false;
  bool run_ok = false;
  bool stable = false;
  double sim_time_sec = 0.0;
  double alt_start_m = 0.0;
  double alt_end_m = 0.0;
  double speed_start_mps = 0.0;
  double speed_end_mps = 0.0;
  double throttle_pos_end = 0.0;
  double throttle_pos_1_end = 0.0;
  double thrust_end_lbs = 0.0;
  double thrust_1_end_lbs = 0.0;
};

struct F22EnvelopeProbeParam {
  double altitude_m = 0.0;
  double speed_mps = 0.0;
  double alpha_deg = 0.0;
  double gamma_deg = 0.0;
  double throttle = 0.0;
  bool do_trim = false;
};

struct F22EnvelopeProbeResult {
  bool loaded = false;
  bool run_ic_ok = false;
  bool trim_attempted = false;
  bool trim_succeeded = false;
  bool run_ok = false;
  bool stable = false;
  double sim_time_sec = 0.0;
  double alt_start_m = 0.0;
  double alt_end_m = 0.0;
  double speed_start_mps = 0.0;
  double speed_end_mps = 0.0;
  double min_speed_mps = std::numeric_limits<double>::max();
  double min_altitude_m = std::numeric_limits<double>::max();
  double alpha_start_deg = 0.0;
  double alpha_end_deg = 0.0;
  double theta_start_rad = 0.0;
  double theta_end_rad = 0.0;
  double gamma_start_rad = 0.0;
  double gamma_end_rad = 0.0;
  double qbar_start_pa = 0.0;
  double qbar_end_pa = 0.0;
  double throttle_pos_end = 0.0;
  double throttle_pos_1_end = 0.0;
  double thrust_end_lbs = 0.0;
  double thrust_1_end_lbs = 0.0;
};

struct WaypointProbeParam {
  std::string model;
  double altitude_m;
  double speed_mps;
};

struct WaypointProbeResult {
  double target_distance_m = 0.0;
  double init_distance_m = 0.0;
  double min_distance_m = 0.0;
  double final_distance_m = 0.0;
  double sim_time_sec = 0.0;
  double final_altitude_m = 0.0;
  double final_speed_mps = 0.0;
  double final_heading_error_rad = 0.0;
  double max_abs_heading_error_rad = 0.0;
  double max_abs_roll_rad = 0.0;
  double max_abs_aileron_cmd = 0.0;
  bool completed = false;
  bool crashed = false;
  bool stopped = false;
};

struct OrbitProbeParam {
  std::string model;
  double altitude_m;
  double speed_mps;
};

struct OrbitProbeResult {
  double orbit_radius_m = 0.0;
  double center_distance_m = 0.0;
  double final_distance_m = 0.0;
  double min_distance_m = 0.0;
  double max_distance_m = 0.0;
  double sim_time_sec = 0.0;
  double final_altitude_m = 0.0;
  double min_altitude_m = 0.0;
  double final_speed_mps = 0.0;
  double max_abs_roll_rad = 0.0;
  double max_abs_heading_error_rad = 0.0;
  bool crashed = false;
  bool stopped = false;
};

std::vector<AircraftProbeParam> ProbeAircraft() {
  return {
      {"A4", 2000.0, 120.0},   {"F4N", 2000.0, 130.0},  {"F80C", 2000.0, 120.0},
      {"f15", 3000.0, 200.0},  {"f16", 3000.0, 200.0},  {"f22", 3000.0, 200.0},
      {"OV10", 500.0, 70.0},   {"B17", 1000.0, 80.0},   {"Boeing314", 500.0, 70.0},
      {"C130", 1000.0, 90.0},  {"DHC6", 500.0, 55.0},   {"L410", 1000.0, 90.0},
      {"737", 3000.0, 130.0},  {"B747", 3000.0, 140.0}, {"Concorde", 5000.0, 150.0},
      {"MD11", 3000.0, 140.0}, {"c172p", 500.0, 50.0},  {"c172r", 500.0, 50.0},
      {"c172x", 500.0, 50.0},  {"c182", 500.0, 55.0},   {"c310", 500.0, 65.0},
  };
}

std::vector<WaypointProbeParam> WaypointProbeAircraft() {
  return {
      {"A4", 2000.0, 120.0},   {"F4N", 2000.0, 130.0},  {"F80C", 2000.0, 120.0},
      {"f15", 3000.0, 200.0},  {"f16", 3000.0, 200.0},  {"f22", 3000.0, 200.0},
      {"737", 3000.0, 130.0},  {"B747", 3000.0, 140.0}, {"Concorde", 5000.0, 150.0},
      {"MD11", 3000.0, 140.0}, {"c310", 500.0, 65.0},
  };
}

std::vector<OrbitProbeParam> OrbitProbeAircraft() {
  return {
      {"f16", 3000.0, 200.0},
      {"Concorde", 5000.0, 150.0},
      {"Concorde", 10000.0, 250.0},
  };
}

config::FlightDynamicConfig MakeConfig(const AircraftProbeParam& aircraft, bool do_trim) {
  config::FlightDynamicConfig config;
  config.aircraft_model = aircraft.model;
  config.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  config.dt_sec = kDt;
  config.do_trim = do_trim;
  config.silent_mode = true;
  config.initial_kinematics.position_frame = coordinate::PositionFrame::kLla;
  config.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  config.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  config.initial_kinematics.position_lla_deg_m.altitude_m = aircraft.altitude_m;
  config.initial_kinematics.velocity_mps.x_mps = aircraft.speed_mps;
  config.initial_kinematics.velocity_mps.y_mps = 0.0;
  config.initial_kinematics.velocity_mps.z_mps = 0.0;
  config.initial_kinematics.attitude_deg.roll_deg = 0.0;
  config.initial_kinematics.attitude_deg.pitch_deg = 0.0;
  config.initial_kinematics.attitude_deg.yaw_deg = 0.0;
  return config;
}

config::FlightDynamicConfig MakeWaypointConfig(const WaypointProbeParam& aircraft) {
  AircraftProbeParam param{aircraft.model, aircraft.altitude_m, aircraft.speed_mps};
  return MakeConfig(param, true);
}

config::FlightDynamicConfig MakeF22Config(bool do_trim) {
  AircraftProbeParam param{"f22", 3000.0, 200.0};
  return MakeConfig(param, do_trim);
}

bool IsStable(const model::VehicleState& state) {
  return std::isfinite(state.altitude_geod_m) && std::isfinite(state.vtrue_mps) &&
         std::isfinite(state.phi_rad) && std::isfinite(state.theta_rad) &&
         std::isfinite(state.psi_rad) && state.mass_kg > 0.0 && state.altitude_geod_m > 0.0 &&
         state.vtrue_mps > 1.0;
}

ProbeSnapshot RunProjectFreeFlight(adapter::JsbsimAdapter& adapter) {
  ProbeSnapshot snapshot;
  const auto start = model::VehicleStateMapper::Map(
      adapter.GetPropagate(), adapter.GetAccelerations(), adapter.GetFdmExec(), 0.0);
  snapshot.alt_start_m = start.altitude_geod_m;
  snapshot.vt_start_mps = start.vtrue_mps;

  const int steps = static_cast<int>(kProbeRunSec / kDt);
  for (int i = 0; i < steps; ++i) {
    snapshot.run_ok = adapter.Run();
    snapshot.sim_time_sec += kDt;
    if (!snapshot.run_ok) {
      break;
    }
  }

  const auto end =
      model::VehicleStateMapper::Map(adapter.GetPropagate(), adapter.GetAccelerations(),
                                     adapter.GetFdmExec(), snapshot.sim_time_sec);
  snapshot.alt_end_m = end.altitude_geod_m;
  snapshot.vt_end_mps = end.vtrue_mps;
  snapshot.stable = snapshot.run_ok && IsStable(end);
  return snapshot;
}

AileronProbe RunAileronProbe(const config::FlightDynamicConfig& config, double command) {
  adapter::JsbsimAdapter adapter(config);
  const auto start = model::VehicleStateMapper::Map(
      adapter.GetPropagate(), adapter.GetAccelerations(), adapter.GetFdmExec(), 0.0);
  const double left_start = adapter.HasProperty("fcs/left-aileron-pos-rad")
                                ? adapter.GetProperty("fcs/left-aileron-pos-rad")
                                : 0.0;
  const double right_start = adapter.HasProperty("fcs/right-aileron-pos-rad")
                                 ? adapter.GetProperty("fcs/right-aileron-pos-rad")
                                 : 0.0;

  adapter.SetProperty("fcs/aileron-cmd-norm", command);
  for (int i = 0; i < 200; ++i) {
    if (!adapter.Run()) {
      break;
    }
  }

  const auto end = model::VehicleStateMapper::Map(
      adapter.GetPropagate(), adapter.GetAccelerations(), adapter.GetFdmExec(), 200.0 * kDt);
  AileronProbe result;
  result.roll_delta_rad = end.phi_rad - start.phi_rad;
  if (adapter.HasProperty("fcs/left-aileron-pos-rad")) {
    result.left_aileron_delta_rad = adapter.GetProperty("fcs/left-aileron-pos-rad") - left_start;
  }
  if (adapter.HasProperty("fcs/right-aileron-pos-rad")) {
    result.right_aileron_delta_rad = adapter.GetProperty("fcs/right-aileron-pos-rad") - right_start;
  }
  return result;
}

double PropertyOrZero(const adapter::JsbsimAdapter& adapter, const std::string& name) {
  return adapter.HasProperty(name) ? adapter.GetProperty(name) : 0.0;
}

double PropertyOrZero(JSBSim::FGFDMExec& fdm, const std::string& name) {
  return fdm.GetPropertyManager()->GetNode(name) != nullptr ? fdm.GetPropertyValue(name) : 0.0;
}

void SetPropertyIfPresent(JSBSim::FGFDMExec& fdm, const std::string& name, double value) {
  if (fdm.GetPropertyManager()->GetNode(name) != nullptr) {
    fdm.SetPropertyValue(name, value);
  }
}

void RetractLandingGearIfModeled(JSBSim::FGFDMExec& fdm) {
  SetPropertyIfPresent(fdm, "gear/gear-cmd-norm", 0.0);
  SetPropertyIfPresent(fdm, "gear/gear-pos-norm", 0.0);
}

F22EnvelopeProbeResult RunF22EnvelopeProbe(const F22EnvelopeProbeParam& param) {
  F22EnvelopeProbeResult result;
  JSBSim::FGFDMExec fdm;
  fdm.SetDebugLevel(0);
  fdm.SetRootDir(SGPath(FD_JSBSIM_ROOT_DIR));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetEnginePath(SGPath("engine"));
  fdm.SetSystemsPath(SGPath("systems"));
  fdm.Setdt(kDt);
  result.loaded = fdm.LoadModel("f22", true);
  if (!result.loaded) {
    return result;
  }

  auto ic = fdm.GetIC();
  ic->SetLatitudeDegIC(0.0);
  ic->SetLongitudeDegIC(0.0);
  ic->SetAltitudeASLFtIC(param.altitude_m * kMToFt);
  ic->SetVtrueFpsIC(param.speed_mps * kMToFt);
  ic->SetAlphaDegIC(param.alpha_deg);
  ic->SetBetaDegIC(0.0);
  ic->SetFlightPathAngleDegIC(param.gamma_deg);
  ic->SetPhiDegIC(0.0);
  ic->SetPsiDegIC(0.0);

  result.run_ic_ok = fdm.RunIC();
  if (!result.run_ic_ok) {
    return result;
  }
  fdm.GetPropulsion()->InitRunning(-1);
  RetractLandingGearIfModeled(fdm);
  SetPropertyIfPresent(fdm, "fcs/throttle-cmd-norm", param.throttle);
  SetPropertyIfPresent(fdm, "fcs/throttle-cmd-norm[1]", param.throttle);

  if (param.do_trim) {
    result.trim_attempted = true;
    try {
      fdm.DoTrim(0);
      result.trim_succeeded = true;
    } catch (...) {
      result.trim_succeeded = false;
    }
    fdm.GetPropulsion()->InitRunning(-1);
    RetractLandingGearIfModeled(fdm);
    SetPropertyIfPresent(fdm, "fcs/throttle-cmd-norm", param.throttle);
    SetPropertyIfPresent(fdm, "fcs/throttle-cmd-norm[1]", param.throttle);
  }

  const auto start =
      model::VehicleStateMapper::Map(*fdm.GetPropagate(), *fdm.GetAccelerations(), fdm, 0.0);
  result.alt_start_m = start.altitude_geod_m;
  result.speed_start_mps = start.vtrue_mps;
  result.min_speed_mps = start.vtrue_mps;
  result.min_altitude_m = start.altitude_geod_m;
  result.alpha_start_deg = PropertyOrZero(fdm, "aero/alpha-deg");
  result.theta_start_rad = start.theta_rad;
  result.gamma_start_rad = PropertyOrZero(fdm, "flight-path/gamma-rad");
  result.qbar_start_pa = start.qbar_pa;

  const int steps = static_cast<int>(20.0 / kDt);
  for (int i = 0; i < steps; ++i) {
    result.run_ok = fdm.Run();
    result.sim_time_sec += kDt;
    const auto state = model::VehicleStateMapper::Map(
        *fdm.GetPropagate(), *fdm.GetAccelerations(), fdm, result.sim_time_sec);
    result.min_speed_mps = std::min(result.min_speed_mps, state.vtrue_mps);
    result.min_altitude_m = std::min(result.min_altitude_m, state.altitude_geod_m);
    if (!result.run_ok || state.altitude_geod_m <= 0.0 || !std::isfinite(state.vtrue_mps)) {
      break;
    }
  }

  const auto end =
      model::VehicleStateMapper::Map(*fdm.GetPropagate(), *fdm.GetAccelerations(), fdm,
                                     result.sim_time_sec);
  result.alt_end_m = end.altitude_geod_m;
  result.speed_end_mps = end.vtrue_mps;
  result.alpha_end_deg = PropertyOrZero(fdm, "aero/alpha-deg");
  result.theta_end_rad = end.theta_rad;
  result.gamma_end_rad = PropertyOrZero(fdm, "flight-path/gamma-rad");
  result.qbar_end_pa = end.qbar_pa;
  result.throttle_pos_end = PropertyOrZero(fdm, "fcs/throttle-pos-norm");
  result.throttle_pos_1_end = PropertyOrZero(fdm, "fcs/throttle-pos-norm[1]");
  result.thrust_end_lbs = PropertyOrZero(fdm, "propulsion/engine/thrust-lbs");
  result.thrust_1_end_lbs = PropertyOrZero(fdm, "propulsion/engine[1]/thrust-lbs");
  result.stable = result.run_ok && std::isfinite(end.altitude_geod_m) &&
                  std::isfinite(end.vtrue_mps) && end.altitude_geod_m > 0.0 &&
                  result.min_speed_mps > 30.0 && end.mass_kg > 0.0;
  return result;
}

F22CommandProbeResult RunF22CommandProbe(bool do_trim, double command) {
  const auto config = MakeF22Config(do_trim);
  adapter::JsbsimAdapter adapter(config);

  F22CommandProbeResult result;
  result.scenario = "command_response";
  result.do_trim = do_trim;
  result.command = command;
  result.trim_attempted = adapter.TrimAttempted();
  result.trim_succeeded = adapter.TrimSucceeded();

  const auto start = model::VehicleStateMapper::Map(
      adapter.GetPropagate(), adapter.GetAccelerations(), adapter.GetFdmExec(), 0.0);
  result.alt_start_m = start.altitude_geod_m;
  result.speed_start_mps = start.vtrue_mps;
  result.roll_start_rad = start.phi_rad;
  result.roll_rate_start_rad_sec = PropertyOrZero(adapter, "velocities/p-aero-rad_sec");

  adapter.SetProperty("fcs/aileron-cmd-norm", command);
  const int steps = static_cast<int>(2.0 / kDt);
  for (int i = 0; i < steps; ++i) {
    result.run_ok = adapter.Run();
    result.sim_time_sec += kDt;
    if (!result.run_ok) {
      break;
    }
  }

  const auto end =
      model::VehicleStateMapper::Map(adapter.GetPropagate(), adapter.GetAccelerations(),
                                     adapter.GetFdmExec(), result.sim_time_sec);
  result.alt_end_m = end.altitude_geod_m;
  result.speed_end_mps = end.vtrue_mps;
  result.roll_end_rad = end.phi_rad;
  result.roll_rate_end_rad_sec = PropertyOrZero(adapter, "velocities/p-aero-rad_sec");
  result.aileron_cmd_end = PropertyOrZero(adapter, "fcs/aileron-cmd-norm");
  result.roll_rate_cmd_end = PropertyOrZero(adapter, "fcs/roll-rate-cmd");
  result.roll_rate_error_end = PropertyOrZero(adapter, "fcs/roll-rate-error");
  result.roll_rate_integrator_end = PropertyOrZero(adapter, "fcs/roll-rate-integrator");
  result.roll_cmd_end = PropertyOrZero(adapter, "fcs/roll-cmd");
  result.aileron_act_end = PropertyOrZero(adapter, "fcs/aileron-act");
  result.left_aileron_pos_end_rad = PropertyOrZero(adapter, "fcs/left-aileron-pos-rad");
  result.right_aileron_pos_end_rad = PropertyOrZero(adapter, "fcs/right-aileron-pos-rad");
  result.stable = result.run_ok && IsStable(end);
  return result;
}

F22ThrottleProbeResult RunF22ThrottleProbe(bool do_trim, double command) {
  const auto config = MakeF22Config(do_trim);
  adapter::JsbsimAdapter adapter(config);

  F22ThrottleProbeResult result;
  result.do_trim = do_trim;
  result.command = command;
  result.trim_attempted = adapter.TrimAttempted();
  result.trim_succeeded = adapter.TrimSucceeded();

  const auto start = model::VehicleStateMapper::Map(
      adapter.GetPropagate(), adapter.GetAccelerations(), adapter.GetFdmExec(), 0.0);
  result.alt_start_m = start.altitude_geod_m;
  result.speed_start_mps = start.vtrue_mps;

  adapter.SetProperty("fcs/throttle-cmd-norm", command);
  if (adapter.HasProperty("fcs/throttle-cmd-norm[1]")) {
    adapter.SetProperty("fcs/throttle-cmd-norm[1]", command);
  }

  const int steps = static_cast<int>(10.0 / kDt);
  for (int i = 0; i < steps; ++i) {
    result.run_ok = adapter.Run();
    result.sim_time_sec += kDt;
    if (!result.run_ok) {
      break;
    }
  }

  const auto end =
      model::VehicleStateMapper::Map(adapter.GetPropagate(), adapter.GetAccelerations(),
                                     adapter.GetFdmExec(), result.sim_time_sec);
  result.alt_end_m = end.altitude_geod_m;
  result.speed_end_mps = end.vtrue_mps;
  result.throttle_pos_end = PropertyOrZero(adapter, "fcs/throttle-pos-norm");
  result.throttle_pos_1_end = PropertyOrZero(adapter, "fcs/throttle-pos-norm[1]");
  result.thrust_end_lbs = PropertyOrZero(adapter, "propulsion/engine/thrust-lbs");
  result.thrust_1_end_lbs = PropertyOrZero(adapter, "propulsion/engine[1]/thrust-lbs");
  result.stable = result.run_ok && IsStable(end);
  return result;
}

double DistanceToTargetM(const model::VehicleState& state, const guidance::Waypoint& target) {
  constexpr double kEarthRadiusM = 6378137.0;
  return std::hypot(target.latitude_rad - state.latitude_rad,
                    target.longitude_rad - state.longitude_rad) *
         kEarthRadiusM;
}

WaypointProbeResult RunWaypointProbe(const WaypointProbeParam& aircraft, double target_distance_m) {
  constexpr double kInvSqrt2 = 0.7071067811865475;
  constexpr double kEarthRadiusM = 6378137.0;
  constexpr int kMaxSteps = 80000;

  FlightManager fm(MakeWaypointConfig(aircraft));
  guidance::Waypoint target;
  const double axis_distance_m = target_distance_m * kInvSqrt2;
  target.latitude_rad = axis_distance_m / kEarthRadiusM;
  target.longitude_rad = axis_distance_m / kEarthRadiusM;
  target.altitude_m = aircraft.altitude_m;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target = target;
  fm.PushManeuver(cmd);

  WaypointProbeResult result;
  result.target_distance_m = target_distance_m;
  result.init_distance_m = DistanceToTargetM(fm.GetVehicleState(), target);
  result.min_distance_m = result.init_distance_m;

  for (int i = 0; i < kMaxSteps; ++i) {
    const bool running = fm.Step(kDt);
    const auto& state = fm.GetVehicleState();
    const double distance_m = DistanceToTargetM(state, target);
    result.min_distance_m = std::min(result.min_distance_m, distance_m);
    result.max_abs_heading_error_rad = std::max(result.max_abs_heading_error_rad,
                                                std::abs(fm.GetAutopilot().GetAngleToHeadingRad()));
    result.max_abs_roll_rad = std::max(result.max_abs_roll_rad, std::abs(state.phi_rad));
    if (fm.GetAdapter().HasProperty("fcs/aileron-cmd-norm")) {
      result.max_abs_aileron_cmd =
          std::max(result.max_abs_aileron_cmd,
                   std::abs(fm.GetAdapter().GetProperty("fcs/aileron-cmd-norm")));
    }

    if (state.altitude_geod_m <= 0.0) {
      result.crashed = true;
      break;
    }
    if (fm.GetState() == FlightManagerState::kCompleted) {
      result.completed = true;
      break;
    }
    if (!running) {
      result.stopped = true;
      break;
    }
  }

  const auto& final_state = fm.GetVehicleState();
  result.final_distance_m = DistanceToTargetM(final_state, target);
  result.sim_time_sec = final_state.sim_time_sec;
  result.final_altitude_m = final_state.altitude_geod_m;
  result.final_speed_mps = final_state.vtrue_mps;
  result.final_heading_error_rad = fm.GetAutopilot().GetAngleToHeadingRad();
  return result;
}

OrbitProbeResult RunOrbitProbe(const OrbitProbeParam& aircraft, double orbit_radius_m,
                               double center_distance_m) {
  constexpr double kInvSqrt2 = 0.7071067811865475;
  constexpr double kEarthRadiusM = 6378137.0;
  constexpr int kMaxSteps = 60000;

  AircraftProbeParam aircraft_config{aircraft.model, aircraft.altitude_m, aircraft.speed_mps};
  FlightManager fm(MakeConfig(aircraft_config, true));

  guidance::Waypoint center;
  const double axis_distance_m = center_distance_m * kInvSqrt2;
  center.latitude_rad = axis_distance_m / kEarthRadiusM;
  center.longitude_rad = axis_distance_m / kEarthRadiusM;
  center.altitude_m = aircraft.altitude_m;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target = center;
  cmd.value = orbit_radius_m;
  fm.PushManeuver(cmd);

  OrbitProbeResult result;
  result.orbit_radius_m = orbit_radius_m;
  result.center_distance_m = center_distance_m;
  result.min_distance_m = std::numeric_limits<double>::max();
  result.min_altitude_m = std::numeric_limits<double>::max();

  for (int i = 0; i < kMaxSteps; ++i) {
    const bool running = fm.Step(kDt);
    const auto& state = fm.GetVehicleState();
    const double distance_m = std::hypot(center.latitude_rad - state.latitude_rad,
                                         center.longitude_rad - state.longitude_rad) *
                              kEarthRadiusM;
    result.min_distance_m = std::min(result.min_distance_m, distance_m);
    result.max_distance_m = std::max(result.max_distance_m, distance_m);
    result.min_altitude_m = std::min(result.min_altitude_m, state.altitude_geod_m);
    result.max_abs_roll_rad = std::max(result.max_abs_roll_rad, std::abs(state.phi_rad));
    result.max_abs_heading_error_rad = std::max(result.max_abs_heading_error_rad,
                                                std::abs(fm.GetAutopilot().GetAngleToHeadingRad()));

    if (state.altitude_geod_m <= 0.0) {
      result.crashed = true;
      break;
    }
    if (!running) {
      result.stopped = true;
      break;
    }
  }

  const auto& final_state = fm.GetVehicleState();
  result.final_distance_m = std::hypot(center.latitude_rad - final_state.latitude_rad,
                                       center.longitude_rad - final_state.longitude_rad) *
                            kEarthRadiusM;
  result.sim_time_sec = final_state.sim_time_sec;
  result.final_altitude_m = final_state.altitude_geod_m;
  result.final_speed_mps = final_state.vtrue_mps;
  return result;
}

bool ResetFileExists(const std::string& model) {
  std::ifstream file(std::string(FD_JSBSIM_ROOT_DIR) + "/aircraft/" + model + "/reset00.xml");
  return file.good();
}

ProbeSnapshot RunResetFreeFlight(const AircraftProbeParam& aircraft) {
  ProbeSnapshot snapshot;
  if (!ResetFileExists(aircraft.model)) {
    return snapshot;
  }

  JSBSim::FGFDMExec fdm;
  fdm.SetDebugLevel(0);
  fdm.SetRootDir(SGPath(FD_JSBSIM_ROOT_DIR));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetEnginePath(SGPath("engine"));
  fdm.SetSystemsPath(SGPath("systems"));
  fdm.Setdt(kDt);
  if (!fdm.LoadModel(aircraft.model, true)) {
    return snapshot;
  }
  if (!fdm.GetIC()->Load(SGPath("reset00.xml"), true)) {
    return snapshot;
  }
  if (!fdm.RunIC()) {
    return snapshot;
  }
  fdm.GetPropulsion()->InitRunning(-1);

  const auto start =
      model::VehicleStateMapper::Map(*fdm.GetPropagate(), *fdm.GetAccelerations(), fdm, 0.0);
  snapshot.alt_start_m = start.altitude_geod_m;
  snapshot.vt_start_mps = start.vtrue_mps;

  const int steps = static_cast<int>(kProbeRunSec / kDt);
  for (int i = 0; i < steps; ++i) {
    snapshot.run_ok = fdm.Run();
    snapshot.sim_time_sec += kDt;
    if (!snapshot.run_ok) {
      break;
    }
  }

  const auto end = model::VehicleStateMapper::Map(*fdm.GetPropagate(), *fdm.GetAccelerations(), fdm,
                                                  snapshot.sim_time_sec);
  snapshot.alt_end_m = end.altitude_geod_m;
  snapshot.vt_end_mps = end.vtrue_mps;
  snapshot.stable = snapshot.run_ok && std::isfinite(end.altitude_geod_m) &&
                    std::isfinite(end.vtrue_mps) && end.mass_kg > 0.0;
  return snapshot;
}

std::string CsvBool(bool value) { return value ? "1" : "0"; }

std::string LateralInterfaceName(autopilot::LateralControlInterface lateral_interface) {
  switch (lateral_interface) {
    case autopilot::LateralControlInterface::kDirectSurface:
      return "direct_surface";
    case autopilot::LateralControlInterface::kGenericAutopilotBridge:
      return "generic_ap_bridge";
    case autopilot::LateralControlInterface::kOwnAutopilot:
      return "own_ap";
    case autopilot::LateralControlInterface::kFbwRateCommand:
      return "fbw_rate_command";
  }
  return "unknown";
}

void WriteProjectRow(std::ostream& out, const AircraftProbeParam& aircraft, bool do_trim) {
  const auto config = MakeConfig(aircraft, do_trim);
  try {
    adapter::JsbsimAdapter adapter(config);
    autopilot::Autopilot ap(adapter);
    const auto& profile = ap.GetControlProfile();
    const ProbeSnapshot snapshot = RunProjectFreeFlight(adapter);
    const AileronProbe pos = RunAileronProbe(config, 0.2);
    const AileronProbe neg = RunAileronProbe(config, -0.2);

    out << aircraft.model << ",project_air_" << (do_trim ? "trim_on" : "trim_off") << ","
        << CsvBool(adapter.TrimAttempted()) << "," << CsvBool(adapter.TrimSucceeded()) << ","
        << CsvBool(adapter.HasProperty("ap/heading_hold")) << ","
        << CsvBool(adapter.HasProperty("ap/autopilot-roll-on")) << ","
        << CsvBool(adapter.HasProperty("fcs/fbw-override")) << ","
        << CsvBool(profile.has_roll_rate_command) << ","
        << CsvBool(adapter.HasProperty("fcs/aileron-cmd-norm")) << ","
        << LateralInterfaceName(profile.lateral_interface) << "," << profile.engine_count << ","
        << CsvBool(snapshot.run_ok) << "," << CsvBool(snapshot.stable) << ","
        << snapshot.sim_time_sec << "," << snapshot.alt_start_m << "," << snapshot.alt_end_m << ","
        << snapshot.alt_end_m - snapshot.alt_start_m << "," << snapshot.vt_start_mps << ","
        << snapshot.vt_end_mps << "," << pos.roll_delta_rad << "," << neg.roll_delta_rad << ","
        << pos.left_aileron_delta_rad << "," << pos.right_aileron_delta_rad << ",ok\n";
  } catch (const std::exception& ex) {
    out << aircraft.model << ",project_air_" << (do_trim ? "trim_on" : "trim_off")
        << ",0,0,0,0,0,0,0,unknown,0,0,0,0,0,0,0,0,0,0,0,0,0,error:" << ex.what() << "\n";
  }
}

void WriteResetRow(std::ostream& out, const AircraftProbeParam& aircraft) {
  const ProbeSnapshot snapshot = RunResetFreeFlight(aircraft);
  const std::string note = ResetFileExists(aircraft.model) ? "ok" : "no_reset00";
  out << aircraft.model << ",jsbsim_reset00"
      << ",0,0,0,0,0,0,0,unknown,0"
      << "," << CsvBool(snapshot.run_ok) << "," << CsvBool(snapshot.stable) << ","
      << snapshot.sim_time_sec << "," << snapshot.alt_start_m << "," << snapshot.alt_end_m << ","
      << snapshot.alt_end_m - snapshot.alt_start_m << "," << snapshot.vt_start_mps << ","
      << snapshot.vt_end_mps << ",0,0,0,0," << note << "\n";
}

TEST(FdAircraftProbe, EmitsAircraftProfileCsv) {
  const char* enabled = std::getenv("FD_RUN_AIRCRAFT_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    GTEST_SKIP() << "set FD_RUN_AIRCRAFT_PROBE=1 to emit aircraft probe CSV";
  }

  const char* csv_env = std::getenv("FD_AIRCRAFT_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_aircraft_probe.csv";
  std::ofstream out(csv_path);
  ASSERT_TRUE(out.is_open()) << csv_path;

  out << std::fixed << std::setprecision(6);
  out << "model,scenario,trim_attempted,trim_succeeded,has_own_ap,"
         "has_generic_ap,has_fbw,has_roll_rate_command,has_aileron_cmd,"
         "lateral_interface,engine_count,run_ok,stable,sim_time_sec,alt_start_m,"
         "alt_end_m,alt_delta_m,vt_start_mps,"
         "vt_end_mps,roll_pos_delta_rad,roll_neg_delta_rad,"
         "left_aileron_pos_delta_rad,right_aileron_pos_delta_rad,note\n";

  for (const auto& aircraft : ProbeAircraft()) {
    WriteProjectRow(out, aircraft, true);
    WriteProjectRow(out, aircraft, false);
    WriteResetRow(out, aircraft);
  }
}

TEST(FdAircraftProbe, EmitsWaypointSweepCsv) {
  const char* enabled = std::getenv("FD_RUN_WAYPOINT_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    GTEST_SKIP() << "set FD_RUN_WAYPOINT_PROBE=1 to emit waypoint sweep CSV";
  }

  const char* csv_env = std::getenv("FD_WAYPOINT_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_waypoint_probe.csv";
  std::ofstream out(csv_path);
  ASSERT_TRUE(out.is_open()) << csv_path;

  out << std::fixed << std::setprecision(6);
  out << "model,target_distance_m,init_distance_m,min_distance_m,"
         "final_distance_m,completed,crashed,stopped,sim_time_sec,"
         "final_altitude_m,final_speed_mps,final_heading_error_rad,"
         "max_abs_heading_error_rad,max_abs_roll_rad,max_abs_aileron_cmd\n";

  for (const auto& aircraft : WaypointProbeAircraft()) {
    for (double target_distance_m : {5000.0, 10000.0, 20000.0}) {
      const WaypointProbeResult result = RunWaypointProbe(aircraft, target_distance_m);
      out << aircraft.model << "," << result.target_distance_m << "," << result.init_distance_m
          << "," << result.min_distance_m << "," << result.final_distance_m << ","
          << CsvBool(result.completed) << "," << CsvBool(result.crashed) << ","
          << CsvBool(result.stopped) << "," << result.sim_time_sec << "," << result.final_altitude_m
          << "," << result.final_speed_mps << "," << result.final_heading_error_rad << ","
          << result.max_abs_heading_error_rad << "," << result.max_abs_roll_rad << ","
          << result.max_abs_aileron_cmd << "\n";
    }
  }
}

TEST(FdAircraftProbe, EmitsOrbitSweepCsv) {
  const char* enabled = std::getenv("FD_RUN_ORBIT_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    GTEST_SKIP() << "set FD_RUN_ORBIT_PROBE=1 to emit orbit sweep CSV";
  }

  const char* csv_env = std::getenv("FD_ORBIT_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_orbit_probe.csv";
  std::ofstream out(csv_path);
  ASSERT_TRUE(out.is_open()) << csv_path;

  out << std::fixed << std::setprecision(6);
  out << "model,altitude_m,speed_mps,orbit_radius_m,center_distance_m,"
         "final_distance_m,min_distance_m,max_distance_m,sim_time_sec,"
         "final_altitude_m,min_altitude_m,final_speed_mps,max_abs_roll_rad,"
         "max_abs_heading_error_rad,crashed,stopped\n";

  for (const auto& aircraft : OrbitProbeAircraft()) {
    for (double radius_m : {1000.0, 3000.0, 6000.0, 10000.0, 20000.0}) {
      const OrbitProbeResult result = RunOrbitProbe(aircraft, radius_m, radius_m * 2.0);
      out << aircraft.model << "," << aircraft.altitude_m << "," << aircraft.speed_mps << ","
          << result.orbit_radius_m << "," << result.center_distance_m << ","
          << result.final_distance_m << "," << result.min_distance_m << "," << result.max_distance_m
          << "," << result.sim_time_sec << "," << result.final_altitude_m << ","
          << result.min_altitude_m << "," << result.final_speed_mps << ","
          << result.max_abs_roll_rad << "," << result.max_abs_heading_error_rad << ","
          << CsvBool(result.crashed) << "," << CsvBool(result.stopped) << "\n";
    }
  }
}

TEST(FdAircraftProbe, EmitsF22FocusedCsv) {
  const char* enabled = std::getenv("FD_RUN_F22_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    GTEST_SKIP() << "set FD_RUN_F22_PROBE=1 to emit f22 focused CSV";
  }

  const char* csv_env = std::getenv("FD_F22_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_f22_probe.csv";
  std::ofstream out(csv_path);
  ASSERT_TRUE(out.is_open()) << csv_path;

  out << std::fixed << std::setprecision(6);
  out << "kind,scenario,do_trim,command,target_distance_m,trim_attempted,"
         "trim_succeeded,run_ok,stable,completed,crashed,stopped,sim_time_sec,"
         "alt_start_m,alt_end_m,speed_start_mps,speed_end_mps,roll_start_rad,"
         "roll_end_rad,roll_rate_start_rad_sec,roll_rate_end_rad_sec,"
         "init_distance_m,min_distance_m,final_distance_m,"
         "final_heading_error_rad,max_abs_heading_error_rad,max_abs_roll_rad,"
         "max_abs_aileron_cmd,aileron_cmd_end,roll_rate_cmd_end,"
         "roll_rate_error_end,roll_rate_integrator_end,roll_cmd_end,"
         "aileron_act_end,left_aileron_pos_end_rad,right_aileron_pos_end_rad\n";

  for (bool do_trim : {true, false}) {
    {
      adapter::JsbsimAdapter adapter(MakeF22Config(do_trim));
      const ProbeSnapshot snapshot = RunProjectFreeFlight(adapter);
      out << "free_flight,project_air," << CsvBool(do_trim) << ",0,0,"
          << CsvBool(adapter.TrimAttempted()) << "," << CsvBool(adapter.TrimSucceeded()) << ","
          << CsvBool(snapshot.run_ok) << "," << CsvBool(snapshot.stable) << ",0,0,0,"
          << snapshot.sim_time_sec << "," << snapshot.alt_start_m << "," << snapshot.alt_end_m
          << "," << snapshot.vt_start_mps << "," << snapshot.vt_end_mps << ",0,0,0,0,0,0,0,0,0,0,0,"
          << PropertyOrZero(adapter, "fcs/aileron-cmd-norm") << ","
          << PropertyOrZero(adapter, "fcs/roll-rate-cmd") << ","
          << PropertyOrZero(adapter, "fcs/roll-rate-error") << ","
          << PropertyOrZero(adapter, "fcs/roll-rate-integrator") << ","
          << PropertyOrZero(adapter, "fcs/roll-cmd") << ","
          << PropertyOrZero(adapter, "fcs/aileron-act") << ","
          << PropertyOrZero(adapter, "fcs/left-aileron-pos-rad") << ","
          << PropertyOrZero(adapter, "fcs/right-aileron-pos-rad") << "\n";
    }

    for (double command : {-0.4, 0.4}) {
      const F22CommandProbeResult result = RunF22CommandProbe(do_trim, command);
      out << "command," << result.scenario << "," << CsvBool(result.do_trim) << ","
          << result.command << ",0," << CsvBool(result.trim_attempted) << ","
          << CsvBool(result.trim_succeeded) << "," << CsvBool(result.run_ok) << ","
          << CsvBool(result.stable) << ",0,0,0," << result.sim_time_sec << "," << result.alt_start_m
          << "," << result.alt_end_m << "," << result.speed_start_mps << "," << result.speed_end_mps
          << "," << result.roll_start_rad << "," << result.roll_end_rad << ","
          << result.roll_rate_start_rad_sec << "," << result.roll_rate_end_rad_sec
          << ",0,0,0,0,0,0,0," << result.aileron_cmd_end << "," << result.roll_rate_cmd_end << ","
          << result.roll_rate_error_end << "," << result.roll_rate_integrator_end << ","
          << result.roll_cmd_end << "," << result.aileron_act_end << ","
          << result.left_aileron_pos_end_rad << "," << result.right_aileron_pos_end_rad << "\n";
    }
  }

  for (const WaypointProbeParam& aircraft :
       {WaypointProbeParam{"f22", 3000.0, 200.0}, WaypointProbeParam{"f22", 5000.0, 250.0},
        WaypointProbeParam{"f22", 10000.0, 300.0}}) {
    for (double target_distance_m : {5000.0, 10000.0, 20000.0}) {
      const WaypointProbeResult result = RunWaypointProbe(aircraft, target_distance_m);
      out << "waypoint,fly_to_alt" << aircraft.altitude_m << "_spd" << aircraft.speed_mps << ",1,0,"
          << result.target_distance_m << ",1,0,1," << CsvBool(!result.crashed) << ","
          << CsvBool(result.completed) << "," << CsvBool(result.crashed) << ","
          << CsvBool(result.stopped) << "," << result.sim_time_sec << "," << aircraft.altitude_m
          << "," << result.final_altitude_m << "," << aircraft.speed_mps << ","
          << result.final_speed_mps << ",0,0,0,0," << result.init_distance_m << ","
          << result.min_distance_m << "," << result.final_distance_m << ","
          << result.final_heading_error_rad << "," << result.max_abs_heading_error_rad << ","
          << result.max_abs_roll_rad << "," << result.max_abs_aileron_cmd << ",0,0,0,0,0,0,0,0\n";
    }
  }
}

TEST(FdAircraftProbe, EmitsF22ThrottleCsv) {
  const char* enabled = std::getenv("FD_RUN_F22_THROTTLE_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    GTEST_SKIP() << "set FD_RUN_F22_THROTTLE_PROBE=1 to emit f22 throttle CSV";
  }

  const char* csv_env = std::getenv("FD_F22_THROTTLE_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_f22_throttle_probe.csv";
  std::ofstream out(csv_path);
  ASSERT_TRUE(out.is_open()) << csv_path;

  out << std::fixed << std::setprecision(6);
  out << "do_trim,command,trim_attempted,trim_succeeded,run_ok,stable,"
         "sim_time_sec,alt_start_m,alt_end_m,speed_start_mps,speed_end_mps,"
         "throttle_pos_end,throttle_pos_1_end,thrust_end_lbs,thrust_1_end_lbs\n";

  for (bool do_trim : {true, false}) {
    for (double command : {0.3, 0.7, 1.0}) {
      const F22ThrottleProbeResult result = RunF22ThrottleProbe(do_trim, command);
      out << CsvBool(result.do_trim) << "," << result.command << ","
          << CsvBool(result.trim_attempted) << "," << CsvBool(result.trim_succeeded) << ","
          << CsvBool(result.run_ok) << "," << CsvBool(result.stable) << "," << result.sim_time_sec
          << "," << result.alt_start_m << "," << result.alt_end_m << "," << result.speed_start_mps
          << "," << result.speed_end_mps << "," << result.throttle_pos_end << ","
          << result.throttle_pos_1_end << "," << result.thrust_end_lbs << ","
          << result.thrust_1_end_lbs << "\n";
    }
  }
}

TEST(FdAircraftProbe, EmitsF22EnvelopeCsv) {
  const char* enabled = std::getenv("FD_RUN_F22_ENVELOPE_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    GTEST_SKIP() << "set FD_RUN_F22_ENVELOPE_PROBE=1 to emit f22 envelope CSV";
  }

  const char* csv_env = std::getenv("FD_F22_ENVELOPE_PROBE_CSV");
  const std::string csv_path =
      csv_env != nullptr ? csv_env : "/tmp/1q_f22_envelope_probe.csv";
  std::ofstream out(csv_path);
  ASSERT_TRUE(out.is_open()) << csv_path;

  out << std::fixed << std::setprecision(6);
  out << "altitude_m,speed_mps,alpha_deg,gamma_deg,throttle,do_trim,"
         "loaded,run_ic_ok,trim_attempted,trim_succeeded,run_ok,stable,"
         "sim_time_sec,alt_start_m,alt_end_m,speed_start_mps,speed_end_mps,"
         "min_speed_mps,min_altitude_m,alpha_start_deg,alpha_end_deg,"
         "theta_start_rad,theta_end_rad,gamma_start_rad,gamma_end_rad,"
         "qbar_start_pa,qbar_end_pa,throttle_pos_end,throttle_pos_1_end,"
         "thrust_end_lbs,thrust_1_end_lbs\n";

  for (double altitude_m : {3000.0, 5000.0, 10000.0}) {
    for (double speed_mps : {200.0, 250.0, 300.0, 350.0}) {
      for (double alpha_deg : {-2.0, 0.0, 4.0, 8.0}) {
        for (double gamma_deg : {-2.0, 0.0, 2.0}) {
          for (bool do_trim : {false, true}) {
            const F22EnvelopeProbeParam param{altitude_m, speed_mps, alpha_deg, gamma_deg,
                                              1.0, do_trim};
            const F22EnvelopeProbeResult result = RunF22EnvelopeProbe(param);
            out << param.altitude_m << "," << param.speed_mps << "," << param.alpha_deg << ","
                << param.gamma_deg << "," << param.throttle << "," << CsvBool(param.do_trim)
                << "," << CsvBool(result.loaded) << "," << CsvBool(result.run_ic_ok) << ","
                << CsvBool(result.trim_attempted) << "," << CsvBool(result.trim_succeeded) << ","
                << CsvBool(result.run_ok) << "," << CsvBool(result.stable) << ","
                << result.sim_time_sec << "," << result.alt_start_m << "," << result.alt_end_m
                << "," << result.speed_start_mps << "," << result.speed_end_mps << ","
                << result.min_speed_mps << "," << result.min_altitude_m << ","
                << result.alpha_start_deg << "," << result.alpha_end_deg << ","
                << result.theta_start_rad << "," << result.theta_end_rad << ","
                << result.gamma_start_rad << "," << result.gamma_end_rad << ","
                << result.qbar_start_pa << "," << result.qbar_end_pa << ","
                << result.throttle_pos_end << "," << result.throttle_pos_1_end << ","
                << result.thrust_end_lbs << "," << result.thrust_1_end_lbs << "\n";
          }
        }
      }
    }
  }
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq
