// FD 开发期验证工具：机型能力探针 → 三种 CSV 导出。
// 用法：aircraft_probe_csv（按环境变量门控导出）：
//   FD_RUN_AIRCRAFT_PROBE=1 → 机型剖面/襟副翼探针（FD_AIRCRAFT_PROBE_CSV 或 /tmp/1q_aircraft_probe.csv）
//   FD_RUN_WAYPOINT_PROBE=1 → 航点寻的扫描（FD_WAYPOINT_PROBE_CSV 或 /tmp/1q_waypoint_probe.csv）
//   FD_RUN_ORBIT_PROBE=1    → 盘旋扫描（FD_ORBIT_PROBE_CSV 或 /tmp/1q_orbit_probe.csv）
// 无任何开启时仅打印提示（零耗时）；CSV 供人工/脚本分析，不设断言。
// （2026-08-10 自 GTest fd_aircraft_probe_test.cpp 迁出：本文件是数据导出
// 而非断言，GTest 分区只保留确定性断言。）

#include <algorithm>
#include <cmath>
#include <cstdio>
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
constexpr double kDt = 0.01;

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
      {"f15", 3000.0, 200.0},  {"f16", 3000.0, 200.0},
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
      {"f15", 3000.0, 200.0},  {"f16", 3000.0, 200.0},
      {"737", 3000.0, 130.0},  {"B747", 3000.0, 140.0}, {"Concorde", 5000.0, 150.0},
      {"MD11", 3000.0, 140.0}, {"c310", 500.0, 65.0},
  };
}

std::vector<OrbitProbeParam> OrbitProbeAircraft() {
  return {
      {"f16", 3000.0, 200.0},
      {"Concorde", 15000.0, 500.0},     // supersonic: clean orbit, no crash
      {"Concorde", 10000.0, 250.0},      // transonic: marginal
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

int EmitAircraftProfileCsv() {
  const char* enabled = std::getenv("FD_RUN_AIRCRAFT_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    std::printf("aircraft_probe_csv: FD_RUN_AIRCRAFT_PROBE 未开启，跳过机型剖面导出（设 =1 开启）\n");
    return 0;
  }

  const char* csv_env = std::getenv("FD_AIRCRAFT_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_aircraft_probe.csv";
  std::ofstream out(csv_path);
  if (!out.is_open()) {
    std::fprintf(stderr, "aircraft_probe_csv: 无法打开 %s\n", csv_path.c_str());
    return 1;
  }

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
  std::printf("aircraft_probe_csv: 已导出 %s\n", csv_path.c_str());
  return 0;
}

int EmitWaypointSweepCsv() {
  const char* enabled = std::getenv("FD_RUN_WAYPOINT_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    std::printf("aircraft_probe_csv: FD_RUN_WAYPOINT_PROBE 未开启，跳过航点扫描导出（设 =1 开启）\n");
    return 0;
  }

  const char* csv_env = std::getenv("FD_WAYPOINT_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_waypoint_probe.csv";
  std::ofstream out(csv_path);
  if (!out.is_open()) {
    std::fprintf(stderr, "aircraft_probe_csv: 无法打开 %s\n", csv_path.c_str());
    return 1;
  }

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
  std::printf("aircraft_probe_csv: 已导出 %s\n", csv_path.c_str());
  return 0;
}

int EmitOrbitSweepCsv() {
  const char* enabled = std::getenv("FD_RUN_ORBIT_PROBE");
  if (enabled == nullptr || std::string(enabled) != "1") {
    std::printf("aircraft_probe_csv: FD_RUN_ORBIT_PROBE 未开启，跳过盘旋扫描导出（设 =1 开启）\n");
    return 0;
  }

  const char* csv_env = std::getenv("FD_ORBIT_PROBE_CSV");
  const std::string csv_path = csv_env != nullptr ? csv_env : "/tmp/1q_orbit_probe.csv";
  std::ofstream out(csv_path);
  if (!out.is_open()) {
    std::fprintf(stderr, "aircraft_probe_csv: 无法打开 %s\n", csv_path.c_str());
    return 1;
  }

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
  std::printf("aircraft_probe_csv: 已导出 %s\n", csv_path.c_str());
  return 0;
}

}  // namespace
}  // namespace flight_dynamic
}  // namespace oneq

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "--help") {
    std::printf("aircraft_probe_csv — FD 机型能力探针 CSV 导出工具\n"
                "用法：aircraft_probe_csv\n"
                "环境变量门控（可同时开启多个）：\n"
                "  FD_RUN_AIRCRAFT_PROBE=1  机型剖面/襟副翼探针 → FD_AIRCRAFT_PROBE_CSV | /tmp/1q_aircraft_probe.csv\n"
                "  FD_RUN_WAYPOINT_PROBE=1  航点寻的扫描 → FD_WAYPOINT_PROBE_CSV | /tmp/1q_waypoint_probe.csv\n"
                "  FD_RUN_ORBIT_PROBE=1     盘旋扫描 → FD_ORBIT_PROBE_CSV | /tmp/1q_orbit_probe.csv\n");
    return 0;
  }
  int status = 0;
  status |= oneq::flight_dynamic::EmitAircraftProfileCsv();
  status |= oneq::flight_dynamic::EmitWaypointSweepCsv();
  status |= oneq::flight_dynamic::EmitOrbitSweepCsv();
  return status;
}
