/// Standalone: run standard maneuvers for one or all aircraft, output CSV summary.
/// Usage: maneuver_sweep_csv <aircraft_model|ALL> [output.csv]
///
/// CSV columns: model, maneuver, outcome, steps, time_s, alt_min_m, alt_max_m,
///              spd_min_mps, spd_max_mps, pitch_max_deg, roll_max_deg, crashed

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"

using namespace oneq::flight_dynamic;

namespace {

constexpr double kDt = 0.01;

struct ManeuverResult {
  std::string aircraft;
  std::string maneuver;
  std::string outcome;
  int steps = 0;
  double time_s = 0;
  double alt_min = 1e9, alt_max = -1e9;
  double spd_min = 1e9, spd_max = -1e9;
  double pitch_max = 0, roll_max = 0;
  bool crashed = false;
};

void TrackState(FlightManager& fm, ManeuverResult& r) {
  const auto& s = fm.GetVehicleState();
  if (s.altitude_geod_m < r.alt_min) r.alt_min = s.altitude_geod_m;
  if (s.altitude_geod_m > r.alt_max) r.alt_max = s.altitude_geod_m;
  if (s.vtrue_mps < r.spd_min) r.spd_min = s.vtrue_mps;
  if (s.vtrue_mps > r.spd_max) r.spd_max = s.vtrue_mps;
  double p = std::abs(s.theta_rad) * 57.2958;
  double ro = std::abs(s.phi_rad) * 57.2958;
  if (p > r.pitch_max) r.pitch_max = p;
  if (ro > r.roll_max) r.roll_max = ro;
  if (s.altitude_agl_m <= 0) r.crashed = true;
}

int RunUntilState(FlightManager& fm, int max_steps, ManeuverResult& r) {
  for (int i = 0; i < max_steps; ++i) {
    fm.Step(kDt);
    TrackState(fm, r);
    auto st = fm.GetState();
    if (st == FlightManagerState::kCompleted) {
      r.outcome = "completed";
      return i + 1;
    }
    if (st == FlightManagerState::kAborted) {
      r.outcome = r.crashed ? "crashed" : "aborted";
      return i + 1;
    }
  }
  r.outcome = "timeout";
  return max_steps;
}

config::FlightDynamicConfig MakeConfig(const std::string& model) {
  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = true;
  cfg.silent_mode = true;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 500.0;
  cfg.initial_kinematics.velocity_mps.x_mps = 50.0;
  return cfg;
}

void RunFlyToWaypoint(const std::string& model, std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "FlyToWaypoint";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFlyToWaypoint;
  cmd.target.latitude_rad = 0.0008;
  cmd.target.longitude_rad = 0.0008;
  cmd.target.altitude_m = 500.0;
  cmd.target.radius_m = 200.0;
  fm.PushManeuver(cmd);
  r.steps = RunUntilState(fm, 40000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunOrbit(const std::string& model, std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "Orbit";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = 0.001;
  cmd.target.longitude_rad = 0.001;
  cmd.target.altitude_m = 500.0;
  cmd.value = 2000.0;
  cmd.duration_sec = 30.0;
  fm.PushManeuver(cmd);
  r.steps = RunUntilState(fm, 10000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunSetHeading(const std::string& model, std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "SetHeading";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetHeading;
  cmd.value = 1.57;  // 90 degrees
  fm.PushManeuver(cmd);
  r.steps = RunUntilState(fm, 5000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunSetAltitude(const std::string& model, std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "SetAltitude";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetAltitude;
  cmd.value = 800.0;
  fm.PushManeuver(cmd);
  r.steps = RunUntilState(fm, 40000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunSetPitch(const std::string& model, std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "SetPitch";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSetPitch;
  cmd.value = 5.0;
  cmd.duration_sec = 5.0;
  fm.PushManeuver(cmd);
  r.steps = RunUntilState(fm, 1000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunOrbitTimed(const std::string& model, std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "OrbitTimed";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target.latitude_rad = 0.001;
  cmd.target.longitude_rad = 0.001;
  cmd.target.altitude_m = 500.0;
  cmd.value = 2000.0;
  cmd.duration_sec = 5.0;
  fm.PushManeuver(cmd);
  r.steps = RunUntilState(fm, 2000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunQueueOrbitThenHeading(const std::string& model,
                              std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "QueueOrbitHeading";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand orbit;
  orbit.type = guidance::ManeuverType::kOrbit;
  orbit.target.latitude_rad = 0.001;
  orbit.target.longitude_rad = 0.001;
  orbit.target.altitude_m = 500.0;
  orbit.value = 2000.0;
  orbit.duration_sec = 3.0;
  fm.PushManeuver(orbit);
  ManeuverCommand hdg;
  hdg.type = guidance::ManeuverType::kSetHeading;
  hdg.value = 0.0;
  fm.PushManeuver(hdg);
  r.steps = RunUntilState(fm, 5000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void RunQueueFlyToThenOrbit(const std::string& model,
                            std::vector<ManeuverResult>& results) {
  ManeuverResult r;
  r.aircraft = model;
  r.maneuver = "QueueFlyOrbit";
  FlightManager fm(MakeConfig(model));
  if (fm.GetState() != FlightManagerState::kReady) {
    r.outcome = "init_failed";
    results.push_back(r);
    return;
  }
  ManeuverCommand fly;
  fly.type = guidance::ManeuverType::kFlyToWaypoint;
  fly.target.latitude_rad = 0.0005;
  fly.target.longitude_rad = 0.0005;
  fly.target.altitude_m = 500.0;
  fly.target.radius_m = 200.0;
  fm.PushManeuver(fly);
  ManeuverCommand orbit;
  orbit.type = guidance::ManeuverType::kOrbit;
  orbit.target.latitude_rad = 0.001;
  orbit.target.longitude_rad = 0.001;
  orbit.target.altitude_m = 500.0;
  orbit.value = 2000.0;
  orbit.duration_sec = 3.0;
  fm.PushManeuver(orbit);
  r.steps = RunUntilState(fm, 20000, r);
  r.time_s = r.steps * kDt;
  results.push_back(r);
}

void WriteResults(FILE* out, const std::vector<ManeuverResult>& results) {
  fprintf(out, "aircraft,maneuver,outcome,steps,time_s,alt_min_m,alt_max_m,"
          "spd_min_mps,spd_max_mps,pitch_max_deg,roll_max_deg,crashed\n");
  for (const auto& r : results) {
    fprintf(out, "%s,%s,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d\n",
            r.aircraft.c_str(), r.maneuver.c_str(), r.outcome.c_str(),
            r.steps, r.time_s,
            r.alt_min, r.alt_max, r.spd_min, r.spd_max,
            r.pitch_max, r.roll_max, r.crashed ? 1 : 0);
  }
}

const char* kAllAircraft[] = {
    "f16", "f22", "c172x", "c310", "f15", "Concorde",
    "B17", "C130", "L410", "737", "B747", "MD11",
    "A4", "F4N", "F80C", "OV10", "DHC6",
};

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: maneuver_sweep_csv <aircraft|ALL> [output.csv]\n";
    return 1;
  }
  std::string sel(argv[1]);
  const char* out_path = (argc >= 3) ? argv[2] : nullptr;
  FILE* out = out_path ? fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }

  std::vector<std::string> models;
  if (sel == "ALL") {
    for (const char* m : kAllAircraft) models.push_back(m);
  } else {
    models.push_back(sel);
  }

  std::vector<ManeuverResult> all_results;
  for (const auto& model : models) {
    fprintf(stderr, "%s: ", model.c_str());
    RunFlyToWaypoint(model, all_results);       fprintf(stderr, "F");
    RunOrbit(model, all_results);               fprintf(stderr, "O");
    RunSetHeading(model, all_results);          fprintf(stderr, "H");
    RunSetAltitude(model, all_results);         fprintf(stderr, "A");
    RunSetPitch(model, all_results);            fprintf(stderr, "P");
    RunOrbitTimed(model, all_results);          fprintf(stderr, "T");
    RunQueueOrbitThenHeading(model, all_results);  fprintf(stderr, "Q");
    RunQueueFlyToThenOrbit(model, all_results);    fprintf(stderr, "q\n");
  }

  WriteResults(out, all_results);
  if (out_path) fclose(out);
  fprintf(stderr, "Done: %d results\n", (int)all_results.size());
  return 0;
}
