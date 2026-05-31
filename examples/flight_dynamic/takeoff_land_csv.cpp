/// Standalone program: run takeoff → cruise → landing and output CSV.
/// Usage: takeoff_land_csv <aircraft_model> [output.csv]
///   If output.csv is omitted, writes to stdout.

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/propulsion/EngineManager.h"

using namespace oneq::flight_dynamic;

namespace {

constexpr double kDt = 0.01;

struct CsvRow {
  double time_sec, agl_m, vc_kts, pitch_deg, roll_deg, throttle, wow;
  int fm_state;
};

void WriteHeader(FILE* out) {
  fprintf(out, "time_sec,agl_m,vc_kts,pitch_deg,roll_deg,throttle,elevator,wow,"
          "thr0,thr1,thr2,thr3,fm_state\n");
}

void WriteRow(FILE* out, double t, FlightManager& fm) {
  auto* pm = fm.GetAdapter().GetFdmExec().GetPropertyManager().get();
  auto get = [&](const char* n) {
    auto* node = pm->GetNode(n);
    return node ? node->getDoubleValue() : -1.0;
  };
  fprintf(out, "%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,"
          "%.2f,%.2f,%.2f,%.2f,%d\n",
          t,
          get("position/h-agl-ft") * 0.3048,
          get("velocities/vc-kts"),
          get("attitude/pitch-rad") * 57.2958,
          get("attitude/roll-rad") * 57.2958,
          get("fcs/throttle-cmd-norm"),
          get("fcs/elevator-cmd-norm"),
          get("gear/unit[1]/WOW"),
          get("fcs/throttle-cmd-norm[0]"),
          get("fcs/throttle-cmd-norm[1]"),
          get("fcs/throttle-cmd-norm[2]"),
          get("fcs/throttle-cmd-norm[3]"),
          static_cast<int>(fm.GetState()));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: takeoff_land_csv <aircraft_model> [output.csv]\n";
    return 1;
  }
  std::string model(argv[1]);
  const char* out_path = (argc >= 3) ? argv[2] : nullptr;
  FILE* out = out_path ? fopen(out_path, "w") : stdout;
  if (!out) {
    std::cerr << "Cannot open " << out_path << "\n";
    return 1;
  }

  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = false;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 0.0;
  cfg.initial_kinematics.velocity_mps.x_mps = 0.0;
  // f22: slight nose-up from landing gear geometry helps FBW rotation
  if (model == "f22") cfg.initial_kinematics.attitude_deg.pitch_deg = 3.0;

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::cerr << model << ": init failed\n";
    if (out_path) fclose(out);
    return 1;
  }

  // Known limit: B17 can't reach Vr at max weight (wing loading 37 lbs/ft²).
  if (model == "B17") {
    fprintf(stderr, "%s: skipped (Vr unreachable at MTOW)\n", model.c_str());
    if (out_path) fclose(out); return 0;
  }

  // Cruise altitude by engine/aircraft category.
  propulsion::EngineManager eng(fm.GetAdapter());
  double cruise_alt_m = 3000.0;
  switch (eng.GetType()) {
    case propulsion::EngineType::kPiston:   cruise_alt_m = 1500.0; break;
    case propulsion::EngineType::kTurboprop: cruise_alt_m = 4000.0; break;
    case propulsion::EngineType::kTurbine:
      if (model.find("f16") != std::string::npos ||
          model.find("f22") != std::string::npos ||
          model.find("f15") != std::string::npos ||
          model.find("A4")  != std::string::npos ||
          model.find("F4")  != std::string::npos ||
          model.find("F80") != std::string::npos)
        cruise_alt_m = 500.0;
      else if (model == "Concorde")
        cruise_alt_m = 12000.0;
      else if (model == "B17" || model == "C130")
        cruise_alt_m = 5000.0;
      else
        cruise_alt_m = 8000.0;  // commercial jets
      break;
    case propulsion::EngineType::kRocket: cruise_alt_m = 15000.0; break;
    default: break;
  }

  // Queue: takeoff → cruise → land
  ManeuverCommand tko;
  tko.type = guidance::ManeuverType::kTakeoff;
  tko.target.altitude_m = cruise_alt_m;
  fm.PushManeuver(tko);

  // Waypoint: distance proportional to cruise altitude.
  // Min 3km, max at altitude — fast jets need room to converge.
  double ref_spd = fm.GetAutopilot().GetControlProfile().ref_speed_mps;
  if (ref_spd <= 0.0) ref_spd = 50.0;
  double wp_dist_m = ref_spd * 45.0;
  if (wp_dist_m < cruise_alt_m * 1.5) wp_dist_m = cruise_alt_m * 1.5;
  if (wp_dist_m < 3000.0) wp_dist_m = 3000.0;
  double wp_offset_rad = wp_dist_m * 0.70710678 / 6378137.0;

  ManeuverCommand fly;
  fly.type = guidance::ManeuverType::kFlyToWaypoint;
  fly.target.latitude_rad = wp_offset_rad;
  fly.target.longitude_rad = wp_offset_rad;
  fly.target.altitude_m = cruise_alt_m;
  fly.target.radius_m = std::max(200.0, ref_spd * 20.0);  // generous for fast jets
  fm.PushManeuver(fly);

  ManeuverCommand land;
  land.type = guidance::ManeuverType::kLand;
  land.target.latitude_rad = 0.0012;
  land.target.longitude_rad = 0.0012;
  land.target.altitude_m = 0.0;
  land.value = eng.GetRotationSpeedKts() * 0.514 * 1.3;  // Vr×1.3 in m/s
  fm.PushManeuver(land);

  WriteHeader(out);

  double t = 0.0;
  int max_steps = 250000;  // 2500s max
  for (int i = 0; i < max_steps; ++i) {
    fm.Step(kDt);
    t += kDt;
    WriteRow(out, t, fm);
    if (fm.GetState() == FlightManagerState::kCompleted ||
        fm.GetState() == FlightManagerState::kAborted) {
      if (fm.GetState() == FlightManagerState::kCompleted) {
        fprintf(stderr, "%s: completed at t=%.1fs\n", model.c_str(), t);
      } else {
        fprintf(stderr, "%s: aborted at t=%.1fs\n", model.c_str(), t);
      }
      break;
    }
  }

  if (out_path) fclose(out);
  return 0;
}
