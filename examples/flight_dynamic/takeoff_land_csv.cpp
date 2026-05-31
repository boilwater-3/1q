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

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::cerr << model << ": init failed\n";
    if (out_path) fclose(out);
    return 1;
  }

  // Queue: takeoff → cruise (speed-adaptive distance) → land
  ManeuverCommand tko;
  tko.type = guidance::ManeuverType::kTakeoff;
  tko.target.altitude_m = 400.0;
  fm.PushManeuver(tko);

  // Waypoint distance: 45s of cruise at ref_speed, min 3km.
  double ref_spd = fm.GetAutopilot().GetControlProfile().ref_speed_mps;
  if (ref_spd <= 0.0) ref_spd = 50.0;
  double wp_dist_m = ref_spd * 45.0;
  if (wp_dist_m < 3000.0) wp_dist_m = 3000.0;
  double wp_offset_rad = wp_dist_m * 0.70710678 / 6378137.0;

  ManeuverCommand fly;
  fly.type = guidance::ManeuverType::kFlyToWaypoint;
  fly.target.latitude_rad = wp_offset_rad;
  fly.target.longitude_rad = wp_offset_rad;
  fly.target.altitude_m = 400.0;
  fly.target.radius_m = 200.0;
  fm.PushManeuver(fly);

  ManeuverCommand land;
  land.type = guidance::ManeuverType::kLand;
  land.target.latitude_rad = 0.0012;
  land.target.longitude_rad = 0.0012;
  land.target.altitude_m = 0.0;
  land.value = 45.0;
  fm.PushManeuver(land);

  WriteHeader(out);

  double t = 0.0;
  int max_steps = 200000;  // 2000s max
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
