/// Standalone program: run takeoff → cruise → landing and output CSV.
/// Usage:
///   takeoff_land_csv <aircraft_model> [output.csv]
///   takeoff_land_csv --heading-alt <aircraft_model> [output.csv]
///   takeoff_land_csv --pitch-test <aircraft_model> [output.csv]
///
/// Modes:
///   default     → 3 fly-to waypoints along the 45° diagonal
///   --heading-alt → SetHeading/SetAltitude pairs in cruise, no fly-to
///   --pitch-test → SetPitch sweep after takeoff, then land

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"
#include "flight_dynamic/adapter/JsbsimAdapter.h"
#include "flight_dynamic/propulsion/EngineManager.h"
#include "models/FGGroundReactions.h"

using namespace oneq::flight_dynamic;

namespace {

constexpr double kDt = 0.01;

// ─── Scenario geometry (not aircraft-specific) ────────────────────────────
//
// These are TEST-SCENARIO parameters — they define the shape of the
// takeoff→fly-to→land mission, not aircraft capability.  They live here
// (in the example program) rather than in the core library because
// different scenarios will want different geometries.
//
// The core library provides:
//   - profile.cruise_altitude_m    →  default cruise altitude by category
//   - profile.cruise_speed_mps     →  default cruise speed by category
//   - profile.max_speed_mps        →  speed ceiling
//   - ConfigureForApproach()       →  approach speed priority chain

struct ScenarioConfig {
  double cruise_altitude_m = 0.0;       // 0 → use profile default
  double waypoint_distance_m = 0.0;     // 0 → auto-calculate from speed/altitude
  double landing_lat_rad = 0.0;         // landing target latitude (radians)
  double landing_lon_rad = 0.0;         // landing target longitude (radians)
  int max_steps = 350000;               // simulation step budget
  double altitude_tolerance_m = 10.0;   // SetAltitude convergence
  double heading_tolerance_rad = 0.035; // SetHeading convergence (~2 deg)
};

ScenarioConfig MakeScenario(FlightManager& fm) {
  ScenarioConfig cfg;
  const auto& profile = fm.GetAutopilot().GetControlProfile();
  const auto& engines = fm.GetEngineManager();

  // ── Cruise altitude: scenario-level default by aircraft category ──
  // Derived from profile detection booleans (not model names), so new
  // aircraft get a reasonable default without code changes.  This is a
  // SCENARIO parameter — the same aircraft could cruise at different
  // altitudes in different missions.
  const bool has_fbw = profile.has_fbw_override || profile.has_roll_rate_command;
  const bool is_heavy = profile.engine_count >= 4 ||
      (profile.pitch_moi_lbsft2 > 0.0 && std::log10(profile.pitch_moi_lbsft2) > 7.0);
  const bool is_medium = !is_heavy && profile.pitch_moi_lbsft2 > 0.0 &&
      std::log10(profile.pitch_moi_lbsft2) > 6.0;

  if (has_fbw) {
    cfg.cruise_altitude_m = 500.0;       // FBW fighter: low circuit
  } else if (is_heavy && !profile.has_mixture) {
    cfg.cruise_altitude_m = 10000.0;     // heavy jet transport
  } else if (is_medium && !profile.has_mixture) {
    cfg.cruise_altitude_m = 8000.0;      // medium jet transport (737-class)
  } else if (profile.has_mixture) {
    cfg.cruise_altitude_m = 1500.0;      // piston (all sizes)
  } else {
    // Light turbine / turboprop: use weight-based sub-classification.
    // (Handles OV10-style misclassified turboprops at ~6500 lbs.)
    double weight_lbs = 0.0;
    {
      auto* pm = fm.GetAdapter().GetFdmExec().GetPropertyManager().get();
      auto* w = pm->GetNode("inertia/weight-lbs");
      if (w) weight_lbs = w->getDoubleValue();
    }
    switch (engines.GetType()) {
      case propulsion::EngineType::kRocket:
        cfg.cruise_altitude_m = 15000.0;
        break;
      default:
        cfg.cruise_altitude_m = 4000.0;
        break;
    }
    (void)weight_lbs;  // reserved for future weight-based tuning
  }

  // ── Waypoint distance ──
  // Base distance: ref_speed × 60s (~3 km for c172x, ~30 km for B747).
  // ref_speed_mps is the energy-management reference speed — it is
  // intentionally generous to provide room for fly-to convergence.
  double ref_spd = profile.ref_speed_mps;
  if (ref_spd <= 0.0) ref_spd = 50.0;
  double wp_dist = ref_spd * 60.0;

  // Fast aircraft need longer legs because their takeoff rolls cover
  // large distances at high speed, and fly-to convergence needs room.
  constexpr double kFastAircraftSpeedMps = 150.0;
  constexpr double kFastAircraftDistanceFactor = 80.0;
  if (profile.max_speed_mps > kFastAircraftSpeedMps) {
    wp_dist = std::max(wp_dist, profile.max_speed_mps * kFastAircraftDistanceFactor);
  }

  // Floor: at least 1.5× cruise altitude or 3 km, whichever is larger.
  constexpr double kMinWaypointDistanceM = 3000.0;
  if (wp_dist < cfg.cruise_altitude_m * 1.5) {
    wp_dist = cfg.cruise_altitude_m * 1.5;
  }
  if (wp_dist < kMinWaypointDistanceM) wp_dist = kMinWaypointDistanceM;

  cfg.waypoint_distance_m = wp_dist;

  // ── Landing target ──
  // The landing point is an independent scenario location (simulating a
  // destination airport).  It is NOT derived from the waypoint — a real
  // flight may land at an airport that is not on the outbound route.
  // For this test scenario we place it ~7.6 km diagonal from origin,
  // which gives the aircraft room to descend after passing the waypoint.
  constexpr double kDefaultLandingOffsetRad = 0.0012;
  cfg.landing_lat_rad = kDefaultLandingOffsetRad;
  cfg.landing_lon_rad = kDefaultLandingOffsetRad;

  return cfg;
}

// ─── Model skip detection ─────────────────────────────────────────────────
//
// Some JSBSim aircraft models are not compatible with fixed-wing
// takeoff/landing cycles.  Detection uses property-tree probing, not
// model-name strings, so new aircraft are handled without code changes.

struct SkipReason {
  bool should_skip = false;
  const char* reason = nullptr;
};

SkipReason CheckSkip(FlightManager& fm, const ScenarioConfig& cfg) {
  const auto& profile = fm.GetAutopilot().GetControlProfile();

  // Multirotor: no wingspan → not a fixed-wing aircraft.
  double bw_ft = 0.0;
  {
    auto* pm = fm.GetAdapter().GetFdmExec().GetPropertyManager().get();
    auto* node = pm->GetNode("metrics/bw-ft");
    if (node) bw_ft = node->getDoubleValue();
  }
  if (bw_ft < 1.0) {
    return {true, "multirotor / no fixed wing (bw-ft < 1.0)"};
  }

  // Cruise altitude unreachable: profile derived 0 — shouldn't happen
  // but guard against it.
  if (cfg.cruise_altitude_m <= 0.0) {
    return {true, "cruise altitude not available"};
  }

  (void)profile;
  return {false, nullptr};
}

// ─── CSV output (diagnostic trace) ────────────────────────────────────────

struct CsvRow {
  double time_sec, agl_m, vc_kts, pitch_deg, roll_deg, throttle, wow;
  int fm_state;
};

void WriteHeader(FILE* out) {
  fprintf(out,
          "time_sec,agl_m,vc_kts,pitch_deg,roll_deg,throttle,elevator,wow,aileron,"
          "gear_cmd,gear_pos,"
          "thr0,thr1,thr2,thr3,"
          "gear0_wow,gear0_comp_ft,gear0_comp_vel_fps,gear0_comp_force_lbs,"
          "gear0_fbx_lbs,gear0_fby_lbs,gear0_fbz_lbs,"
          "gear1_wow,gear1_comp_ft,gear1_comp_vel_fps,gear1_comp_force_lbs,"
          "gear1_fbx_lbs,gear1_fby_lbs,gear1_fbz_lbs,"
          "gear2_wow,gear2_comp_ft,gear2_comp_vel_fps,gear2_comp_force_lbs,"
          "gear2_fbx_lbs,gear2_fby_lbs,gear2_fbz_lbs,"
          "grx_lbs,gry_lbs,grz_lbs,"
          "fbx_prop_lbs,engine0_thrust_lbs,engine1_thrust_lbs,engine0_rpm,engine1_rpm,"
          "fbx_gear_lbs,fby_gear_lbs,fbz_gear_lbs,"
          "fm_state\n");
}

void WriteRow(FILE* out, double t, FlightManager& fm) {
  auto* pm = fm.GetAdapter().GetFdmExec().GetPropertyManager().get();
  auto get = [&](const char* n) {
    auto* node = pm->GetNode(n);
    return node ? node->getDoubleValue() : -1.0;
  };
  auto ground = fm.GetAdapter().GetFdmExec().GetGroundReactions();
  auto gear_value = [&](int index, const char* field) {
    if (!ground || index >= static_cast<int>(ground->GetNumGearUnits())) {
      return -1.0;
    }
    auto gear = ground->GetGearUnit(index);
    if (!gear) {
      return -1.0;
    }
    std::string key(field);
    if (key == "wow") return gear->GetWOW() ? 1.0 : 0.0;
    if (key == "comp") return gear->GetCompLen();
    if (key == "vel") return gear->GetCompVel();
    if (key == "force") return gear->GetCompForce();
    if (key == "fbx") return gear->GetBodyForces()(1);
    if (key == "fby") return gear->GetBodyForces()(2);
    if (key == "fbz") return gear->GetBodyForces()(3);
    return -1.0;
  };
  auto ground_force = [&](int index) {
    if (!ground) {
      return -1.0;
    }
    return ground->GetForces(index);
  };
  fprintf(out,
          "%.2f,%.2f,%.1f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,"
          "%.2f,%.2f,"
          "%.2f,%.2f,%.2f,%.2f,"
          "%.1f,%.4f,%.3f,%.1f,"
          "%.1f,%.1f,%.1f,"
          "%.1f,%.4f,%.3f,%.1f,"
          "%.1f,%.1f,%.1f,"
          "%.1f,%.4f,%.3f,%.1f,"
          "%.1f,%.1f,%.1f,"
          "%.1f,%.1f,%.1f,"
          "%.1f,%.1f,%.1f,%.1f,%.1f,"
          "%.1f,%.1f,%.1f,"
          "%d\n",
          t, get("position/h-agl-ft") * 0.3048, get("velocities/vc-kts"),
          get("attitude/pitch-rad") * 57.2958, get("attitude/roll-rad") * 57.2958,
          get("fcs/throttle-cmd-norm"), get("fcs/elevator-cmd-norm"), get("gear/unit[1]/WOW"),
          get("fcs/aileron-cmd-norm"),
          get("gear/gear-cmd-norm"), get("gear/gear-pos-norm"),
          get("fcs/throttle-cmd-norm[0]"), get("fcs/throttle-cmd-norm[1]"),
          get("fcs/throttle-cmd-norm[2]"), get("fcs/throttle-cmd-norm[3]"), gear_value(0, "wow"),
          gear_value(0, "comp"), gear_value(0, "vel"), gear_value(0, "force"), gear_value(0, "fbx"),
          gear_value(0, "fby"), gear_value(0, "fbz"), gear_value(1, "wow"), gear_value(1, "comp"),
          gear_value(1, "vel"), gear_value(1, "force"), gear_value(1, "fbx"), gear_value(1, "fby"),
          gear_value(1, "fbz"), gear_value(2, "wow"), gear_value(2, "comp"), gear_value(2, "vel"),
          gear_value(2, "force"), gear_value(2, "fbx"), gear_value(2, "fby"), gear_value(2, "fbz"),
          ground_force(1), ground_force(2), ground_force(3), get("forces/fbx-prop-lbs"),
          get("propulsion/engine[0]/thrust-lbs"), get("propulsion/engine[1]/thrust-lbs"),
          get("propulsion/engine[0]/propeller-rpm"), get("propulsion/engine[1]/propeller-rpm"),
          get("forces/fbx-gear-lbs"), get("forces/fby-gear-lbs"), get("forces/fbz-gear-lbs"),
          static_cast<int>(fm.GetState()));
}

}  // namespace

// ─── Maneuver sequence builders ────────────────────────────────────────────

void BuildWaypointSequence(FlightManager& fm, const ScenarioConfig& sc) {
  ManeuverCommand tko;
  tko.type = guidance::ManeuverType::kTakeoff;
  tko.target.altitude_m = sc.cruise_altitude_m;
  fm.PushManeuver(tko);

  double wp_offset_rad =
      sc.waypoint_distance_m * 0.70710678 / 6378137.0;
  double wp_radius = std::max(200.0, fm.GetAutopilot().GetControlProfile().ref_speed_mps * 20.0);

  // Three waypoints along the same 45° diagonal.
  struct { double fraction; double radius_scale; } wp_seq[] = {
    {1.0/3.0, 0.35},
    {2.0/3.0, 0.55},
    {1.0,     1.0},
  };
  for (const auto& w : wp_seq) {
    ManeuverCommand fly;
    fly.type = guidance::ManeuverType::kFlyToWaypoint;
    fly.target.latitude_rad = wp_offset_rad * w.fraction;
    fly.target.longitude_rad = wp_offset_rad * w.fraction;
    fly.target.altitude_m = sc.cruise_altitude_m;
    fly.target.radius_m = std::max(200.0, wp_radius * w.radius_scale);
    fm.PushManeuver(fly);
  }

  ManeuverCommand land;
  land.type = guidance::ManeuverType::kLand;
  land.target.latitude_rad = sc.landing_lat_rad;
  land.target.longitude_rad = sc.landing_lon_rad;
  land.target.altitude_m = 0.0;
  land.value = 0.0;
  fm.PushManeuver(land);
}

void BuildHeadingAltSequence(FlightManager& fm, const ScenarioConfig& sc) {
  ManeuverCommand tko;
  tko.type = guidance::ManeuverType::kTakeoff;
  tko.target.altitude_m = sc.cruise_altitude_m;
  fm.PushManeuver(tko);

  // Three SetHeading → SetAltitude pairs that exercise convergence on
  // discrete heading targets (2° tolerance) and altitude targets (10m).
  // The land phase computes its own heading to the landing point, so
  // the exact final heading is not critical.
  // Altitude deltas are kept modest (+100m) to avoid stall near ceiling.
  struct { double heading_rad; double alt_delta_m; } ha_seq[] = {
    {0.523599,  200.0},   //  30° right (toward NE-ish), climb 200m
    {1.047197,  0.0},     //  60° right, return to cruise altitude
    {0.523599,  200.0},   //  30° right again, climb 200m
  };
  for (const auto& h : ha_seq) {
    ManeuverCommand hdg;
    hdg.type = guidance::ManeuverType::kSetHeading;
    hdg.value = h.heading_rad;
    hdg.heading_tolerance_rad = sc.heading_tolerance_rad;
    fm.PushManeuver(hdg);

    ManeuverCommand alt;
    alt.type = guidance::ManeuverType::kSetAltitude;
    alt.value = sc.cruise_altitude_m + h.alt_delta_m;
    alt.altitude_tolerance_m = sc.altitude_tolerance_m;
    fm.PushManeuver(alt);
  }

  ManeuverCommand land;
  land.type = guidance::ManeuverType::kLand;
  land.target.latitude_rad = sc.landing_lat_rad;
  land.target.longitude_rad = sc.landing_lon_rad;
  land.target.altitude_m = 0.0;
  land.value = 0.0;
  fm.PushManeuver(land);
}

void BuildPitchTestSequence(FlightManager& fm, const ScenarioConfig& sc) {
  ManeuverCommand tko;
  tko.type = guidance::ManeuverType::kTakeoff;
  tko.target.altitude_m = sc.cruise_altitude_m;
  fm.PushManeuver(tko);

  // SetPitch sweep: exercise the pitch PD controller at multiple targets.
  // Each pitch command runs for a fixed duration; completion is time-based.
  // With speed protection (L1: speed_hold, L2: pitch relief), the aircraft
  // will reach its physical pitch limit — the equilibrium between thrust
  // and drag at the commanded pitch angle.  Durations are long enough for
  // the aircraft to reach steady-state at each target.
  struct { double pitch_deg; double duration_sec; } pitch_seq[] = {
      {5.0, 15.0},    // mild nose-up (baseline, should always succeed)
      {-5.0, 15.0},   // mild nose-down (speed increases, safe)
      {15.0, 20.0},   // aggressive — tests physical limit for most types
      {25.0, 20.0},   // extreme — only high-TWR fighters may sustain this
      {0.0, 10.0},    // recover to level
  };
  for (const auto& p : pitch_seq) {
    ManeuverCommand cmd;
    cmd.type = guidance::ManeuverType::kSetPitch;
    cmd.value = p.pitch_deg;
    cmd.duration_sec = p.duration_sec;
    fm.PushManeuver(cmd);
  }

  ManeuverCommand land;
  land.type = guidance::ManeuverType::kLand;
  land.target.latitude_rad = sc.landing_lat_rad;
  land.target.longitude_rad = sc.landing_lon_rad;
  land.target.altitude_m = 0.0;
  land.value = 0.0;
  fm.PushManeuver(land);
}

// ─── Run loop ──────────────────────────────────────────────────────────────

void RunSimulation(FlightManager& fm, const ScenarioConfig& sc, FILE* out,
                   const std::string& model, const char* out_path) {
  WriteHeader(out);
  double t = 0.0;
  bool reached_cruise_alt = false;
  bool timeout_logged = false;

  // Extended budget: if the normal step budget runs out, keep running up to
  // 3× the original limit so the aircraft can complete its full cycle.
  const int extended_max_steps = sc.max_steps * 3;

  for (int i = 0; i < extended_max_steps; ++i) {
    fm.Step(kDt);
    t += kDt;

    WriteRow(out, t, fm);

    // ── Cruise-altitude reached detection ──
    if (!reached_cruise_alt) {
      auto* pm = fm.GetAdapter().GetFdmExec().GetPropertyManager().get();
      auto* node = pm->GetNode("position/h-agl-ft");
      double agl_m = node ? node->getDoubleValue() * 0.3048 : 0.0;
      if (agl_m >= sc.cruise_altitude_m - sc.altitude_tolerance_m) {
        reached_cruise_alt = true;
        fprintf(stderr, "%s: ★ reached cruise altitude %.0f m at t=%.1fs\n",
                model.c_str(), sc.cruise_altitude_m, t);
      }
    }

    if (fm.GetState() == FlightManagerState::kCompleted ||
        fm.GetState() == FlightManagerState::kAborted) {
      if (fm.GetState() == FlightManagerState::kCompleted) {
        fprintf(stderr, "%s: completed at t=%.1fs\n", model.c_str(), t);
      } else {
        fprintf(stderr, "%s: aborted at t=%.1fs\n", model.c_str(), t);
      }
      return;
    }

    // Log timeout once but keep going.
    if (!timeout_logged && (i + 1) >= sc.max_steps) {
      timeout_logged = true;
      fprintf(stderr, "%s: timeout at t=%.1fs, continuing to completion...\n",
              model.c_str(), t);
    }
  }
  fprintf(stderr, "%s: extended timeout at t=%.1fs\n", model.c_str(), t);
}

// ─── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: takeoff_land_csv [--heading-alt|--pitch-test] <aircraft_model> [output.csv]\n";
    return 1;
  }

  bool heading_alt_mode = false;
  bool pitch_test_mode = false;
  int arg_i = 1;
  if (argc > 1) {
    std::string flag(argv[1]);
    if (flag == "--heading-alt") {
      heading_alt_mode = true;
      arg_i = 2;
    } else if (flag == "--pitch-test") {
      pitch_test_mode = true;
      arg_i = 2;
    }
  }
  if (arg_i >= argc) {
    std::cerr << "Usage: takeoff_land_csv [--heading-alt|--pitch-test] <aircraft_model> [output.csv]\n";
    return 1;
  }

  std::string model(argv[arg_i]);
  const char* out_path = (argc > arg_i + 1) ? argv[arg_i + 1] : nullptr;
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
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 0.0;
  cfg.initial_kinematics.velocity_mps.x_mps = 0.0;

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::cerr << model << ": init failed\n";
    if (out_path) fclose(out);
    return 1;
  }

  ScenarioConfig sc = MakeScenario(fm);

  auto skip = CheckSkip(fm, sc);
  if (skip.should_skip) {
    fprintf(stderr, "%s: skipped (%s)\n", model.c_str(), skip.reason);
    if (out_path) fclose(out);
    return 0;
  }

  if (heading_alt_mode) {
    BuildHeadingAltSequence(fm, sc);
  } else if (pitch_test_mode) {
    BuildPitchTestSequence(fm, sc);
  } else {
    BuildWaypointSequence(fm, sc);
  }

  RunSimulation(fm, sc, out, model, out_path);

  if (out_path) fclose(out);
  return 0;
}
