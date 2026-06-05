/// @file
/// @brief 跑道巡逻进场航迹导出 — 为每个进场场景生成轨迹 CSV
///
/// 用法：
///   racetrack_approach_trace [model] [output_dir]
///
/// 输出每个场景一个 CSV 文件，包含 racetrack-frame 坐标 (x_prime, y_prime)。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"

using namespace oneq::flight_dynamic;
using oneq::flight_dynamic::guidance::Waypoint;

namespace {

constexpr double kDt = 0.01;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;

double FeasibleTurnRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
  double tan_b = std::tan(max_bank_deg * kDegToRad);
  if (tan_b < 0.01) tan_b = 0.01;
  double min_r = (speed_mps * speed_mps) / (9.80665 * tan_b);
  return std::max(nominal_r, min_r * 1.2);
}

Waypoint LocalToWaypoint(double ref_lat_rad, double ref_lon_rad,
                          double north_m, double east_m, double altitude_m) {
  double cos_lat = std::cos(ref_lat_rad);
  Waypoint wp;
  wp.latitude_rad = ref_lat_rad + north_m / kEarthRadiusM;
  wp.longitude_rad = ref_lon_rad + east_m / (kEarthRadiusM * cos_lat);
  wp.altitude_m = altitude_m;
  return wp;
}

struct ApproachScenario {
  const char* name;
  double offset_north_m;
  double offset_east_m;
  double heading_deg;
  const char* description;
};

const ApproachScenario kScenarios[] = {
    {"aligned",      0.0,      0.0,     0.0,   "ideal aligned"},
    {"behind",       -5000.0,  0.0,     0.0,   "5km behind"},
    {"left_side",    0.0,      -5000.0, 0.0,   "5km left"},
    {"right_side",   0.0,      5000.0,  0.0,   "5km right"},
    {"far_behind",   -10000.0, 0.0,     0.0,   "10km behind"},
    {"far_right",    0.0,      10000.0, 0.0,   "10km right"},
    {"diagonal_se",  -5000.0,  5000.0,  0.0,   "5km SE diagonal"},
    {"diagonal_nw",  5000.0,   -5000.0, 0.0,   "5km NW diagonal"},
};

int RunTrace(const std::string& model, double altitude_m,
             double init_spd, double radius_m, double duration_sec,
             const ApproachScenario& scenario, FILE* out) {
  double heading_rad = 0.0;
  double leg_length_m = 10000.0;
  int num_laps = 3;

  Waypoint ac_pos = LocalToWaypoint(0.0, 0.0,
                                     scenario.offset_north_m,
                                     scenario.offset_east_m,
                                     altitude_m);

  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = true;
  cfg.silent_mode = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = ac_pos.latitude_rad * 180.0 / M_PI;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = ac_pos.longitude_rad * 180.0 / M_PI;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = altitude_m;
  cfg.initial_kinematics.velocity_mps.x_mps = init_spd;
  cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {0.0, 0.0, 0.0};

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::fprintf(stderr, "  [%s] INIT FAILED\n", scenario.name);
    return 1;
  }

  const auto& prof = fm.GetAutopilot().GetControlProfile();
  double actual_speed = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : init_spd;
  double max_bank = prof.max_roll_angle_deg;
  double feasible_r = FeasibleTurnRadiusM(actual_speed, max_bank, radius_m);

  fm.GetAutopilot().SetSpeedTargetMps(actual_speed);

  guidance::Waypoint start;
  start.latitude_rad = 0.0;
  start.longitude_rad = 0.0;
  start.altitude_m = altitude_m;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kRacetrack;
  cmd.target = start;
  cmd.value = heading_rad;
  cmd.duration_sec = leg_length_m;
  cmd.heading_tolerance_rad = feasible_r;
  cmd.altitude_tolerance_m = static_cast<double>(num_laps);
  fm.PushManeuver(cmd);

  std::fprintf(out,
      "# scenario=%s  offset=(%.0fN,%.0fE)m  r=%.0fm  leg=%.0fm\n"
      "time_sec,x_prime_m,y_prime_m,heading_rad,roll_deg,err_m\n",
      scenario.name, scenario.offset_north_m, scenario.offset_east_m,
      feasible_r, leg_length_m);

  double cos_h = std::cos(heading_rad);
  double sin_h = std::sin(heading_rad);

  int max_steps = static_cast<int>(duration_sec / kDt);
  // Sample every 10 steps (0.1s) to keep file size manageable
  for (int step = 0; step < max_steps; ++step) {
    bool running = fm.Step(kDt);
    if (!running) break;
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) break;

    if (step % 10 != 0) continue;

    double d_lat = s.latitude_rad - start.latitude_rad;
    double d_lon = s.longitude_rad - start.longitude_rad;
    double north = d_lat * kEarthRadiusM;
    double east = d_lon * kEarthRadiusM * std::cos(start.latitude_rad);

    double x_prime = east * cos_h - north * sin_h;
    double y_prime = east * sin_h + north * cos_h;

    // Distance to racetrack
    double err = 0.0;
    double r = feasible_r;
    double L = leg_length_m;
    if (y_prime >= 0.0 && y_prime <= L) {
      err = std::min(std::abs(x_prime), std::abs(x_prime - 2.0 * r));
    } else if (y_prime > L) {
      err = std::abs(std::hypot(x_prime - r, y_prime - L) - r);
    } else {
      err = std::abs(std::hypot(x_prime - r, y_prime) - r);
    }

    std::fprintf(out, "%.2f,%.1f,%.1f,%.4f,%.2f,%.1f\n",
                 step * kDt, x_prime, y_prime,
                 s.psi_rad, s.phi_rad * 180.0 / M_PI, err);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string model = (argc >= 2) ? argv[1] : "c172p";
  std::string outdir = (argc >= 3) ? argv[2] : "/tmp/racetrack_approach_";

  double alt_m = 500.0, spd = 50.0, nominal_r = 1000.0;
  struct Spec { const char* m; double a; double s; double r; };
  Spec specs[] = {
      {"f16", 3000.0, 200.0, 12000.0}, {"c172p", 500.0, 50.0, 1000.0},
      {"c182", 1000.0, 55.0, 800.0},   {"c310", 500.0, 65.0, 1000.0},
      {"737", 3000.0, 130.0, 4000.0},  {"DHC6", 500.0, 55.0, 800.0},
  };
  for (const auto& s : specs) {
    if (model == s.m) { alt_m = s.a; spd = s.s; nominal_r = s.r; break; }
  }

  double duration_sec = 800.0;

  std::fprintf(stderr, "\n═══ Racetrack Approach Trace: %s  dur=%.0fs ═══\n",
               model.c_str(), duration_sec);

  for (const auto& sc : kScenarios) {
    std::string path = outdir + sc.name + ".csv";
    std::fprintf(stderr, "  [%-12s] %s → %s\n", sc.name, sc.description, path.c_str());
    FILE* out = std::fopen(path.c_str(), "w");
    if (!out) { std::fprintf(stderr, "    Cannot open %s\n", path.c_str()); continue; }
    RunTrace(model, alt_m, spd, nominal_r, duration_sec, sc, out);
    std::fclose(out);
  }

  std::fprintf(stderr, "\n═══ Done ═══\n");
  return 0;
}
