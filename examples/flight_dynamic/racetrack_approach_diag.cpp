/// @file
/// @brief 跑道巡逻进场诊断 — 测试不同方向/位置进入 racetrack 的轨迹质量
///
/// 用法：
///   racetrack_approach_diag [model]
///
/// 测试 8 种进场场景，对比理想 racetrack 路径的误差。

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

double DistanceToRacetrack(double x_prime, double y_prime, double r, double leg_len) {
  if (y_prime >= 0.0 && y_prime <= leg_len) {
    return std::min(std::abs(x_prime - 0.0), std::abs(x_prime - 2.0 * r));
  } else if (y_prime > leg_len) {
    double dist_to_center = std::hypot(x_prime - r, y_prime - leg_len);
    return std::abs(dist_to_center - r);
  } else {
    double dist_to_center = std::hypot(x_prime - r, y_prime - 0.0);
    return std::abs(dist_to_center - r);
  }
}

double FeasibleTurnRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
  double tan_b = std::tan(max_bank_deg * kDegToRad);
  if (tan_b < 0.01) tan_b = 0.01;
  double min_r = (speed_mps * speed_mps) / (9.80665 * tan_b);
  return std::max(nominal_r, min_r * 1.2);
}

/// Approach scenario: position offset and initial heading.
struct ApproachScenario {
  const char* name;
  double offset_north_m;  // offset from racetrack start (north)
  double offset_east_m;   // offset from racetrack start (east)
  double heading_deg;      // initial heading (deg, 0=N, 90=E)
  const char* description;
};

const ApproachScenario kScenarios[] = {
    {"aligned",      0.0,      0.0,     0.0,   "理想对齐: 起点, 朝北"},
    {"behind",       -5000.0,  0.0,     0.0,   "后方5km: 朝北飞向起点"},
    {"left_side",    0.0,      -5000.0, 0.0,   "左方5km: 朝北, 需右偏"},
    {"right_side",   0.0,      5000.0,  0.0,   "右方5km: 朝北, 需左偏"},
    {"far_behind",   -10000.0, 0.0,     0.0,   "后方10km: 远距离正面"},
    {"far_right",    0.0,      10000.0, 0.0,   "右方10km: 远距离侧方"},
    {"diagonal_se",  -5000.0,  5000.0,  0.0,   "东南5km: 朝北斜向"},
    {"diagonal_nw",  5000.0,   -5000.0, 0.0,   "西北5km: 朝北斜向"},
    {"opposite",     0.0,      0.0,     180.0, "同位置反向: 需180°转弯"},
    {"far_east_270", 0.0,      15000.0, 270.0, "东15km朝西: 远距离侧方"},
};

Waypoint LocalToWaypoint(double ref_lat_rad, double ref_lon_rad,
                          double north_m, double east_m, double altitude_m) {
  double cos_lat = std::cos(ref_lat_rad);
  Waypoint wp;
  wp.latitude_rad = ref_lat_rad + north_m / kEarthRadiusM;
  wp.longitude_rad = ref_lon_rad + east_m / (kEarthRadiusM * cos_lat);
  wp.altitude_m = altitude_m;
  return wp;
}

struct ScenarioResult {
  const char* name;
  bool crashed;
  double max_err_m;
  double avg_err_m;         // overall average error
  double lap_avg_err_m;     // average error after first lap converges
  double converge_time_sec; // time to first get within 2× radius
  double first_lap_err_m;   // average error during first lap
  int laps_completed;
};

ScenarioResult RunScenario(const std::string& model, double altitude_m,
                           double init_spd, double radius_m, double duration_sec,
                           const ApproachScenario& scenario) {
  ScenarioResult result = {};
  result.name = scenario.name;

  // Racetrack start at origin, heading North.
  double heading_rad = 0.0;
  double leg_length_m = 10000.0;
  int num_laps = 5;

  // Aircraft start position: offset from racetrack start.
  Waypoint ac_pos = LocalToWaypoint(0.0, 0.0,
                                     scenario.offset_north_m,
                                     scenario.offset_east_m,
                                     altitude_m);

  // Always start heading North to ensure trim succeeds.
  // After a warm-up period, turn to the scenario heading.
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
    result.crashed = true;
    return result;
  }

  const auto& prof = fm.GetAutopilot().GetControlProfile();
  double actual_speed = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : init_spd;
  double max_bank = prof.max_roll_angle_deg;
  double feasible_r = FeasibleTurnRadiusM(actual_speed, max_bank, radius_m);

  fm.GetAutopilot().SetSpeedTargetMps(actual_speed);

  // Phase 1: warm-up (2s) + turn to scenario heading (30s).
  // This ensures the aircraft is flying stably in the scenario heading
  // before the racetrack command is issued.
  double target_heading_rad = scenario.heading_deg * kDegToRad;
  if (std::abs(target_heading_rad) > 0.01) {
    ManeuverCommand hdg_cmd;
    hdg_cmd.type = guidance::ManeuverType::kSetHeading;
    hdg_cmd.value = target_heading_rad;
    hdg_cmd.heading_tolerance_rad = 0.05;  // ~3°
    fm.PushManeuver(hdg_cmd);
  }

  // Warm-up + heading settle: 200 steps (2s) + up to 3000 steps (30s) for heading.
  int warmup_steps = 200;
  int heading_settle_max = 3000;
  for (int i = 0; i < warmup_steps; ++i) {
    if (!fm.Step(kDt)) { result.crashed = true; return result; }
  }
  // Wait for heading to settle (or timeout).
  for (int i = 0; i < heading_settle_max; ++i) {
    if (!fm.Step(kDt)) { result.crashed = true; return result; }
  }

  // Phase 2: issue racetrack command from current position/heading.
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

  double cos_h = std::cos(heading_rad);
  double sin_h = std::sin(heading_rad);

  int max_steps = static_cast<int>(duration_sec / kDt);
  double err_sum = 0.0;
  double max_err = 0.0;
  int err_count = 0;
  bool converged = false;
  double converge_time = duration_sec;

  // Track per-lap errors (estimate lap from y_prime cycling)
  double lap_err_sum = 0.0;
  int lap_err_count = 0;
  int prev_y_sign = 0;
  int laps = 0;

  for (int step = 0; step < max_steps; ++step) {
    bool running = fm.Step(kDt);
    if (!running) { result.crashed = true; break; }
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { result.crashed = true; break; }

    double d_lat = s.latitude_rad - start.latitude_rad;
    double d_lon = s.longitude_rad - start.longitude_rad;
    double north = d_lat * kEarthRadiusM;
    double east = d_lon * kEarthRadiusM * std::cos(start.latitude_rad);

    double x_prime = east * cos_h - north * sin_h;
    double y_prime = east * sin_h + north * cos_h;

    double err = DistanceToRacetrack(x_prime, y_prime, feasible_r, leg_length_m);

    err_sum += err;
    err_count++;
    if (err > max_err) max_err = err;

    // Convergence: first time error drops below 2× radius
    if (!converged && err < feasible_r * 2.0) {
      converged = true;
      converge_time = step * kDt;
    }

    // Lap detection: y_prime crosses zero from negative to positive = new lap
    int y_sign = (y_prime >= 0.0) ? 1 : -1;
    if (prev_y_sign < 0 && y_sign > 0 && step > 100) {
      laps++;
    }
    prev_y_sign = y_sign;

    // Accumulate error only after convergence (lap quality)
    if (converged) {
      lap_err_sum += err;
      lap_err_count++;
    }
  }

  result.max_err_m = max_err;
  result.avg_err_m = err_count > 0 ? err_sum / err_count : 0.0;
  result.lap_avg_err_m = lap_err_count > 0 ? lap_err_sum / lap_err_count : result.avg_err_m;
  result.converge_time_sec = converge_time;
  result.first_lap_err_m = result.avg_err_m;  // simplified
  result.laps_completed = laps;

  return result;
}

}  // namespace

int main(int argc, char** argv) {
  std::string model = (argc >= 2) ? argv[1] : "c172p";

  // Find spec
  double alt_m = 500.0;
  double spd = 50.0;
  double nominal_r = 1000.0;

  struct Spec { const char* m; double a; double s; double r; };
  Spec specs[] = {
      {"f16", 3000.0, 200.0, 12000.0}, {"f15", 3000.0, 200.0, 6000.0},
      {"A4", 2000.0, 120.0, 2500.0},   {"F4N", 2000.0, 130.0, 3000.0},
      {"F80C", 2000.0, 120.0, 2500.0}, {"OV10", 500.0, 70.0, 1000.0},
      {"737", 3000.0, 130.0, 4000.0},  {"B747", 3000.0, 140.0, 5000.0},
      {"c172p", 500.0, 50.0, 1000.0},  {"c172r", 500.0, 50.0, 1000.0},
      {"c182", 1000.0, 55.0, 800.0},   {"c310", 500.0, 65.0, 1000.0},
      {"C130", 5000.0, 90.0, 2500.0},  {"DHC6", 500.0, 55.0, 800.0},
  };
  for (const auto& s : specs) {
    if (model == s.m) { alt_m = s.a; spd = s.s; nominal_r = s.r; break; }
  }

  double duration_sec = 600.0;

  std::fprintf(stderr, "\n══════════════════════════════════════════════════════════\n");
  std::fprintf(stderr, "  Racetrack Approach Diagnosis: %s\n", model.c_str());
  std::fprintf(stderr, "  alt=%.0fm  speed=%.0fm/s  nominal_r=%.0fm  leg=10000m\n",
               alt_m, spd, nominal_r);
  std::fprintf(stderr, "  duration=%.0fs (≈%.0f laps ideal)\n", duration_sec,
               duration_sec * spd / (2.0 * 10000.0 + 2.0 * M_PI * nominal_r));
  std::fprintf(stderr, "══════════════════════════════════════════════════════════\n\n");

  // Output CSV header
  std::printf("scenario,description,crashed,max_err_m,avg_err_m,lap_avg_err_m,"
              "converge_sec,laps,ratio\n");

  for (const auto& sc : kScenarios) {
    std::fprintf(stderr, "  [%-12s] %s ...\n", sc.name, sc.description);
    auto r = RunScenario(model, alt_m, spd, nominal_r, duration_sec, sc);

    double ratio = nominal_r > 0.0 ? r.lap_avg_err_m / nominal_r : 0.0;
    const char* verdict = "GOOD";
    if (r.crashed) verdict = "CRASHED";
    else if (ratio > 1.0) verdict = "BAD";
    else if (ratio > 0.5) verdict = "WARN";

    std::printf("%s,\"%s\",%s,%.1f,%.1f,%.1f,%.1f,%d,%.3f\n",
                sc.name, sc.description,
                r.crashed ? "YES" : "no",
                r.max_err_m, r.avg_err_m, r.lap_avg_err_m,
                r.converge_time_sec, r.laps_completed, ratio);

    std::fprintf(stderr, "    → %s  avg_err=%.0fm  lap_avg=%.0fm  ratio=%.3f  "
                "converge=%.1fs  laps=%d\n",
                verdict, r.avg_err_m, r.lap_avg_err_m, ratio,
                r.converge_time_sec, r.laps_completed);
  }

  std::fprintf(stderr, "\n═══ Done ═══\n");
  return 0;
}
