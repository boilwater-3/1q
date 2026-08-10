/// @file
/// @brief Figure-8 进场航迹导出 — 为每个进场场景生成轨迹 CSV
///
/// 用法：
///   figure8_approach_trace [model] [output_dir]
///
/// 输出每个场景一个 CSV 文件，包含相对圆心的 (north_m, east_m) 坐标。
/// 用于验证 Figure-8 从任意位置/方向进场时的轨迹质量。

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

double FeasibleRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
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

// ── 进场场景定义 ──
// 圆心在 (0, 0)。飞机初始位置相对圆心偏移。
// 所有场景初始航向朝北（0°），通过 DoTrim 初始化。
// "反向" 场景使用 SetHeading 机动转向南。

struct ApproachScenario {
  const char* name;
  double offset_north_m;
  double offset_east_m;
  double heading_deg;       // 0 = use SetHeading for non-North
  const char* description;
};

const ApproachScenario kScenarios[] = {
    {"aligned",       0.0,      0.0,     0.0,    "at center (baseline)"},
    {"on_circle_s",   0.0,      0.0,     0.0,    "on circle south (tangent)"},
    {"behind",       -5000.0,   0.0,     0.0,    "5km south of center"},
    {"left_side",     0.0,     -5000.0,  0.0,    "5km west of center"},
    {"right_side",    0.0,      5000.0,  0.0,    "5km east of center"},
    {"far_behind",   -10000.0,  0.0,     0.0,    "10km south of center"},
    {"far_right",     0.0,      10000.0, 0.0,    "10km east of center"},
    {"diagonal_se",  -5000.0,   5000.0,  0.0,    "5km SE diagonal"},
    {"reverse_180",   0.0,      0.0,     180.0,  "at center, heading south"},
};

int RunTrace(const std::string& model, double altitude_m,
             double init_spd, double nominal_r, double duration_sec,
             const ApproachScenario& scenario, FILE* out) {
  int num_cycles = 3;

  // Figure-8 center at origin (0, 0)
  Waypoint center;
  center.latitude_rad = 0.0;
  center.longitude_rad = 0.0;
  center.altitude_m = altitude_m;

  // Aircraft starting position = center + offset
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
  double feasible_r = FeasibleRadiusM(actual_speed, max_bank, nominal_r);

  // For "on_circle_s": start exactly on the circle south of center
  // heading north = CW tangent direction
  if (std::string(scenario.name) == "on_circle_s") {
    ac_pos = LocalToWaypoint(0.0, 0.0, -feasible_r, 0.0, altitude_m);
    // Re-init is not needed since we just need the center offset.
    // The aircraft is already at (0,0) from the aligned config.
    // Instead, just set the center SOUTH of the aircraft by radius.
    center.latitude_rad = -feasible_r / kEarthRadiusM;
  }

  // Push Figure-8 maneuver
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kFigure8;
  cmd.target = center;
  cmd.value = feasible_r;                // radius
  cmd.duration_sec = 0.0;                // axis heading (0 = North)
  cmd.heading_tolerance_rad = static_cast<double>(num_cycles);
  cmd.altitude_tolerance_m = static_cast<double>(num_cycles);

  // Non-North heading: push SetHeading BEFORE Figure-8.
  // FlightManager executes maneuvers from the queue in order,
  // so SetHeading completes first, then Figure-8 starts automatically.
  // This avoids the kCompleted state issue where Step() returns false
  // after the last queued maneuver finishes.
  if (scenario.heading_deg != 0.0) {
    // Warm-up heading North first (stabilize trim)
    for (int i = 0; i < 200; ++i) {
      if (!fm.Step(kDt)) return 1;
    }
    ManeuverCommand hdg_cmd;
    hdg_cmd.type = guidance::ManeuverType::kSetHeading;
    hdg_cmd.value = scenario.heading_deg * kDegToRad;
    hdg_cmd.heading_tolerance_rad = 0.1;  // ~6° tolerance for quick turn
    fm.PushManeuver(hdg_cmd);
  }

  fm.GetAutopilot().SetSpeedTargetMps(actual_speed);
  fm.PushManeuver(cmd);

  std::fprintf(out,
      "# scenario=%s  offset=(%.0fN,%.0fE)m  r=%.0fm  cycles=%d\n"
      "time_sec,north_m,east_m,heading_rad,roll_deg,dist_to_center_m,bearing_rad\n",
      scenario.name, scenario.offset_north_m, scenario.offset_east_m,
      feasible_r, num_cycles);

  int max_steps = static_cast<int>(duration_sec / kDt);
  double prev_bearing = 0.0;
  double bearing_accum = 0.0;
  int phase = 0;  // 0=CW, 1=CCW
  int phase_switch_count = 0;

  for (int step = 0; step < max_steps; ++step) {
    bool running = fm.Step(kDt);
    if (!running) break;
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) break;

    // Sample every 10 steps (0.1s)
    if (step % 10 != 0) continue;

    double cos_lat = std::cos(center.latitude_rad);
    double dn = (s.latitude_rad - center.latitude_rad) * kEarthRadiusM;
    double de = (s.longitude_rad - center.longitude_rad) * kEarthRadiusM * cos_lat;
    double dist = std::hypot(dn, de);
    double bearing = std::atan2(de, dn);

    // Track bearing accumulation to detect CW/CCW phase switches
    double db = bearing - prev_bearing;
    while (db > M_PI) db -= 2.0 * M_PI;
    while (db < -M_PI) db += 2.0 * M_PI;
    bearing_accum += db;
    prev_bearing = bearing;

    // Detect phase switch (2π accumulated)
    if (std::abs(bearing_accum) >= 2.0 * M_PI - 0.1) {
      phase = 1 - phase;
      phase_switch_count++;
      bearing_accum = 0.0;
    }

    std::fprintf(out, "%.2f,%.1f,%.1f,%.4f,%.2f,%.1f,%.4f\n",
                 step * kDt, dn, de,
                 s.psi_rad, s.phi_rad * 180.0 / M_PI,
                 dist, bearing);
  }

  std::fprintf(stderr, "    phase_switches=%d  final_phase=%s\n",
               phase_switch_count, phase == 0 ? "CW" : "CCW");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string model = (argc >= 2) ? argv[1] : "c172p";
  std::string outdir = (argc >= 3) ? argv[2] : "/tmp/figure8_approach_";

  double alt_m = 500.0, spd = 50.0, nominal_r = 1000.0;
  struct Spec { const char* m; double a; double s; double r; };
  Spec specs[] = {
      {"f16", 3000.0, 200.0, 5000.0}, {"c172p", 500.0, 50.0, 1000.0},
      {"c182", 1000.0, 55.0, 800.0},  {"c310", 500.0, 65.0, 1000.0},
      {"737", 3000.0, 130.0, 4000.0}, {"DHC6", 500.0, 55.0, 800.0},
  };
  for (const auto& s : specs) {
    if (model == s.m) { alt_m = s.a; spd = s.s; nominal_r = s.r; break; }
  }

  double duration_sec = 1200.0;  // ~4-5 cycles for c172p

  std::fprintf(stderr, "\n═══ Figure-8 Approach Trace: %s  dur=%.0fs ═══\n",
               model.c_str(), duration_sec);

  for (const auto& sc : kScenarios) {
    std::string path = outdir + sc.name + ".csv";
    std::fprintf(stderr, "  [%-14s] %s → %s\n", sc.name, sc.description, path.c_str());
    FILE* out = std::fopen(path.c_str(), "w");
    if (!out) {
      std::fprintf(stderr, "    Cannot open %s\n", path.c_str());
      continue;
    }
    RunTrace(model, alt_m, spd, nominal_r, duration_sec, sc, out);
    std::fclose(out);
  }

  std::fprintf(stderr, "\n═══ Done ═══\n");
  return 0;
}
