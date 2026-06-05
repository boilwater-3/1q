/// @file
/// @brief 长时间盘旋稳定性测试 — 运行 N 圈，测量半径/高度/速度漂移
///
/// 使用方位角（bearing from center）累加检测完整绕圈，而非航向累加。
/// 方位角累加 2π = 飞行器绕中心完成一整圈。
///
/// 用法：
///   orbit_long_duration <model> <radius_m> <num_orbits> [speed_mps [output.csv]]
///
/// 输出 CSV：每圈一行，包含平均/最小/最大距离、高度、滚转、速度
///
/// 示例：
///   ./orbit_long_duration f16 5000 10 /tmp/f16_10orbits.csv
///   ./orbit_long_duration c172p 800 5 50 /tmp/c172p_5orbits.csv

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

namespace {

constexpr double kDt = 0.01;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = M_PI / 180.0;

double DistanceToCenterM(double lat, double lon, const guidance::Waypoint& c) {
  double cos_lat = std::cos(c.latitude_rad);
  double dn = (lat - c.latitude_rad) * kEarthRadiusM;
  double de = (lon - c.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::hypot(dn, de);
}

double BearingFromCenterRad(double lat, double lon, const guidance::Waypoint& c) {
  double cos_lat = std::cos(c.latitude_rad);
  double dn = (lat - c.latitude_rad) * kEarthRadiusM;
  double de = (lon - c.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::atan2(de, dn);
}

double NormalizeRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double MinTurnRadiusM(double v, double bank_deg) {
  double tb = std::tan(bank_deg * kDegToRad);
  if (tb < 0.01) tb = 0.01;
  return (v * v) / (9.80665 * tb);
}

guidance::Waypoint CenterSouth(double r, double alt) {
  guidance::Waypoint c;
  c.latitude_rad = -r / kEarthRadiusM;
  c.longitude_rad = 0.0;
  c.altitude_m = alt;
  return c;
}

double OrbitPeriodSec(double r, double v) {
  if (v < 1.0) v = 1.0;
  return 2.0 * M_PI * r / v;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: orbit_long_duration <model> <radius_m> <num_orbits>"
                 " [speed_mps [output.csv]]\n";
    return 1;
  }

  std::string model(argv[1]);
  double radius_m = std::atof(argv[2]);
  int num_orbits = std::atoi(argv[3]);
  double speed_mps = (argc >= 5) ? std::atof(argv[4]) : 200.0;
  const char* out_path = (argc >= 6) ? argv[5] : nullptr;

  if (radius_m < 1.0 || num_orbits < 1 || speed_mps < 1.0) return 1;

  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = true;
  cfg.silent_mode = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 3000.0;
  cfg.initial_kinematics.velocity_mps.x_mps = speed_mps;
  cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {0.0, 0.0, 0.0};

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::cerr << "Failed init " << model << "\n"; return 1;
  }

  double max_bank = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double feasible_r = std::max(radius_m, MinTurnRadiusM(speed_mps, max_bank) * 1.2);
  if (feasible_r != radius_m)
    std::fprintf(stderr, "  radius %.0f -> feasible %.0f (min_turn %.0f, bank %.0f)\n",
                 radius_m, feasible_r, MinTurnRadiusM(speed_mps, max_bank), max_bank);

  auto center = CenterSouth(feasible_r, 3000.0);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target = center;
  cmd.value = feasible_r;
  fm.PushManeuver(cmd);

  double period = OrbitPeriodSec(feasible_r, speed_mps);
  int settle = std::min(static_cast<int>(period * 1.0 / kDt), 30000);
  std::fprintf(stderr, "[%s] r=%.0f v=%.0f T=%.1fs settle=%d=%.1fs\n",
               model.c_str(), feasible_r, speed_mps, period, settle, settle * kDt);
  for (int i = 0; i < settle; ++i) {
    if (!fm.Step(kDt) || fm.GetVehicleState().altitude_geod_m <= 0.0)
      { std::cerr << "CRASHED during settle\n"; return 1; }
  }

  FILE* out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }
  std::fprintf(out,
      "orbit,start_s,end_s,"
      "avg_dist_m,min_dist_m,max_dist_m,dist_start,dist_end,dist_drift,"
      "avg_alt_m,min_alt_m,max_alt_m,alt_drift,"
      "avg_roll_deg,avg_speed_mps,bearing_accum_rad\n");

  int orbits = 0;
  double bearing_accum = 0.0;
  double prev_bearing = BearingFromCenterRad(
      fm.GetVehicleState().latitude_rad, fm.GetVehicleState().longitude_rad, center);

  struct { double start; double dist_start; double min_d=1e9, max_d=0, sum_d=0, min_a=1e9, max_a=0, sum_a=0, sum_r=0, sum_s=0; int n=0; } cur;
  cur.start = fm.GetVehicleState().sim_time_sec;
  cur.dist_start = DistanceToCenterM(
      fm.GetVehicleState().latitude_rad, fm.GetVehicleState().longitude_rad, center);

  double orbit0_dist = cur.dist_start;
  int max_steps = num_orbits * static_cast<int>(period / kDt) * 5;
  if (max_steps < 50000) max_steps = 50000;
  bool crashed = false;

  for (int step = 0; step < max_steps && orbits < num_orbits; ++step) {
    if (!fm.Step(kDt)) { crashed = true; break; }
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { crashed = true; break; }

    double bearing = BearingFromCenterRad(s.latitude_rad, s.longitude_rad, center);
    bearing_accum += NormalizeRad(bearing - prev_bearing);
    prev_bearing = bearing;

    double dist = DistanceToCenterM(s.latitude_rad, s.longitude_rad, center);
    cur.min_d = std::min(cur.min_d, dist);
    cur.max_d = std::max(cur.max_d, dist);
    cur.sum_d += dist;
    cur.min_a = std::min(cur.min_a, s.altitude_geod_m);
    cur.max_a = std::max(cur.max_a, s.altitude_geod_m);
    cur.sum_a += s.altitude_geod_m;
    cur.sum_r += std::abs(s.phi_rad) * 180.0 / M_PI;
    cur.sum_s += s.vtrue_mps;
    cur.n++;

    // 方位角累积 2π = 一圈
    if (std::abs(bearing_accum) >= 2.0 * M_PI - 0.1) {
      ++orbits;
      double avg_d = cur.sum_d / cur.n;
      double avg_a = cur.sum_a / cur.n;
      double avg_r = cur.sum_r / cur.n;
      double avg_s = cur.sum_s / cur.n;
      double end_t = s.sim_time_sec;

      std::fprintf(out,
          "%d,%.3f,%.3f,"
          "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
          "%.1f,%.1f,%.1f,%.1f,"
          "%.2f,%.2f,%.4f\n",
          orbits, cur.start, end_t,
          avg_d, cur.min_d, cur.max_d, cur.dist_start, dist, dist - cur.dist_start,
          avg_a, cur.min_a, cur.max_a, cur.max_a - cur.min_a,
          avg_r, avg_s, bearing_accum);

      double pct = static_cast<double>(orbits) / num_orbits * 100.0;
      if (pct >= 10.0 * (static_cast<int>(pct) / 10) || orbits == num_orbits)
        std::fprintf(stderr, "  orbit %3d/%d  dist=%.0f(+-%.0f) drift=%+.0f alt=%.0f roll=%.1f\n",
                     orbits, num_orbits, avg_d, cur.max_d - cur.min_d,
                     dist - orbit0_dist, avg_a, avg_r);

      cur = {};
      cur.start = end_t;
      cur.dist_start = dist;
      cur.min_d = 1e9; cur.min_a = 1e9;
      bearing_accum = 0.0;
    }
  }

  if (out_path) std::fclose(out);

  std::fprintf(stderr, "\n=== Long Duration: %s r=%.0f %d/%d orbits ===\n",
               model.c_str(), feasible_r, orbits, num_orbits);
  if (crashed) { std::fprintf(stderr, "CRASHED\n"); return 1; }
  if (orbits < 1) {
    double d = DistanceToCenterM(fm.GetVehicleState().latitude_rad,
                                  fm.GetVehicleState().longitude_rad, center);
    std::fprintf(stderr, "No complete orbits (accum=%.2f rad, dist=%.0f)\n",
                 bearing_accum, d);
    return 1;
  }
  std::fprintf(stderr, "OK\n");
  return 0;
}
