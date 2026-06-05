/// @file
/// @brief 跑道盘旋逐点轨迹导出 — 每步记录位置、距离、误差
///
/// 输出 CSV 可用于 Matplotlib 绘图 / Google Earth KML 转换。
///
/// 用法：
///   racetrack_trace_csv <model> <radius_m> <duration_sec> [output.csv]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "1q/flight_dynamic/FlightManager.h"
#include "1q/flight_dynamic/autopilot/Autopilot.h"
#include "1q/flight_dynamic/config/FlightDynamicConfig.h"

using namespace oneq::flight_dynamic;

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

double MinTurnRadiusM(double speed_mps, double max_bank_deg) {
  double tan_b = std::tan(max_bank_deg * kDegToRad);
  if (tan_b < 0.01) tan_b = 0.01;
  return (speed_mps * speed_mps) / (9.80665 * tan_b);
}

double FeasibleTurnRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
  double min_r = MinTurnRadiusM(speed_mps, max_bank_deg);
  return std::max(nominal_r, min_r * 1.2);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: racetrack_trace_csv <model> <radius_m> <duration_sec> [output.csv]\n";
    return 1;
  }

  std::string model(argv[1]);
  double radius_m = std::atof(argv[2]);
  double duration_sec = std::atof(argv[3]);
  if (radius_m < 1.0 || duration_sec < 1.0) {
    std::cerr << "radius_m and duration_sec must be > 0\n";
    return 1;
  }

  const char* out_path = (argc >= 5) ? argv[4] : nullptr;

  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = true;
  cfg.silent_mode = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = 500.0;
  double init_speed = 60.0;  // fallback for trim
  cfg.initial_kinematics.velocity_mps.x_mps = init_speed;
  cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {0.0, 0.0, 0.0}; // Heading North

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::cerr << "Failed to init " << model << "\n";
    return 1;
  }

  // Use profile cruise speed for feasibility computation.
  const auto& prof = fm.GetAutopilot().GetControlProfile();
  double actual_speed = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : init_speed;
  double max_bank = prof.max_roll_angle_deg;
  double feasible_r = FeasibleTurnRadiusM(actual_speed, max_bank, radius_m);
  if (feasible_r > radius_m) {
    std::fprintf(stderr, "  radius %.0f → feasible %.0f\n", radius_m, feasible_r);
  }

  guidance::Waypoint start;
  start.latitude_rad = 0.0;
  start.longitude_rad = 0.0;
  start.altitude_m = 500.0;

  double leg_length_m = 10000.0;
  double heading_rad = 0.0;
  int num_laps = 10;

  fm.GetAutopilot().SetSpeedTargetMps(actual_speed);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kRacetrack;
  cmd.target = start;
  cmd.value = heading_rad;
  cmd.duration_sec = leg_length_m;
  cmd.heading_tolerance_rad = feasible_r;
  cmd.altitude_tolerance_m = static_cast<double>(num_laps);
  fm.PushManeuver(cmd);

  FILE* out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }

  std::fprintf(out,
      "sim_time_sec,lat_rad,lon_rad,alt_m,dist_m,radius_m,"
      "err_m,err_pct,heading_rad,roll_deg,pitch_deg,"
      "vtrue_mps,vc_mps\n");

  int max_steps = static_cast<int>(duration_sec / kDt);
  bool crashed = false;

  double cos_h = std::cos(heading_rad);
  double sin_h = std::sin(heading_rad);

  for (int step = 0; step < max_steps; ++step) {
    bool running = fm.Step(kDt);
    if (!running) { crashed = true; break; }
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { crashed = true; break; }

    double d_lat = s.latitude_rad - start.latitude_rad;
    double d_lon = s.longitude_rad - start.longitude_rad;
    double north = d_lat * kEarthRadiusM;
    double east = d_lon * kEarthRadiusM * std::cos(start.latitude_rad);

    double x_prime = east * cos_h - north * sin_h;
    double y_prime = east * sin_h + north * cos_h;

    double dist = DistanceToRacetrack(x_prime, y_prime, feasible_r, leg_length_m);
    double err = dist; // The error is just the distance to the path
    double err_pct = err / feasible_r * 100.0;

    std::fprintf(out,
        "%.3f,%.10f,%.10f,%.2f,%.2f,%.2f,"
        "%.2f,%.4f,%.6f,%.4f,%.4f,"
        "%.4f,%.4f\n",
        s.sim_time_sec,
        s.latitude_rad, s.longitude_rad, s.altitude_geod_m, dist, feasible_r,
        err, err_pct,
        s.psi_rad, s.phi_rad * 180.0 / M_PI, s.theta_rad * 180.0 / M_PI,
        s.vtrue_mps, s.vc_mps);
  }

  if (out_path) std::fclose(out);

  std::fprintf(stderr,
      "\n═══ Racetrack Trace: %s  r=%.0fm  dur=%.0fs ═══\n",
      model.c_str(), feasible_r, duration_sec);
  if (crashed) {
    std::fprintf(stderr, "  STATUS: CRASHED at %.2fs\n", fm.GetVehicleState().sim_time_sec);
  }

  return crashed ? 1 : 0;
}
