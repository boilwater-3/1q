/// @file
/// @brief 跑道盘旋逐点轨迹导出 — 每步记录位置、距离、误差
///
/// 用法：
///   racetrack_trace_csv ALL [output_prefix] [duration_sec]
///   racetrack_trace_csv <model> <radius_m> <duration_sec> [output.csv]
///
/// ALL 模式：为全部机型各生成一份轨迹 CSV，文件名为 <prefix><model>_trace.csv。
/// 单机模式与原来一致，支持自定义半径和时长。

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

// Aircraft spec matching racetrack_quality_csv
struct AircraftSpec {
  std::string model;
  double altitude_m;
  double speed_mps;
  double nominal_radius_m;
};

const AircraftSpec kAllAircraft[] = {
    {"f16", 3000.0, 200.0, 12000.0},
    {"f15", 3000.0, 200.0, 6000.0},
    {"A4", 2000.0, 120.0, 2500.0},
    {"F4N", 2000.0, 130.0, 3000.0},
    {"F80C", 2000.0, 120.0, 2500.0},
    {"OV10", 500.0, 70.0, 1000.0},
    {"737", 3000.0, 130.0, 4000.0},
    {"B747", 3000.0, 140.0, 5000.0},
    {"MD11", 3000.0, 140.0, 5000.0},
    {"C130", 5000.0, 90.0, 2500.0},
    {"B17", 1000.0, 60.0, 2000.0},
    {"Boeing314", 500.0, 70.0, 1500.0},
    {"L410", 3000.0, 90.0, 2500.0},
    {"DHC6", 500.0, 55.0, 800.0},
    {"Concorde", 10000.0, 250.0, 8000.0},
    {"c172p", 500.0, 50.0, 1000.0},
    {"c172r", 500.0, 50.0, 1000.0},
    {"c172x", 500.0, 50.0, 1000.0},
    {"c182", 1000.0, 55.0, 800.0},
    {"c310", 500.0, 65.0, 1000.0},
};

/// Run a single racetrack trace for a given model and write CSV.
/// Returns 0 on success, 1 on crash.
int RunOneTrace(const std::string& model, double altitude_m, double init_spd,
                double radius_m, double duration_sec, FILE* out) {
  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = true;
  cfg.silent_mode = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = altitude_m;
  cfg.initial_kinematics.velocity_mps.x_mps = init_spd;
  cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {0.0, 0.0, 0.0};

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::fprintf(stderr, "  [%s] INIT FAILED\n", model.c_str());
    return 1;
  }

  // Use profile cruise speed for feasibility.
  const auto& prof = fm.GetAutopilot().GetControlProfile();
  double actual_speed = prof.cruise_speed_mps > 0.0 ? prof.cruise_speed_mps : init_spd;
  double max_bank = prof.max_roll_angle_deg;
  double feasible_r = FeasibleTurnRadiusM(actual_speed, max_bank, radius_m);

  guidance::Waypoint start;
  start.latitude_rad = 0.0;
  start.longitude_rad = 0.0;
  start.altitude_m = altitude_m;

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

  // Write header with model metadata
  std::fprintf(out,
      "# model=%s  altitude=%.0fm  speed=%.0fm/s  feasible_r=%.0fm\n"
      "sim_time_sec,lat_rad,lon_rad,alt_m,dist_m,radius_m,"
      "err_m,err_pct,heading_rad,roll_deg,pitch_deg,"
      "vtrue_mps,vc_mps\n",
      model.c_str(), altitude_m, actual_speed, feasible_r);

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
    double err = dist;
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

  return crashed ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  // ── ALL mode ──
  if (argc >= 2 && std::string(argv[1]) == "ALL") {
    std::string prefix = (argc >= 3) ? argv[2] : "/tmp/racetrack_trace_";
    double duration_sec = (argc >= 4) ? std::atof(argv[3]) : 600.0;
    if (duration_sec < 1.0) duration_sec = 600.0;

    int ok_count = 0, fail_count = 0;
    for (const auto& spec : kAllAircraft) {
      std::string path = prefix + spec.model + "_trace.csv";
      std::fprintf(stderr, "  [%s] → %s\n", spec.model.c_str(), path.c_str());
      FILE* out = std::fopen(path.c_str(), "w");
      if (!out) {
        std::fprintf(stderr, "    Cannot open %s\n", path.c_str());
        fail_count++;
        continue;
      }
      double r = spec.nominal_radius_m;
      int ret = RunOneTrace(spec.model, spec.altitude_m, spec.speed_mps,
                            r, duration_sec, out);
      std::fclose(out);
      if (ret == 0) ok_count++;
      else fail_count++;
    }
    std::fprintf(stderr, "\n═══ ALL done: %d OK, %d FAILED ═══\n", ok_count, fail_count);
    return fail_count > 0 ? 1 : 0;
  }

  // ── Single model mode ──
  if (argc < 4) {
    std::cerr << "Usage:\n"
              << "  racetrack_trace_csv ALL [output_prefix] [duration_sec]\n"
              << "  racetrack_trace_csv <model> <radius_m> <duration_sec> [output.csv]\n";
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
  FILE* out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }

  // Look up spec altitude/speed, or use sensible defaults.
  double alt_m = 500.0;
  double spd = 60.0;
  for (const auto& spec : kAllAircraft) {
    if (spec.model == model) {
      alt_m = spec.altitude_m;
      spd = spec.speed_mps;
      break;
    }
  }

  int ret = RunOneTrace(model, alt_m, spd, radius_m, duration_sec, out);
  if (out_path) std::fclose(out);

  std::fprintf(stderr,
      "\n═══ Racetrack Trace: %s  r=%.0fm  dur=%.0fs ═══\n",
      model.c_str(), radius_m, duration_sec);
  if (ret != 0) {
    std::fprintf(stderr, "  STATUS: FAILED\n");
  }
  return ret;
}
