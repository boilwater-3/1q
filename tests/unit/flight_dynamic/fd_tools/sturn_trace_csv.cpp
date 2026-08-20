/// @file
/// @brief S型机动逐点轨迹导出 — 每步记录位置、航向、高度、倾斜角
///
/// 输出 CSV 可用于 Matplotlib 绘图。
///
/// 用法：
///   sturn_trace_csv ALL [output_prefix] [duration_sec]
///   sturn_trace_csv <model> [output.csv]
///
/// 示例（单机）：
///   sturn_trace_csv f16 /tmp/f16_sturn.csv
///
/// 示例（全部机型）：
///   sturn_trace_csv ALL /tmp/sturn_trace_
///
/// 可视化：
///   python3 -c "
///   import pandas as pd, matplotlib.pyplot as plt
///   d = pd.read_csv('/tmp/f16_sturn.csv', comment='#')
///   fig, ((a1,a2),(a3,a4)) = plt.subplots(2,2,figsize=(12,8))
///   a1.scatter(d.east_m, d.north_m, s=1, c=d.sim_time_sec, cmap='viridis')
///   a1.set_title('trajectory (color=time)'); a1.set_aspect('equal')
///   a1.set_xlabel('east m'); a1.set_ylabel('north m')
///   a2.plot(d.sim_time_sec, d.heading_deg); a2.set_ylabel('heading deg')
///   a2.set_title('heading vs time')
///   a3.plot(d.sim_time_sec, d.alt_m); a3.set_ylabel('altitude m')
///   a3.set_title('altitude vs time')
///   a4.plot(d.sim_time_sec, d.roll_deg); a4.set_ylabel('roll deg')
///   a4.set_title('roll vs time')
///   plt.tight_layout(); plt.savefig('/tmp/sturn_f16.png'); print('saved')
///   "

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

struct AircraftSpec {
  std::string model;
  double altitude_m;
  double speed_mps;
  double amplitude_deg;
  double period_sec;
  double duration_sec;
};

const AircraftSpec kAllAircraft[] = {
    {"f16",      3000.0, 200.0, 15.0, 10.0, 30.0},
    {"f15",      3000.0, 200.0, 15.0, 10.0, 30.0},
    {"A4",       2000.0, 120.0, 15.0, 10.0, 30.0},
    {"F4N",      2000.0, 130.0, 15.0, 10.0, 30.0},
    {"F80C",     2000.0, 120.0, 15.0, 10.0, 30.0},
    {"OV10",      500.0,  70.0, 15.0, 10.0, 30.0},
    {"737",      3000.0, 130.0, 15.0, 12.0, 30.0},
    {"B747",     3000.0, 140.0, 15.0, 12.0, 30.0},
    {"MD11",     3000.0, 140.0, 15.0, 12.0, 30.0},
    {"C130",     1000.0,  90.0, 15.0, 10.0, 30.0},
    {"B17",      1000.0,  80.0, 15.0, 10.0, 30.0},
    {"Boeing314", 500.0,  70.0, 15.0, 10.0, 30.0},
    {"L410",     1000.0,  90.0, 15.0, 10.0, 30.0},
    {"DHC6",      500.0,  55.0, 15.0, 10.0, 30.0},
    {"Concorde",15000.0, 500.0, 15.0, 12.0, 30.0},
    {"c172p",     500.0,  50.0, 15.0, 10.0, 30.0},
    {"c172r",     500.0,  50.0, 15.0, 10.0, 30.0},
    {"c172x",     500.0,  50.0, 15.0, 10.0, 30.0},
    {"c182",      500.0,  55.0, 15.0, 10.0, 30.0},
    {"c310",      500.0,  65.0, 15.0, 10.0, 30.0},
};

/// Run a single S-Turn trace and write CSV.
/// Returns 0 on success, 1 on crash.
int RunOneTrace(const std::string& model, double altitude_m, double init_spd,
                double amplitude_deg, double period_sec, double duration_sec,
                FILE* out) {
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

  // S-Turn: ManeuverCommand field convention:
  //   value                = base_heading_rad (0 = north)
  //   heading_tolerance_rad = amplitude_deg
  //   altitude_tolerance_m  = period_sec
  //   duration_sec          = total duration
  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kSTurn;
  cmd.value = 0.0;
  cmd.heading_tolerance_rad = amplitude_deg;
  cmd.altitude_tolerance_m = period_sec;
  cmd.duration_sec = duration_sec;
  fm.PushManeuver(cmd);

  double ref_lat = cfg.initial_kinematics.position_lla_deg_m.latitude_deg * M_PI / 180.0;
  double ref_lon = cfg.initial_kinematics.position_lla_deg_m.longitude_deg * M_PI / 180.0;

  // Header
  std::fprintf(out,
      "# model=%s  altitude=%.0fm  speed=%.0fm/s  amplitude=%.0fdeg  period=%.1fs  duration=%.0fs\n"
      "sim_time_sec,lat_rad,lon_rad,alt_m,north_m,east_m,"
      "heading_deg,roll_deg,pitch_deg,vtrue_mps\n",
      model.c_str(), altitude_m, init_spd, amplitude_deg, period_sec, duration_sec);

  int max_steps = static_cast<int>(duration_sec / kDt);
  bool crashed = false;
  double alt_min = 1e9, alt_max = -1e9;
  double roll_max = 0.0;

  for (int step = 0; step < max_steps; ++step) {
    bool running = fm.Step(kDt);
    if (!running) { crashed = true; break; }
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { crashed = true; break; }

    double north_m = (s.latitude_rad - ref_lat) * kEarthRadiusM;
    double east_m = (s.longitude_rad - ref_lon) * kEarthRadiusM * std::cos(ref_lat);

    if (s.altitude_geod_m < alt_min) alt_min = s.altitude_geod_m;
    if (s.altitude_geod_m > alt_max) alt_max = s.altitude_geod_m;
    double r = std::abs(s.phi_rad) * 57.2958;
    if (r > roll_max) roll_max = r;

    // Subsample: output every 10 steps (10 Hz) to keep CSV size manageable.
    if (step % 10 == 0) {
      std::fprintf(out,
          "%.3f,%.10f,%.10f,%.2f,%.2f,%.2f,"
          "%.4f,%.4f,%.4f,%.4f\n",
          s.sim_time_sec,
          s.latitude_rad, s.longitude_rad, s.altitude_geod_m,
          north_m, east_m,
          s.psi_rad * 57.2958,
          s.phi_rad * 57.2958,
          s.theta_rad * 57.2958,
          s.vtrue_mps);
    }
  }

  // Summary to stderr
  std::fprintf(stderr,
      "  [%s] %s  alt=[%.0f, %.0f]m  roll_max=%.1f°\n",
      model.c_str(),
      crashed ? "CRASHED" : "OK",
      alt_min, alt_max, roll_max);

  return crashed ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  // ── ALL mode ──
  if (argc >= 2 && std::string(argv[1]) == "ALL") {
    std::string prefix = (argc >= 3) ? argv[2] : "/tmp/sturn_trace_";
    int ok_count = 0, fail_count = 0;
    for (const auto& spec : kAllAircraft) {
      std::string path = prefix + spec.model + "_trace.csv";
      FILE* out = std::fopen(path.c_str(), "w");
      if (!out) {
        std::fprintf(stderr, "    Cannot open %s\n", path.c_str());
        fail_count++;
        continue;
      }
      int ret = RunOneTrace(spec.model, spec.altitude_m, spec.speed_mps,
                            spec.amplitude_deg, spec.period_sec, spec.duration_sec, out);
      std::fclose(out);
      if (ret == 0) ok_count++;
      else fail_count++;
    }
    std::fprintf(stderr, "\n═══ ALL done: %d OK, %d FAILED ═══\n", ok_count, fail_count);
    return fail_count > 0 ? 1 : 0;
  }

  // ── Single model mode ──
  if (argc < 2) {
    std::cerr << "Usage:\n"
              << "  sturn_trace_csv ALL [output_prefix] [duration_sec]\n"
              << "  sturn_trace_csv <model> [output.csv]\n";
    return 1;
  }

  std::string model(argv[1]);
  const char* out_path = (argc >= 3) ? argv[2] : nullptr;

  // Look up spec or use defaults.
  double alt_m = 2000.0;
  double spd = 100.0;
  double amplitude = 15.0;
  double period = 10.0;
  double duration = 30.0;
  for (const auto& spec : kAllAircraft) {
    if (spec.model == model) {
      alt_m = spec.altitude_m;
      spd = spec.speed_mps;
      amplitude = spec.amplitude_deg;
      period = spec.period_sec;
      duration = spec.duration_sec;
      break;
    }
  }

  FILE* out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }

  int ret = RunOneTrace(model, alt_m, spd, amplitude, period, duration, out);
  if (out_path) std::fclose(out);

  if (out_path) {
    std::fprintf(stderr, "\n  Trace written to: %s\n", out_path);
  }
  return ret;
}
