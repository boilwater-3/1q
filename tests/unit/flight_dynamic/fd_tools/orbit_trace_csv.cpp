/// @file
/// @brief 盘旋逐点轨迹导出 — 每步记录位置、到中心距离、半径误差
///
/// 输出 CSV 可用于 Matplotlib 绘图 / Google Earth KML 转换。
///
/// 用法：
///   orbit_trace_csv <model> <radius_m> <duration_sec> [output.csv]
///
/// 示例（可视化）：
///   orbit_trace_csv f16 5000 60 /tmp/f16_trace.csv
///   python3 -c "
///   import pandas as pd, matplotlib.pyplot as plt
///   d = pd.read_csv('/tmp/f16_trace.csv')
///   plt.figure(figsize=(12,4))
///   plt.subplot(121); plt.plot(d.dist_m); plt.axhline(d.radius_m[0], ls='--')
///   plt.title('distance to center')
///   plt.subplot(122); plt.scatter(d.lon_deg, d.lat_deg, s=1, c=d.sim_time_sec)
///   plt.title('trajectory (color=time)'); plt.axis('equal')
///   plt.tight_layout(); plt.show()
///   "

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

double DistanceToCenterM(double lat_rad, double lon_rad, const guidance::Waypoint& c) {
  double cos_lat = std::cos(c.latitude_rad);
  double dn = (lat_rad - c.latitude_rad) * kEarthRadiusM;
  double de = (lon_rad - c.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::hypot(dn, de);
}

double NormalizeRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double MinTurnRadiusM(double speed_mps, double max_bank_deg) {
  double tan_b = std::tan(max_bank_deg * kDegToRad);
  if (tan_b < 0.01) tan_b = 0.01;
  return (speed_mps * speed_mps) / (9.80665 * tan_b);
}

guidance::Waypoint CenterSouth(double radius_m, double alt_m) {
  guidance::Waypoint c;
  c.latitude_rad = -radius_m / kEarthRadiusM;
  c.longitude_rad = 0.0;
  c.altitude_m = alt_m;
  return c;
}

double GetInitialSpeedMps(const std::string& model) {
  if (model == "c172p" || model == "c172r" || model == "c172x" || 
      model == "c182" || model == "c310" || model == "L410" || model == "DHC6") {
    return 60.0;
  }
  if (model == "C130" || model == "B17" || model == "Boeing314" || 
      model == "OV10" || model == "F80C") {
    return 120.0;
  }
  if (model == "Concorde") {
    return 250.0;
  }
  return 200.0; // Default for fast jets and airliners (f16, f15, 737, etc.)
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: orbit_trace_csv <model> <radius_m> <duration_sec> [output.csv]\n";
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

  // 配置
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
  double init_speed = GetInitialSpeedMps(model);
  cfg.initial_kinematics.velocity_mps.x_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.y_mps = init_speed;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {90.0, 0.0, 0.0};

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    std::cerr << "Failed to init " << model << "\n";
    return 1;
  }

  double max_bank = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double feasible_r = radius_m;
  double min_r = MinTurnRadiusM(init_speed, max_bank);
  if (feasible_r < min_r * 1.2) {
    feasible_r = min_r * 1.2;
    std::fprintf(stderr, "  radius %.0f → feasible %.0f (min turn %.0f)\n",
                 radius_m, feasible_r, min_r);
  }

  auto center = CenterSouth(feasible_r, 3000.0);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target = center;
  cmd.value = feasible_r;
  fm.PushManeuver(cmd);

  // 打开输出
  FILE* out = out_path ? std::fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }

  // CSV header
  std::fprintf(out,
      "sim_time_sec,lat_rad,lon_rad,alt_m,dist_m,radius_m,"
      "err_m,err_pct,heading_rad,roll_deg,pitch_deg,"
      "vtrue_mps,vc_mps\n");

  int max_steps = static_cast<int>(duration_sec / kDt);
  double prev_err = 0.0;
  double max_abs_err = 0.0;
  double min_abs_err = 1e9;
  double err_integral = 0.0;
  bool crashed = false;

  // ── 逐步记录 ──
  for (int step = 0; step < max_steps; ++step) {
    bool running = fm.Step(kDt);
    if (!running) { crashed = true; break; }
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { crashed = true; break; }

    double dist = DistanceToCenterM(s.latitude_rad, s.longitude_rad, center);
    double err = dist - feasible_r;
    double err_pct = err / feasible_r * 100.0;

    max_abs_err = std::max(max_abs_err, std::abs(err));
    min_abs_err = std::min(min_abs_err, std::abs(err));
    err_integral += std::abs(err) * kDt;
    prev_err = err;

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

  // ── 摘要 ──
  std::fprintf(stderr,
      "\n═══ Orbit Trace: %s  r=%.0fm  dur=%.0fs ═══\n",
      model.c_str(), feasible_r, duration_sec);
  if (crashed) {
    std::fprintf(stderr, "  STATUS: CRASHED at %.2fs\n", fm.GetVehicleState().sim_time_sec);
  } else {
    double final_dist = DistanceToCenterM(
        fm.GetVehicleState().latitude_rad, fm.GetVehicleState().longitude_rad, center);
    double final_err = final_dist - feasible_r;
    double time = fm.GetVehicleState().sim_time_sec;
    double avg_abs_err = err_integral / time;
    int n_orbits = static_cast<int>(std::abs(fm.GetVehicleState().psi_rad * max_steps * kDt * init_speed));

    std::fprintf(stderr, "  Final distance:  %.2f m  (target %.0f m, err %+.1f m = %+.1f%%)\n",
                 final_dist, feasible_r, final_err, final_err / feasible_r * 100.0);
    std::fprintf(stderr, "  Max |error|:     %.2f m\n", max_abs_err);
    std::fprintf(stderr, "  Min |error|:     %.2f m\n", min_abs_err);
    std::fprintf(stderr, "  Avg |error|:     %.2f m  (integrated over %.0f s = %.0f steps)\n",
                 avg_abs_err, time, time / kDt);
    std::fprintf(stderr, "  Final altitude:  %.1f m  (target %.0f m)\n",
                 fm.GetVehicleState().altitude_geod_m, 3000.0);
    std::fprintf(stderr, "  Final speed:     %.1f m/s\n", fm.GetVehicleState().vtrue_mps);
  }

  if (out_path) {
    std::fprintf(stderr, "\n  Trace written to: %s\n", out_path);
    std::fprintf(stderr, "  Visualize: python3 -c \"\n");
    std::fprintf(stderr, "    import pandas as pd, matplotlib.pyplot as plt\n");
    std::fprintf(stderr, "    d = pd.read_csv('%s')\n", out_path);
    std::fprintf(stderr, "    fig, ((a1,a2),(a3,a4)) = plt.subplots(2,2,figsize=(12,8))\n");
    std::fprintf(stderr, "    a1.plot(d.sim_time_sec, d.dist_m); a1.axhline(d.radius_m[0],c='r',ls='--')\n");
    std::fprintf(stderr, "    a1.set_ylabel('distance m'); a1.set_xlabel('time s')\n");
    std::fprintf(stderr, "    a2.plot(d.sim_time_sec, d.err_pct); a2.axhline(0,c='r',ls='--')\n");
    std::fprintf(stderr, "    a2.set_ylabel('error %%'); a2.set_xlabel('time s')\n");
    std::fprintf(stderr, "    a3.scatter(d.lon_rad, d.lat_rad, s=1, c=d.dist_m, cmap='viridis')\n");
    std::fprintf(stderr, "    a3.set_title('trajectory (color=distance to center)'); a3.set_aspect('equal')\n");
    std::fprintf(stderr, "    a4.plot(d.sim_time_sec, d.alt_m); a4.set_ylabel('alt m')\n");
    std::fprintf(stderr, "    plt.tight_layout(); plt.savefig('/tmp/orbit_trace.png')\n");
    std::fprintf(stderr, "    print('Saved /tmp/orbit_trace.png')\n");
    std::fprintf(stderr, "  \"\n");
  }

  return crashed ? 1 : 0;
}
