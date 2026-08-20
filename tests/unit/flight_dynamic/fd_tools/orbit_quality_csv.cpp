/// @file
/// @brief 盘旋（Orbit）质量分析 — 独立 CSV 导出工具
///
/// 用法：
///   orbit_quality_csv <aircraft_model|ALL> [output.csv]
///
/// 输出多半径扫描（0.5×, 1.0×, 2.0× 标称半径），每行包含：
///
///   维度 1 — 轨迹稳定性：avg/min/max_distance_m, radius_error_ratio
///   维度 2 — 运动学表现：avg/min/max_speed_mps, segment_angular_rate_cv, total_bearing_deg
///   维度 3 — 姿态与朝向：heading_reversal_rate, max_heading_step_deg, total_heading_deg,
///                         max_roll_deg, roll_limit_deg, roll_over_limit_ratio, roll_severe_count
///   维度 4 — 切入与切出：exit_max_roll_step_deg, exit_max_alt_jump_m
///   维度 5 — 边界与极端：min/max_altitude_m, crashed
///
/// 示例：
///   ./orbit_quality_csv f16 /tmp/f16_orbit.csv
///   ./orbit_quality_csv ALL /tmp/all_orbit.csv

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
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

// ── 几何工具 ───────────────────────────────────────────────────────────────

double NormalizeRad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double DistanceToCenterM(const model::VehicleState& s, const guidance::Waypoint& c) {
  double cos_lat = std::cos(c.latitude_rad);
  double dn = (s.latitude_rad - c.latitude_rad) * kEarthRadiusM;
  double de = (s.longitude_rad - c.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::hypot(dn, de);
}

double BearingFromCenterRad(const model::VehicleState& s, const guidance::Waypoint& c) {
  double cos_lat = std::cos(c.latitude_rad);
  double dn = (s.latitude_rad - c.latitude_rad) * kEarthRadiusM;
  double de = (s.longitude_rad - c.longitude_rad) * kEarthRadiusM * cos_lat;
  return std::atan2(de, dn);
}

double MinTurnRadiusM(double speed_mps, double max_bank_deg) {
  double tan_bank = std::tan(max_bank_deg * kDegToRad);
  if (tan_bank < 0.01) tan_bank = 0.01;
  return (speed_mps * speed_mps) / (9.80665 * tan_bank);
}

double FeasibleOrbitRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
  double min_r = MinTurnRadiusM(speed_mps, max_bank_deg);
  return std::max(nominal_r, min_r * 1.2);
}

/// 中心在飞行器南侧，使初始航向（北）= 顺时针切线方向
guidance::Waypoint MakeCenter(double radius_m, double altitude_m) {
  guidance::Waypoint c;
  c.latitude_rad = -radius_m / kEarthRadiusM;
  c.longitude_rad = 0.0;
  c.altitude_m = altitude_m;
  return c;
}

// ── 飞机参数 ───────────────────────────────────────────────────────────────

struct AircraftSpec {
  std::string model;
  double altitude_m;
  double speed_mps;
  double nominal_radius_m;
};

const AircraftSpec kAllAircraft[] = {
    // 战斗机
    {"f16", 3000.0, 200.0, 5000.0},
    {"f15", 3000.0, 200.0, 6000.0},
    {"A4", 2000.0, 120.0, 2500.0},
    {"F4N", 2000.0, 130.0, 3000.0},
    {"F80C", 2000.0, 120.0, 2500.0},
    {"OV10", 500.0, 70.0, 1000.0},
    // 运输机
    {"737", 3000.0, 130.0, 4000.0},
    {"B747", 3000.0, 140.0, 5000.0},
    {"MD11", 3000.0, 140.0, 5000.0},
    {"C130", 1000.0, 90.0, 2500.0},
    {"B17", 1000.0, 80.0, 2000.0},
    {"Boeing314", 500.0, 70.0, 1500.0},
    {"L410", 1000.0, 90.0, 2500.0},
    {"DHC6", 500.0, 55.0, 800.0},
    {"Concorde", 10000.0, 250.0, 10000.0},
    // 通用航空
    {"c172p", 500.0, 50.0, 1000.0},
    {"c172r", 500.0, 50.0, 1000.0},
    {"c172x", 500.0, 50.0, 1000.0},
    {"c182", 500.0, 55.0, 800.0},
    {"c310", 500.0, 65.0, 1000.0},
};

// ── 探测结果 ───────────────────────────────────────────────────────────────

struct OrbitQualityRow {
  std::string model;
  double altitude_m = 0.0, speed_mps = 0.0;
  double radius_m = 0.0, feasible_radius_m = 0.0;
  // 半径精度
  double avg_dist_m = 0.0, min_dist_m = 0.0, max_dist_m = 0.0;
  double radius_error_ratio = 0.0;
  // 速度
  double avg_speed_mps = 0.0, min_speed_mps = 0.0, max_speed_mps = 0.0;
  // 角速度
  double angular_rate_cv = 0.0, total_bearing_deg = 0.0;
  // 航向
  double heading_reversal_rate = 0.0, max_heading_step_deg = 0.0, total_heading_deg = 0.0;
  // 滚转
  double max_roll_deg = 0.0, roll_limit_deg = 0.0;
  double roll_over_limit_ratio = 0.0;
  int roll_severe_count = 0;
  // 高度
  double min_alt_m = 0.0, max_alt_m = 0.0;
  // 切出
  double exit_max_roll_step_deg = 0.0, exit_max_alt_jump_m = 0.0;
  // 状态
  bool crashed = false, stopped = false;
};

// ── 探测主逻辑 ─────────────────────────────────────────────────────────────

OrbitQualityRow RunOrbitProbe(const AircraftSpec& spec, double radius_m) {
  OrbitQualityRow row;
  row.model = spec.model;
  row.altitude_m = spec.altitude_m;
  row.speed_mps = spec.speed_mps;
  row.radius_m = radius_m;
  row.min_alt_m = std::numeric_limits<double>::max();

  config::FlightDynamicConfig cfg;
  cfg.aircraft_model = spec.model;
  cfg.aircraft_root_dir = FD_JSBSIM_ROOT_DIR;
  cfg.dt_sec = kDt;
  cfg.do_trim = true;
  cfg.silent_mode = true;
  cfg.initial_kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  cfg.initial_kinematics.position_lla_deg_m.latitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.longitude_deg = 0.0;
  cfg.initial_kinematics.position_lla_deg_m.altitude_m = spec.altitude_m;
  cfg.initial_kinematics.velocity_mps.x_mps = spec.speed_mps;
  cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {0.0, 0.0, 0.0};

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    row.stopped = true;
    return row;
  }

  double max_bank = fm.GetAutopilot().GetControlProfile().max_roll_angle_deg;
  double feasible_r = FeasibleOrbitRadiusM(spec.speed_mps, max_bank, radius_m);
  row.feasible_radius_m = feasible_r;
  row.roll_limit_deg = max_bank;

  auto center = MakeCenter(feasible_r, spec.altitude_m);

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kOrbit;
  cmd.target = center;
  cmd.value = feasible_r;
  fm.PushManeuver(cmd);

  // ── 收敛期：轨道周期的 1/4 或至少 2000 步 ──
  double period = 2.0 * M_PI * feasible_r / std::max(spec.speed_mps, 10.0);
  int settle = std::max(2000, static_cast<int>(period / kDt / 4.0));
  settle = std::min(settle, 20000);

  for (int i = 0; i < settle; ++i) {
    if (!fm.Step(kDt)) { row.stopped = true; return row; }
    if (fm.GetVehicleState().altitude_geod_m <= 0.0) { row.crashed = true; return row; }
  }

  // ── 稳态采集：1000 步 ──
  constexpr int kSamples = 1000;
  std::vector<double> distances, speeds, headings, bearing_deltas;
  distances.reserve(kSamples);
  speeds.reserve(kSamples);
  headings.reserve(kSamples);
  bearing_deltas.reserve(kSamples);

  double prev_bearing = BearingFromCenterRad(fm.GetVehicleState(), center);

  for (int i = 0; i < kSamples; ++i) {
    if (!fm.Step(kDt)) { row.stopped = true; return row; }
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { row.crashed = true; return row; }

    distances.push_back(DistanceToCenterM(s, center));
    speeds.push_back(s.vtrue_mps);
    headings.push_back(s.psi_rad);

    double b = BearingFromCenterRad(s, center);
    bearing_deltas.push_back(NormalizeRad(b - prev_bearing));
    prev_bearing = b;

    double roll = std::abs(s.phi_rad) * 180.0 / M_PI;
    row.max_roll_deg = std::max(row.max_roll_deg, roll);
    if (roll > row.roll_limit_deg) row.roll_over_limit_ratio += 1.0;
    if (roll > row.roll_limit_deg * 2.0) ++row.roll_severe_count;

    row.min_alt_m = std::min(row.min_alt_m, s.altitude_geod_m);
    row.max_alt_m = std::max(row.max_alt_m, s.altitude_geod_m);
  }

  row.roll_over_limit_ratio /= kSamples;
  if (row.min_alt_m == std::numeric_limits<double>::max()) row.min_alt_m = 0.0;

  // ── 半径统计 ──
  row.avg_dist_m = std::accumulate(distances.begin(), distances.end(), 0.0) / kSamples;
  row.min_dist_m = *std::min_element(distances.begin(), distances.end());
  row.max_dist_m = *std::max_element(distances.begin(), distances.end());
  row.radius_error_ratio = std::abs(row.avg_dist_m - feasible_r) / feasible_r;

  // ── 速度统计 ──
  row.avg_speed_mps = std::accumulate(speeds.begin(), speeds.end(), 0.0) / kSamples;
  row.min_speed_mps = *std::min_element(speeds.begin(), speeds.end());
  row.max_speed_mps = *std::max_element(speeds.begin(), speeds.end());

  // ── 角速度 CV（段间） ──
  constexpr int kSegSteps = 100;
  constexpr int kSegs = kSamples / kSegSteps;
  std::vector<double> seg_omegas;
  for (int s = 0; s < kSegs; ++s) {
    double tot = 0.0;
    for (int j = 0; j < kSegSteps; ++j) tot += bearing_deltas[static_cast<size_t>(s * kSegSteps + j)];
    seg_omegas.push_back(tot / (kSegSteps * kDt));
  }
  row.total_bearing_deg = std::abs(std::accumulate(bearing_deltas.begin(), bearing_deltas.end(), 0.0))
                          * 180.0 / M_PI;
  double mw = std::accumulate(seg_omegas.begin(), seg_omegas.end(), 0.0) / kSegs;
  double var = 0.0;
  for (double w : seg_omegas) { double d = w - mw; var += d * d; }
  var /= kSegs;
  row.angular_rate_cv = std::sqrt(var) / (std::abs(mw) + 1e-9);

  // ── 航向平滑度 ──
  double max_hdg_step = 0.0;
  int revs = 0;
  for (size_t i = 1; i < headings.size(); ++i) {
    double delta = NormalizeRad(headings[i] - headings[i - 1]);
    max_hdg_step = std::max(max_hdg_step, std::abs(delta));
    if (i >= 2) {
      double prev = NormalizeRad(headings[i - 1] - headings[i - 2]);
      if (std::abs(prev) > 0.01 && std::abs(delta) > 0.01 && prev * delta < 0.0) ++revs;
    }
  }
  row.max_heading_step_deg = max_hdg_step * 180.0 / M_PI;
  row.heading_reversal_rate = static_cast<double>(revs) / kSamples;
  double tot_hdg = 0.0;
  for (size_t i = 1; i < headings.size(); ++i)
    tot_hdg += NormalizeRad(headings[i] - headings[i - 1]);
  row.total_heading_deg = std::abs(tot_hdg) * 180.0 / M_PI;

  // ── 切出质量 ──
  ManeuverCommand hdg_cmd;
  hdg_cmd.type = guidance::ManeuverType::kSetHeading;
  hdg_cmd.value = NormalizeRad(fm.GetVehicleState().psi_rad + M_PI / 2.0);
  fm.PushManeuver(hdg_cmd);

  double prev_roll = fm.GetVehicleState().phi_rad;
  double prev_alt = fm.GetVehicleState().altitude_geod_m;
  for (int i = 0; i < 300; ++i) {
    fm.Step(kDt);
    double rs = std::abs(fm.GetVehicleState().phi_rad - prev_roll) * 180.0 / M_PI;
    double aj = std::abs(fm.GetVehicleState().altitude_geod_m - prev_alt);
    row.exit_max_roll_step_deg = std::max(row.exit_max_roll_step_deg, rs);
    row.exit_max_alt_jump_m = std::max(row.exit_max_alt_jump_m, aj);
    prev_roll = fm.GetVehicleState().phi_rad;
    prev_alt = fm.GetVehicleState().altitude_geod_m;
  }

  return row;
}

// ── CSV 输出 ────────────────────────────────────────────────────────────────

void WriteHeader(FILE* out) {
  fprintf(out, "model,altitude_m,speed_mps,radius_m,feasible_radius_m,"
                "avg_distance_m,min_distance_m,max_distance_m,radius_error_ratio,"
                "avg_speed_mps,min_speed_mps,max_speed_mps,"
                "angular_rate_cv,total_bearing_deg,"
                "heading_reversal_rate,max_heading_step_deg,total_heading_deg,"
                "max_roll_deg,roll_limit_deg,roll_over_limit_ratio,roll_severe_count,"
                "min_altitude_m,max_altitude_m,"
                "exit_max_roll_step_deg,exit_max_alt_jump_m,"
                "crashed,stopped\n");
}

void WriteRow(FILE* out, const OrbitQualityRow& r) {
  fprintf(out,
      "%s,%.1f,%.1f,%.1f,%.1f,"
      "%.1f,%.1f,%.1f,%.4f,"
      "%.2f,%.2f,%.2f,"
      "%.4f,%.2f,"
      "%.4f,%.4f,%.2f,"
      "%.2f,%.1f,%.4f,%d,"
      "%.1f,%.1f,"
      "%.4f,%.4f,"
      "%d,%d\n",
      r.model.c_str(), r.altitude_m, r.speed_mps, r.radius_m, r.feasible_radius_m,
      r.avg_dist_m, r.min_dist_m, r.max_dist_m, r.radius_error_ratio,
      r.avg_speed_mps, r.min_speed_mps, r.max_speed_mps,
      r.angular_rate_cv, r.total_bearing_deg,
      r.heading_reversal_rate, r.max_heading_step_deg, r.total_heading_deg,
      r.max_roll_deg, r.roll_limit_deg, r.roll_over_limit_ratio, r.roll_severe_count,
      r.min_alt_m, r.max_alt_m,
      r.exit_max_roll_step_deg, r.exit_max_alt_jump_m,
      r.crashed ? 1 : 0, r.stopped ? 1 : 0);
}

}  // namespace

// ── main ───────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "Usage: orbit_quality_csv <aircraft_model|ALL> [output.csv]\n\n"
        << "Generate orbit quality metrics across radius sweep (0.5x, 1.0x, 2.0x nominal).\n"
        << "CSV columns cover 5 dimensions:\n"
        << "  1. Radius accuracy & stability\n"
        << "  2. Speed stability & angular rate consistency\n"
        << "  3. Heading smoothness & roll envelope\n"
        << "  4. Entry/exit transition quality\n"
        << "  5. Edge case robustness\n\n"
        << "Examples:\n"
        << "  orbit_quality_csv f16 /tmp/f16_orbit.csv\n"
        << "  orbit_quality_csv ALL /tmp/all_orbit.csv\n";
    return 1;
  }

  std::string sel(argv[1]);
  const char* out_path = (argc >= 3) ? argv[2] : nullptr;
  FILE* out = out_path ? fopen(out_path, "w") : stdout;
  if (!out) { std::cerr << "Cannot open " << out_path << "\n"; return 1; }

  std::vector<AircraftSpec> specs;
  if (sel == "ALL") {
    for (const auto& s : kAllAircraft) specs.push_back(s);
  } else {
    auto it = std::find_if(std::begin(kAllAircraft), std::end(kAllAircraft),
                           [&](const AircraftSpec& s) { return s.model == sel; });
    if (it == std::end(kAllAircraft)) {
      std::cerr << "Unknown aircraft: " << sel
                << ". Use one of: f16 737 c172p Concorde ALL\n";
      return 1;
    }
    specs.push_back(*it);
  }

  WriteHeader(out);

  for (const auto& spec : specs) {
    for (double mult : {0.5, 1.0, 2.0}) {
      double test_r = spec.nominal_radius_m * mult;
      fprintf(stderr, "%s r=%.0f: ", spec.model.c_str(), test_r);
      auto row = RunOrbitProbe(spec, test_r);
      if (row.crashed) fprintf(stderr, "CRASHED\n");
      else if (row.stopped) fprintf(stderr, "STOPPED\n");
      else fprintf(stderr, "ok (dist=%.0f err=%.2f)\n", row.avg_dist_m, row.radius_error_ratio);
      WriteRow(out, row);
    }
  }

  if (out_path) fclose(out);
  fprintf(stderr, "Done.\n");
  return 0;
}
