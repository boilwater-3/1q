/// @file
/// @brief 跑道（Racetrack）质量分析 — 独立 CSV 导出工具

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

double MinTurnRadiusM(double speed_mps, double max_bank_deg) {
  double tan_bank = std::tan(max_bank_deg * kDegToRad);
  if (tan_bank < 0.01) tan_bank = 0.01;
  return (speed_mps * speed_mps) / (9.80665 * tan_bank);
}

double FeasibleTurnRadiusM(double speed_mps, double max_bank_deg, double nominal_r) {
  double min_r = MinTurnRadiusM(speed_mps, max_bank_deg);
  return std::max(nominal_r, min_r * 1.2);
}

// Distance to the racetrack path
// x_prime, y_prime: coordinate in the racetrack-aligned frame
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

// ── 飞机参数 ───────────────────────────────────────────────────────────────

struct AircraftSpec {
  std::string model;
  double altitude_m;
  double speed_mps;
  double nominal_radius_m;
};

const AircraftSpec kAllAircraft[] = {
    // 战斗机
    {"f16", 3000.0, 200.0, 12000.0},
    {"f15", 3000.0, 200.0, 6000.0},
    {"A4", 2000.0, 120.0, 2500.0},
    {"F4N", 2000.0, 130.0, 3000.0},
    {"F80C", 2000.0, 120.0, 2500.0},
    {"OV10", 500.0, 70.0, 1000.0},
    // 运输机
    {"737", 3000.0, 130.0, 4000.0},
    {"B747", 3000.0, 140.0, 5000.0},
    {"MD11", 3000.0, 140.0, 5000.0},
    {"C130", 5000.0, 90.0, 2500.0},
    {"B17", 1000.0, 60.0, 2000.0},
    {"Boeing314", 500.0, 70.0, 1500.0},
    {"L410", 3000.0, 90.0, 2500.0},
    {"DHC6", 500.0, 55.0, 800.0},
    {"Concorde", 10000.0, 250.0, 8000.0},
    // 通用航空
    {"c172p", 500.0, 50.0, 1000.0},
    {"c172r", 500.0, 50.0, 1000.0},
    {"c172x", 500.0, 50.0, 1000.0},
    {"c182", 1000.0, 55.0, 800.0},
    {"c310", 500.0, 65.0, 1000.0},
};

// ── 探测结果 ───────────────────────────────────────────────────────────────

struct RacetrackQualityRow {
  std::string model;
  double altitude_m = 0.0, speed_mps = 0.0;
  double radius_m = 0.0, feasible_radius_m = 0.0;
  double leg_length_m = 0.0;
  // 轨迹精度
  double avg_err_m = 0.0, min_err_m = 0.0, max_err_m = 0.0;
  double cross_track_error_ratio = 0.0;
  // 速度
  double avg_speed_mps = 0.0, min_speed_mps = 0.0, max_speed_mps = 0.0;
  // 航向
  double max_heading_step_deg = 0.0, total_heading_deg = 0.0;
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

RacetrackQualityRow RunRacetrackProbe(const AircraftSpec& spec, double radius_m, double leg_len_m, int num_laps) {
  RacetrackQualityRow row;
  row.model = spec.model;
  row.altitude_m = spec.altitude_m;
  row.speed_mps = spec.speed_mps;
  row.radius_m = radius_m;
  row.leg_length_m = leg_len_m;
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
  cfg.initial_kinematics.velocity_mps.x_mps = spec.speed_mps; // North
  cfg.initial_kinematics.velocity_mps.y_mps = 0.0;
  cfg.initial_kinematics.velocity_mps.z_mps = 0.0;
  cfg.initial_kinematics.attitude_deg = {0.0, 0.0, 0.0}; // Heading North

  FlightManager fm(cfg);
  if (fm.GetState() != FlightManagerState::kReady) {
    row.stopped = true;
    return row;
  }

  // ── 预热：测量飞机实际稳定速度 ──
  // FBW 飞机（如 f16）的实际飞行速度可能远高于 profile 巡航速度。
  // 必须用实测速度计算 feasible_r，否则转弯半径不够大。
  // 但某些机型（C130, B17, L410）trim 后不稳定，无机动预热会崩溃。
  // 策略：先尝试短预热（500步=5s），失败则回退到 profile 速度。
  const auto& prof = fm.GetAutopilot().GetControlProfile();
  double max_bank = prof.max_roll_angle_deg;
  double actual_speed = spec.speed_mps;
  if (prof.cruise_speed_mps > 0.0) actual_speed = prof.cruise_speed_mps;

  bool warmup_ok = true;
  fm.GetAutopilot().SetSpeedTargetMps(actual_speed);
  fm.GetAutopilot().SetSpeedHold(true);
  fm.GetAutopilot().SetAltitudeTargetM(spec.altitude_m);
  fm.GetAutopilot().SetAltitudeHold(true);
  double peak_speed = 0.0;
  for (int i = 0; i < 2000; ++i) {
    if (!fm.Step(kDt)) { warmup_ok = false; break; }
    if (fm.GetVehicleState().altitude_geod_m <= 0.0) { warmup_ok = false; break; }
    double spd = fm.GetVehicleState().vtrue_mps;
    if (spd > peak_speed) peak_speed = spd;
  }
  // 如果预热成功且峰值速度高于 profile，使用峰值（捕捉 FBW 加速）
  if (warmup_ok && peak_speed > actual_speed) {
    actual_speed = peak_speed;
  } else if (!warmup_ok) {
    // 预热崩溃：重置 FlightManager，使用 profile 速度
    fm.Reset(cfg);
    if (fm.GetState() != FlightManagerState::kReady) {
      row.stopped = true;
      return row;
    }
  }

  double feasible_r = FeasibleTurnRadiusM(actual_speed, max_bank, radius_m);
  row.feasible_radius_m = feasible_r;
  row.roll_limit_deg = max_bank;
  row.speed_mps = actual_speed;  // report actual, not spec

  guidance::Waypoint start;
  start.latitude_rad = 0.0;
  start.longitude_rad = 0.0;
  start.altitude_m = spec.altitude_m;
  double heading_rad = 0.0;

  ManeuverCommand cmd;
  cmd.type = guidance::ManeuverType::kRacetrack;
  cmd.target = start;
  cmd.value = heading_rad;
  cmd.duration_sec = leg_len_m;
  cmd.heading_tolerance_rad = feasible_r;
  cmd.altitude_tolerance_m = static_cast<double>(num_laps);
  fm.GetAutopilot().SetSpeedTargetMps(actual_speed);
  fm.PushManeuver(cmd);

  // ── 收敛期：直线段通常不需要很久，但为了稳定我们先走 1000 步 ──
  int settle = 1000;
  for (int i = 0; i < settle; ++i) {
    if (!fm.Step(kDt)) { row.stopped = true; return row; }
    if (fm.GetVehicleState().altitude_geod_m <= 0.0) { row.crashed = true; return row; }
  }

  // ── 稳态采集：模拟所需圈数所需时间 ──
  double perimeter = 2.0 * leg_len_m + 2.0 * M_PI * feasible_r;
  double total_time = (perimeter * num_laps) / std::max(actual_speed, 10.0);
  int kSamples = static_cast<int>(total_time / kDt);
  kSamples = std::min(kSamples, 100000);

  std::vector<double> errors, speeds, headings;
  errors.reserve(kSamples);
  speeds.reserve(kSamples);
  headings.reserve(kSamples);

  double cos_h = std::cos(heading_rad);
  double sin_h = std::sin(heading_rad);

  for (int i = 0; i < kSamples; ++i) {
    if (!fm.Step(kDt)) { row.stopped = true; break; } // Might finish naturally
    const auto& s = fm.GetVehicleState();
    if (s.altitude_geod_m <= 0.0) { row.crashed = true; return row; }

    double d_lat = s.latitude_rad - start.latitude_rad;
    double d_lon = s.longitude_rad - start.longitude_rad;
    double north = d_lat * kEarthRadiusM;
    double east = d_lon * kEarthRadiusM * std::cos(start.latitude_rad);

    double x_prime = east * cos_h - north * sin_h;
    double y_prime = east * sin_h + north * cos_h;

    double dist_err = DistanceToRacetrack(x_prime, y_prime, feasible_r, leg_len_m);
    errors.push_back(dist_err);
    speeds.push_back(s.vtrue_mps);
    headings.push_back(s.psi_rad);

    double roll = std::abs(s.phi_rad) * 180.0 / M_PI;
    row.max_roll_deg = std::max(row.max_roll_deg, roll);
    if (roll > row.roll_limit_deg) row.roll_over_limit_ratio += 1.0;
    if (roll > row.roll_limit_deg * 2.0) ++row.roll_severe_count;

    row.min_alt_m = std::min(row.min_alt_m, s.altitude_geod_m);
    row.max_alt_m = std::max(row.max_alt_m, s.altitude_geod_m);
  }

  if (errors.empty()) { row.stopped = true; return row; }

  row.roll_over_limit_ratio /= errors.size();
  if (row.min_alt_m == std::numeric_limits<double>::max()) row.min_alt_m = 0.0;

  // ── 误差统计 ──
  row.avg_err_m = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
  row.min_err_m = *std::min_element(errors.begin(), errors.end());
  row.max_err_m = *std::max_element(errors.begin(), errors.end());
  row.cross_track_error_ratio = row.avg_err_m / feasible_r;

  // ── 速度统计 ──
  row.avg_speed_mps = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
  row.min_speed_mps = *std::min_element(speeds.begin(), speeds.end());
  row.max_speed_mps = *std::max_element(speeds.begin(), speeds.end());

  // ── 航向平滑度 ──
  double max_hdg_step = 0.0;
  for (size_t i = 1; i < headings.size(); ++i) {
    double delta = NormalizeRad(headings[i] - headings[i - 1]);
    max_hdg_step = std::max(max_hdg_step, std::abs(delta));
  }
  row.max_heading_step_deg = max_hdg_step * 180.0 / M_PI;
  double tot_hdg = 0.0;
  for (size_t i = 1; i < headings.size(); ++i)
    tot_hdg += NormalizeRad(headings[i] - headings[i - 1]);
  row.total_heading_deg = std::abs(tot_hdg) * 180.0 / M_PI;

  return row;
}

// ── CSV 输出 ────────────────────────────────────────────────────────────────

void WriteHeader(FILE* out) {
  fprintf(out, "model,altitude_m,speed_mps,radius_m,feasible_radius_m,leg_length_m,"
                "avg_err_m,min_err_m,max_err_m,cross_track_error_ratio,"
                "avg_speed_mps,min_speed_mps,max_speed_mps,"
                "max_heading_step_deg,total_heading_deg,"
                "max_roll_deg,roll_limit_deg,roll_over_limit_ratio,roll_severe_count,"
                "min_altitude_m,max_altitude_m,"
                "exit_max_roll_step_deg,exit_max_alt_jump_m,"
                "crashed,stopped\n");
}

void WriteRow(FILE* out, const RacetrackQualityRow& r) {
  fprintf(out,
      "%s,%.1f,%.1f,%.1f,%.1f,%.1f,"
      "%.1f,%.1f,%.1f,%.4f,"
      "%.2f,%.2f,%.2f,"
      "%.4f,%.2f,"
      "%.2f,%.1f,%.4f,%d,"
      "%.1f,%.1f,"
      "%.4f,%.4f,"
      "%d,%d\n",
      r.model.c_str(), r.altitude_m, r.speed_mps, r.radius_m, r.feasible_radius_m, r.leg_length_m,
      r.avg_err_m, r.min_err_m, r.max_err_m, r.cross_track_error_ratio,
      r.avg_speed_mps, r.min_speed_mps, r.max_speed_mps,
      r.max_heading_step_deg, r.total_heading_deg,
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
        << "Usage: racetrack_quality_csv <aircraft_model|ALL> [output.csv]\n\n"
        << "Examples:\n"
        << "  racetrack_quality_csv f16 /tmp/f16_racetrack.csv\n"
        << "  racetrack_quality_csv ALL /tmp/all_racetrack.csv\n";
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
      std::cerr << "Unknown aircraft: " << sel << "\n";
      return 1;
    }
    specs.push_back(*it);
  }

  WriteHeader(out);

  double leg_length_m = 10000.0;
  int num_laps = 2;

  for (const auto& spec : specs) {
    for (double mult : {0.5, 1.0, 2.0}) {
      double test_r = spec.nominal_radius_m * mult;
      fprintf(stderr, "%s r=%.0f, leg=%.0f: ", spec.model.c_str(), test_r, leg_length_m);
      auto row = RunRacetrackProbe(spec, test_r, leg_length_m, num_laps);
      if (row.crashed) fprintf(stderr, "CRASHED\n");
      else if (row.stopped) fprintf(stderr, "STOPPED\n");
      else fprintf(stderr, "ok (err=%.1f ratio=%.2f)\n", row.avg_err_m, row.cross_track_error_ratio);
      WriteRow(out, row);
    }
  }

  if (out_path) fclose(out);
  fprintf(stderr, "Done.\n");
  return 0;
}
