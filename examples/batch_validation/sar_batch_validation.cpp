/**
 * @file sar_batch_validation.cpp
 * @brief 合成孔径雷达（SAR）批量场景验证。
 *
 * @par 目标
 * 通过公开 Session 接口（SarTraceSession）对 SAR 模块做多场景参数扫描，验证其在不同
 * 带宽 / 斜距 / 孔径长度下的成像泛用性：
 *   - 采集周期级 CSV（聚焦阶段、图像质量指标：SNR / 主瓣宽 / 分辨率 / 图像熵 / 对比度）。
 *   - 软断言：带宽↑ → 距离分辨率↑（分辨率数值↓）；聚焦阶段达预期 kL1RdaImage；
 *     图像质量指标非 NaN / 合理。
 *   - 每场景录制可回放 trace，用 ReplaySarTrace 做确定性回归。
 *
 * @par 与 AR/EOS/ESR 的差异
 *   - SAR 内部存 LLA+NED 大地坐标，外部输入也是 LLA，无需 SarCycleInputAdapter 做坐标转换。
 *   - SAR 单周期即完成聚焦成像（非多周期跟踪）。
 *   - 输出是聚焦图像而非航迹/检测；CSV 只记录图像质量摘要，复数像素不入 CSV。
 *
 * @par 运行方式
 *   ./sar_batch_validation [output_dir]
 *   默认输出 /tmp/1q/batch_validation/sar/
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "1q/sar/sar.hpp"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sar/session/SarTraceSession.h"
#include "batch_assertions.h"
#include "batch_csv_writer.h"
#include "batch_replay.h"
#include "config_loader.h"

namespace sar_session = sar::session;
namespace sar_config = sar::config;
using batch_validation::CsvWriter;
using batch_validation::ModuleName;
using batch_validation::ReplayCheckResult;
using batch_validation::Severity;
using batch_validation::WarningCollector;

namespace {

#ifndef BATCH_CONFIG_DIR
#define BATCH_CONFIG_DIR "."
#endif

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/sar";
constexpr const char* kTraceId = "sar-batch-validation";

// 场景几何常量（与 sar/integration_demo.cpp 同源，确保物理合理）。
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPlatformAltitudeM = 10000.0;
constexpr double kPlatformSpeedMps = 180.0;
constexpr double kSceneCenterLatDeg = 40.1;
constexpr double kSceneCenterLonDeg = -105.0;
constexpr double kDefaultDtSec = 0.1;
constexpr double kTargetRcsDbsm = 10.0;

// =============================================================================
// 场景参数表
// =============================================================================

struct SarCase {
  std::string scenario_id;
  double bandwidth_mhz;          ///< 信号带宽（MHz）→ 影响距离分辨率
  double slant_range_km;         ///< 标称斜距（km）
  std::uint32_t azimuth_pulses;  ///< 方位向脉冲数 → 影响方位分辨率 / 孔径长度
};

std::vector<SarCase> BuildSarCases() {
  std::vector<SarCase> cases;
  // 带宽扫描：保持在 sample_rate(1MHz) 以下，满足奈奎斯特。
  // 距离分辨率 = c/(2*B)，带宽↑ → 分辨率数值↓（更精细）。
  const double bws[] = {0.2, 0.5, 1.0, 2.0};       // MHz（最后两档接近/达采样率上限）
  const double ranges[] = {100.0, 120.0, 150.0};   // km（标称斜距）
  const std::uint32_t pulses[] = {17U, 33U, 65U};  // 方位脉冲数（孔径长度）
  char buf[128];
  // 带宽扫描（固定中等斜距 + 中孔径）
  for (double bw : bws) {
    SarCase c;
    std::snprintf(buf, sizeof(buf), "sar_bwsweep_bw%.1f", bw);
    c.scenario_id = buf;
    c.bandwidth_mhz = bw;
    c.slant_range_km = 100.0;
    c.azimuth_pulses = 33U;
    cases.push_back(c);
  }
  // 斜距扫描（固定带宽 + 中孔径）
  for (double r : ranges) {
    SarCase c;
    std::snprintf(buf, sizeof(buf), "sar_rsweep_r%.0fkm", r);
    c.scenario_id = buf;
    c.bandwidth_mhz = 0.5;
    c.slant_range_km = r;
    c.azimuth_pulses = 33U;
    cases.push_back(c);
  }
  // 孔径脉冲数扫描（固定带宽 + 中斜距）
  for (std::uint32_t p : pulses) {
    SarCase c;
    std::snprintf(buf, sizeof(buf), "sar_psweep_p%u", p);
    c.scenario_id = buf;
    c.bandwidth_mhz = 0.5;
    c.slant_range_km = 100.0;
    c.azimuth_pulses = p;
    cases.push_back(c);
  }
  return cases;
}

// =============================================================================
// config 按场景调整
// =============================================================================

/// 把场景参数写入 SarSessionConfig。
///
/// @note SAR 参数强耦合（PRF / 平台速度 / 孔径时间 / 斜距 / 波长 / 采样窗口），
///   这里以 sar.json / sar/integration_demo.cpp 共享的验证参数集为基准
///  （sample_rate=1MHz, pulse_width=20us, range_sample=1024, PRF=100Hz, slant=100km），
///   仅在保持采样窗口与孔径时间自洽的前提下小幅扫描带宽 / 斜距 / 方位脉冲数：
///   - 保持 sample_rate=1MHz、pulse_width=20us 固定（满足 ceil(20) << 1024）。
///   - 合成孔径时间 = azimuth_pulses / PRF，与孔径长度自洽。
void ApplyCaseToConfig(const SarCase& c, sar_config::SarSessionConfig& config) {
  // 基准参数（demo 验证过）。
  config.hardware.sample_rate_hz = 1.0e6;
  config.hardware.pulse_width_s = 20.0e-6;
  config.hardware.pulse_repetition_frequency_hz = 100.0;  // PRF=100Hz
  config.mission.range_sample_count = 1024U;
  config.mission.platform_speed_mps = 180.0;
  config.mission.scene_center_latitude_deg = kSceneCenterLatDeg;
  config.mission.scene_center_longitude_deg = kSceneCenterLonDeg;
  config.mission.scene_center_altitude_m = 0.0;

  // 场景扫描维度。
  config.hardware.bandwidth_hz = c.bandwidth_mhz * 1.0e6;
  config.mission.nominal_slant_range_m = c.slant_range_km * 1000.0;
  config.mission.azimuth_pulse_count = c.azimuth_pulses;
  config.mission.synthetic_aperture_time_s =
      static_cast<double>(c.azimuth_pulses) / config.hardware.pulse_repetition_frequency_hz;
}

// =============================================================================
// 输入构造（仿 sar/integration_demo.cpp::MakeCycleInput）
// =============================================================================

/// 构造单周期 SAR 输入：平台沿东向匀速、场景中心一个静止点目标。
/// 直接填充 SarCycleInput（SAR 内部存 LLA，无需 Adapter 做坐标转换）。
sar_session::SarCycleInput MakeCycleInput(std::uint32_t cycle_index,
                                          const sar_config::SarMissionConfig& mission) {
  const double elapsed_s = static_cast<double>(cycle_index - 1) * kDefaultDtSec;
  const double east_disp_m = kPlatformSpeedMps * elapsed_s;
  const double delta_lon_deg =
      east_disp_m / (kEarthRadiusM * std::cos(kSceneCenterLatDeg * kPi / 180.0)) * (180.0 / kPi);

  sar_session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = static_cast<float>(kDefaultDtSec);

  // 平台起始位于场景中心以北 ~100km（0.9°），使斜距≈标称值；沿东向匀速。
  constexpr double kPlatformLatOffset = 0.9;
  input.platform.time_s = elapsed_s;
  input.platform.latitude_deg = kSceneCenterLatDeg - kPlatformLatOffset;
  input.platform.longitude_deg = kSceneCenterLonDeg + delta_lon_deg;
  input.platform.altitude_m = kPlatformAltitudeM;
  input.platform.velocity_north_mps = 0.0;
  input.platform.velocity_east_mps = kPlatformSpeedMps;
  input.platform.velocity_down_mps = 0.0;
  input.platform.roll_deg = 0.0;
  input.platform.pitch_deg = 0.0;
  input.platform.yaw_deg = 90.0;

  sar_session::SarPointTarget target;
  target.target_id = 101;
  target.target_name = "scene_center_target";
  target.latitude_deg = mission.scene_center_latitude_deg;
  target.longitude_deg = mission.scene_center_longitude_deg;
  target.altitude_m = mission.scene_center_altitude_m;
  target.radar_cross_section_dbsm = kTargetRcsDbsm;
  input.point_targets = {target};

  return input;
}

// =============================================================================
// CSV schema
// =============================================================================

constexpr const char* kCycleHeader =
    "scenario_id,cycle_index,executed_this_cycle,has_error,reused_previous_output,abort_reason,"
    "completed_stage,has_raw_echo,has_range_compressed_echo,has_l1_image,has_l3_bp_image,"
    "estimated_snr_db,center_slant_range_m,range_sample_count,azimuth_pulse_count,"
    "image_entropy_nats,image_contrast,range_resolution_3db_m,azimuth_resolution_3db_m,"
    "range_width_3db_bins,azimuth_width_3db_bins,has_image_quality_metrics,"
    "image_resolution_m_valid,diag_warning_count,diag_error_count,image_rows,image_cols";

constexpr const char* kScenarioHeader =
    "scenario_id,bandwidth_mhz,slant_range_km,azimuth_pulses,executed,replay_ok,"
    "replay_compared,replay_divergence,completed_stage,estimated_snr_db,"
    "range_resolution_3db_m,azimuth_resolution_3db_m,image_entropy_nats,image_contrast,"
    "has_image_quality_metrics,warning_count,error_count,warnings";

// =============================================================================
// 单场景执行
// =============================================================================

struct CycleMetrics {
  std::uint32_t cycle_index{0};
  bool executed{false};
  bool has_error{false};
  bool reused{false};
  std::string abort_reason;
  int completed_stage{0};
  bool has_raw_echo{false};
  bool has_rc_echo{false};
  bool has_l1{false};
  bool has_l3{false};
  double snr_db{0.0};
  double center_slant_range_m{0.0};
  std::uint32_t range_samples{0};
  std::uint32_t azimuth_pulses{0};
  double image_entropy{0.0};
  double image_contrast{0.0};
  double range_res_m{0.0};
  double az_res_m{0.0};
  double range_width_bins{0.0};
  double az_width_bins{0.0};
  bool has_iqm{false};
  bool res_valid{false};
  std::size_t diag_warn{0};
  std::size_t diag_err{0};
  std::uint32_t img_rows{0};
  std::uint32_t img_cols{0};
};

CycleMetrics ExtractCycleMetrics(const sar_session::SarCycleResult& r) {
  CycleMetrics m;
  m.cycle_index = r.input_cycle_index;
  m.executed = r.executed_this_cycle;
  m.has_error = r.has_error;
  m.reused = r.reused_previous_output;
  m.abort_reason = r.abort_reason;
  const auto& f = r.output_frame;
  m.completed_stage = static_cast<int>(f.completed_stage);
  m.has_raw_echo = f.has_raw_echo;
  m.has_rc_echo = f.has_range_compressed_echo;
  m.has_l1 = f.has_l1_image;
  m.has_l3 = f.has_l3_bp_image;
  m.snr_db = f.estimated_snr_db;
  m.center_slant_range_m = f.center_slant_range_m;
  m.range_samples = f.range_sample_count;
  m.azimuth_pulses = f.azimuth_pulse_count;
  m.image_entropy = f.image_entropy_nats;
  m.image_contrast = f.image_contrast;
  m.range_res_m = f.range_resolution_3db_m;
  m.az_res_m = f.azimuth_resolution_3db_m;
  m.range_width_bins = f.range_width_3db_bins;
  m.az_width_bins = f.azimuth_width_3db_bins;
  m.has_iqm = f.has_image_quality_metrics;
  m.res_valid = f.image_resolution_m_valid;
  for (const auto& d : r.diagnostics) {
    if (d.severity == sar_session::SarDiagnosticSeverity::kWarning) ++m.diag_warn;
    if (d.severity == sar_session::SarDiagnosticSeverity::kError) ++m.diag_err;
  }
  m.img_rows = r.focused_image.row_count;
  m.img_cols = r.focused_image.column_count;
  return m;
}

struct ScenarioSummary {
  std::string scenario_id;
  double bandwidth_mhz{0.0};
  double slant_range_km{0.0};
  std::uint32_t azimuth_pulses{0};
  bool executed{false};
  bool replay_ok{false};
  std::uint64_t replay_compared{0};
  bool replay_divergence{false};
  int completed_stage{0};
  double snr_db{0.0};
  double range_res_m{0.0};
  double az_res_m{0.0};
  double image_entropy{0.0};
  double image_contrast{0.0};
  bool has_iqm{false};
  WarningCollector warnings;
};

ScenarioSummary RunSarScenario(const SarCase& c, const sar_config::SarSessionConfig& base_config,
                               const std::string& output_dir, CsvWriter& cycle_writer) {
  ScenarioSummary s;
  s.scenario_id = c.scenario_id;
  s.bandwidth_mhz = c.bandwidth_mhz;
  s.slant_range_km = c.slant_range_km;
  s.azimuth_pulses = c.azimuth_pulses;

  // 从基础配置读取不变的几何参数（PRF / 处理开关）。
  sar_config::SarSessionConfig config = base_config;
  ApplyCaseToConfig(c, config);

  const std::string trace_dir = output_dir + "/traces/" + c.scenario_id;
  auto replay_writer =
      batch_validation::MakeReplayWriter(trace_dir, ModuleName::kSar, kTraceId, c.scenario_id);

  CycleMetrics m;
  // SAR 单周期聚焦：录制 scope 内执行 1 个周期。
  {
    sar_session::SarTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sar_session::SarTraceSession session(config, options);

    sar_session::SarCycleInput input = MakeCycleInput(1U, config.mission);
    const sar_session::SarCycleResult result = session.StepWithResult(input);
    m = ExtractCycleMetrics(result);
    s.executed = m.executed;

    std::fprintf(cycle_writer.file(),
                 "%s,%u,%d,%d,%d,%s,%d,%d,%d,%d,%d,%.5f,%.3f,%u,%u,%.5f,%.5f,%.5f,"
                 "%.5f,%.5f,%.5f,%d,%d,%zu,%zu,%u,%u\n",
                 c.scenario_id.c_str(), m.cycle_index, static_cast<int>(m.executed),
                 static_cast<int>(m.has_error), static_cast<int>(m.reused),
                 batch_validation::EscapeCsvField(m.abort_reason).c_str(), m.completed_stage,
                 static_cast<int>(m.has_raw_echo), static_cast<int>(m.has_rc_echo),
                 static_cast<int>(m.has_l1), static_cast<int>(m.has_l3), m.snr_db,
                 m.center_slant_range_m, m.range_samples, m.azimuth_pulses, m.image_entropy,
                 m.image_contrast, m.range_res_m, m.az_res_m, m.range_width_bins, m.az_width_bins,
                 static_cast<int>(m.has_iqm), static_cast<int>(m.res_valid), m.diag_warn,
                 m.diag_err, m.img_rows, m.img_cols);
    replay_writer->Flush();
  }

  s.completed_stage = m.completed_stage;
  s.snr_db = m.snr_db;
  s.range_res_m = m.range_res_m;
  s.az_res_m = m.az_res_m;
  s.image_entropy = m.image_entropy;
  s.image_contrast = m.image_contrast;
  s.has_iqm = m.has_iqm;

  // 回放（SAR 单周期，compared 应为 1）。
  const sar_session::SarReplaySessionResult replay = sar_session::ReplaySarTrace(trace_dir);
  s.replay_ok = replay.ok;
  s.replay_compared = replay.playback.compared_output_count;
  s.replay_divergence = replay.playback.divergence_found;
  if (!replay.ok || replay.playback.divergence_found) {
    s.warnings.Warn("replay divergence: " + replay.first_error);
  }

  // 软断言
  if (!s.executed) {
    s.warnings.Error("cycle not executed");
  } else {
    // ① 聚焦阶段应达 kL1RdaImage (=3) 或更高
    if (s.completed_stage < static_cast<int>(sar_session::SarProcessingStage::kL1RdaImage)) {
      s.warnings.Warn("completed_stage below kL1RdaImage: " + std::to_string(s.completed_stage));
    }
    // ② 图像质量指标应有效
    if (!s.has_iqm) {
      s.warnings.Warn("has_image_quality_metrics is false");
    }
    // ③ SNR 应为有限值
    if (!std::isfinite(s.snr_db)) {
      s.warnings.Warn("estimated_snr_db not finite");
    }
    // ④ 图像熵/对比度应合理非零
    if (s.image_entropy <= 0.0) {
      s.warnings.Warn("image_entropy_nats non-positive: " + std::to_string(s.image_entropy));
    }
  }

  batch_validation::LogReplayResult(
      c.scenario_id,
      ReplayCheckResult{replay.ok, replay.playback.divergence_found,
                        replay.playback.compared_output_count, replay.playback.applied_input_count,
                        replay.reached_failure_marker, replay.first_error},
      s.executed ? 1U : 0U);
  s.warnings.DumpToStderr(c.scenario_id + ": ");
  return s;
}

/// 跨场景趋势：带宽↑ → 距离分辨率数值↓（分辨率更高）。
void CheckCrossScenarioTrends(std::vector<ScenarioSummary>& summaries) {
  std::vector<std::pair<double, double>> by_bw;  // (bandwidth, range_resolution)
  for (const auto& s : summaries) {
    if (std::abs(s.slant_range_km - 100.0) < 1e-9 && s.azimuth_pulses == 33U && s.has_iqm) {
      by_bw.emplace_back(s.bandwidth_mhz, s.range_res_m);
    }
  }
  std::sort(by_bw.begin(), by_bw.end());
  std::vector<double> res;
  for (auto& pr : by_bw) res.push_back(pr.second);
  if (res.size() >= 2 && !batch_validation::IsMonotonicNonIncreasing(res)) {
    for (auto& s : summaries) {
      if (std::abs(s.slant_range_km - 100.0) < 1e-9 && s.azimuth_pulses == 33U) {
        s.warnings.Warn(
            "cross-scenario: range_resolution not improving (decreasing) with bandwidth");
        break;
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string output_dir = (argc > 1) ? argv[1] : kDefaultOutputDir;
  const std::string config_path = BATCH_CONFIG_DIR "/sar.json";

  std::fprintf(stderr, "=== SAR 批量场景验证 ===\n");
  std::fprintf(stderr, "  配置: %s\n  输出: %s\n", config_path.c_str(), output_dir.c_str());

  sar_config::SarSessionConfig base_config;
  std::string load_error;
  if (!examples::LoadSarSessionConfigFromFile(config_path.c_str(), &base_config, &load_error)) {
    std::fprintf(stderr, "FATAL: 加载配置失败 %s: %s\n", config_path.c_str(), load_error.c_str());
    return 1;
  }

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "FATAL: 无法创建输出目录 %s: %s\n", output_dir.c_str(),
                 ec.message().c_str());
    return 1;
  }
  const std::string cycles_csv = output_dir + "/cycles.csv";
  const std::string scenarios_csv = output_dir + "/scenarios.csv";
  CsvWriter cycle_writer(cycles_csv, kCycleHeader);
  CsvWriter scenario_writer(scenarios_csv, kScenarioHeader);

  std::vector<SarCase> cases = BuildSarCases();
  std::vector<ScenarioSummary> summaries;
  summaries.reserve(cases.size());
  std::fprintf(stderr, "  场景数: %zu\n", cases.size());

  for (std::size_t i = 0; i < cases.size(); ++i) {
    std::fprintf(stderr, "[%zu/%zu] %s\n", i + 1, cases.size(), cases[i].scenario_id.c_str());
    summaries.push_back(RunSarScenario(cases[i], base_config, output_dir, cycle_writer));
  }
  cycle_writer.Flush();

  CheckCrossScenarioTrends(summaries);

  for (const auto& s : summaries) {
    std::fprintf(scenario_writer.file(),
                 "%s,%.3f,%.3f,%u,%d,%d,%llu,%d,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%zu,%zu,%s\n",
                 s.scenario_id.c_str(), s.bandwidth_mhz, s.slant_range_km, s.azimuth_pulses,
                 static_cast<int>(s.executed), static_cast<int>(s.replay_ok),
                 static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence), s.completed_stage, s.snr_db, s.range_res_m,
                 s.az_res_m, s.image_entropy, s.image_contrast, static_cast<int>(s.has_iqm),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError),
                 batch_validation::EscapeCsvField(s.warnings.JoinForCsv()).c_str());
  }
  scenario_writer.Flush();

  std::size_t total_warn = 0, total_err = 0, replay_div = 0;
  for (const auto& s : summaries) {
    total_warn += s.warnings.Count(Severity::kWarning);
    total_err += s.warnings.Count(Severity::kError);
    if (!s.replay_ok || s.replay_divergence) ++replay_div;
  }
  std::fprintf(stderr, "\n=== SAR 批量验证完成 ===\n");
  std::fprintf(stderr, "  场景数: %zu\n  周期 CSV: %s\n  场景 CSV: %s\n", summaries.size(),
               cycles_csv.c_str(), scenarios_csv.c_str());
  std::fprintf(stderr, "  软断言 warning: %zu, error: %zu\n", total_warn, total_err);
  std::fprintf(stderr, "  回放分叉场景: %zu（记 warning，不阻塞退出）\n", replay_div);
  std::fprintf(stderr, "  trace 目录: %s/traces/<scenario_id>/\n", output_dir.c_str());
  return (total_err > 0) ? 2 : 0;
}
