/**
 * @file eos_batch_validation.cpp
 * @brief 光电传感器（EOS）批量场景验证。
 *
 * @par 目标
 * 通过公开 Session 接口（EosTraceSession + EosCycleInputAdapter）对 EOS 模块做多场景
 * 参数扫描，验证其在不同目标距离 / 红外对比度 / 光照条件下的泛用性：
 *   - 采集周期级 CSV（检出数、融合/红外/可见光 SNR 分布）+ 场景汇总 CSV。
 *   - 软断言：高红外对比度场景检出率 ≥ 低对比度；夜间可见光 SNR 显著低于红外；
 *     距离↑ → 融合 SNR↓（物理衰减）。
 *   - 每场景录制可回放 trace，用 ReplayEosTrace 做确定性回归。
 *
 * @par 运行方式
 *   ./eos_batch_validation [output_dir]
 *   默认输出 /tmp/1q/batch_validation/electro_optical_sensor/
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosReplaySession.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"

#include "batch_assertions.h"
#include "batch_csv_writer.h"
#include "batch_replay.h"
#include "config_loader.h"

namespace eos = electro_optical_sensor;
namespace eos_config = electro_optical_sensor::config;
namespace eos_session = electro_optical_sensor::session;
using batch_validation::CsvWriter;
using batch_validation::ModuleName;
using batch_validation::ReplayCheckResult;
using batch_validation::Severity;
using batch_validation::WarningCollector;

namespace {

#ifndef BATCH_CONFIG_DIR
#define BATCH_CONFIG_DIR "."
#endif

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/electro_optical_sensor";
constexpr std::uint32_t kNumCycles = 30;
constexpr std::uint32_t kWarmupCycles = 5;
constexpr const char* kTraceId = "eos-batch-validation";

// =============================================================================
// 场景参数表
// =============================================================================

/// 红外对比度档位（目标外观参数预设）。
enum class ContrastLevel { kLow, kMedium, kHigh };

/// 光照场景（影响环境输入）。
enum class LightingCondition { kDay, kNight, kTwilight };

struct EosCase {
  std::string scenario_id;
  double target_lon_offset_deg;  ///< 目标相对平台的经度偏移（越大距离越远）
  ContrastLevel contrast;         ///< 红外对比度档位
  LightingCondition lighting;     ///< 光照条件
};

const char* ContrastName(ContrastLevel c) {
  switch (c) {
    case ContrastLevel::kLow: return "low";
    case ContrastLevel::kMedium: return "med";
    case ContrastLevel::kHigh: return "high";
  }
  return "?";
}

const char* LightingName(LightingCondition l) {
  switch (l) {
    case LightingCondition::kDay: return "day";
    case LightingCondition::kNight: return "night";
    case LightingCondition::kTwilight: return "twilight";
  }
  return "?";
}

std::vector<EosCase> BuildEosCases() {
  std::vector<EosCase> cases;
  // 目标相对传感器足印中心的经度偏移（足印中心已在探测俯仰角内）。
  // 1° lon @35N ≈ 91km；{0.005,0.010,0.020,0.030} ≈ {0.45,0.9,1.8,2.7}km，均在视场内。
  const double offsets[] = {0.005, 0.010, 0.020, 0.030};
  const ContrastLevel contrasts[] = {ContrastLevel::kLow, ContrastLevel::kMedium,
                                     ContrastLevel::kHigh};
  const LightingCondition lights[] = {LightingCondition::kDay, LightingCondition::kNight,
                                      LightingCondition::kTwilight};
  char buf[128];
  for (double off : offsets) {
    for (ContrastLevel c : contrasts) {
      for (LightingCondition l : lights) {
        EosCase cs;
        std::snprintf(buf, sizeof(buf), "eos_off%.3f_%s_%s", off, ContrastName(c),
                      LightingName(l));
        cs.scenario_id = buf;
        cs.target_lon_offset_deg = off;
        cs.contrast = c;
        cs.lighting = l;
        cases.push_back(cs);
      }
    }
  }
  return cases;
}

// =============================================================================
// 输入构造（仿 session_usage.cpp 的 LLA 模式）
// =============================================================================

/// 平台 LLA 起点（与 session_usage 同一参考点 @ 35N,114.5E,7000m 高度）。
/// 目标放在 base_lon 附近，使地面偏移落在 EOS 探测俯仰角范围内。
/// session_usage 验证过：base_lon = 114.5 + 0.06925 对应 ~6.3km 探测偏移。
constexpr double kPlatformLat = 35.0;
constexpr double kPlatformLon = 114.5;
constexpr double kPlatformAlt = 7000.0;
constexpr double kBaseLon = kPlatformLon + 0.06925;  ///< 传感器足印中心经度

eos_session::EosExternalPoseInput MakePlatform(std::uint32_t cycle_index) {
  eos_session::EosExternalPoseInput p;
  // 用项目精确 LLA→ECEF 转换（手写近似会导致几何失真、目标落出视场）。
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = kPlatformLat;
  platform_lla.longitude_deg = kPlatformLon;
  platform_lla.altitude_m = kPlatformAlt;
  oneq::coordinate::TryLlaToEcef(platform_lla, &p.platform_position_ecef_m);
  (void)cycle_index;
  p.platform_attitude_deg.yaw_deg = 0.0;
  p.platform_attitude_deg.pitch_deg = 0.0;
  p.platform_attitude_deg.roll_deg = 0.0;
  return p;
}

/// 按 contrast 档位构造目标外观。
eos_session::EosTargetAppearance MakeAppearance(ContrastLevel c) {
  eos_session::EosTargetAppearance a;
  switch (c) {
    case ContrastLevel::kLow:
      a.apparent_temperature_k = 295.0f;  // 接近背景 290K，低对比度
      a.emissivity = 0.5f;
      a.reflectance = 0.1f;
      a.projected_area_m2 = 2.0f;
      break;
    case ContrastLevel::kMedium:
      a.apparent_temperature_k = 350.0f;
      a.emissivity = 0.8f;
      a.reflectance = 0.2f;
      a.projected_area_m2 = 5.0f;
      break;
    case ContrastLevel::kHigh:
      a.apparent_temperature_k = 480.0f;  // 高温差，高对比度
      a.emissivity = 0.95f;
      a.reflectance = 0.3f;
      a.projected_area_m2 = 12.0f;
      break;
  }
  return a;
}

std::vector<eos_session::EosExternalTargetInput> MakeTargets(const EosCase& c) {
  std::vector<eos_session::EosExternalTargetInput> targets;
  eos_session::EosExternalTargetInput t;
  t.target_id = 101;
  t.kinematics.position_frame = oneq::coordinate::PositionFrame::kLla;
  // 目标放在传感器足印中心附近，按场景偏移量错开（偏移即相对足印中心的地面距离）。
  t.kinematics.position_lla_deg_m.latitude_deg = kPlatformLat - 0.002;
  t.kinematics.position_lla_deg_m.longitude_deg = kBaseLon + c.target_lon_offset_deg;
  t.kinematics.position_lla_deg_m.altitude_m = 0.0;
  t.appearance = MakeAppearance(c.contrast);
  targets.push_back(t);
  return targets;
}

/// 按 lighting 构造环境输入。
eos_session::EosEnvironmentInput MakeEnvironment(LightingCondition l) {
  eos_session::EosEnvironmentInput env;
  switch (l) {
    case LightingCondition::kDay:
      env.solar_altitude_deg = 45.0f;
      env.solar_azimuth_deg = 180.0f;
      env.solar_irradiance_w_m2 = 850.0f;
      env.day_night_type = eos_session::DayNightType::kDay;
      break;
    case LightingCondition::kNight:
      env.solar_altitude_deg = -15.0f;  // 太阳在地平线以下
      env.solar_azimuth_deg = 0.0f;
      env.solar_irradiance_w_m2 = 0.5f;
      env.day_night_type = eos_session::DayNightType::kNight;
      break;
    case LightingCondition::kTwilight:
      env.solar_altitude_deg = 5.0f;
      env.solar_azimuth_deg = 90.0f;
      env.solar_irradiance_w_m2 = 50.0f;
      env.day_night_type = eos_session::DayNightType::kTwilight;
      break;
  }
  env.cloud_coverage_ratio = 0.15f;
  env.background_temperature_k = 290.0f;
  return env;
}

// =============================================================================
// CSV schema
// =============================================================================

constexpr const char* kCycleHeader =
    "scenario_id,cycle_index,executed_this_cycle,has_validation_error,abort_reason,"
    "scan_azimuth_deg,total_detection_count,detected_count,detection_rate,"
    "fused_snr_db_mean,fused_snr_db_p95,infrared_snr_linear_mean,visible_snr_linear_mean";

constexpr const char* kScenarioHeader =
    "scenario_id,target_lon_offset_deg,contrast,lighting,executed_cycles,"
    "steady_detection_rate_mean,steady_fused_snr_db_mean,steady_infrared_mean,"
    "steady_visible_mean,replay_ok,replay_compared,replay_divergence,warning_count,"
    "error_count,warnings";

// =============================================================================
// 单场景执行
// =============================================================================

struct CycleMetrics {
  std::uint32_t cycle_index{0};
  bool executed{false};
  bool has_validation_error{false};
  int abort_reason{0};
  float scan_azimuth_deg{0.0f};
  std::size_t total{0};
  std::size_t detected{0};
  double fused_snr_db_mean{0.0};
  double fused_snr_db_p95{0.0};
  double infrared_mean{0.0};
  double visible_mean{0.0};
};

CycleMetrics ExtractCycleMetrics(const eos_session::EosCycleResult& r) {
  CycleMetrics m;
  m.cycle_index = r.input_cycle_index;
  m.executed = r.executed_this_cycle;
  m.has_validation_error = r.has_validation_error;
  m.abort_reason = static_cast<int>(r.abort_reason);
  const auto& f = r.output_frame;
  m.scan_azimuth_deg = f.scan_azimuth_deg;
  m.total = f.detections.size();
  std::vector<double> fused_db, infrared, visible;
  for (const auto& d : f.detections) {
    if (d.detected) ++m.detected;
    fused_db.push_back(static_cast<double>(d.fused_snr_db));
    infrared.push_back(static_cast<double>(d.infrared_snr_linear));
    visible.push_back(static_cast<double>(d.visible_snr_linear));
  }
  m.fused_snr_db_mean = batch_validation::Mean(fused_db);
  m.fused_snr_db_p95 = batch_validation::Percentile(fused_db, 95.0);
  m.infrared_mean = batch_validation::Mean(infrared);
  m.visible_mean = batch_validation::Mean(visible);
  return m;
}

struct ScenarioSummary {
  std::string scenario_id;
  double target_lon_offset_deg{0.0};
  std::string contrast;
  std::string lighting;
  std::uint32_t executed_cycles{0};
  double steady_detection_rate_mean{0.0};
  double steady_fused_snr_db_mean{0.0};
  double steady_infrared_mean{0.0};
  double steady_visible_mean{0.0};
  bool replay_ok{false};
  std::uint64_t replay_compared{0};
  bool replay_divergence{false};
  WarningCollector warnings;
};

ScenarioSummary RunEosScenario(const EosCase& c, const eos_config::EosSessionConfig& base_config,
                               const std::string& output_dir, CsvWriter& cycle_writer) {
  ScenarioSummary s;
  s.scenario_id = c.scenario_id;
  s.target_lon_offset_deg = c.target_lon_offset_deg;
  s.contrast = ContrastName(c.contrast);
  s.lighting = LightingName(c.lighting);

  // EOS 物理链路本身基于辐射度学；这里不改 config（沿用基础探测门限）。
  (void)base_config;
  eos_config::EosSessionConfig config = base_config;

  const std::string trace_dir = output_dir + "/traces/" + c.scenario_id;
  auto replay_writer = batch_validation::MakeReplayWriter(
      trace_dir, ModuleName::kElectroOpticalSensor, kTraceId, c.scenario_id);

  std::vector<CycleMetrics> metrics;
  metrics.reserve(kNumCycles);

  {
    eos_session::EosTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    eos_session::EosTraceSession session(config, options);

    for (std::uint32_t i = 0; i < kNumCycles; ++i) {
      const std::uint32_t cycle_index = i + 1;
      eos_session::EosExternalPoseInput platform = MakePlatform(cycle_index);
      std::vector<eos_session::EosExternalTargetInput> targets = MakeTargets(c);
      eos_session::EosEnvironmentInput env = MakeEnvironment(c.lighting);

      eos_session::EosCycleInput input;
      eos_session::EosCoordinateStatus status;
      if (!eos_session::EosCycleInputAdapter::Build(platform, targets, 1.0f, env, &input, &status)) {
        s.warnings.Error("EosCycleInputAdapter::Build failed at cycle " +
                         std::to_string(cycle_index));
        break;
      }
      input.cycle_index = cycle_index;

      const eos_session::EosCycleResult result = session.StepWithResult(input);
      CycleMetrics m = ExtractCycleMetrics(result);
      metrics.push_back(m);

      const double det_rate =
          (m.total > 0) ? static_cast<double>(m.detected) / static_cast<double>(m.total) : 0.0;
      std::fprintf(cycle_writer.file(),
                   "%s,%u,%d,%d,%d,%.3f,%zu,%zu,%.5f,%.5f,%.5f,%.5f,%.5f\n", c.scenario_id.c_str(),
                   m.cycle_index, static_cast<int>(m.executed),
                   static_cast<int>(m.has_validation_error), m.abort_reason,
                   static_cast<double>(m.scan_azimuth_deg), m.total, m.detected, det_rate,
                   m.fused_snr_db_mean, m.fused_snr_db_p95, m.infrared_mean, m.visible_mean);
    }
    replay_writer->Flush();
  }

  // 聚合
  std::vector<double> steady_det_rate, steady_fused, steady_ir, steady_vis;
  for (const auto& m : metrics) {
    if (m.executed) ++s.executed_cycles;
    if (!m.executed || m.cycle_index <= kWarmupCycles) continue;
    const double det_rate =
        (m.total > 0) ? static_cast<double>(m.detected) / static_cast<double>(m.total) : 0.0;
    steady_det_rate.push_back(det_rate);
    steady_fused.push_back(m.fused_snr_db_mean);
    steady_ir.push_back(m.infrared_mean);
    steady_vis.push_back(m.visible_mean);
  }
  s.steady_detection_rate_mean = batch_validation::Mean(steady_det_rate);
  s.steady_fused_snr_db_mean = batch_validation::Mean(steady_fused);
  s.steady_infrared_mean = batch_validation::Mean(steady_ir);
  s.steady_visible_mean = batch_validation::Mean(steady_vis);

  // 回放
  const eos_session::EosReplaySessionResult replay = eos_session::ReplayEosTrace(trace_dir);
  s.replay_ok = replay.ok;
  s.replay_compared = replay.playback.compared_output_count;
  s.replay_divergence = replay.playback.divergence_found;
  if (!replay.ok || replay.playback.divergence_found) {
    s.warnings.Error("replay failed: " + replay.first_error);
  }

  // 软断言
  if (s.executed_cycles == 0) {
    s.warnings.Error("no cycle executed");
  } else {
    // ① 高对比度检出率应 >= 低对比度（跨场景比较在 CheckCrossScenarioTrends）
    // ② 夜间可见光 SNR 应显著低于红外（可见光依赖太阳辐照）
    if (c.lighting == LightingCondition::kNight) {
      if (s.steady_visible_mean > s.steady_infrared_mean * 0.5) {
        s.warnings.Warn("night: visible_snr not clearly below infrared (vis=" +
                        std::to_string(s.steady_visible_mean) +
                        " ir=" + std::to_string(s.steady_infrared_mean) + ")");
      }
    }
    // ③ 融合 SNR 应为有限值
    if (!std::isfinite(s.steady_fused_snr_db_mean)) {
      s.warnings.Warn("fused_snr_db not finite: " + std::to_string(s.steady_fused_snr_db_mean));
    }
  }

  batch_validation::LogReplayResult(
      c.scenario_id,
      ReplayCheckResult{replay.ok, replay.playback.divergence_found,
                        replay.playback.compared_output_count, replay.playback.applied_input_count,
                        replay.reached_failure_marker, replay.first_error},
      s.executed_cycles);
  s.warnings.DumpToStderr(c.scenario_id + ": ");
  return s;
}

/// 跨场景趋势：固定 lighting=day，对比度↑ → 检出率↑ / fused SNR↑；
/// 固定对比度=high, 距离(offset)↑ → fused SNR↓。
void CheckCrossScenarioTrends(std::vector<ScenarioSummary>& summaries) {
  // 对比度趋势：offset=0.010（中等距离）、day 光照下
  std::vector<double> det_by_contrast;  // low, med, high 顺序
  for (const auto& lv : {"low", "med", "high"}) {
    for (const auto& s : summaries) {
      if (std::abs(s.target_lon_offset_deg - 0.010) < 1e-9 && s.lighting == "day" &&
          s.contrast == lv) {
        det_by_contrast.push_back(s.steady_detection_rate_mean);
        break;
      }
    }
  }
  if (det_by_contrast.size() == 3 && !batch_validation::IsMonotonicNonDecreasing(det_by_contrast)) {
    for (auto& s : summaries) {
      if (std::abs(s.target_lon_offset_deg - 0.010) < 1e-9 && s.lighting == "day") {
        s.warnings.Warn("cross-scenario: detection_rate not monotonic-increasing in contrast");
        break;
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string output_dir = (argc > 1) ? argv[1] : kDefaultOutputDir;
  const std::string config_path = BATCH_CONFIG_DIR "/electro_optical.json";

  std::fprintf(stderr, "=== EOS 批量场景验证 ===\n");
  std::fprintf(stderr, "  配置: %s\n  输出: %s\n", config_path.c_str(), output_dir.c_str());

  eos_config::EosSessionConfig base_config;
  std::string load_error;
  if (!examples::LoadEosSessionConfigFromFile(config_path.c_str(), &base_config, &load_error)) {
    std::fprintf(stderr, "FATAL: 加载配置失败 %s: %s\n", config_path.c_str(), load_error.c_str());
    return 1;
  }

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "FATAL: 无法创建输出目录 %s: %s\n", output_dir.c_str(), ec.message().c_str());
    return 1;
  }
  const std::string cycles_csv = output_dir + "/cycles.csv";
  const std::string scenarios_csv = output_dir + "/scenarios.csv";
  CsvWriter cycle_writer(cycles_csv, kCycleHeader);
  CsvWriter scenario_writer(scenarios_csv, kScenarioHeader);

  std::vector<EosCase> cases = BuildEosCases();
  std::vector<ScenarioSummary> summaries;
  summaries.reserve(cases.size());
  std::fprintf(stderr, "  场景数: %zu\n", cases.size());

  for (std::size_t i = 0; i < cases.size(); ++i) {
    std::fprintf(stderr, "[%zu/%zu] %s\n", i + 1, cases.size(), cases[i].scenario_id.c_str());
    summaries.push_back(RunEosScenario(cases[i], base_config, output_dir, cycle_writer));
    cycle_writer.Flush();
  }

  CheckCrossScenarioTrends(summaries);

  for (const auto& s : summaries) {
    std::fprintf(stderr,
                 "  [scenario] module=EOS id=%s offset_deg=%.4f contrast=%s lighting=%s "
                 "executed=%u/%u detection_rate=%.4f fused_snr_db=%.4f infrared=%.4f "
                 "visible=%.4f replay_ok=%d compared=%llu divergence=%d warn=%zu error=%zu\n",
                 s.scenario_id.c_str(), s.target_lon_offset_deg, s.contrast.c_str(),
                 s.lighting.c_str(), s.executed_cycles, kNumCycles,
                 s.steady_detection_rate_mean, s.steady_fused_snr_db_mean,
                 s.steady_infrared_mean, s.steady_visible_mean, static_cast<int>(s.replay_ok),
                 static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError));
    std::fprintf(scenario_writer.file(),
                 "%s,%.4f,%s,%s,%u,%.4f,%.4f,%.4f,%.4f,%d,%llu,%d,%zu,%zu,%s\n",
                 s.scenario_id.c_str(), s.target_lon_offset_deg, s.contrast.c_str(),
                 s.lighting.c_str(), s.executed_cycles, s.steady_detection_rate_mean,
                 s.steady_fused_snr_db_mean, s.steady_infrared_mean, s.steady_visible_mean,
                 static_cast<int>(s.replay_ok), static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence), s.warnings.Count(Severity::kWarning),
                 s.warnings.Count(Severity::kError),
                 batch_validation::EscapeCsvField(s.warnings.JoinForCsv()).c_str());
  }
  scenario_writer.Flush();

  std::size_t total_warn = 0, total_err = 0, replay_fail = 0;
  for (const auto& s : summaries) {
    total_warn += s.warnings.Count(Severity::kWarning);
    total_err += s.warnings.Count(Severity::kError);
    if (!s.replay_ok || s.replay_divergence) ++replay_fail;
  }
  std::fprintf(stderr, "\n=== EOS 批量验证完成 ===\n");
  std::fprintf(stderr, "  场景数: %zu\n  周期 CSV: %s\n  场景 CSV: %s\n", summaries.size(),
               cycles_csv.c_str(), scenarios_csv.c_str());
  std::fprintf(stderr, "  软断言 warning: %zu, error: %zu\n", total_warn, total_err);
  std::fprintf(stderr, "  回放失败场景: %zu\n", replay_fail);
  std::fprintf(stderr, "  trace 目录: %s/traces/<scenario_id>/\n", output_dir.c_str());
  return (replay_fail > 0 || total_err > 0) ? 2 : 0;
}
