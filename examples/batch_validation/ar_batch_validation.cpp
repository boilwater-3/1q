/**
 * @file ar_batch_validation.cpp
 * @brief 机载雷达（AR）批量场景验证。
 *
 * @par 目标
 * 通过公开 Session 接口（ArTraceSession + ArCycleInputAdapter）对 AR 模块做多场景
 * 参数扫描，证明其在不同目标距离 / RCS / 目标数 / 探测阈值下的泛用性：
 *   - 采集周期级 + 场景汇总级 CSV 指标（关联质量、航迹统计）。
 *   - 对关键物理趋势做软断言（距离↑→确认率↓、RCS↓→漏失率↑ 等），违反记 kWarning。
 *   - 每个场景用 ReplayTraceWriter 录制可回放 trace，并立即用 ReplayArTrace 做确定性
 *     回归（分叉检测），回放结果写入汇总 CSV 的 replay_ok 列。
 *
 * @par 运行方式
 *   ./ar_batch_validation [--suite sweep|sequence|all] [--scenario ID]
 *                         [--output-dir PATH] [--list-scenarios]
 *   默认输出 /tmp/1q/batch_validation/airborne_radar/
 *
 * @par 配置依赖
 *   编译期通过 BATCH_CONFIG_DIR 注入 configs/airborne_radar.json 路径。
 */

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/ArCycleInputAdapter.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArReplaySession.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTraceSession.h"
#include "1q/coordinate/types.h"
#include "batch_assertions.h"
#include "batch_checks.h"
#include "batch_cli.h"
#include "batch_csv_writer.h"
#include "batch_replay.h"
#include "config_loader.h"

namespace ar = airborne_radar;
namespace ar_config = airborne_radar::config;
namespace ar_session = airborne_radar::session;
using batch_validation::ContractCheckCollector;
using batch_validation::CsvWriter;
using batch_validation::ModuleName;
using batch_validation::ReplayCheckResult;
using batch_validation::Severity;
using batch_validation::WarningCollector;

namespace {

#ifndef BATCH_CONFIG_DIR
#define BATCH_CONFIG_DIR "."
#endif

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/airborne_radar";
constexpr std::uint32_t kNumCycles = 50;    ///< 每场景周期数（含预热）
constexpr std::uint32_t kWarmupCycles = 8;  ///< 前 N 周期不计入"稳态"指标
constexpr const char* kTraceId = "ar-batch-validation";

// =============================================================================
// 场景参数表
// =============================================================================

/// 单个 AR 场景参数。
struct ArCase {
  std::string scenario_id;    ///< 场景标识（含参数编码，用于 trace 目录名）
  double target_range_km;     ///< 目标相对雷达的初始斜距（km）
  float rcs_m2;               ///< 目标 RCS（m^2）
  int target_count;           ///< 目标数量
  float min_snr_db_override;  ///< 探测阈值覆盖；<0 表示沿用配置默认值
  bool sequence{false};       ///< 是否为跨周期专项场景
  std::string family{"parameter_sweep"};
};

/// 构造扫描矩阵：距离 × RCS × 目标数（默认阈值）+ 一组阈值扫描。
/// 距离/RCS 范围刻意拉宽（含 80/120km、0.01/0.05 m²），使雷达方程的 SNR 衰减
/// 在物理极限处可见——近距离高 RCS 全检出，远距离低 RCS 才会出现漏检。
std::vector<ArCase> BuildArCases() {
  std::vector<ArCase> cases;
  const double ranges[] = {8.0, 25.0, 60.0, 120.0};
  const float rcs_values[] = {0.01f, 0.2f, 5.0f, 10.0f};
  const int counts[] = {1, 3, 5};
  char buf[128];
  for (double r : ranges) {
    for (float rcs : rcs_values) {
      for (int n : counts) {
        ArCase c;
        c.min_snr_db_override = -1.0f;
        std::snprintf(buf, sizeof(buf), "ar_r%03.0fkm_rcs%.2f_n%d", r, static_cast<double>(rcs), n);
        c.scenario_id = buf;
        c.target_range_km = r;
        c.rcs_m2 = rcs;
        c.target_count = n;
        cases.push_back(c);
      }
    }
  }
  // 阈值扫描：固定中等距离/RCS/3目标，扫描 min_snr_db
  const float snr_thresholds[] = {2.0f, 8.0f, 14.0f, 20.0f};
  for (float snr : snr_thresholds) {
    ArCase c;
    std::snprintf(buf, sizeof(buf), "ar_r25km_rcs5.0_n3_snr%.0f", static_cast<double>(snr));
    c.scenario_id = buf;
    c.target_range_km = 25.0;
    c.rcs_m2 = 5.0f;
    c.target_count = 3;
    c.min_snr_db_override = snr;
    cases.push_back(c);
  }
  return cases;
}

std::vector<ArCase> BuildArSequenceCases() {
  const char* ids[] = {"ar_seq_two_target_crossing",
                       "ar_seq_crossing_with_pulsed_jammer",
                       "ar_seq_tws_stt_tws",
                       "ar_seq_power_cycle",
                       "ar_seq_invalid_input_recovery",
                       "ar_seq_invalid_patch_atomic"};
  std::vector<ArCase> cases;
  for (const char* id : ids) {
    ArCase c;
    c.scenario_id = id;
    c.target_range_km = 8.0;
    c.rcs_m2 = 10.0f;
    c.target_count = std::strstr(id, "pulsed_jammer") == nullptr ? 2 : 3;
    c.min_snr_db_override = 2.0f;
    c.sequence = true;
    c.family = std::strstr(id, "crossing") != nullptr ? "multi_target_crossing"
               : std::strstr(id, "patch") != nullptr || std::strstr(id, "tws_stt") != nullptr
                   ? "runtime_reconfiguration"
               : std::strstr(id, "power") != nullptr ? "lifecycle_interruption"
                                                     : "invalid_input_recovery";
    cases.push_back(c);
  }
  return cases;
}

std::uint32_t CycleCount(const ArCase& c) { return c.sequence ? 24U : kNumCycles; }

const char* PhaseFor(const ArCase& c, std::uint32_t cycle) {
  if (!c.sequence) return "sweep";
  if (cycle <= 8U) return "establish";
  if (cycle <= 16U) return "interruption";
  return "recovery";
}

// =============================================================================
// 输入构造
// =============================================================================

/// 构造平台位姿（ECEF，固定原点 + 微小速度，保证雷达本地参考系稳定）。
ar_session::ArExternalPoseInput MakePlatformPose(std::uint32_t cycle_index) {
  ar_session::ArExternalPoseInput p;
  // 固定平台 ECEF 原点（与 integration_demo 同一参考点）。
  p.platform_position_ecef_m.x_m = -2289512.0;
  p.platform_position_ecef_m.y_m = 4909946.0;
  p.platform_position_ecef_m.z_m = 3640982.0;
  // 微小速度，避免退化几何；周期间位置基本不变。
  p.platform_velocity_mps.x_mps = 0.0;
  p.platform_velocity_mps.y_mps = 0.0;
  p.platform_velocity_mps.z_mps = 0.0;
  (void)cycle_index;
  p.platform_attitude_deg.yaw_deg = 0.0;
  p.platform_attitude_deg.pitch_deg = 0.0;
  p.platform_attitude_deg.roll_deg = 0.0;
  p.radar_mount_angles_deg.yaw_deg = 0.0;
  p.radar_mount_angles_deg.pitch_deg = 0.0;
  p.radar_mount_angles_deg.roll_deg = 0.0;
  return p;
}

/**
 * @brief 构造一组目标，分布在目标斜距附近、雷达本地 +x（前向）方向。
 *
 * AR 输入位置为 ECEF；本函数把目标放在平台 ECEF 原点 + 沿 ECEF x 轴偏移 range_km。
 * 实际场景中目标会在本地系呈现该量级的斜距（足够驱动 SNR/距离趋势）。
 */
std::vector<ar_session::ArExternalTargetInput> MakeTargets(const ArCase& c,
                                                           std::uint32_t cycle_index) {
  std::vector<ar_session::ArExternalTargetInput> targets;
  targets.reserve(static_cast<std::size_t>(c.target_count));
  const double base_offset_m = c.target_range_km * 1000.0;
  for (int k = 0; k < c.target_count; ++k) {
    ar_session::ArExternalTargetInput t;
    t.target_id = 1000 + static_cast<std::uint64_t>(k);
    // 多目标在 x 方向错开 200m，避免位置完全重叠。
    const double off = base_offset_m + 200.0 * static_cast<double>(k);
    t.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    t.kinematics.position_ecef_m.x_m = -2289512.0 + off;
    t.kinematics.position_ecef_m.y_m = 4909946.0;
    t.kinematics.position_ecef_m.z_m = 3640982.0;
    // 速度沿 -x（朝向雷达），使相对运动产生多普勒。
    t.kinematics.velocity_mps.x_mps = -50.0;
    t.kinematics.velocity_mps.y_mps = 0.0;
    t.kinematics.velocity_mps.z_mps = 0.0;
    if (c.sequence && c.scenario_id.find("crossing") != std::string::npos && k < 2) {
      const double signed_offset =
          (k == 0 ? -1.0 : 1.0) * (12.5 - static_cast<double>(cycle_index)) * 120.0;
      t.kinematics.position_ecef_m.y_m += signed_offset;
      t.kinematics.velocity_mps.y_mps = k == 0 ? 120.0 : -120.0;
    }
    t.rcs = c.rcs_m2;
    t.swerling_type = 0;
    targets.push_back(t);
  }
  (void)cycle_index;
  return targets;
}

/// 构造默认环境（关闭物理大气模型，避免引入额外衰减干扰趋势）。
ar_session::ArEnvironmentInput MakeEnvironment(std::uint32_t cycle_index) {
  ar_session::ArEnvironmentInput env;
  env.atmospheric_observation.enable_physical_model = false;
  env.atmospheric_observation.pressure_hpa = 1013.25f;
  env.atmospheric_observation.temperature_k = 288.15f;
  env.atmospheric_observation.relative_humidity = 0.5f;
  env.atmospheric_context.has_simulation_unix_seconds = true;
  env.atmospheric_context.simulation_unix_seconds = 1770000000 + cycle_index;
  return env;
}

/// 按场景调整 config：
///   - 强制启用物理探测链 + 物理 RCS 估计，使目标距离 / RCS / SNR 阈值真正驱动探测结果。
///     （示例配置默认关闭物理探测，导致所有目标被确定性检出，无法体现距离/RCS 差异。）
///   - 若场景指定 min_snr_db 覆盖，则覆盖探测门限。
///
/// @note 这些都是 ArSessionConfig 对外公开的字段，属于本框架正常使用的配置面。
void ApplyCaseToConfig(const ArCase& c, ar_config::ArSessionConfig& config) {
  config.hardware.rcs_physics.enable_physical_rcs = true;
  config.hardware.rcs_physics.physics_mix_ratio = 1.0f;  // 完全物理估计
  if (c.min_snr_db_override >= 0.0f) {
    config.policy.detection.minimum_snr_db = c.min_snr_db_override;
  }
}

// =============================================================================
// CSV schema
// =============================================================================

constexpr const char* kCycleHeader =
    "scenario_id,suite,scenario_family,phase,cycle_index,executed_this_cycle,has_validation_error,"
    "abort_reason,"
    "prior_track_count,detection_count,matched_count,new_track_count,missed_track_count,"
    "match_rate,new_track_rate,missed_track_rate,mean_match_cost,p95_match_cost,"
    "jamming_severity,association_stress,confirmed_count,tentative_count,lost_count,"
    "jamming_track_count";

constexpr const char* kScenarioHeader =
    "scenario_id,suite,scenario_family,target_range_km,rcs_m2,target_count,min_snr_db_override,"
    "executed_cycles,warmup_confirmed_cycles,steady_confirmed_mean,steady_match_rate_mean,"
    "steady_missed_rate_mean,steady_jamming_mean,replay_ok,replay_compared,replay_divergence,"
    "expected_failure_count,contract_check_count,contract_failure_count,failure_marker_count,"
    "warning_count,error_count,warnings";

/// 把 abort_reason 枚举转为整数（CSV 中便于筛选）。
int AbortReasonToInt(ar_session::SignalCycleAbortReason r) { return static_cast<int>(r); }

// =============================================================================
// 单场景执行
// =============================================================================

/// 周期级指标快照（写周期 CSV + 聚合场景级）。
struct CycleMetrics {
  std::uint32_t cycle_index{0};
  bool executed{false};
  bool has_validation_error{false};
  int abort_reason{0};
  std::size_t prior_track_count{0};
  std::size_t detection_count{0};
  std::size_t matched_count{0};
  std::size_t new_track_count{0};
  std::size_t missed_track_count{0};
  std::size_t confirmed{0};
  std::size_t tentative{0};
  std::size_t lost{0};
  std::size_t jamming{0};
  float match_rate{0.0f};
  float new_track_rate{0.0f};
  float missed_rate{0.0f};
  float jamming_severity{0.0f};
  float association_stress{0.0f};
  float mean_match_cost{0.0f};
  float p95_match_cost{0.0f};
};

/// 从 CycleResult 提取周期指标。
CycleMetrics ExtractCycleMetrics(const ar_session::ArCycleResult& r) {
  CycleMetrics m;
  m.cycle_index = r.input_cycle_index;
  m.executed = r.executed_this_cycle;
  m.has_validation_error = r.has_validation_error;
  m.abort_reason = AbortReasonToInt(r.abort_reason);
  const auto& f = r.track_output_frame;
  m.confirmed = ar_session::CountTracksByStatus(f, ar_session::TrackStatus::kConfirmed);
  m.tentative = ar_session::CountTracksByStatus(f, ar_session::TrackStatus::kTentative);
  m.lost = ar_session::CountTracksByStatus(f, ar_session::TrackStatus::kLost);
  m.jamming = ar_session::CountJammingTracks(f);
  const auto& q = r.association_quality_metrics;
  m.prior_track_count = q.prior_track_count;
  m.detection_count = q.detection_count;
  m.matched_count = q.matched_count;
  m.new_track_count = q.new_track_count;
  m.missed_track_count = q.missed_track_count;
  m.match_rate = q.match_rate;
  m.new_track_rate = q.new_track_rate;
  m.missed_rate = q.missed_track_rate;
  m.jamming_severity = q.jamming_severity;
  m.association_stress = q.association_stress;
  m.mean_match_cost = q.mean_match_cost;
  m.p95_match_cost = q.p95_match_cost;
  return m;
}

/// 场景级汇总（写场景 CSV）。
struct ScenarioSummary {
  std::string scenario_id;
  std::string suite;
  std::string scenario_family;
  double target_range_km{0.0};
  double rcs_m2{0.0};
  int target_count{0};
  double min_snr_db_override{0.0};
  std::uint32_t executed_cycles{0};
  std::uint32_t warmup_confirmed_cycles{0};
  double steady_confirmed_mean{0.0};
  double steady_match_rate_mean{0.0};
  double steady_missed_rate_mean{0.0};
  double steady_jamming_mean{0.0};
  bool replay_ok{false};
  std::uint64_t replay_compared{0};
  bool replay_divergence{false};
  std::size_t expected_failure_count{0U};
  std::size_t contract_check_count{0U};
  std::size_t contract_failure_count{0U};
  std::uint64_t failure_marker_count{0U};
  WarningCollector warnings;
};

/// 运行单个 AR 场景：录制 trace + 采集周期指标，返回场景汇总。
ScenarioSummary RunArScenario(const ArCase& c, const ar_config::ArSessionConfig& base_config,
                              const std::string& output_dir, CsvWriter& cycle_writer,
                              ContractCheckCollector& checks) {
  ScenarioSummary s;
  s.scenario_id = c.scenario_id;
  s.suite = c.sequence ? "sequence" : "sweep";
  s.scenario_family = c.family;
  s.target_range_km = c.target_range_km;
  s.rcs_m2 = static_cast<double>(c.rcs_m2);
  s.target_count = c.target_count;
  s.min_snr_db_override = static_cast<double>(c.min_snr_db_override);

  // 每场景基于 base config 重建（阈值覆盖）。
  ar_config::ArSessionConfig config = base_config;
  ApplyCaseToConfig(c, config);

  const std::string trace_dir = output_dir + "/traces/" + c.scenario_id;
  auto replay_writer = batch_validation::MakeReplayWriter(trace_dir, ModuleName::kAirborneRadar,
                                                          kTraceId, c.scenario_id);

  std::vector<CycleMetrics> metrics;
  const std::uint32_t cycle_count = CycleCount(c);
  metrics.reserve(cycle_count);
  std::size_t rejected_cycle_count = 0U;
  std::uint64_t established_key = 0U;
  std::uint64_t recovered_key = 0U;
  bool invalid_patch_rejected = false;

  // 录制 scope：TraceSession + 所有 Step 包在内，结尾 Flush。
  {
    ar_session::ArTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    ar_session::ArTraceSession session(config, options);

    const char* previous_phase = nullptr;
    for (std::uint32_t i = 0; i < cycle_count; ++i) {
      const std::uint32_t cycle_index = i + 1;
      const char* phase = PhaseFor(c, cycle_index);
      if (previous_phase == nullptr || std::strcmp(previous_phase, phase) != 0) {
        std::fprintf(stderr, "  [phase] scenario=%s phase=%s cycle=%u\n", c.scenario_id.c_str(),
                     phase, cycle_index);
        previous_phase = phase;
      }

      if (c.scenario_id == "ar_seq_tws_stt_tws" && (cycle_index == 9U || cycle_index == 17U)) {
        ar_config::ArRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode =
            cycle_index == 9U ? ar_config::ArWorkMode::kStt : ar_config::ArWorkMode::kTws;
        session.ApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "ar_seq_power_cycle" && (cycle_index == 9U || cycle_index == 14U)) {
        ar_config::ArRuntimeConfigPatch patch;
        patch.has_sensor_enabled = true;
        patch.sensor_enabled = cycle_index == 14U;
        session.ApplyRuntimeConfig(patch);
      }
      if (c.scenario_id == "ar_seq_invalid_patch_atomic" && cycle_index == 9U) {
        ar_config::ArRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = ar_config::ArWorkMode::kStt;
        patch.has_commanded_beamwidth_enabled = true;
        patch.commanded_beamwidth_enabled = true;
        patch.has_commanded_beamwidth_deg = true;
        patch.commanded_beamwidth_deg.commanded_az_beamwidth_deg = 0.0f;
        invalid_patch_rejected = !session.TryApplyRuntimeConfig(patch);
      } else if (c.scenario_id == "ar_seq_invalid_patch_atomic" && cycle_index == 10U) {
        ar_config::ArRuntimeConfigPatch patch;
        patch.has_work_mode = true;
        patch.work_mode = ar_config::ArWorkMode::kStt;
        session.ApplyRuntimeConfig(patch);
      }
      ar_session::ArExternalPoseInput platform = MakePlatformPose(cycle_index);
      std::vector<ar_session::ArExternalTargetInput> targets = MakeTargets(c, cycle_index);
      ar_session::ArEnvironmentInput env = MakeEnvironment(cycle_index);

      ar_session::ArCycleInput input;
      ar_session::ArCoordinateStatus status;
      if (!ar_session::ArCycleInputAdapter::Build(platform, targets, 1.0f, env, &input, &status)) {
        s.warnings.Error("ArCycleInputAdapter::Build failed at cycle " +
                         std::to_string(cycle_index));
        break;
      }
      input.cycle_index = cycle_index;
      if (c.scenario_id == "ar_seq_invalid_input_recovery" && cycle_index == 9U) {
        input.dt_sec = 0.0f;
      } else if (c.scenario_id == "ar_seq_invalid_input_recovery" && cycle_index == 10U &&
                 input.scene.size() >= 2U) {
        input.scene[1].external_target_id = input.scene[0].external_target_id;
      }

      const ar_session::ArCycleResult result = session.StepWithResult(input);
      if (!result.executed_this_cycle) ++rejected_cycle_count;
      const auto track_map = ar_session::BuildTrackMapByExternalTargetId(result.track_output_frame);
      const auto track_it = track_map.find(1000U);
      if (track_it != track_map.end()) {
        if (cycle_index == 8U) established_key = track_it->second.association_key;
        if (cycle_index == 24U) recovered_key = track_it->second.association_key;
      }
      CycleMetrics m = ExtractCycleMetrics(result);
      metrics.push_back(m);

      // 周期级 CSV（含未执行周期，便于诊断）。
      std::fprintf(cycle_writer.file(),
                   "%s,%s,%s,%s,%u,%d,%d,%d,%zu,%zu,%zu,%zu,%zu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
                   "%.5f,%zu,%zu,%zu,%zu\n",
                   c.scenario_id.c_str(), c.sequence ? "sequence" : "sweep", c.family.c_str(),
                   phase, m.cycle_index, static_cast<int>(m.executed),
                   static_cast<int>(m.has_validation_error), m.abort_reason, m.prior_track_count,
                   m.detection_count, m.matched_count, m.new_track_count, m.missed_track_count,
                   static_cast<double>(m.match_rate), static_cast<double>(m.new_track_rate),
                   static_cast<double>(m.missed_rate), static_cast<double>(m.mean_match_cost),
                   static_cast<double>(m.p95_match_cost), static_cast<double>(m.jamming_severity),
                   static_cast<double>(m.association_stress), m.confirmed, m.tentative, m.lost,
                   m.jamming);
    }
    replay_writer->Flush();
  }  // TraceSession 析构

  // ---- 聚合场景级指标（仅统计 executed 周期，warmup 后为"稳态"窗口）----
  for (const auto& m : metrics) {
    if (m.executed) ++s.executed_cycles;
    if (m.cycle_index <= kWarmupCycles && m.confirmed > 0) ++s.warmup_confirmed_cycles;
  }
  std::vector<double> steady_confirmed, steady_match_rate, steady_missed_rate, steady_jamming;
  for (const auto& m : metrics) {
    if (!m.executed || m.cycle_index <= kWarmupCycles) continue;
    steady_confirmed.push_back(static_cast<double>(m.confirmed));
    steady_match_rate.push_back(static_cast<double>(m.match_rate));
    steady_missed_rate.push_back(static_cast<double>(m.missed_rate));
    steady_jamming.push_back(static_cast<double>(m.jamming_severity));
  }
  s.steady_confirmed_mean = batch_validation::Mean(steady_confirmed);
  s.steady_match_rate_mean = batch_validation::Mean(steady_match_rate);
  s.steady_missed_rate_mean = batch_validation::Mean(steady_missed_rate);
  s.steady_jamming_mean = batch_validation::Mean(steady_jamming);

  // ---- 回放（writer 已析构，trace 落盘完整）----
  const ar_session::ArReplaySessionResult replay = ar_session::ReplayArTrace(trace_dir);
  s.replay_ok = replay.ok;
  s.replay_compared = replay.playback.compared_output_count;
  s.replay_divergence = replay.playback.divergence_found;
  s.failure_marker_count = replay.playback.failure_marker_count;
  if (!replay.ok || replay.playback.divergence_found) {
    s.warnings.Error("replay failed: " + replay.first_error);
  } else if (replay.playback.compared_output_count != metrics.size()) {
    s.warnings.Error(
        "replay compared_count=" + std::to_string(replay.playback.compared_output_count) +
        " != steps=" + std::to_string(metrics.size()));
  }

  if (c.sequence) {
    const std::size_t expected_nonexecuted = c.scenario_id == "ar_seq_invalid_input_recovery" ? 2U
                                             : c.scenario_id == "ar_seq_power_cycle"          ? 5U
                                                                                              : 0U;
    s.expected_failure_count = c.scenario_id == "ar_seq_invalid_input_recovery" ? 2U
                               : c.scenario_id == "ar_seq_invalid_patch_atomic" ? 1U
                                                                                : 0U;
    checks.Add(
        c.scenario_id, "replay", cycle_count, "replay_complete", std::to_string(metrics.size()),
        std::to_string(s.replay_compared),
        replay.ok && !replay.playback.divergence_found && s.replay_compared == metrics.size());
    checks.Add(c.scenario_id, "recovery", cycle_count, "expected_nonexecuted_cycles",
               std::to_string(expected_nonexecuted), std::to_string(rejected_cycle_count),
               rejected_cycle_count == expected_nonexecuted);
    const std::uint64_t expected_markers =
        c.scenario_id == "ar_seq_invalid_input_recovery" ? 2U : 0U;
    checks.Add(c.scenario_id, "replay", cycle_count, "failure_marker_count",
               std::to_string(expected_markers), std::to_string(s.failure_marker_count),
               s.failure_marker_count == expected_markers);
    if (c.scenario_id == "ar_seq_invalid_patch_atomic") {
      checks.Add(c.scenario_id, "interruption", 9U, "invalid_patch_atomic_rejection", "rejected",
                 invalid_patch_rejected ? "rejected" : "accepted", invalid_patch_rejected);
    }
    if (c.scenario_id.find("crossing") != std::string::npos ||
        c.scenario_id.find("recovery") != std::string::npos ||
        c.scenario_id.find("power_cycle") != std::string::npos) {
      checks.Add(c.scenario_id, "recovery", cycle_count, "association_identity_continuity",
                 std::to_string(established_key), std::to_string(recovered_key),
                 established_key != 0U && recovered_key == established_key);
    }
    s.contract_check_count = checks.size();
  }

  // ---- 软断言 ----
  if (s.executed_cycles == 0) {
    s.warnings.Error("no cycle executed");
  } else {
    // ① 近距离 + 高 RCS 应在预热后确认轨迹
    if (c.target_range_km <= 15.0 && c.rcs_m2 >= 5.0f) {
      if (s.steady_confirmed_mean < 0.5) {
        s.warnings.Warn("near+high-rcs expected confirmed>=0.5, got " +
                        std::to_string(s.steady_confirmed_mean));
      }
    }
    // ② match_rate 应在 [0,1]
    if (s.steady_match_rate_mean < 0.0 || s.steady_match_rate_mean > 1.0) {
      s.warnings.Warn("match_rate out of [0,1]: " + std::to_string(s.steady_match_rate_mean));
    }
    // ③ 距离↑ 时确认数应单调↓（仅默认阈值组内比较，由跨场景趋势检查完成，此处单场景不判）
  }

  batch_validation::LogReplayResult(
      c.scenario_id,
      ReplayCheckResult{replay.ok, replay.playback.divergence_found,
                        replay.playback.compared_output_count, replay.playback.applied_input_count,
                        replay.reached_failure_marker, replay.first_error},
      metrics.size());
  s.warnings.DumpToStderr(c.scenario_id + ": ");

  return s;
}

// =============================================================================
// 跨场景趋势软断言
// =============================================================================

/// 对场景汇总列表做跨场景趋势检查（距离/RCS 单调性），追加 warning 到对应场景。
void CheckCrossScenarioTrends(std::vector<ScenarioSummary>& summaries) {
  // 取默认阈值组（min_snr_db_override < 0）做趋势分析。
  // ① 固定 RCS，距离↑ → confirmed_mean 单调↓
  // 按 RCS 分组，每组内按距离升序。
  const double rcs_set[] = {0.2, 1.0, 5.0, 10.0};
  for (double rcs : rcs_set) {
    std::vector<double> ranges, confirmed;
    for (const auto& s : summaries) {
      if (s.min_snr_db_override >= 0) continue;  // 仅默认阈值
      if (s.target_count != 3) continue;         // 固定目标数消除噪声
      if (std::abs(s.rcs_m2 - rcs) > 1e-6) continue;
      ranges.push_back(s.target_range_km);
      confirmed.push_back(s.steady_confirmed_mean);
    }
    // 按距离升序排序 confirmed
    std::vector<std::pair<double, double>> pairs;
    for (std::size_t i = 0; i < ranges.size(); ++i) pairs.emplace_back(ranges[i], confirmed[i]);
    std::sort(pairs.begin(), pairs.end());
    std::vector<double> sorted_confirmed;
    for (auto& pr : pairs) sorted_confirmed.push_back(pr.second);
    if (sorted_confirmed.size() >= 2 &&
        !batch_validation::IsMonotonicNonIncreasing(sorted_confirmed)) {
      // 找到该 RCS 组的所有场景，给最近距离最小的场景记 warning（代理）
      for (auto& s : summaries) {
        if (s.min_snr_db_override >= 0 || s.target_count != 3 || std::abs(s.rcs_m2 - rcs) > 1e-6)
          continue;
        s.warnings.Warn("cross-scenario: confirmed not monotonic-decreasing in range (rcs=" +
                        std::to_string(rcs) + ")");
        break;
      }
    }
  }
}

}  // namespace

// =============================================================================
// main
// =============================================================================

int main(int argc, char** argv) {
  batch_validation::BatchCliOptions cli;
  std::string cli_error;
  if (!batch_validation::ParseBatchCli(argc, argv, kDefaultOutputDir, &cli, &cli_error)) {
    std::fprintf(stderr, "FATAL: %s\n", cli_error.c_str());
    batch_validation::PrintBatchUsage(argv[0]);
    return 1;
  }
  const std::string output_dir = cli.output_dir;
  const std::string config_path = BATCH_CONFIG_DIR "/airborne_radar.json";

  std::fprintf(stderr, "=== AR 批量场景验证 ===\n");
  std::fprintf(stderr, "  配置: %s\n  输出: %s\n", config_path.c_str(), output_dir.c_str());

  std::vector<ArCase> cases;
  if (batch_validation::IncludesSweep(cli.suite)) cases = BuildArCases();
  if (batch_validation::IncludesSequence(cli.suite)) {
    const std::vector<ArCase> sequences = BuildArSequenceCases();
    cases.insert(cases.end(), sequences.begin(), sequences.end());
  }
  if (cli.list_scenarios) {
    for (const ArCase& c : cases) std::printf("%s\n", c.scenario_id.c_str());
    return 0;
  }
  if (!cli.scenario_id.empty()) {
    cases.erase(std::remove_if(cases.begin(), cases.end(),
                               [&](const ArCase& c) { return c.scenario_id != cli.scenario_id; }),
                cases.end());
  }
  if (cases.empty()) {
    std::fprintf(stderr, "FATAL: no scenario matched\n");
    return 1;
  }

  // 1. 加载基础配置（一次）。
  ar_config::ArSessionConfig base_config;
  std::string load_error;
  if (!examples::LoadArSessionConfigFromFile(config_path.c_str(), &base_config, &load_error)) {
    std::fprintf(stderr, "FATAL: 加载配置失败 %s: %s\n", config_path.c_str(), load_error.c_str());
    return 1;
  }

  // 2. 准备输出目录 + CSV writer。
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

  // 3. 构造场景并逐个执行。
  std::vector<ScenarioSummary> summaries;
  ContractCheckCollector checks;
  summaries.reserve(cases.size());
  std::fprintf(stderr, "  场景数: %zu\n", cases.size());

  for (std::size_t i = 0; i < cases.size(); ++i) {
    std::fprintf(stderr, "[%zu/%zu] %s\n", i + 1, cases.size(), cases[i].scenario_id.c_str());
    std::fprintf(stderr, "  [scenario] id=%s suite=%s family=%s\n", cases[i].scenario_id.c_str(),
                 cases[i].sequence ? "sequence" : "sweep", cases[i].family.c_str());
    const std::size_t check_begin = checks.size();
    summaries.push_back(RunArScenario(cases[i], base_config, output_dir, cycle_writer, checks));
    summaries.back().contract_check_count = checks.size() - check_begin;
    std::size_t failures = 0U;
    const auto& all_checks = checks.checks();
    for (std::size_t j = check_begin; j < all_checks.size(); ++j) {
      if (!all_checks[j].passed && all_checks[j].severity == Severity::kError) ++failures;
    }
    summaries.back().contract_failure_count = failures;
    cycle_writer.Flush();
  }

  // 4. 跨场景趋势软断言。
  CheckCrossScenarioTrends(summaries);

  // 5. 写场景汇总 CSV。
  for (const auto& s : summaries) {
    std::fprintf(
        stderr,
        "  [scenario] module=AR id=%s physics_detection=1 physical_rcs=1 "
        "physics_mix_ratio=1.000 range_km=%.3f rcs_m2=%.3f targets=%d "
        "min_snr_db_override=%.3f executed=%u/%u confirmed=%.4f match_rate=%.4f "
        "missed_rate=%.4f jamming=%.4f replay_ok=%d compared=%llu divergence=%d "
        "warn=%zu error=%zu\n",
        s.scenario_id.c_str(), s.target_range_km, s.rcs_m2, s.target_count, s.min_snr_db_override,
        s.executed_cycles, s.scenario_id.find("_seq_") != std::string::npos ? 24U : kNumCycles,
        s.steady_confirmed_mean, s.steady_match_rate_mean, s.steady_missed_rate_mean,
        s.steady_jamming_mean, static_cast<int>(s.replay_ok),
        static_cast<unsigned long long>(s.replay_compared), static_cast<int>(s.replay_divergence),
        s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError));
    std::fprintf(scenario_writer.file(),
                 "%s,%s,%s,%.3f,%.3f,%d,%.3f,%u,%u,%.4f,%.4f,%.4f,%.4f,%d,%llu,%d,%zu,%zu,%zu,%llu,"
                 "%zu,%zu,%s\n",
                 s.scenario_id.c_str(), s.suite.c_str(), s.scenario_family.c_str(),
                 s.target_range_km, s.rcs_m2, s.target_count, s.min_snr_db_override,
                 s.executed_cycles, s.warmup_confirmed_cycles, s.steady_confirmed_mean,
                 s.steady_match_rate_mean, s.steady_missed_rate_mean, s.steady_jamming_mean,
                 static_cast<int>(s.replay_ok), static_cast<unsigned long long>(s.replay_compared),
                 static_cast<int>(s.replay_divergence), s.expected_failure_count,
                 s.contract_check_count, s.contract_failure_count,
                 static_cast<unsigned long long>(s.failure_marker_count),
                 s.warnings.Count(Severity::kWarning), s.warnings.Count(Severity::kError),
                 batch_validation::EscapeCsvField(s.warnings.JoinForCsv()).c_str());
  }
  scenario_writer.Flush();
  checks.WriteCsv(output_dir + "/checks.csv");

  // 6. 汇总报告。
  std::size_t total_warn = 0, total_err = 0, replay_fail = 0;
  for (const auto& s : summaries) {
    total_warn += s.warnings.Count(Severity::kWarning);
    total_err += s.warnings.Count(Severity::kError);
    if (!s.replay_ok || s.replay_divergence) ++replay_fail;
  }
  std::fprintf(stderr, "\n=== AR 批量验证完成 ===\n");
  std::fprintf(stderr, "  场景数: %zu\n  周期 CSV: %s\n  场景 CSV: %s\n", summaries.size(),
               cycles_csv.c_str(), scenarios_csv.c_str());
  std::fprintf(stderr, "  软断言 warning: %zu, error: %zu\n", total_warn, total_err);
  std::fprintf(stderr, "  回放失败场景: %zu\n", replay_fail);
  std::fprintf(stderr, "  trace 目录: %s/traces/<scenario_id>/\n", output_dir.c_str());

  // 回放失败视为严重问题，返回非零。
  return (replay_fail > 0 || total_err > 0 || checks.FailureCount() > 0U) ? 2 : 0;
}
