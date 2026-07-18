/**
 * @file sbirs_batch_validation.cpp
 * @brief SBIRS 距离、温度与投影面积的批量物理场景验证。
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsReplaySession.h"
#include "1q/sbirs_sensor/session/SbirsTraceSession.h"
#include "batch_assertions.h"
#include "batch_csv_writer.h"
#include "batch_replay.h"

namespace sbirs_config = sbirs_sensor::config;
namespace sbirs_output = sbirs_sensor::output;
namespace sbirs_session = sbirs_sensor::session;
using batch_validation::CsvWriter;
using batch_validation::ModuleName;
using batch_validation::ReplayCheckResult;
using batch_validation::Severity;
using batch_validation::WarningCollector;

namespace {

constexpr const char* kDefaultOutputDir = "/tmp/1q/batch_validation/sbirs_sensor";
constexpr const char* kTraceId = "sbirs-batch-validation";
constexpr double kSatelliteXM = 7000000.0;

struct SbirsCase {
  std::string scenario_id;
  double range_km{0.0};
  float temperature_k{0.0f};
  float projected_area_m2{0.0f};
};

std::vector<SbirsCase> BuildCases() {
  const double ranges_km[] = {1000.0, 2000.0, 4000.0};
  const float temperatures_k[] = {800.0f, 1400.0f, 2200.0f};
  const float areas_m2[] = {10.0f, 500.0f, 5000.0f};
  std::vector<SbirsCase> cases;
  char id[128];
  for (double range_km : ranges_km) {
    for (float temperature_k : temperatures_k) {
      for (float area_m2 : areas_m2) {
        std::snprintf(id, sizeof(id), "sbirs_r%.0fkm_t%.0f_a%.0f", range_km,
                      static_cast<double>(temperature_k), static_cast<double>(area_m2));
        cases.push_back({id, range_km, temperature_k, area_m2});
      }
    }
  }
  return cases;
}

sbirs_session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_config::SbirsSessionConfig MakeConfig() {
  sbirs_config::SbirsSessionConfig config;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  return config;
}

sbirs_session::SbirsCycleInput MakeInput(const SbirsCase& scenario, std::uint32_t cycle_index) {
  sbirs_session::SbirsSceneTarget target;
  target.target_id = 101U;
  target.target_name = "batch_ir_target";
  target.position_ecef_m = Vector(kSatelliteXM + scenario.range_km * 1000.0, 0.0, 0.0);
  target.temperature_k = scenario.temperature_k;
  target.emissivity = 0.9f;
  target.projected_area_m2 = scenario.projected_area_m2;
  return sbirs_session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(kSatelliteXM, 0.0, 0.0))
      .AddTarget(target)
      .Build();
}

const char* StageName(sbirs_output::SbirsObservationStage stage) {
  switch (stage) {
    case sbirs_output::SbirsObservationStage::kWideFieldSearch:
      return "wfov_search";
    case sbirs_output::SbirsObservationStage::kNarrowFieldAcquisition:
      return "nfov_acquisition";
    case sbirs_output::SbirsObservationStage::kNarrowFieldTrack:
      return "nfov_track";
  }
  return "none";
}

struct ScenarioSummary {
  SbirsCase scenario;
  std::size_t executed_cycles{0U};
  std::size_t detection_count{0U};
  float max_snr_linear{0.0f};
  std::string final_stage{"none"};
  bool replay_ok{false};
  std::uint64_t replay_compared{0U};
  bool replay_divergence{false};
  WarningCollector warnings;
};

constexpr const char* kCycleHeader =
    "scenario_id,cycle_index,executed,validation_error,reused,abort_reason,scan_azimuth_deg,"
    "detection_count,max_snr_linear,observation_stage";
constexpr const char* kScenarioHeader =
    "scenario_id,range_km,temperature_k,projected_area_m2,executed_cycles,detection_count,"
    "max_snr_linear,final_stage,replay_ok,replay_compared,replay_divergence,warning_count,"
    "error_count,warnings";

ScenarioSummary RunScenario(const SbirsCase& scenario, const std::string& output_dir,
                            CsvWriter& cycle_writer) {
  ScenarioSummary summary;
  summary.scenario = scenario;
  const std::string trace_dir = output_dir + "/traces/" + scenario.scenario_id;
  auto replay_writer = batch_validation::MakeReplayWriter(trace_dir, ModuleName::kSbirsSensor,
                                                          kTraceId, scenario.scenario_id);
  {
    sbirs_session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    sbirs_session::SbirsTraceSession session(MakeConfig(), options);
    for (std::uint32_t cycle = 1U; cycle <= 2U; ++cycle) {
      const sbirs_session::SbirsCycleResult result =
          session.StepWithResult(MakeInput(scenario, cycle));
      if (result.executed_this_cycle) ++summary.executed_cycles;
      summary.detection_count += result.output_frame.detections.size();
      float cycle_max_snr = 0.0f;
      const char* stage = "none";
      for (const auto& detection : result.output_frame.detections) {
        cycle_max_snr = std::max(cycle_max_snr, detection.infrared_snr_linear);
        stage = StageName(detection.observation_stage);
      }
      summary.max_snr_linear = std::max(summary.max_snr_linear, cycle_max_snr);
      summary.final_stage = stage;
      std::fprintf(
          cycle_writer.file(), "%s,%u,%d,%d,%d,%d,%.5f,%zu,%.9g,%s\n", scenario.scenario_id.c_str(),
          cycle, static_cast<int>(result.executed_this_cycle),
          static_cast<int>(result.has_validation_error),
          static_cast<int>(result.reused_previous_output), static_cast<int>(result.abort_reason),
          result.output_frame.scan_azimuth_deg, result.output_frame.detections.size(),
          static_cast<double>(cycle_max_snr), stage);
    }
    replay_writer->Flush();
  }

  const sbirs_session::SbirsReplaySessionResult replay = sbirs_session::ReplaySbirsTrace(trace_dir);
  summary.replay_ok = replay.ok;
  summary.replay_compared = replay.playback.compared_output_count;
  summary.replay_divergence = replay.playback.divergence_found;
  if (summary.executed_cycles != 2U) {
    summary.warnings.Error("not all cycles executed");
  }
  if (!replay.ok || replay.playback.divergence_found || summary.replay_compared != 2U) {
    summary.warnings.Warn("replay divergence: " + replay.first_error);
  }
  batch_validation::LogReplayResult(
      scenario.scenario_id,
      ReplayCheckResult{replay.ok, replay.playback.divergence_found,
                        replay.playback.compared_output_count, replay.playback.applied_input_count,
                        replay.reached_failure_marker, replay.first_error},
      2U);
  summary.warnings.DumpToStderr(scenario.scenario_id + ": ");
  return summary;
}

void CheckTemperatureTrend(std::vector<ScenarioSummary>& summaries) {
  for (double range_km : {1000.0, 2000.0, 4000.0}) {
    for (float area_m2 : {10.0f, 500.0f, 5000.0f}) {
      std::vector<std::pair<float, float>> samples;
      for (const auto& summary : summaries) {
        if (summary.scenario.range_km == range_km &&
            summary.scenario.projected_area_m2 == area_m2) {
          samples.emplace_back(summary.scenario.temperature_k, summary.max_snr_linear);
        }
      }
      std::sort(samples.begin(), samples.end());
      for (std::size_t i = 1U; i < samples.size(); ++i) {
        if (samples[i].second < samples[i - 1U].second) {
          for (auto& summary : summaries) {
            if (summary.scenario.range_km == range_km &&
                summary.scenario.projected_area_m2 == area_m2) {
              summary.warnings.Warn("SNR decreased as target temperature increased");
              break;
            }
          }
          break;
        }
      }
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string output_dir = argc > 1 ? argv[1] : kDefaultOutputDir;
  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::fprintf(stderr, "FATAL: cannot create %s: %s\n", output_dir.c_str(), ec.message().c_str());
    return 1;
  }
  CsvWriter cycle_writer(output_dir + "/cycles.csv", kCycleHeader);
  CsvWriter scenario_writer(output_dir + "/scenarios.csv", kScenarioHeader);
  const std::vector<SbirsCase> cases = BuildCases();
  std::vector<ScenarioSummary> summaries;
  summaries.reserve(cases.size());
  std::fprintf(stderr, "=== SBIRS batch validation ===\n  scenarios: %zu\n", cases.size());
  for (std::size_t i = 0U; i < cases.size(); ++i) {
    std::fprintf(stderr, "[%zu/%zu] %s\n", i + 1U, cases.size(), cases[i].scenario_id.c_str());
    summaries.push_back(RunScenario(cases[i], output_dir, cycle_writer));
  }
  CheckTemperatureTrend(summaries);
  for (const auto& summary : summaries) {
    std::fprintf(stderr,
                 "  [scenario] module=SBIRS id=%s range_km=%.0f temperature_k=%.0f area_m2=%.0f "
                 "executed=%zu/2 detections=%zu max_snr_linear=%.9g final_stage=%s replay_ok=%d "
                 "compared=%llu divergence=%d warn=%zu error=%zu\n",
                 summary.scenario.scenario_id.c_str(), summary.scenario.range_km,
                 static_cast<double>(summary.scenario.temperature_k),
                 static_cast<double>(summary.scenario.projected_area_m2), summary.executed_cycles,
                 summary.detection_count, static_cast<double>(summary.max_snr_linear),
                 summary.final_stage.c_str(), static_cast<int>(summary.replay_ok),
                 static_cast<unsigned long long>(summary.replay_compared),
                 static_cast<int>(summary.replay_divergence),
                 summary.warnings.Count(Severity::kWarning),
                 summary.warnings.Count(Severity::kError));
    std::fprintf(
        scenario_writer.file(), "%s,%.0f,%.0f,%.0f,%zu,%zu,%.9g,%s,%d,%llu,%d,%zu,%zu,%s\n",
        summary.scenario.scenario_id.c_str(), summary.scenario.range_km,
        static_cast<double>(summary.scenario.temperature_k),
        static_cast<double>(summary.scenario.projected_area_m2), summary.executed_cycles,
        summary.detection_count, static_cast<double>(summary.max_snr_linear),
        summary.final_stage.c_str(), static_cast<int>(summary.replay_ok),
        static_cast<unsigned long long>(summary.replay_compared),
        static_cast<int>(summary.replay_divergence), summary.warnings.Count(Severity::kWarning),
        summary.warnings.Count(Severity::kError),
        batch_validation::EscapeCsvField(summary.warnings.JoinForCsv()).c_str());
  }
  std::size_t errors = 0U;
  std::size_t warnings = 0U;
  for (const auto& summary : summaries) {
    errors += summary.warnings.Count(Severity::kError);
    warnings += summary.warnings.Count(Severity::kWarning);
  }
  std::fprintf(stderr, "=== SBIRS summary: scenarios=%zu warnings=%zu errors=%zu ===\n",
               summaries.size(), warnings, errors);
  return errors == 0U ? 0 : 2;
}
