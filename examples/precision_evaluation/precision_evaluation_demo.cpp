/**
 * @file precision_evaluation_demo.cpp
 * @brief 精度评估层集成参考示例（需求 3.2.1.6.3：五项定位误差 + AHP 综合评分）。
 *
 * 默认场景 `scenes/sbirs_dual_sat_fix/sbirs_dual_sat_fix.json`（原硬编码几何外置）：
 *  - 双星：主星 (7e6,0,0)、辅星 (0,7e6,0)（ECEF 静止），各自扫描中心对准目标群
 *    （主星 ≈79.5°、辅星 ≈351.9°）；
 *  - 目标 1（key=1）：径向下降弹道，供落点预测误差样本；目标 2（key=2）：邻近
 *    位置径向上升弹道，供发射点预测误差样本（两目标角距 ~2°，均在双星扫描带
 *    与宽视场内）；
 *  - 误差模型保持库默认（attitude σ=0.01°、固定 seed）；SNR 门限放宽到 0.001
 *    保证双星稳定检出。内部双 SBIRS 不承担 sbirs_acceptance.log 齐套。
 *
 * 每周期：调用方推进真值（p += v·dt）→ PrecisionEvaluationSession::Step 驱动
 * 内部双星 SBIRS + 融合（强制逐航迹滤波）+ 按间隔推演；结束 Summarize 汇总五指标
 * + AHP，并写入 precision_acceptance.log（需 ONEQ_ENABLE_PRECISION_EVALUATION_LOG）。
 *
 * 运行：precision_evaluation_demo [--scene <path>] [--cycles <n>] [--output-dir <dir>]
 */

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#if defined(_MSC_VER) && _MSC_VER < 1910
#include <filesystem>
namespace demo_fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace demo_fs = std::filesystem;
#endif

#include "1q/coordinate/types.h"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "json_reader.h"

namespace pe = precision_evaluation;

namespace {

#ifndef PE_DEFAULT_SCENE_FILE
#define PE_DEFAULT_SCENE_FILE \
  "examples/precision_evaluation/scenes/sbirs_dual_sat_fix/sbirs_dual_sat_fix.json"
#endif
#ifndef PE_DEFAULT_OUTPUT_DIR
#define PE_DEFAULT_OUTPUT_DIR "examples/precision_evaluation/log"
#endif

struct LoadedScene {
  std::uint32_t cycles{60U};
  float dt_sec{1.0f};
  double utc_julian_day{2451544.2230698913};
  pe::PrecisionEvaluationConfig config{};
  pe::DualSatEphemerisInput ephemeris{};
  std::vector<pe::EvaluationTruthTarget> truth{};
};

bool ReadVec3(const examples::JsonValue& value, double* x, double* y, double* z,
              const char* name, std::string* error) {
  if (value.IsNull() || value.Size() != 3U) {
    *error = std::string(name) + " must be [x,y,z]";
    return false;
  }
  *x = value[static_cast<std::size_t>(0)].AsDouble();
  *y = value[static_cast<std::size_t>(1)].AsDouble();
  *z = value[static_cast<std::size_t>(2)].AsDouble();
  return true;
}

sbirs_sensor::config::SbirsSessionConfig LoadSatelliteConfig(const examples::JsonValue& block) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w =
      block.Has("noise_equivalent_power_w")
          ? static_cast<float>(block["noise_equivalent_power_w"].AsDouble())
          : 1.0e-18f;
  config.hardware.integration_time_sec =
      block.Has("integration_time_sec")
          ? static_cast<float>(block["integration_time_sec"].AsDouble())
          : 1.0f;
  config.mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  config.mission.scan_start_az_deg = static_cast<float>(block["scan_start_az_deg"].AsDouble());
  config.mission.scan_span_deg =
      block.Has("scan_span_deg") ? static_cast<float>(block["scan_span_deg"].AsDouble()) : 11.0f;
  config.mission.scan_rate_deg_per_sec =
      block.Has("scan_rate_deg_per_sec")
          ? static_cast<float>(block["scan_rate_deg_per_sec"].AsDouble())
          : 1.0f;
  config.mission.wide_field_fov_az_deg =
      block.Has("wide_field_fov_az_deg")
          ? static_cast<float>(block["wide_field_fov_az_deg"].AsDouble())
          : 20.0f;
  config.mission.wide_field_fov_el_deg =
      block.Has("wide_field_fov_el_deg")
          ? static_cast<float>(block["wide_field_fov_el_deg"].AsDouble())
          : 20.0f;
  config.mission.narrow_field_fov_az_deg =
      block.Has("narrow_field_fov_az_deg")
          ? static_cast<float>(block["narrow_field_fov_az_deg"].AsDouble())
          : 5.0f;
  config.mission.narrow_field_fov_el_deg =
      block.Has("narrow_field_fov_el_deg")
          ? static_cast<float>(block["narrow_field_fov_el_deg"].AsDouble())
          : 5.0f;
  config.policy.detection.wide_min_snr_linear =
      block.Has("wide_min_snr_linear")
          ? static_cast<float>(block["wide_min_snr_linear"].AsDouble())
          : 0.001f;
  config.policy.detection.narrow_min_snr_linear =
      block.Has("narrow_min_snr_linear")
          ? static_cast<float>(block["narrow_min_snr_linear"].AsDouble())
          : 0.001f;
  return config;
}

bool LoadEphemerisSat(const examples::JsonValue& block, oneq::coordinate::EcefPositionM* position,
                      oneq::coordinate::EcefVelocityMps* velocity, std::string* error,
                      const char* tag) {
  double px = 0.0;
  double py = 0.0;
  double pz = 0.0;
  if (!ReadVec3(block["position_ecef_m"], &px, &py, &pz,
                (std::string(tag) + ".position_ecef_m").c_str(), error)) {
    return false;
  }
  *position = oneq::coordinate::EcefPositionM(px, py, pz);
  if (block.Has("velocity_ecef_mps")) {
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    if (!ReadVec3(block["velocity_ecef_mps"], &vx, &vy, &vz,
                  (std::string(tag) + ".velocity_ecef_mps").c_str(), error)) {
      return false;
    }
    *velocity = oneq::coordinate::EcefVelocityMps(vx, vy, vz);
  }
  return true;
}

bool LoadScene(const char* path, LoadedScene* scene, std::string* error) {
  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error)) {
    return false;
  }
  if (root.Has("cycles")) {
    const std::int64_t cycles = root["cycles"].AsInt();
    if (cycles <= 0) {
      *error = "cycles must be > 0";
      return false;
    }
    scene->cycles = static_cast<std::uint32_t>(cycles);
  }
  if (root.Has("dt_sec")) {
    scene->dt_sec = static_cast<float>(root["dt_sec"].AsDouble());
  }
  if (root.Has("utc_julian_day")) {
    scene->utc_julian_day = root["utc_julian_day"].AsDouble();
  }
  if (root.Has("inference_horizon_sec")) {
    scene->config.inference.prediction_horizon_sec = root["inference_horizon_sec"].AsDouble();
  } else {
    scene->config.inference.prediction_horizon_sec = 1200.0;
  }

  const examples::JsonValue& satellites = root["satellites"];
  if (satellites.IsNull() || satellites.Size() != 2U) {
    *error = "satellites must be an array of two objects (A then B)";
    return false;
  }
  scene->config.satellite_a = LoadSatelliteConfig(satellites[static_cast<std::size_t>(0)]);
  scene->config.satellite_b = LoadSatelliteConfig(satellites[static_cast<std::size_t>(1)]);
  if (!LoadEphemerisSat(satellites[static_cast<std::size_t>(0)],
                        &scene->ephemeris.satellite_a_position_ecef_m,
                        &scene->ephemeris.satellite_a_velocity_ecef_m_per_s, error,
                        "satellites[0]")) {
    return false;
  }
  if (!LoadEphemerisSat(satellites[static_cast<std::size_t>(1)],
                        &scene->ephemeris.satellite_b_position_ecef_m,
                        &scene->ephemeris.satellite_b_velocity_ecef_m_per_s, error,
                        "satellites[1]")) {
    return false;
  }

  const examples::JsonValue& targets = root["targets"];
  if (targets.IsNull() || targets.Size() == 0U) {
    *error = "targets must be a non-empty array";
    return false;
  }
  scene->truth.clear();
  for (std::size_t i = 0U; i < targets.Size(); ++i) {
    const examples::JsonValue& block = targets[i];
    pe::EvaluationTruthTarget truth;
    truth.key = static_cast<std::uint64_t>(block["key"].AsInt());
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    if (!ReadVec3(block["position_ecef_m"], &px, &py, &pz, "targets[].position_ecef_m", error)) {
      return false;
    }
    truth.position_ecef_m = oneq::coordinate::EcefPositionM(px, py, pz);
    if (block.Has("velocity_ecef_mps")) {
      double vx = 0.0;
      double vy = 0.0;
      double vz = 0.0;
      if (!ReadVec3(block["velocity_ecef_mps"], &vx, &vy, &vz, "targets[].velocity_ecef_mps",
                    error)) {
        return false;
      }
      truth.has_velocity = true;
      truth.velocity_ecef_m_per_s = oneq::coordinate::EcefVelocityMps(vx, vy, vz);
    }
    if (block.Has("radiant_intensity_w_per_sr")) {
      truth.radiant_intensity_w_per_sr = block["radiant_intensity_w_per_sr"].AsDouble();
    }
    scene->truth.push_back(truth);
  }
  return true;
}

void SetProcessEnv(const char* name, const std::string& value) {
#if defined(_WIN32)
  _putenv_s(name, value.c_str());
#else
  setenv(name, value.c_str(), /*overwrite=*/1);
#endif
}

void BindAcceptanceLogPaths(const std::string& output_dir) {
  SetProcessEnv("ONEQ_PRECISION_ACCEPTANCE_LOG_PATH",
                output_dir + "/precision_acceptance.log");
  SetProcessEnv("ONEQ_SBIRS_ACCEPTANCE_LOG_PATH", output_dir + "/sbirs_acceptance.log");
  SetProcessEnv("ONEQ_FUSION_ACCEPTANCE_LOG_PATH", output_dir + "/fusion_acceptance.log");
  SetProcessEnv("ONEQ_INFERENCE_ACCEPTANCE_LOG_PATH",
                output_dir + "/inference_acceptance.log");
}

const char* MetricName(pe::PrecisionMetric metric) {
  switch (metric) {
    case pe::PrecisionMetric::kAngular:
      return "angular (deg)";
    case pe::PrecisionMetric::kDualSatFix:
      return "dual_sat_fix (m)";
    case pe::PrecisionMetric::kVelocity:
      return "velocity (m/s)";
    case pe::PrecisionMetric::kImpactPoint:
      return "impact_point (m)";
    case pe::PrecisionMetric::kLaunchPoint:
      return "launch_point (m)";
  }
  return "unknown";
}

void PrintReport(const pe::PrecisionEvaluationReport& report) {
  std::cout << "\n=== Precision Evaluation Report ===\n";
  std::cout << "metric               count          mean          rmse           p95           max\n";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    const pe::ErrorMetricSummary& m = report.metrics[i];
    std::cout << MetricName(static_cast<pe::PrecisionMetric>(i)) << std::string(
                     22U - std::string(MetricName(static_cast<pe::PrecisionMetric>(i))).size(), ' ')
              << m.count << std::string(9U - std::to_string(m.count).size(), ' ')
              << m.mean << "  " << m.rmse << "  " << m.p95 << "  " << m.max << "\n";
  }
  std::cout << "\nahp: valid=" << (report.ahp_valid ? 1 : 0)
            << " consistent=" << (report.ahp.is_consistent ? 1 : 0)
            << " lambda_max=" << report.ahp.lambda_max
            << " ci=" << report.ahp.consistency_index
            << " cr=" << report.ahp.consistency_ratio << "\n";
  std::cout << "weights:";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    std::cout << " " << report.ahp.weights[i];
  }
  std::cout << "\nscores (score = ref/(ref+rmse), contribution = w*score):\n";
  for (std::size_t i = 0U; i < pe::kPrecisionMetricCount; ++i) {
    std::cout << "  " << MetricName(static_cast<pe::PrecisionMetric>(i))
              << " ref=" << report.reference_errors[i]
              << " score=" << report.metric_scores[i]
              << " contribution=" << report.metric_contributions[i] << "\n";
  }
  std::cout << "composite score = " << report.composite_score << "\n";
}

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " [--scene <path>] [--cycles <n>] [--output-dir <dir>]\n"
            << "  --scene       JSON 场景（默认 " << PE_DEFAULT_SCENE_FILE << "）\n"
            << "  --cycles      覆盖场景 cycles\n"
            << "  --output-dir  验收文件目录（默认 " << PE_DEFAULT_OUTPUT_DIR
            << "/<场景名>/）\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string scene_path = PE_DEFAULT_SCENE_FILE;
  std::string output_dir = PE_DEFAULT_OUTPUT_DIR;
  bool output_dir_overridden = false;
  bool cycles_overridden = false;
  std::uint32_t cycles_override = 0U;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--cycles" && i + 1 < argc) {
      cycles_overridden = true;
      cycles_override = static_cast<std::uint32_t>(std::atoi(argv[++i]));
    } else if (arg == "--scene" && i + 1 < argc) {
      scene_path = argv[++i];
    } else if (arg == "--output-dir" && i + 1 < argc) {
      output_dir = argv[++i];
      output_dir_overridden = true;
    } else {
      PrintUsage(argv[0]);
      return 1;
    }
  }

  LoadedScene scene;
  std::string error;
  if (!LoadScene(scene_path.c_str(), &scene, &error)) {
    std::cerr << "Failed to load scene \"" << scene_path << "\": " << error << "\n";
    return 1;
  }
  if (cycles_overridden) {
    scene.cycles = cycles_override;
  }
  if (scene.cycles == 0U) {
    std::cerr << "Invalid cycle count: must be > 0\n";
    return 1;
  }

  if (!output_dir_overridden) {
    std::string slug = scene_path;
    for (std::size_t i = 0U; i < slug.size(); ++i) {
      if (slug[i] == '\\') {
        slug[i] = '/';
      }
    }
    while (!slug.empty() && slug[slug.size() - 1U] == '/') {
      slug.erase(slug.size() - 1U);
    }
    const std::size_t slash = slug.find_last_of('/');
    const std::string parent = (slash == std::string::npos) ? std::string() : slug.substr(0U, slash);
    const std::size_t parent_slash = parent.find_last_of('/');
    const std::string parent_name =
        (parent_slash == std::string::npos) ? parent : parent.substr(parent_slash + 1U);
    if (!parent_name.empty()) {
      output_dir = std::string(PE_DEFAULT_OUTPUT_DIR) + "/" + parent_name;
    }
  }
  std::error_code fs_error;
  demo_fs::create_directories(output_dir, fs_error);
  if (fs_error) {
    std::cerr << "Failed to create output dir \"" << output_dir << "\": " << fs_error.message()
              << "\n";
    return 1;
  }
  BindAcceptanceLogPaths(output_dir);

#if !defined(PE_ACCEPTANCE_LOG_ENABLED) || !PE_ACCEPTANCE_LOG_ENABLED
  std::cerr << "warning: ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF; "
            << output_dir << "/precision_acceptance.log will not be written\n";
#endif

  pe::PrecisionEvaluationSession session(scene.config);

  std::cout << "precision_evaluation demo: " << scene_path << ", " << scene.cycles << " cycles x "
            << scene.dt_sec << " s\n"
            << "acceptance logs -> " << output_dir
            << " (precision/sbirs/fusion/inference_acceptance.log)\n";
  std::uint32_t dual_sat_cycles = 0U;
  for (std::uint32_t cycle = 1U; cycle <= scene.cycles; ++cycle) {
    // 真值弹道由调用方推进（评估会话不拥有真值；同单测约定 p += v·dt）。
    for (std::size_t i = 0U; i < scene.truth.size(); ++i) {
      pe::EvaluationTruthTarget& target = scene.truth[i];
      target.position_ecef_m.x_m +=
          static_cast<double>(scene.dt_sec) * target.velocity_ecef_m_per_s.x_mps;
      target.position_ecef_m.y_m +=
          static_cast<double>(scene.dt_sec) * target.velocity_ecef_m_per_s.y_mps;
      target.position_ecef_m.z_m +=
          static_cast<double>(scene.dt_sec) * target.velocity_ecef_m_per_s.z_mps;
    }
    const pe::PrecisionEvaluationCycleResult result =
        session.Step(cycle, scene.dt_sec, scene.utc_julian_day, scene.ephemeris, scene.truth);
    if (!result.dual_sat.empty()) {
      ++dual_sat_cycles;
    }
    std::cout << "cycle=" << cycle << " angular=" << result.angular.size()
              << " dual_sat=" << result.dual_sat.size()
              << " velocity=" << result.velocity.size()
              << " keypoints=" << result.keypoints.size() << "\n";
  }

  const pe::PrecisionEvaluationReport report = session.Summarize();
  PrintReport(report);
  std::cout << "dual_sat_cycles=" << dual_sat_cycles << "/" << scene.cycles << "\n";

  // 自检：五指标均有样本、AHP 矩阵合法求解、综合分 ∈ (0,1]，且周期内有双星交会。
  if (!report.all_metrics_sampled || !report.ahp_valid || !(report.composite_score > 0.0) ||
      report.composite_score > 1.0 || dual_sat_cycles == 0U) {
    std::cerr << "SMOKE FAILED: all_metrics_sampled=" << (report.all_metrics_sampled ? 1 : 0)
              << " ahp_valid=" << (report.ahp_valid ? 1 : 0)
              << " composite=" << report.composite_score
              << " dual_sat_cycles=" << dual_sat_cycles << "\n";
    return 1;
  }
  return 0;
}
