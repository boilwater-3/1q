/**
 * @file sbirs_dual_sat_fix/main.cpp
 * @brief 精度评估场景可执行（需求 3.2.1.6.3：五项定位误差 + AHP 综合评分）。
 *
 * 两个卫星实体各挂 SBIRS 组件，地面站实体挂融合组件（内含 FusionEngine +
 * PrecisionEvaluationSession）。每周期：调用方推进真值 → 卫星探测帧写入收件箱
 * → 地面站适配（源 4 / 源 104）后 FusionEngine::Update，再对照真值打分。
 * 场景由 ONEQ_SCENE_JSON 钉死（本目录 sbirs_dual_sat_fix.json）；兼容目标
 * precision_evaluation_demo 与本可执行同源。
 *
 * 运行：sbirs_dual_sat_fix / precision_evaluation_demo [--scene <path>] [--cycles <n>] [--output-dir <dir>]
 */

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "app/fs_compat.h"

#include "1q/coordinate/types.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "json_reader.h"
#include "app/output_dir.h"
#include "components/fusion_component.h"
#include "components/sbirs_sensor_component.h"
#include "core/scene_types.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/acceptance_paths.h"

namespace pe = precision_evaluation;

namespace {

#ifndef ONEQ_SCENE_JSON
#define ONEQ_SCENE_JSON \
  "examples/scenes/sbirs_dual_sat_fix/sbirs_dual_sat_fix.json"
#endif
#ifndef PE_DEFAULT_OUTPUT_DIR
#define PE_DEFAULT_OUTPUT_DIR "examples/log"
#endif

struct LoadedScene {
  std::uint32_t cycles{60U};
  std::string log_dir{};
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
  // 场景自带日志输出目录（强制约束）：相对路径（基点 examples/log/），无 ..。
  if (!root["log_dir"].IsString() || root["log_dir"].AsString().empty()) {
    *error = "missing required \"log_dir\"（相对 examples/log/ 的路径）";
    return false;
  }
  scene->log_dir = root["log_dir"].AsString();
  if (scene->log_dir[0] == '/' ||
      scene->log_dir.find("..") != std::string::npos) {
    *error = "invalid \"log_dir\"：须为相对路径且不含 ..";
    return false;
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

std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargets(
    const std::vector<pe::EvaluationTruthTarget>& truth) {
  std::vector<sbirs_sensor::session::SbirsSceneTarget> targets;
  targets.reserve(truth.size());
  for (std::size_t i = 0U; i < truth.size(); ++i) {
    const pe::EvaluationTruthTarget& item = truth[i];
    sbirs_sensor::session::SbirsSceneTarget scene_target;
    scene_target.target_id = item.key;
    scene_target.target_name = "eval_truth";
    scene_target.position_ecef_m = sbirs_sensor::session::SbirsVector3M{
        item.position_ecef_m.x_m, item.position_ecef_m.y_m, item.position_ecef_m.z_m};
    scene_target.radiant_intensity_w_per_sr = item.radiant_intensity_w_per_sr;
    scene_target.active = item.active;
    scene_target.has_velocity_ecef_m_per_s = item.has_velocity;
    scene_target.velocity_ecef_m_per_s = sbirs_sensor::session::SbirsVector3M{
        item.velocity_ecef_m_per_s.x_mps, item.velocity_ecef_m_per_s.y_mps,
        item.velocity_ecef_m_per_s.z_mps};
    targets.push_back(scene_target);
  }
  return targets;
}

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0
            << " [--scene <path>] [--cycles <n>] [--output-dir <dir>]\n"
            << "  --scene       JSON 场景（默认 " << ONEQ_SCENE_JSON << "）\n"
            << "  --cycles      覆盖场景 cycles\n"
            << "  --output-dir  验收文件目录（默认 " << PE_DEFAULT_OUTPUT_DIR
            << "/<场景名>/）\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string scene_path = ONEQ_SCENE_JSON;
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
    output_dir = std::string(PE_DEFAULT_OUTPUT_DIR) + "/" + scene.log_dir;
  }
  if (component_attachment::app::IsInsideTempArea(output_dir)) {
    std::cerr << "refusing output dir \"" << output_dir
              << "\": 场景产物禁止落到系统临时目录（强制约束；输出位置由场景 "
                 "JSON 的 log_dir 声明，--output-dir 仅允许非临时目录覆盖）\n";
    return 1;
  }
  std::error_code fs_error;
  app_fs::create_directories(output_dir, fs_error);
  if (fs_error) {
    std::cerr << "Failed to create output dir \"" << output_dir << "\": " << fs_error.message()
              << "\n";
    return 1;
  }
  component_attachment::app::InitIntegrationLog(output_dir);

#if !defined(PE_ACCEPTANCE_LOG_ENABLED) || !PE_ACCEPTANCE_LOG_ENABLED
  std::cerr << "warning: ONEQ_ENABLE_PRECISION_EVALUATION_LOG=OFF; "
            << output_dir << "/precision_acceptance.log will not be written\n";
#endif

  scene.config.fusion.enable_track_filtering = true;
  std::uint32_t source_a = scene.config.satellite_a_source_id;
  std::uint32_t source_b = scene.config.satellite_b_source_id;
  if (source_b == source_a) {
    source_b = source_a + 100U;
  }

  component_attachment::AppSceneState app_scene;
  app_scene.sbirs_utc_julian_day = scene.utc_julian_day;
  component_attachment::World world(app_scene);

  const sbirs_sensor::session::SbirsVector3M pose_a(
      scene.ephemeris.satellite_a_position_ecef_m.x_m,
      scene.ephemeris.satellite_a_position_ecef_m.y_m,
      scene.ephemeris.satellite_a_position_ecef_m.z_m);
  const sbirs_sensor::session::SbirsVector3M vel_a(
      scene.ephemeris.satellite_a_velocity_ecef_m_per_s.x_mps,
      scene.ephemeris.satellite_a_velocity_ecef_m_per_s.y_mps,
      scene.ephemeris.satellite_a_velocity_ecef_m_per_s.z_mps);
  const sbirs_sensor::session::SbirsEulerAnglesDeg att_a(
      scene.ephemeris.satellite_a_attitude_yaw_deg, scene.ephemeris.satellite_a_attitude_pitch_deg,
      scene.ephemeris.satellite_a_attitude_roll_deg);
  const sbirs_sensor::session::SbirsVector3M pose_b(
      scene.ephemeris.satellite_b_position_ecef_m.x_m,
      scene.ephemeris.satellite_b_position_ecef_m.y_m,
      scene.ephemeris.satellite_b_position_ecef_m.z_m);
  const sbirs_sensor::session::SbirsVector3M vel_b(
      scene.ephemeris.satellite_b_velocity_ecef_m_per_s.x_mps,
      scene.ephemeris.satellite_b_velocity_ecef_m_per_s.y_mps,
      scene.ephemeris.satellite_b_velocity_ecef_m_per_s.z_mps);
  const sbirs_sensor::session::SbirsEulerAnglesDeg att_b(
      scene.ephemeris.satellite_b_attitude_yaw_deg, scene.ephemeris.satellite_b_attitude_pitch_deg,
      scene.ephemeris.satellite_b_attitude_roll_deg);

  component_attachment::Entity& satellite_a = world.CreateEntity("satellite_a");
  satellite_a.Attach(std::unique_ptr<component_attachment::SbirsSensorComponent>(
      new component_attachment::SbirsSensorComponent(
          sbirs_sensor::session::SbirsSession::Create(scene.config.satellite_a), source_a, pose_a,
          vel_a, att_a)));
  component_attachment::Entity& satellite_b = world.CreateEntity("satellite_b");
  satellite_b.Attach(std::unique_ptr<component_attachment::SbirsSensorComponent>(
      new component_attachment::SbirsSensorComponent(
          sbirs_sensor::session::SbirsSession::Create(scene.config.satellite_b), source_b, pose_b,
          vel_b, att_b)));

  component_attachment::Entity& ground_station = world.CreateEntity("ground_station");
  ground_station.Attach(std::unique_ptr<component_attachment::FusionComponent>(
      new component_attachment::FusionComponent(
          std::unique_ptr<fusion::FusionEngine>(new fusion::FusionEngine(scene.config.fusion)),
          std::unique_ptr<pe::PrecisionEvaluationSession>(
              new pe::PrecisionEvaluationSession(scene.config)),
          source_a, source_b)));
  component_attachment::FusionComponent* fusion =
      ground_station.Find<component_attachment::FusionComponent>();

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
    app_scene.cycle = cycle;
    app_scene.t_sec = static_cast<double>(cycle) * static_cast<double>(scene.dt_sec);
    app_scene.sbirs_targets = MakeSbirsTargets(scene.truth);
    app_scene.detection_pool.clear();
    app_scene.sbirs_ground_station_inbox.clear();
    fusion->SetEvaluationInputs(scene.ephemeris, scene.truth);
    component_attachment::app::BeginViewLogCycle(cycle);
    world.Step(static_cast<double>(scene.dt_sec));
    const pe::PrecisionEvaluationCycleResult& result = fusion->last_evaluation();
    if (!result.dual_sat.empty()) {
      ++dual_sat_cycles;
    }
    std::cout << "cycle=" << cycle << " angular=" << result.angular.size()
              << " dual_sat=" << result.dual_sat.size()
              << " velocity=" << result.velocity.size()
              << " keypoints=" << result.keypoints.size() << "\n";
  }

  const pe::PrecisionEvaluationReport report = fusion->SummarizeEvaluation();
  PrintReport(report);
  std::cout << "dual_sat_cycles=" << dual_sat_cycles << "/" << scene.cycles << "\n";
  component_attachment::app::FlushIntegrationLog();

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
