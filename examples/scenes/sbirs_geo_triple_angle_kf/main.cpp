/**
 * @file sbirs_geo_triple_angle_kf/main.cpp
 * @brief 三颗地球静止轨道卫星 + 实验角度域线性 KF（kAngleCvKf）。
 *
 * 几何与消息地面站同 sbirs_dual_sat_fix_messages；每星 Estimated 后端改为
 * 4 维视线角恒速 KF。精度评估交会仍只用前两颗星（库 API 是双视线）。
 */

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "app/command_routing.h"
#include "app/fs_compat.h"

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "json_reader.h"
#include "app/output_dir.h"
#include "components/ground_station_fusion_component.h"
#include "components/sbirs_sensor_component.h"
#include "core/scene_types.h"
#include "core/world.h"
#include "logger/logger.h"
#include "logger/acceptance_paths.h"
#include "logger/acceptance_timing.h"
#include "config_loaders/sbirs_sensor/config_loader_common.h"

namespace pe = precision_evaluation;

namespace {

#ifndef ONEQ_SCENE_JSON
#define ONEQ_SCENE_JSON \
  "examples/scenes/sbirs_geo_triple_angle_kf/sbirs_geo_triple_angle_kf.json"
#endif
#ifndef PE_DEFAULT_OUTPUT_DIR
#define PE_DEFAULT_OUTPUT_DIR "examples/log"
#endif

struct LoadedSatellite {
  std::string id{};
  std::uint32_t source_id{0U};
  sbirs_sensor::config::SbirsSessionConfig config{};
  oneq::coordinate::EcefPositionM position_ecef_m{};
  oneq::coordinate::EcefVelocityMps velocity_ecef_m_per_s{};
  double attitude_yaw_deg{0.0};
  double attitude_pitch_deg{0.0};
  double attitude_roll_deg{0.0};
};

struct LoadedScene {
  std::uint32_t cycles{60U};
  std::string log_dir{};
  float dt_sec{1.0f};
  double utc_julian_day{2451544.2230698913};
  pe::PrecisionEvaluationConfig config{};
  pe::DualSatEphemerisInput ephemeris{};
  std::vector<LoadedSatellite> satellites{};
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

bool ReadLla(const examples::JsonValue& value, oneq::coordinate::LlaPositionDegM* lla,
             const char* name, std::string* error) {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
  if (!ReadVec3(value, &lat_deg, &lon_deg, &alt_m, name, error)) {
    return false;
  }
  *lla = oneq::coordinate::LlaPositionDegM(lat_deg, lon_deg, alt_m);
  if (!oneq::coordinate::IsValid(*lla)) {
    *error = std::string(name) + " is not a valid LLA [lat_deg, lon_deg, alt_m]";
    return false;
  }
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
  if (block.Has("scan_center_el_deg")) {
    config.mission.scan_center_el_deg = static_cast<float>(block["scan_center_el_deg"].AsDouble());
  }
  if (block.Has("max_concurrent_nfov_locks")) {
    config.policy.scheduler.max_concurrent_nfov_locks =
        static_cast<int>(block["max_concurrent_nfov_locks"].AsInt());
  }
  config.policy.detection.wide_min_snr_linear =
      block.Has("wide_min_snr_linear")
          ? static_cast<float>(block["wide_min_snr_linear"].AsDouble())
          : 0.001f;
  config.policy.detection.narrow_min_snr_linear =
      block.Has("narrow_min_snr_linear")
          ? static_cast<float>(block["narrow_min_snr_linear"].AsDouble())
          : 0.001f;
  if (block.Has("tracking_mode")) {
    config.policy.tracking.tracking_mode =
        examples::SbirsTrackingModeFromString(block["tracking_mode"].AsString());
  }
  if (block.Has("estimated_backend")) {
    config.policy.tracking.estimated_backend =
        examples::SbirsEstimatedTrackingBackendFromString(block["estimated_backend"].AsString());
  }
  if (block.Has("process_noise_diff_coeff")) {
    config.policy.tracking.process_noise_diff_coeff =
        static_cast<float>(block["process_noise_diff_coeff"].AsDouble());
  }
  return config;
}

bool LoadEphemerisSat(const examples::JsonValue& block, oneq::coordinate::EcefPositionM* position,
                      oneq::coordinate::EcefVelocityMps* velocity, std::string* error,
                      const char* tag) {
  if (block.Has("position_lla_deg_m")) {
    oneq::coordinate::LlaPositionDegM lla;
    if (!ReadLla(block["position_lla_deg_m"], &lla,
                 (std::string(tag) + ".position_lla_deg_m").c_str(), error)) {
      return false;
    }
    if (!oneq::coordinate::TryLlaToEcef(lla, position)) {
      *error = std::string(tag) + ".position_lla_deg_m could not be converted to ECEF";
      return false;
    }
  } else {
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    if (!ReadVec3(block["position_ecef_m"], &px, &py, &pz,
                  (std::string(tag) + ".position_ecef_m").c_str(), error)) {
      return false;
    }
    *position = oneq::coordinate::EcefPositionM(px, py, pz);
  }
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
  if (root.Has("fusion")) {
    const examples::JsonValue& fusion = root["fusion"];
    if (fusion.Has("track_bearing_init_range_m")) {
      scene->config.fusion.track_bearing_init_range_m =
          fusion["track_bearing_init_range_m"].AsDouble();
    }
    if (fusion.Has("track_bearing_init_range_std_m")) {
      scene->config.fusion.track_bearing_init_range_std_m =
          fusion["track_bearing_init_range_std_m"].AsDouble();
    }
    if (fusion.Has("relay_fov_width_deg")) {
      scene->config.fusion.relay_fov_width_deg = fusion["relay_fov_width_deg"].AsDouble();
    }
  }

  const examples::JsonValue& satellites = root["satellites"];
  if (satellites.IsNull() || satellites.Size() < 2U) {
    *error = "satellites must be an array of at least two objects (first two used for PE)";
    return false;
  }
  scene->satellites.clear();
  scene->satellites.reserve(satellites.Size());
  for (std::size_t i = 0U; i < satellites.Size(); ++i) {
    const examples::JsonValue& block = satellites[i];
    LoadedSatellite sat;
    sat.id = block.Has("id") ? block["id"].AsString() : ("sat" + std::to_string(i));
    sat.source_id = block.Has("source_id")
                        ? static_cast<std::uint32_t>(block["source_id"].AsInt())
                        : (4U + static_cast<std::uint32_t>(i) * 100U);
    sat.config = LoadSatelliteConfig(block);
    if (!LoadEphemerisSat(block, &sat.position_ecef_m, &sat.velocity_ecef_m_per_s, error,
                          ("satellites[" + std::to_string(i) + "]").c_str())) {
      return false;
    }
    if (block.Has("attitude_yaw_deg")) {
      sat.attitude_yaw_deg = block["attitude_yaw_deg"].AsDouble();
    }
    if (block.Has("attitude_pitch_deg")) {
      sat.attitude_pitch_deg = block["attitude_pitch_deg"].AsDouble();
    }
    if (block.Has("attitude_roll_deg")) {
      sat.attitude_roll_deg = block["attitude_roll_deg"].AsDouble();
    }
    scene->satellites.push_back(sat);
  }
  for (std::size_t i = 0U; i < scene->satellites.size(); ++i) {
    for (std::size_t j = i + 1U; j < scene->satellites.size(); ++j) {
      if (scene->satellites[i].source_id == scene->satellites[j].source_id) {
        *error = "satellites[].source_id must be unique";
        return false;
      }
    }
  }
  scene->config.satellite_a = scene->satellites[0].config;
  scene->config.satellite_b = scene->satellites[1].config;
  scene->config.satellite_a_source_id = scene->satellites[0].source_id;
  scene->config.satellite_b_source_id = scene->satellites[1].source_id;
  scene->ephemeris.satellite_a_position_ecef_m = scene->satellites[0].position_ecef_m;
  scene->ephemeris.satellite_a_velocity_ecef_m_per_s = scene->satellites[0].velocity_ecef_m_per_s;
  scene->ephemeris.satellite_a_attitude_yaw_deg = scene->satellites[0].attitude_yaw_deg;
  scene->ephemeris.satellite_a_attitude_pitch_deg = scene->satellites[0].attitude_pitch_deg;
  scene->ephemeris.satellite_a_attitude_roll_deg = scene->satellites[0].attitude_roll_deg;
  scene->ephemeris.satellite_b_position_ecef_m = scene->satellites[1].position_ecef_m;
  scene->ephemeris.satellite_b_velocity_ecef_m_per_s = scene->satellites[1].velocity_ecef_m_per_s;
  scene->ephemeris.satellite_b_attitude_yaw_deg = scene->satellites[1].attitude_yaw_deg;
  scene->ephemeris.satellite_b_attitude_pitch_deg = scene->satellites[1].attitude_pitch_deg;
  scene->ephemeris.satellite_b_attitude_roll_deg = scene->satellites[1].attitude_roll_deg;

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
    oneq::coordinate::LlaPositionDegM target_lla;
    bool have_lla = false;
    if (block.Has("position_lla_deg_m")) {
      if (!ReadLla(block["position_lla_deg_m"], &target_lla, "targets[].position_lla_deg_m",
                   error)) {
        return false;
      }
      if (!oneq::coordinate::TryLlaToEcef(target_lla, &truth.position_ecef_m)) {
        *error = "targets[].position_lla_deg_m could not be converted to ECEF";
        return false;
      }
      have_lla = true;
    } else {
      double px = 0.0;
      double py = 0.0;
      double pz = 0.0;
      if (!ReadVec3(block["position_ecef_m"], &px, &py, &pz, "targets[].position_ecef_m", error)) {
        return false;
      }
      truth.position_ecef_m = oneq::coordinate::EcefPositionM(px, py, pz);
    }
    if (block.Has("velocity_enu_mps")) {
      double ve = 0.0;
      double vn = 0.0;
      double vu = 0.0;
      if (!ReadVec3(block["velocity_enu_mps"], &ve, &vn, &vu, "targets[].velocity_enu_mps",
                    error)) {
        return false;
      }
      if (!have_lla &&
          !oneq::coordinate::TryEcefToLla(truth.position_ecef_m, &target_lla)) {
        *error = "targets[].velocity_enu_mps needs a convertible position for ENU axes";
        return false;
      }
      const oneq::coordinate::EnuVelocityMps enu_velocity(ve, vn, vu);
      if (!oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, target_lla,
                                                  &truth.velocity_ecef_m_per_s)) {
        *error = "targets[].velocity_enu_mps could not be converted to ECEF";
        return false;
      }
      truth.has_velocity = true;
    } else if (block.Has("velocity_ecef_mps")) {
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
  // 评审 2026-08-26 条12：装配层接好落点预报外发事件（on_impact_forecast_published），
  // 推演验收行据此写「分发状态=已发布(事件…)」。
  scene.config.inference.impact_distribution_channel = "on_impact_forecast_published";

  component_attachment::AppSceneState app_scene;
  app_scene.sbirs_utc_julian_day = scene.utc_julian_day;
  component_attachment::World world(app_scene);

  for (std::size_t i = 0U; i < scene.satellites.size(); ++i) {
    const LoadedSatellite& sat = scene.satellites[i];
    const sbirs_sensor::session::SbirsVector3M pose(sat.position_ecef_m.x_m, sat.position_ecef_m.y_m,
                                                    sat.position_ecef_m.z_m);
    const sbirs_sensor::session::SbirsVector3M vel(sat.velocity_ecef_m_per_s.x_mps,
                                                   sat.velocity_ecef_m_per_s.y_mps,
                                                   sat.velocity_ecef_m_per_s.z_mps);
    const sbirs_sensor::session::SbirsEulerAnglesDeg att(
        sat.attitude_yaw_deg, sat.attitude_pitch_deg, sat.attitude_roll_deg);
    component_attachment::Entity& entity = world.CreateEntity("satellite_" + sat.id);
    entity.Attach(std::unique_ptr<component_attachment::SbirsSensorComponent>(
        new component_attachment::SbirsSensorComponent(
            sbirs_sensor::session::SbirsSession::Create(sat.config), sat.source_id, pose, vel, att,
            component_attachment::SbirsGroundDeliveryMode::kMessage)));
  }
  std::vector<std::uint32_t> evaluation_source_ids;
  evaluation_source_ids.push_back(scene.satellites[0].source_id);
  evaluation_source_ids.push_back(scene.satellites[1].source_id);

  // 实体先建：分发来源/对象实体 ID 在引擎构造前注入推演配置（验收判定标准
  // 第25项·修改：已发布行须写实体 ID 对，不用通道名/角色名代替）。
  component_attachment::Entity& ground_station = world.CreateEntity("ground_station");
  component_attachment::Entity& impact_receiver = world.CreateEntity("impact_receiver");
  scene.config.inference.impact_distribution_source_id = ground_station.id();
  scene.config.inference.impact_distribution_target_id = impact_receiver.id();

  // 评审 2026-08-26 条22（方案B）：融合/精度会话初始化墙钟写 integration_events.log
  // （库内验收文件的「初始化时间」行指引到此处）。
  const auto fusion_create_begin = std::chrono::steady_clock::now();
  std::unique_ptr<fusion::FusionEngine> fusion_engine(
      new fusion::FusionEngine(scene.config.fusion));
  const double fusion_create_ms =
      component_attachment::app::SteadyElapsedMs(fusion_create_begin);
  component_attachment::app::LogAcceptanceMs(0U, 0.0, "初始化时间性能测试", "Fusion", fusion_create_ms);
  const auto precision_create_begin = std::chrono::steady_clock::now();
  std::unique_ptr<pe::PrecisionEvaluationSession> precision_session(
      new pe::PrecisionEvaluationSession(scene.config));
  const double precision_create_ms =
      component_attachment::app::SteadyElapsedMs(precision_create_begin);
  component_attachment::app::LogAcceptanceMs(0U, 0.0, "初始化时间性能测试", "Precision",
                                             precision_create_ms);

  ground_station.Attach(
      std::unique_ptr<component_attachment::GroundStationFusionComponent>(
          new component_attachment::GroundStationFusionComponent(std::move(fusion_engine),
                                                                 std::move(precision_session),
                                                                 evaluation_source_ids)));
  component_attachment::GroundStationFusionComponent* fusion =
      ground_station.Find<component_attachment::GroundStationFusionComponent>();

  // 评审 2026-08-26 条12：订阅落点预报外发事件，收件回执写 integration_events.log
  // （验证分发链路真实可达，非仅日志口径；接收方为 impact_receiver 实体）。
  component_attachment::app::ImpactForecastReceiptListener impact_receipt(
      world, static_cast<std::uint64_t>(impact_receiver.id()));
  (void)impact_receipt;

  std::cout << "sbirs_geo_triple_angle_kf: " << scene_path << ", " << scene.cycles
            << " cycles x " << scene.dt_sec << " s, satellites=" << scene.satellites.size()
            << " GEO (ground-station message delivery, kAngleCvKf)\n"
            << "acceptance logs -> " << output_dir
            << " (precision/sbirs/fusion/inference_acceptance.log)\n";
  for (std::size_t i = 0U; i < scene.satellites.size(); ++i) {
    const LoadedSatellite& sat = scene.satellites[i];
    std::cout << "  sat " << sat.id << " source_id=" << sat.source_id << " ecef=("
              << sat.position_ecef_m.x_m << "," << sat.position_ecef_m.y_m << ","
              << sat.position_ecef_m.z_m << ") backend="
              << static_cast<int>(sat.config.policy.tracking.estimated_backend)
              << " q=" << sat.config.policy.tracking.process_noise_diff_coeff << "\n";
  }

  std::ofstream angle_track((output_dir + "/angle_track.csv").c_str());
  std::ofstream angle_error((output_dir + "/pe_angular.csv").c_str());
  std::ofstream dual_sat_csv((output_dir + "/pe_dual_sat.csv").c_str());
  angle_track << "cycle,source_id,target_id,azimuth_deg,elevation_deg\n";
  angle_error << "cycle,satellite_index,key,azimuth_error_deg,elevation_error_deg\n";
  dual_sat_csv << "cycle,key,position_error_m,los_residual_m\n";

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
    fusion->BeginCycle(world, cycle);
    fusion->SetEvaluationInputs(scene.ephemeris, scene.truth);
    component_attachment::app::BeginViewLogCycle(cycle);
    world.Step(static_cast<double>(scene.dt_sec));
    const pe::PrecisionEvaluationCycleResult& result = fusion->last_evaluation();
    if (!result.dual_sat.empty()) {
      ++dual_sat_cycles;
    }
    const std::vector<component_attachment::SbirsFrameSubmittedEvent>& frames =
        fusion->last_sbirs_frames();
    for (std::size_t f = 0U; f < frames.size(); ++f) {
      const component_attachment::SbirsFrameSubmittedEvent& frame = frames[f];
      for (std::size_t d = 0U; d < frame.detections.size(); ++d) {
        const component_attachment::SbirsBearingSample& sample = frame.detections[d];
        angle_track << cycle << "," << frame.source_id << "," << sample.target_id << ","
                    << (static_cast<double>(sample.azimuth_rad) * 57.29577951308232) << ","
                    << (static_cast<double>(sample.elevation_rad) * 57.29577951308232) << "\n";
      }
    }
    for (std::size_t i = 0U; i < result.angular.size(); ++i) {
      const pe::AngularErrorSample& sample = result.angular[i];
      angle_error << sample.cycle_index << "," << sample.satellite_index << "," << sample.key << ","
                  << sample.azimuth_error_deg << "," << sample.elevation_error_deg << "\n";
    }
    for (std::size_t i = 0U; i < result.dual_sat.size(); ++i) {
      const pe::DualSatFixSample& sample = result.dual_sat[i];
      dual_sat_csv << sample.cycle_index << "," << sample.key << "," << sample.position_error_m
                   << "," << sample.los_residual_m << "\n";
    }
    std::cout << "cycle=" << cycle << " inbox_sats=" << fusion->last_sbirs_frame_count()
              << " fused=" << fusion->targets().size() << " angular=" << result.angular.size()
              << " dual_sat=" << result.dual_sat.size()
              << " velocity=" << result.velocity.size()
              << " keypoints=" << result.keypoints.size();
    if (!fusion->targets().empty()) {
      std::cout << " channels=";
      for (std::size_t t = 0U; t < fusion->targets().size(); ++t) {
        const fusion::FusedTarget& target = fusion->targets()[t];
        if (t > 0U) {
          std::cout << " | ";
        }
        std::cout << "k" << target.key << "[";
        for (std::size_t c = 0U; c < target.channels.size(); ++c) {
          if (c > 0U) {
            std::cout << ",";
          }
          std::cout << target.channels[c].source_id << ":" << target.channels[c].sample_count;
        }
        std::cout << "]";
      }
    }
    std::cout << "\n";
  }

  const pe::PrecisionEvaluationReport report = fusion->SummarizeEvaluation();
  PrintReport(report);
  std::cout << "dual_sat_cycles=" << dual_sat_cycles << "/" << scene.cycles << "\n";
  // 验收判定标准 第54项：场景数/总仿真周期由示例层结束时回写。
  component_attachment::app::LogAcceptanceText(
      0U, 0.0, "可支持连续运行次数性能测试",
      std::string("场景数=1 总仿真周期=") + std::to_string(scene.cycles));
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
