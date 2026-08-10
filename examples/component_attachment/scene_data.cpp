/**
 * @file scene_data.cpp
 * @brief 场景描述加载实现（见 scene_data.h）。
 */

#include "scene_data.h"

#include <cstdint>
#include <string>
#include <utility>

#include "1q/navigation/AreaCoveragePlanner.h"
#include "json_reader.h"

namespace component_attachment {
namespace demo {
namespace {

/// 读取浮点字段：缺省返回默认值（遵循 config_loader 静默默认惯例）。
double ReadDouble(const examples::JsonValue& value, const char* key, double default_value) {
  return value[key].IsNull() ? default_value : value[key].AsDouble();
}

/// 读取整数字段：缺省返回默认值。
std::int64_t ReadInt(const examples::JsonValue& value, const char* key,
                     std::int64_t default_value) {
  return value[key].IsNull() ? default_value : value[key].AsInt();
}

/// 必填几何字段校验：缺失时置 error 并返回 false（几何字段静默为 0 会让
/// 场景在错误位置"合理"运行，浪费整轮验证——此处严格于调参字段）。
bool RequireGeometry(const examples::JsonValue& value, const std::string& block,
                     const char* key, std::string* error) {
  if (value[key].IsNull()) {
    *error = "missing required field \"" + std::string(key) + "\" in \"" + block + "\"";
    return false;
  }
  return true;
}

/// 解析可选 coverage 块（区域巡逻任务）：区域（多边形/圆形）+ 规划参数。
/// 区域形态/模式不匹配等结构性错误在此报出；几何深度校验（顶点 < 3、
/// 间距非正、坐标非法等）由 AreaCoveragePlanner 承担（空计划 → 调用方报错）。
bool ParseCoverageTask(const examples::JsonValue& coverage, double cruise_altitude_m,
                       double cruise_speed_mps, CoverageTask* task, std::string* error) {
  if (coverage["kind"].IsString()) {
    const std::string kind = coverage["kind"].AsString();
    if (kind == "polygon") {
      task->area.kind = navigation::CoverageAreaKind::kPolygon;
    } else if (kind == "circle") {
      task->area.kind = navigation::CoverageAreaKind::kCircle;
    } else {
      *error = "invalid \"coverage.kind\" (must be \"polygon\" or \"circle\")";
      return false;
    }
  }
  if (coverage["mode"].IsString()) {
    const std::string mode = coverage["mode"].AsString();
    if (mode == "scan") {
      task->config.mode = navigation::CoverageMode::kScan;
    } else if (mode == "orbit") {
      task->config.mode = navigation::CoverageMode::kOrbit;
    } else {
      *error = "invalid \"coverage.mode\" (must be \"scan\" or \"orbit\")";
      return false;
    }
  }

  if (task->area.kind == navigation::CoverageAreaKind::kPolygon) {
    const examples::JsonValue& vertices = coverage["vertices"];
    if (vertices.IsNull() || vertices.type() != examples::JsonValue::kArray) {
      *error = "\"coverage.vertices\" must be an array (polygon region)";
      return false;
    }
    for (std::size_t i = 0U; i < vertices.Size(); ++i) {
      const examples::JsonValue& vertex = vertices[i];
      const std::string block = "coverage.vertices[" + std::to_string(i) + "]";
      if (!RequireGeometry(vertex, block, "lat_deg", error) ||
          !RequireGeometry(vertex, block, "lon_deg", error)) {
        return false;
      }
      oneq::coordinate::LlaPositionDegM point;
      point.latitude_deg = vertex["lat_deg"].AsDouble();
      point.longitude_deg = vertex["lon_deg"].AsDouble();
      point.altitude_m = ReadDouble(vertex, "alt_m", 0.0);
      task->area.polygon.vertices.push_back(point);
    }
  } else {
    const examples::JsonValue& center = coverage["center"];
    if (center.IsNull() || center.type() != examples::JsonValue::kObject) {
      *error = "\"coverage.center\" must be an object (circle region)";
      return false;
    }
    if (!RequireGeometry(center, "coverage.center", "lat_deg", error) ||
        !RequireGeometry(center, "coverage.center", "lon_deg", error)) {
      return false;
    }
    task->area.circle.center.latitude_deg = center["lat_deg"].AsDouble();
    task->area.circle.center.longitude_deg = center["lon_deg"].AsDouble();
    task->area.circle.center.altitude_m = ReadDouble(center, "alt_m", 0.0);
    task->area.circle.radius_m = ReadDouble(coverage, "radius_m", 0.0);
  }

  task->config.scan_heading_deg = ReadDouble(coverage, "scan_heading_deg",
                                             task->config.scan_heading_deg);
  task->config.scan_spacing_m = ReadDouble(coverage, "scan_spacing_m",
                                           task->config.scan_spacing_m);
  // 高度/速度缺省回退巡航参数（与 platform.waypoints 条目缺省语义一致）。
  task->config.altitude_m = ReadDouble(coverage, "altitude_m", cruise_altitude_m);
  task->config.speed_mps = ReadDouble(coverage, "speed_mps", cruise_speed_mps);
  task->config.arrival_radius_m = ReadDouble(coverage, "arrival_radius_m", 500.0);
  task->config.orbit_segments = static_cast<std::size_t>(
      ReadInt(coverage, "orbit_segments", static_cast<std::int64_t>(task->config.orbit_segments)));
  task->config.orbit_rings = static_cast<std::size_t>(
      ReadInt(coverage, "orbit_rings", static_cast<std::int64_t>(task->config.orbit_rings)));
  return true;
}

}  // namespace

bool LoadSceneData(const char* path, SceneData* scene, std::string* error) {
  if (scene == nullptr || error == nullptr) {
    return false;
  }
  error->clear();

  examples::JsonValue root;
  if (!examples::JsonReader::ParseFile(path, &root, error)) {
    return false;
  }
  if (root.IsNull() || root.type() != examples::JsonValue::kObject) {
    *error = "scene root must be a JSON object";
    return false;
  }

  SceneData out;  // 成员初始化值 = 缺省字段值
  if (root["name"].IsString()) {
    out.name = root["name"].AsString();
  }
  if (root["cycles"].IsInt()) {
    const std::int64_t cycles = root["cycles"].AsInt();
    if (cycles <= 0) {
      *error = "invalid \"cycles\": must be > 0";
      return false;
    }
    out.cycles = static_cast<std::uint32_t>(cycles);
  }
  out.dt_sec = ReadDouble(root, "dt_sec", out.dt_sec);

  // 平台块（必填）：原点/起飞航向/巡航参数 + 可选航路。
  const examples::JsonValue& platform = root["platform"];
  if (platform.IsNull() || platform.type() != examples::JsonValue::kObject) {
    *error = "missing required block \"platform\"";
    return false;
  }
  for (const char* key : {"origin_lat_deg", "origin_lon_deg"}) {
    if (!RequireGeometry(platform, "platform", key, error)) {
      return false;
    }
  }
  out.platform_origin.latitude_deg = platform["origin_lat_deg"].AsDouble();
  out.platform_origin.longitude_deg = platform["origin_lon_deg"].AsDouble();
  out.platform_origin.altitude_m = ReadDouble(platform, "origin_alt_m", 0.0);
  out.initial_heading_deg = ReadDouble(platform, "initial_heading_deg", out.initial_heading_deg);
  out.cruise_altitude_m = ReadDouble(platform, "cruise_altitude_m", out.cruise_altitude_m);
  out.cruise_speed_mps = ReadDouble(platform, "cruise_speed_mps", out.cruise_speed_mps);

  const examples::JsonValue& waypoints = platform["waypoints"];
  if (!waypoints.IsNull()) {
    if (waypoints.type() != examples::JsonValue::kArray) {
      *error = "\"platform.waypoints\" must be an array";
      return false;
    }
    for (std::size_t i = 0U; i < waypoints.Size(); ++i) {
      const examples::JsonValue& wp = waypoints[i];
      if (wp["lat_deg"].IsNull() || wp["lon_deg"].IsNull()) {
        *error = "waypoint[" + std::to_string(i) + "] missing lat_deg/lon_deg";
        return false;
      }
      navigation::RoutePoint point;
      point.position.latitude_deg = wp["lat_deg"].AsDouble();
      point.position.longitude_deg = wp["lon_deg"].AsDouble();
      point.position.altitude_m = ReadDouble(wp, "alt_m", out.cruise_altitude_m);
      point.speed_mps = ReadDouble(wp, "speed_mps", out.cruise_speed_mps);
      point.radius_m = ReadDouble(wp, "radius_m", 500.0);
      out.waypoints.push_back(point);
    }
  }

  // 区域巡逻块（可选顶层块）：与显式航路互斥（航路来源歧义直接报错）。
  // 规划成功时 waypoints 为规划器输出的巡逻航路（planned=true，平台循环巡逻）；
  // 规划失败（顶点不足/间距非正/模式-区域不匹配等 → 空计划）报错退出，
  // 不允许静默退化为直飞。
  const examples::JsonValue& coverage = root["coverage"];
  if (!coverage.IsNull()) {
    if (coverage.type() != examples::JsonValue::kObject) {
      *error = "\"coverage\" must be an object";
      return false;
    }
    if (!waypoints.IsNull()) {
      *error = "\"platform.waypoints\" and \"coverage\" are mutually exclusive "
               "(explicit waypoints vs planned patrol route)";
      return false;
    }
    if (!ParseCoverageTask(coverage, out.cruise_altitude_m, out.cruise_speed_mps,
                           &out.coverage, error)) {
      return false;
    }
    navigation::AreaCoveragePlanner planner;
    out.waypoints = planner.Plan(out.coverage.area, out.coverage.config);
    if (out.waypoints.empty()) {
      *error = "\"coverage\" planning failed: invalid region or mode-kind mismatch "
               "(need >= 3 vertices / positive scan_spacing_m / positive radius_m)";
      return false;
    }
    out.coverage.planned = true;
  }

  // 目标块（必填数组；可为空 = "无目标"场景，冒烟下限由 smoke 块置 0）。
  const examples::JsonValue& targets = root["targets"];
  if (targets.IsNull() || targets.type() != examples::JsonValue::kArray) {
    *error = "missing required block \"targets\" (array)";
    return false;
  }
  for (std::size_t i = 0U; i < targets.Size(); ++i) {
    const examples::JsonValue& t = targets[i];
    for (const char* key : {"id", "azimuth_deg", "range_m", "altitude_m", "rcs_m2"}) {
      if (!RequireGeometry(t, "targets[" + std::to_string(i) + "]", key, error)) {
        return false;
      }
    }
    ScriptedTarget target;
    target.id = static_cast<std::uint32_t>(t["id"].AsInt());
    target.azimuth_deg = t["azimuth_deg"].AsDouble();
    target.range_m = t["range_m"].AsDouble();
    target.altitude_m = t["altitude_m"].AsDouble();
    target.v_east_mps = ReadDouble(t, "v_east_mps", 0.0);
    target.v_north_mps = ReadDouble(t, "v_north_mps", 0.0);
    target.temperature_k = ReadDouble(t, "temperature_k", 0.0);
    target.rcs = t["rcs_m2"].AsDouble();
    target.projected_area_m2 = ReadDouble(t, "projected_area_m2", 0.0);
    target.emitter_center_frequency_hz = ReadDouble(t, "emitter_center_frequency_hz", 0.0);
    // 变速机动表（可选）：start_cycle 必填且严格递增（乱序语义混乱，直接报错）。
    const examples::JsonValue& maneuvers = t["maneuvers"];
    if (!maneuvers.IsNull()) {
      if (maneuvers.type() != examples::JsonValue::kArray) {
        *error = "targets[" + std::to_string(i) + "].maneuvers must be an array";
        return false;
      }
      for (std::size_t m = 0U; m < maneuvers.Size(); ++m) {
        const examples::JsonValue& mv = maneuvers[m];
        if (mv["start_cycle"].IsNull()) {
          *error = "targets[" + std::to_string(i) + "].maneuvers[" +
                   std::to_string(m) + "] missing start_cycle";
          return false;
        }
        TargetManeuver maneuver;
        maneuver.start_cycle = static_cast<std::uint32_t>(mv["start_cycle"].AsInt());
        if (maneuver.start_cycle == 0U ||
            (!target.maneuvers.empty() &&
             maneuver.start_cycle <= target.maneuvers.back().start_cycle)) {
          *error = "targets[" + std::to_string(i) + "].maneuvers[" +
                   std::to_string(m) + "] start_cycle must be > 0 and strictly increasing";
          return false;
        }
        maneuver.v_east_mps = ReadDouble(mv, "v_east_mps", 0.0);
        maneuver.v_north_mps = ReadDouble(mv, "v_north_mps", 0.0);
        target.maneuvers.push_back(maneuver);
      }
    }
    out.targets.push_back(target);
  }

  // ESR 波形块（可选，字段级缺省）。
  const examples::JsonValue& esr = root["esr"];
  if (!esr.IsNull() && esr.type() == examples::JsonValue::kObject) {
    out.esr.peak_gain_dbi = ReadDouble(esr, "peak_gain_dbi", out.esr.peak_gain_dbi);
    out.esr.bandwidth_hz = ReadDouble(esr, "bandwidth_hz", out.esr.bandwidth_hz);
    out.esr.peak_power_w = ReadDouble(esr, "peak_power_w", out.esr.peak_power_w);
    out.esr.pulse_width_s = ReadDouble(esr, "pulse_width_s", out.esr.pulse_width_s);
    out.esr.pri_s = ReadDouble(esr, "pri_s", out.esr.pri_s);
    out.esr.pulse_count =
        static_cast<std::uint32_t>(ReadInt(esr, "pulse_count", out.esr.pulse_count));
    out.esr.timing_seed =
        static_cast<std::uint32_t>(ReadInt(esr, "timing_seed", out.esr.timing_seed));
  }

  // 天基平台块（可选）：凝视目标群质心正上方，高度由场景控制。
  const examples::JsonValue& satellite = root["sbirs_satellite"];
  if (!satellite.IsNull() && satellite.type() == examples::JsonValue::kObject) {
    out.sbirs_satellite_altitude_m =
        ReadDouble(satellite, "altitude_m", out.sbirs_satellite_altitude_m);
  }

  // EOS 扫描块（可选）：覆写 LoadConfigs 的 JSON 原值（下视地面监视 → 水平扫描）。
  const examples::JsonValue& eos = root["eos_scan"];
  if (!eos.IsNull() && eos.type() == examples::JsonValue::kObject) {
    out.eos_frame_rate_hz =
        static_cast<float>(ReadDouble(eos, "frame_rate_hz", out.eos_frame_rate_hz));
    out.eos_scan_rate_deg_per_sec =
        static_cast<float>(ReadDouble(eos, "scan_rate_deg_per_sec", out.eos_scan_rate_deg_per_sec));
    out.eos_scan_start_az_deg =
        static_cast<float>(ReadDouble(eos, "scan_start_az_deg", out.eos_scan_start_az_deg));
    out.eos_scan_end_az_deg =
        static_cast<float>(ReadDouble(eos, "scan_end_az_deg", out.eos_scan_end_az_deg));
    out.eos_scan_center_el_deg =
        static_cast<float>(ReadDouble(eos, "scan_center_el_deg", out.eos_scan_center_el_deg));
    out.eos_boresight_depression_deg =
        static_cast<float>(ReadDouble(eos, "boresight_depression_deg",
                                     out.eos_boresight_depression_deg));
  }

  // SAR 任务几何/链路块（可选）：覆写 sar.json 的远程监视档为场景适配值。
  const examples::JsonValue& sar = root["sar"];
  if (!sar.IsNull() && sar.type() == examples::JsonValue::kObject) {
    out.sar_peak_power_w = ReadDouble(sar, "peak_power_w", out.sar_peak_power_w);
    out.sar_antenna_gain_db = ReadDouble(sar, "antenna_gain_db", out.sar_antenna_gain_db);
    out.sar_max_squint_angle_deg =
        ReadDouble(sar, "max_squint_deg", out.sar_max_squint_angle_deg);
    out.sar_scene_center_latitude_deg =
        ReadDouble(sar, "scene_center_latitude_deg", out.sar_scene_center_latitude_deg);
    out.sar_scene_center_longitude_deg =
        ReadDouble(sar, "scene_center_longitude_deg", out.sar_scene_center_longitude_deg);
    out.sar_scene_center_altitude_m =
        ReadDouble(sar, "scene_center_altitude_m", out.sar_scene_center_altitude_m);
    out.sar_nominal_slant_range_m =
        ReadDouble(sar, "slant_range_m", out.sar_nominal_slant_range_m);
    out.sar_platform_speed_mps =
        ReadDouble(sar, "platform_speed_mps", out.sar_platform_speed_mps);
  }

  // 融合块（可选，缺省 = FusionConfig 默认值）。
  const examples::JsonValue& fusion = root["fusion"];
  if (!fusion.IsNull() && fusion.type() == examples::JsonValue::kObject) {
    out.fusion.position_radius_m =
        ReadDouble(fusion, "position_radius_m", out.fusion.position_radius_m);
    out.fusion.bearing_beamwidth_deg =
        ReadDouble(fusion, "bearing_beamwidth_deg", out.fusion.bearing_beamwidth_deg);
    out.fusion.feature_threshold =
        ReadDouble(fusion, "feature_threshold", out.fusion.feature_threshold);
    out.fusion.window_size = static_cast<std::size_t>(
        ReadInt(fusion, "window_size", static_cast<std::int64_t>(out.fusion.window_size)));
    out.fusion.max_missed_cycles = static_cast<std::size_t>(
        ReadInt(fusion, "max_missed_cycles",
                static_cast<std::int64_t>(out.fusion.max_missed_cycles)));
    const examples::JsonValue& weights = fusion["source_weights"];
    if (!weights.IsNull()) {
      if (weights.type() != examples::JsonValue::kArray) {
        *error = "\"fusion.source_weights\" must be an array";
        return false;
      }
      out.fusion.source_weights.clear();
      for (std::size_t i = 0U; i < weights.Size(); ++i) {
        out.fusion.source_weights.push_back(weights[i].AsDouble());
      }
    }
  }
  out.high_threat_confidence =
      ReadDouble(root, "high_threat_confidence", out.high_threat_confidence);

  // 威胁评估块（可选，缺省 = ThreatEvaluatorConfig 默认值）。
  const examples::JsonValue& threat = root["threat"];
  if (!threat.IsNull() && threat.type() == examples::JsonValue::kObject) {
    out.threat.weight_range =
        ReadDouble(threat, "weight_range", out.threat.weight_range);
    out.threat.weight_speed =
        ReadDouble(threat, "weight_speed", out.threat.weight_speed);
    out.threat.weight_acceleration =
        ReadDouble(threat, "weight_acceleration", out.threat.weight_acceleration);
    out.threat.weight_rcs =
        ReadDouble(threat, "weight_rcs", out.threat.weight_rcs);
    out.threat.weight_target_probability =
        ReadDouble(threat, "weight_target_probability",
                   out.threat.weight_target_probability);
    out.threat.weight_fusion_confidence =
        ReadDouble(threat, "weight_fusion_confidence",
                   out.threat.weight_fusion_confidence);
    out.threat.range_near_m =
        ReadDouble(threat, "range_near_m", out.threat.range_near_m);
    out.threat.range_far_m =
        ReadDouble(threat, "range_far_m", out.threat.range_far_m);
    out.threat.speed_min_mps =
        ReadDouble(threat, "speed_min_mps", out.threat.speed_min_mps);
    out.threat.speed_max_mps =
        ReadDouble(threat, "speed_max_mps", out.threat.speed_max_mps);
    out.threat.acceleration_max_mps2 =
        ReadDouble(threat, "acceleration_max_mps2",
                   out.threat.acceleration_max_mps2);
    out.threat.rcs_min_sqm =
        ReadDouble(threat, "rcs_min_sqm", out.threat.rcs_min_sqm);
    out.threat.rcs_max_sqm =
        ReadDouble(threat, "rcs_max_sqm", out.threat.rcs_max_sqm);
    out.threat.high_threshold =
        ReadDouble(threat, "high_threshold", out.threat.high_threshold);
    out.threat.medium_threshold =
        ReadDouble(threat, "medium_threshold", out.threat.medium_threshold);
  }

  // 冒烟块（可选，缺省全部下限 = 1）。
  const examples::JsonValue& smoke = root["smoke"];
  if (!smoke.IsNull() && smoke.type() == examples::JsonValue::kObject) {
    out.smoke.min_key_events = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_key_events", out.smoke.min_key_events));
    out.smoke.min_sbirs_events = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_sbirs_events", out.smoke.min_sbirs_events));
    out.smoke.min_sar_products = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_sar_products", out.smoke.min_sar_products));
    out.smoke.min_fused_targets = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_fused_targets", out.smoke.min_fused_targets));
  }

  *scene = std::move(out);
  return true;
}

}  // namespace demo
}  // namespace component_attachment
