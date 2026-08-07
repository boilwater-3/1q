/**
 * @file scene_data.cpp
 * @brief 场景描述加载实现（见 scene_data.h）。
 */

#include "scene_data.h"

#include <cstdint>
#include <string>
#include <utility>

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
