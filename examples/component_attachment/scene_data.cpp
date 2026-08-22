/**
 * @file scene_data.cpp
 * @brief 场景描述加载实现（见 scene_data.h）。
 */

#include "scene_data.h"

#include <cstdint>
#include <string>
#include <utility>

#include "1q/navigation/AreaCoveragePlanner.h"
#include "area_division.h"
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

/// 解析可选 coverage 块（区域巡逻任务）/ mission_area 块（编队切分任务）：
/// 区域（多边形/圆形）+ 规划参数，block_name 用于报错消息定位（"coverage"
/// 或 "mission_area"）。区域形态/模式不匹配等结构性错误在此报出；几何深度
/// 校验（顶点 < 3、间距非正、坐标非法等）由 AreaCoveragePlanner 承担
/// （空计划 → 调用方报错）。
bool ParseCoverageTask(const examples::JsonValue& block, const std::string& block_name,
                       double cruise_altitude_m, double cruise_speed_mps,
                       navigation::CoverageArea* area, navigation::CoveragePlanConfig* config,
                       std::string* error) {
  if (block["kind"].IsString()) {
    const std::string kind = block["kind"].AsString();
    if (kind == "polygon") {
      area->kind = navigation::CoverageAreaKind::kPolygon;
    } else if (kind == "circle") {
      area->kind = navigation::CoverageAreaKind::kCircle;
    } else {
      *error = "invalid \"" + block_name + ".kind\" (must be \"polygon\" or \"circle\")";
      return false;
    }
  }
  if (block["mode"].IsString()) {
    const std::string mode = block["mode"].AsString();
    if (mode == "scan") {
      config->mode = navigation::CoverageMode::kScan;
    } else if (mode == "orbit") {
      config->mode = navigation::CoverageMode::kOrbit;
    } else {
      *error = "invalid \"" + block_name + ".mode\" (must be \"scan\" or \"orbit\")";
      return false;
    }
  }

  if (area->kind == navigation::CoverageAreaKind::kPolygon) {
    const examples::JsonValue& vertices = block["vertices"];
    if (vertices.IsNull() || vertices.type() != examples::JsonValue::kArray) {
      *error = "\"" + block_name + ".vertices\" must be an array (polygon region)";
      return false;
    }
    for (std::size_t i = 0U; i < vertices.Size(); ++i) {
      const examples::JsonValue& vertex = vertices[i];
      const std::string vertex_block = block_name + ".vertices[" + std::to_string(i) + "]";
      if (!RequireGeometry(vertex, vertex_block, "lat_deg", error) ||
          !RequireGeometry(vertex, vertex_block, "lon_deg", error)) {
        return false;
      }
      oneq::coordinate::LlaPositionDegM point;
      point.latitude_deg = vertex["lat_deg"].AsDouble();
      point.longitude_deg = vertex["lon_deg"].AsDouble();
      point.altitude_m = ReadDouble(vertex, "alt_m", 0.0);
      area->polygon.vertices.push_back(point);
    }
  } else {
    const examples::JsonValue& center = block["center"];
    if (center.IsNull() || center.type() != examples::JsonValue::kObject) {
      *error = "\"" + block_name + ".center\" must be an object (circle region)";
      return false;
    }
    if (!RequireGeometry(center, block_name + ".center", "lat_deg", error) ||
        !RequireGeometry(center, block_name + ".center", "lon_deg", error)) {
      return false;
    }
    area->circle.center.latitude_deg = center["lat_deg"].AsDouble();
    area->circle.center.longitude_deg = center["lon_deg"].AsDouble();
    area->circle.center.altitude_m = ReadDouble(center, "alt_m", 0.0);
    area->circle.radius_m = ReadDouble(block, "radius_m", 0.0);
  }

  config->scan_heading_deg = ReadDouble(block, "scan_heading_deg", config->scan_heading_deg);
  config->scan_spacing_m = ReadDouble(block, "scan_spacing_m", config->scan_spacing_m);
  // 高度/速度缺省回退巡航参数（与 platform.waypoints 条目缺省语义一致）。
  config->altitude_m = ReadDouble(block, "altitude_m", cruise_altitude_m);
  config->speed_mps = ReadDouble(block, "speed_mps", cruise_speed_mps);
  config->arrival_radius_m = ReadDouble(block, "arrival_radius_m", 500.0);
  config->orbit_segments = static_cast<std::size_t>(
      ReadInt(block, "orbit_segments", static_cast<std::int64_t>(config->orbit_segments)));
  config->orbit_rings = static_cast<std::size_t>(
      ReadInt(block, "orbit_rings", static_cast<std::int64_t>(config->orbit_rings)));
  return true;
}

/// 用规划器把区域任务展开为航路（coverage 块与编队切分共用）：规划失败
/// （空计划 = 顶点不足/间距非正/模式-区域不匹配等）报错退出，不允许静默
/// 退化为直飞。
bool PlanCoverageRoute(const CoverageTask& task, const std::string& block_name,
                       std::vector<navigation::RoutePoint>* waypoints, std::string* error) {
  navigation::AreaCoveragePlanner planner;
  *waypoints = planner.Plan(task.area, task.config);
  if (waypoints->empty()) {
    *error = "\"" + block_name + "\" planning failed: invalid region or "
             "mode-kind mismatch (need >= 3 vertices / positive scan_spacing_m / "
             "positive radius_m)";
    return false;
  }
  return true;
}

/// 解析平台块（platform 块 / platforms[] 数组条目共用）：原点/起飞航向/
/// 巡航参数 + 显式航路或区域巡逻 coverage 块（互斥）。coverage 规划成功时
/// waypoints 为规划器输出的巡逻航路（planned=true，平台循环巡逻）；规划失败
/// （顶点不足/间距非正/模式-区域不匹配等 → 空计划）报错退出，不允许静默
/// 退化为直飞。
bool ParsePlatformBlock(const examples::JsonValue& block, const std::string& block_name,
                        ScenePlatform* platform, std::string* error) {
  for (const char* key : {"origin_lat_deg", "origin_lon_deg"}) {
    if (!RequireGeometry(block, block_name, key, error)) {
      return false;
    }
  }
  platform->origin.latitude_deg = block["origin_lat_deg"].AsDouble();
  platform->origin.longitude_deg = block["origin_lon_deg"].AsDouble();
  platform->origin.altitude_m = ReadDouble(block, "origin_alt_m", 0.0);
  platform->initial_heading_deg =
      ReadDouble(block, "initial_heading_deg", platform->initial_heading_deg);
  platform->cruise_altitude_m =
      ReadDouble(block, "cruise_altitude_m", platform->cruise_altitude_m);
  platform->cruise_speed_mps =
      ReadDouble(block, "cruise_speed_mps", platform->cruise_speed_mps);

  const examples::JsonValue& waypoints = block["waypoints"];
  if (!waypoints.IsNull()) {
    if (waypoints.type() != examples::JsonValue::kArray) {
      *error = "\"" + block_name + ".waypoints\" must be an array";
      return false;
    }
    for (std::size_t i = 0U; i < waypoints.Size(); ++i) {
      const examples::JsonValue& wp = waypoints[i];
      if (wp["lat_deg"].IsNull() || wp["lon_deg"].IsNull()) {
        *error = block_name + " waypoint[" + std::to_string(i) + "] missing lat_deg/lon_deg";
        return false;
      }
      navigation::RoutePoint point;
      point.position.latitude_deg = wp["lat_deg"].AsDouble();
      point.position.longitude_deg = wp["lon_deg"].AsDouble();
      point.position.altitude_m = ReadDouble(wp, "alt_m", platform->cruise_altitude_m);
      point.speed_mps = ReadDouble(wp, "speed_mps", platform->cruise_speed_mps);
      point.radius_m = ReadDouble(wp, "radius_m", 500.0);
      platform->waypoints.push_back(point);
    }
  }

  // 区域巡逻块（可选）：与显式航路互斥（航路来源歧义直接报错）。
  const examples::JsonValue& coverage = block["coverage"];
  if (!coverage.IsNull()) {
    if (coverage.type() != examples::JsonValue::kObject) {
      *error = "\"" + block_name + ".coverage\" must be an object";
      return false;
    }
    if (!waypoints.IsNull()) {
      *error = "\"" + block_name + ".waypoints\" and \"coverage\" are mutually exclusive "
               "(explicit waypoints vs planned patrol route)";
      return false;
    }
    if (!ParseCoverageTask(coverage, "coverage", platform->cruise_altitude_m,
                           platform->cruise_speed_mps, &platform->coverage.area,
                           &platform->coverage.config, error)) {
      return false;
    }
    if (!PlanCoverageRoute(platform->coverage, block_name + ".coverage",
                           &platform->waypoints, error)) {
      return false;
    }
    platform->coverage.planned = true;
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

  // 主平台块（必填）：原点/起飞航向/巡航参数 + 可选航路/区域巡逻。
  const examples::JsonValue& platform = root["platform"];
  if (platform.IsNull() || platform.type() != examples::JsonValue::kObject) {
    *error = "missing required block \"platform\"";
    return false;
  }
  ScenePlatform main_platform;
  if (!ParsePlatformBlock(platform, "platform", &main_platform, error)) {
    return false;
  }
  out.platform_origin = main_platform.origin;
  out.initial_heading_deg = main_platform.initial_heading_deg;
  out.cruise_altitude_m = main_platform.cruise_altitude_m;
  out.cruise_speed_mps = main_platform.cruise_speed_mps;
  out.waypoints = std::move(main_platform.waypoints);
  out.coverage = main_platform.coverage;

  // 从机（可选顶层 platforms[] 数组）：每条目同 platform 块（各自航路/区域 =
  // "不同指令"；纯飞行，不挂传感器）。巡航参数缺省回退主平台值（同编队量级）。
  const examples::JsonValue& platforms = root["platforms"];
  if (!platforms.IsNull()) {
    if (platforms.type() != examples::JsonValue::kArray) {
      *error = "\"platforms\" must be an array";
      return false;
    }
    for (std::size_t i = 0U; i < platforms.Size(); ++i) {
      const examples::JsonValue& entry = platforms[i];
      const std::string block_name = "platforms[" + std::to_string(i) + "]";
      if (entry.IsNull() || entry.type() != examples::JsonValue::kObject) {
        *error = "\"" + block_name + "\" must be an object";
        return false;
      }
      ScenePlatform wing;
      wing.name = "wingman_" + std::to_string(i + 1U);  // 缺省名（可被 name 覆盖）
      if (entry["name"].IsString()) {
        wing.name = entry["name"].AsString();
      }
      wing.cruise_altitude_m = out.cruise_altitude_m;
      wing.cruise_speed_mps = out.cruise_speed_mps;
      if (!ParsePlatformBlock(entry, block_name, &wing, error)) {
        return false;
      }
      out.platforms.push_back(std::move(wing));
    }
  }

  // 编队切分块（可选顶层 mission_area）：主机收到单个覆盖区域 → 加载时自动
  // 切分为每架飞机（主机 + platforms[] 从机）的子区域（分工覆盖：多边形 =
  // 沿扫描航向等宽条带，圆形 = 同心环），再逐机经 AreaCoveragePlanner 自动
  // 生成覆盖航路。与各平台 coverage/waypoints 块互斥（区域来源歧义报错，
  // 同 waypoints/coverage 互斥先例）；编队数 = 1 + platforms.size()，无从机
  // 报错（切分需 >= 2 架飞机）。
  const examples::JsonValue& mission_area = root["mission_area"];
  if (!mission_area.IsNull()) {
    if (mission_area.type() != examples::JsonValue::kObject) {
      *error = "\"mission_area\" must be an object";
      return false;
    }
    FormationMissionArea mission;
    if (!ParseCoverageTask(mission_area, "mission_area", out.cruise_altitude_m,
                           out.cruise_speed_mps, &mission.area, &mission.config, error)) {
      return false;
    }
    if (out.platforms.empty()) {
      *error = "\"mission_area\" requires \"platforms\" (formation division needs "
               ">= 2 aircraft)";
      return false;
    }
    if (!out.waypoints.empty() || out.coverage.planned) {
      *error = "\"mission_area\" and \"platform.waypoints\"/\"platform.coverage\" are "
               "mutually exclusive (single mission area vs per-platform route)";
      return false;
    }
    for (std::size_t i = 0U; i < out.platforms.size(); ++i) {
      if (!out.platforms[i].waypoints.empty() || out.platforms[i].coverage.planned) {
        *error = "\"mission_area\" and \"platforms[" + std::to_string(i) +
                 "].waypoints\"/\".coverage\" are mutually exclusive (single mission "
                 "area vs per-platform route)";
        return false;
      }
    }
    const FormationDivisionResult division =
        DivideArea(mission.area, mission.config, 1U + out.platforms.size());
    if (!division.ok) {
      *error = "\"mission_area\" division failed: " + division.error;
      return false;
    }
    // 主机（下标 0）+ 从机（1..N-1）逐机填入子区域并规划航路（planned=true
    // → FlightComponent 循环巡逻，与 coverage 块语义一致）。
    out.coverage.area = division.sub_areas[0];
    out.coverage.config = division.sub_configs[0];
    if (!PlanCoverageRoute(out.coverage, "mission_area sub-area (host)", &out.waypoints,
                           error)) {
      return false;
    }
    out.coverage.planned = true;
    for (std::size_t i = 0U; i < out.platforms.size(); ++i) {
      out.platforms[i].coverage.area = division.sub_areas[i + 1U];
      out.platforms[i].coverage.config = division.sub_configs[i + 1U];
      const std::string sub_block =
          "mission_area sub-area (" + out.platforms[i].name + ")";
      if (!PlanCoverageRoute(out.platforms[i].coverage, sub_block,
                             &out.platforms[i].waypoints, error)) {
        return false;
      }
      out.platforms[i].coverage.planned = true;
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
    // 实体类型（可选）："air"（缺省）/"ground"（地面目标 = 静止近地运动学点）。
    if (t["type"].IsString()) {
      const std::string type = t["type"].AsString();
      if (type != "air" && type != "ground") {
        *error = "targets[" + std::to_string(i) + "].type must be \"air\" or \"ground\"";
        return false;
      }
      target.type = type;
    }
    target.azimuth_deg = t["azimuth_deg"].AsDouble();
    target.range_m = t["range_m"].AsDouble();
    target.altitude_m = t["altitude_m"].AsDouble();
    target.v_east_mps = ReadDouble(t, "v_east_mps", 0.0);
    target.v_north_mps = ReadDouble(t, "v_north_mps", 0.0);
    target.temperature_k = ReadDouble(t, "temperature_k", 0.0);
    target.rcs = t["rcs_m2"].AsDouble();
    target.projected_area_m2 = ReadDouble(t, "projected_area_m2", 0.0);
    target.radiant_intensity_w_per_sr = ReadDouble(t, "radiant_intensity_w_per_sr", 0.0);
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
    // RIR 特征真值块（可选，rir 字段存在即视为携带）：标量 + 散射器脚本。
    if (t.Has("rir")) {
      const examples::JsonValue& rir = t["rir"];
      if (rir.type() != examples::JsonValue::kObject) {
        *error = "targets[" + std::to_string(i) + "].rir must be an object";
        return false;
      }
      target.has_rir_features = true;
      target.rir_rcs_dbsm = ReadDouble(rir, "rcs_dbsm", 0.0);
      // 极化通道显式给值才铺样（0 dBsm 合法，不能按零值判缺省）。
      target.has_rir_polarization = rir.Has("pol_ch1_dbsm") || rir.Has("pol_ch2_dbsm");
      target.rir_pol_ch1_dbsm = ReadDouble(rir, "pol_ch1_dbsm", 0.0);
      target.rir_pol_ch2_dbsm = ReadDouble(rir, "pol_ch2_dbsm", 0.0);
      target.has_rir_pol_cross = rir.Has("pol_cross_dbsm");
      target.rir_pol_cross_dbsm = ReadDouble(rir, "pol_cross_dbsm", 0.0);
      target.has_rir_pol_phase = rir.Has("pol_phase_vv_deg");
      target.rir_pol_phase_vv_deg = ReadDouble(rir, "pol_phase_vv_deg", 0.0);
      if (rir["truth_model"].IsString()) {
        target.rir_truth_model = rir["truth_model"].AsString();
      }
      const examples::JsonValue& scatterers = rir["scatterers"];
      if (!scatterers.IsNull()) {
        if (scatterers.type() != examples::JsonValue::kArray) {
          *error = "targets[" + std::to_string(i) + "].rir.scatterers must be an array";
          return false;
        }
        for (std::size_t s = 0U; s < scatterers.Size(); ++s) {
          const examples::JsonValue& sc = scatterers[s];
          RirScattererScript scatterer;
          scatterer.offset_m = ReadDouble(sc, "offset_m", 0.0);
          scatterer.rcs_dbsm = ReadDouble(sc, "rcs_dbsm", 0.0);
          target.rir_scatterers.push_back(scatterer);
        }
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

  // 天基平台块（可选）：凝视目标群质心正上方，高度由场景控制；UTC 儒略日
  // 可选覆写（缺省 = 2024-01-01 00:00 UTC，SBIRS ECI 输出参考系必需）；
  // 焦平面几何与宽→窄连续命中门为验收量覆写（缺省 = 库默认）。
  const examples::JsonValue& satellite = root["sbirs_satellite"];
  if (!satellite.IsNull() && satellite.type() == examples::JsonValue::kObject) {
    out.sbirs_satellite_altitude_m =
        ReadDouble(satellite, "altitude_m", out.sbirs_satellite_altitude_m);
    out.sbirs_utc_julian_day =
        ReadDouble(satellite, "utc_julian_day", out.sbirs_utc_julian_day);
    out.sbirs_focal_length_m =
        ReadDouble(satellite, "focal_length_m", out.sbirs_focal_length_m);
    out.sbirs_detector_pixel_pitch_m =
        ReadDouble(satellite, "detector_pixel_pitch_m", out.sbirs_detector_pixel_pitch_m);
    out.sbirs_wide_to_narrow_required_consecutive_hits = static_cast<int>(
        ReadInt(satellite, "wide_to_narrow_required_consecutive_hits",
                out.sbirs_wide_to_narrow_required_consecutive_hits));
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

  // RIR 地基站点块（可选，enabled=true 时挂载识别雷达组件；站点 LLA 为雷达
  // 局部 ENU 原点，指定目标任务可选——识别完成或窗口超时自动回扫）。
  const examples::JsonValue& rir = root["rir"];
  if (!rir.IsNull() && rir.type() == examples::JsonValue::kObject) {
    out.rir_enabled = rir.Has("enabled") ? rir["enabled"].AsBool() : false;
    const examples::JsonValue& site = rir["site"];
    if (!site.IsNull() && site.type() == examples::JsonValue::kObject) {
      out.rir_site_origin.latitude_deg = ReadDouble(site, "lat_deg", out.rir_site_origin.latitude_deg);
      out.rir_site_origin.longitude_deg =
          ReadDouble(site, "lon_deg", out.rir_site_origin.longitude_deg);
      out.rir_site_origin.altitude_m = ReadDouble(site, "alt_m", out.rir_site_origin.altitude_m);
    }
    out.rir_designated_target_id = static_cast<std::uint64_t>(
        ReadInt(rir, "designated_target_id", static_cast<std::int64_t>(out.rir_designated_target_id)));
    out.rir_designation_duration_cycles = static_cast<std::uint32_t>(
        ReadInt(rir, "designation_duration_cycles",
                static_cast<std::int64_t>(out.rir_designation_duration_cycles)));
  }

  const examples::JsonValue& ecm = root["ecm"];
  if (!ecm.IsNull() && ecm.type() == examples::JsonValue::kObject) {
    out.ecm_enabled = ecm.Has("enabled") ? ecm["enabled"].AsBool() : false;
  }

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
    out.smoke.min_rir_recognition_outputs = static_cast<std::uint32_t>(
        ReadInt(smoke, "min_rir_recognition_outputs", out.smoke.min_rir_recognition_outputs));
  }

  *scene = std::move(out);
  return true;
}

}  // namespace demo
}  // namespace component_attachment
