/**
 * @file scene_data_test.cpp
 * @brief 场景描述加载器（examples/component_attachment/scene_data.*）单元测试。
 *
 * 覆盖：基线场景解析（全字段）、缺省块回退、必填几何字段校验、畸形 JSON
 * 报错、空目标数组（"无目标"场景）、场景业务覆写应用（ApplySceneOverrides）。
 * 场景内容以原始字符串内嵌（与 scenes/baseline_takeoff_east.json 同值），
 * 写入临时目录后加载——测试不依赖仓库路径。
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "demo_config.h"
#include "scene_data.h"

namespace ca = component_attachment;
namespace demo = component_attachment::demo;

namespace {

/// 临时场景文件（析构清理），内容来自构造参数。
class ScopedSceneFile {
 public:
  explicit ScopedSceneFile(const std::string& content) {
    static std::atomic<std::uint64_t> counter{0U};
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("scene_data_test_" + std::to_string(stamp) + "_" +
             std::to_string(counter.fetch_add(1U)) + ".json");
    std::ofstream out(path_);
    out << content;
  }

  ~ScopedSceneFile() { std::filesystem::remove(path_); }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

/// 加载临时文件内容，返回场景与错误串（断言加载成功）。
demo::SceneData LoadOk(const std::string& content) {
  ScopedSceneFile file(content);
  demo::SceneData scene;
  std::string error;
  EXPECT_TRUE(demo::LoadSceneData(file.path().c_str(), &scene, &error)) << error;
  return scene;
}

/// 与 scenes/baseline_takeoff_east.json 同值的基线场景内容。
constexpr char kBaselineSceneJson[] = R"json({
  "name": "baseline_takeoff_east",
  "cycles": 400,
  "dt_sec": 1.0,
  "platform": {
    "origin_lat_deg": 30.0,
    "origin_lon_deg": 120.0,
    "origin_alt_m": 0.0,
    "initial_heading_deg": 90.0,
    "cruise_altitude_m": 400.0,
    "cruise_speed_mps": 50.0,
    "waypoints": [
      {"lat_deg": 30.0, "lon_deg": 120.05, "alt_m": 400.0, "speed_mps": 50.0, "radius_m": 500.0},
      {"lat_deg": 30.0, "lon_deg": 120.10, "alt_m": 400.0, "speed_mps": 50.0, "radius_m": 500.0},
      {"lat_deg": 30.0, "lon_deg": 120.15, "alt_m": 400.0, "speed_mps": 50.0, "radius_m": 500.0}
    ]
  },
  "targets": [
    {"id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
     "v_east_mps": 47.0, "v_north_mps": 5.0, "temperature_k": 520.0,
     "rcs_m2": 2.2, "projected_area_m2": 18.0, "emitter_center_frequency_hz": 9.5e9},
    {"id": 1002, "azimuth_deg": 0.0, "range_m": 14000.0, "altitude_m": 400.0,
     "v_east_mps": 47.0, "v_north_mps": -5.0, "temperature_k": 540.0,
     "rcs_m2": 1.4, "projected_area_m2": 15.0, "emitter_center_frequency_hz": 10.0e9}
  ],
  "esr": {
    "peak_gain_dbi": 30.0,
    "bandwidth_hz": 2.0e6,
    "peak_power_w": 5.0e7,
    "pulse_width_s": 1.0e-6,
    "pri_s": 1.0e-3,
    "pulse_count": 200,
    "timing_seed": 42
  },
  "sbirs_satellite": {
    "altitude_m": 500000.0
  },
  "eos_scan": {
    "frame_rate_hz": 10.0,
    "scan_rate_deg_per_sec": 20.0,
    "scan_start_az_deg": 50.0,
    "scan_end_az_deg": 130.0,
    "scan_center_el_deg": 0.0,
    "boresight_depression_deg": 0.0
  },
  "sar": {
    "peak_power_w": 1.0e6,
    "antenna_gain_db": 40.0,
    "max_squint_deg": 10.0,
    "scene_center_latitude_deg": 30.117117117117118,
    "scene_center_longitude_deg": 120.06,
    "scene_center_altitude_m": 400.0,
    "slant_range_m": 13000.0,
    "platform_speed_mps": 50.0
  },
  "fusion": {
    "position_radius_m": 1000.0,
    "bearing_beamwidth_deg": 8.0,
    "feature_threshold": 0.0,
    "window_size": 10,
    "max_missed_cycles": 5,
    "source_weights": [0.0, 1.0, 0.8, 0.6, 0.5]
  },
  "high_threat_confidence": 3.0,
  "smoke": {
    "min_key_events": 1,
    "min_sbirs_events": 1,
    "min_sar_products": 1,
    "min_fused_targets": 1
  }
})json";

}  // namespace

TEST(SceneDataTest, LoadsBaselineScene) {
  const demo::SceneData scene = LoadOk(kBaselineSceneJson);

  EXPECT_EQ(scene.name, "baseline_takeoff_east");
  EXPECT_EQ(scene.cycles, 400U);
  EXPECT_DOUBLE_EQ(scene.dt_sec, 1.0);

  EXPECT_DOUBLE_EQ(scene.platform_origin.latitude_deg, 30.0);
  EXPECT_DOUBLE_EQ(scene.platform_origin.longitude_deg, 120.0);
  EXPECT_DOUBLE_EQ(scene.platform_origin.altitude_m, 0.0);
  EXPECT_DOUBLE_EQ(scene.initial_heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(scene.cruise_altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.cruise_speed_mps, 50.0);

  ASSERT_EQ(scene.waypoints.size(), 3U);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.latitude_deg, 30.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.longitude_deg, 120.05);
  EXPECT_DOUBLE_EQ(scene.waypoints[2].position.longitude_deg, 120.15);
  EXPECT_DOUBLE_EQ(scene.waypoints[1].position.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[1].speed_mps, 50.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[1].radius_m, 500.0);
  // 无 coverage 块：显式航路、非巡逻。
  EXPECT_FALSE(scene.coverage.planned);

  ASSERT_EQ(scene.targets.size(), 2U);
  EXPECT_EQ(scene.targets[0].id, 1001U);
  EXPECT_DOUBLE_EQ(scene.targets[0].azimuth_deg, 0.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].range_m, 12000.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].v_east_mps, 47.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].v_north_mps, 5.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].temperature_k, 520.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].rcs, 2.2);
  EXPECT_DOUBLE_EQ(scene.targets[0].projected_area_m2, 18.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].emitter_center_frequency_hz, 9.5e9);
  EXPECT_EQ(scene.targets[1].id, 1002U);
  EXPECT_DOUBLE_EQ(scene.targets[1].v_north_mps, -5.0);
  EXPECT_DOUBLE_EQ(scene.targets[1].rcs, 1.4);
  EXPECT_DOUBLE_EQ(scene.targets[1].emitter_center_frequency_hz, 10.0e9);

  EXPECT_DOUBLE_EQ(scene.esr.peak_gain_dbi, 30.0);
  EXPECT_DOUBLE_EQ(scene.esr.bandwidth_hz, 2.0e6);
  EXPECT_DOUBLE_EQ(scene.esr.peak_power_w, 5.0e7);
  EXPECT_DOUBLE_EQ(scene.esr.pulse_width_s, 1.0e-6);
  EXPECT_DOUBLE_EQ(scene.esr.pri_s, 1.0e-3);
  EXPECT_EQ(scene.esr.pulse_count, 200U);
  EXPECT_EQ(scene.esr.timing_seed, 42U);

  EXPECT_DOUBLE_EQ(scene.sbirs_satellite_altitude_m, 500000.0);

  EXPECT_FLOAT_EQ(scene.eos_frame_rate_hz, 10.0f);
  EXPECT_FLOAT_EQ(scene.eos_scan_rate_deg_per_sec, 20.0f);
  EXPECT_FLOAT_EQ(scene.eos_scan_start_az_deg, 50.0f);
  EXPECT_FLOAT_EQ(scene.eos_scan_end_az_deg, 130.0f);
  EXPECT_FLOAT_EQ(scene.eos_scan_center_el_deg, 0.0f);
  EXPECT_FLOAT_EQ(scene.eos_boresight_depression_deg, 0.0f);

  EXPECT_DOUBLE_EQ(scene.sar_peak_power_w, 1.0e6);
  EXPECT_DOUBLE_EQ(scene.sar_antenna_gain_db, 40.0);
  EXPECT_DOUBLE_EQ(scene.sar_max_squint_angle_deg, 10.0);
  EXPECT_DOUBLE_EQ(scene.sar_scene_center_latitude_deg, 30.117117117117118);
  EXPECT_DOUBLE_EQ(scene.sar_scene_center_longitude_deg, 120.06);
  EXPECT_DOUBLE_EQ(scene.sar_scene_center_altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.sar_nominal_slant_range_m, 13000.0);
  EXPECT_DOUBLE_EQ(scene.sar_platform_speed_mps, 50.0);

  EXPECT_DOUBLE_EQ(scene.fusion.position_radius_m, 1000.0);
  EXPECT_DOUBLE_EQ(scene.fusion.bearing_beamwidth_deg, 8.0);
  EXPECT_DOUBLE_EQ(scene.fusion.feature_threshold, 0.0);
  EXPECT_EQ(scene.fusion.window_size, 10U);
  EXPECT_EQ(scene.fusion.max_missed_cycles, 5U);
  ASSERT_EQ(scene.fusion.source_weights.size(), 5U);
  EXPECT_DOUBLE_EQ(scene.fusion.source_weights[1], 1.0);
  EXPECT_DOUBLE_EQ(scene.fusion.source_weights[4], 0.5);

  EXPECT_DOUBLE_EQ(scene.high_threat_confidence, 3.0);
  EXPECT_EQ(scene.smoke.min_key_events, 1U);
  EXPECT_EQ(scene.smoke.min_sbirs_events, 1U);
  EXPECT_EQ(scene.smoke.min_sar_products, 1U);
  EXPECT_EQ(scene.smoke.min_fused_targets, 1U);
}

TEST(SceneDataTest, DefaultsWhenBlocksMissing) {
  // 仅平台 + 空目标：全部可缺省块回退默认值。
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": []
  })json");

  EXPECT_EQ(scene.name, "unnamed_scene");
  EXPECT_EQ(scene.cycles, 400U);
  EXPECT_DOUBLE_EQ(scene.dt_sec, 1.0);
  EXPECT_DOUBLE_EQ(scene.initial_heading_deg, 90.0);
  EXPECT_DOUBLE_EQ(scene.cruise_altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.cruise_speed_mps, 50.0);
  EXPECT_TRUE(scene.waypoints.empty());
  EXPECT_TRUE(scene.targets.empty());
  EXPECT_EQ(scene.esr.pulse_count, 200U);
  EXPECT_EQ(scene.esr.timing_seed, 42U);
  EXPECT_DOUBLE_EQ(scene.sbirs_satellite_altitude_m, 500000.0);
  EXPECT_FLOAT_EQ(scene.eos_frame_rate_hz, 10.0f);
  EXPECT_DOUBLE_EQ(scene.sar_peak_power_w, 1.0e6);
  // 融合缺省 = FusionConfig 默认值（方位相干门限 5°、权重空）。
  EXPECT_DOUBLE_EQ(scene.fusion.bearing_beamwidth_deg, 5.0);
  EXPECT_TRUE(scene.fusion.source_weights.empty());
  EXPECT_DOUBLE_EQ(scene.high_threat_confidence, 3.0);
  EXPECT_EQ(scene.smoke.min_key_events, 1U);
  EXPECT_EQ(scene.smoke.min_fused_targets, 1U);
}

TEST(SceneDataTest, WaypointDefaultsToCruiseParameters) {
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {
      "origin_lat_deg": 30.0, "origin_lon_deg": 120.0,
      "cruise_altitude_m": 800.0, "cruise_speed_mps": 60.0,
      "waypoints": [{"lat_deg": 31.0, "lon_deg": 121.0}]
    },
    "targets": []
  })json");

  ASSERT_EQ(scene.waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.latitude_deg, 31.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.longitude_deg, 121.0);
  // alt/speed 缺省回退巡航参数，radius 缺省 500 m。
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.altitude_m, 800.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].speed_mps, 60.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].radius_m, 500.0);
}

TEST(SceneDataTest, FusionAndSmokeOverrides) {
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "fusion": {"bearing_beamwidth_deg": 12.0, "source_weights": [0.0, 0.5]},
    "smoke": {"min_fused_targets": 0}
  })json");

  EXPECT_DOUBLE_EQ(scene.fusion.bearing_beamwidth_deg, 12.0);
  ASSERT_EQ(scene.fusion.source_weights.size(), 2U);
  EXPECT_DOUBLE_EQ(scene.fusion.source_weights[1], 0.5);
  // 未覆写字段保持默认。
  EXPECT_DOUBLE_EQ(scene.fusion.position_radius_m, 1000.0);
  EXPECT_EQ(scene.smoke.min_fused_targets, 0U);
  EXPECT_EQ(scene.smoke.min_key_events, 1U);
}

TEST(SceneDataTest, MissingPlatformBlockFails) {
  ScopedSceneFile file(R"json({"targets": []})json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("platform"), std::string::npos);
}

TEST(SceneDataTest, MissingTargetsBlockFails) {
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0}})json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("targets"), std::string::npos);
}

TEST(SceneDataTest, MissingGeometryFieldFails) {
  // 目标缺 altitude_m（必填几何字段）→ 报错并点名缺失字段。
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{"id": 1001, "azimuth_deg": 0.0, "range_m": 1000.0, "rcs_m2": 1.0}]
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("altitude_m"), std::string::npos);
}

TEST(SceneDataTest, InvalidCyclesFails) {
  ScopedSceneFile file(R"json({
    "cycles": 0,
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": []
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("cycles"), std::string::npos);
}

TEST(SceneDataTest, ParsesTargetManeuvers) {
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{
      "id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
      "rcs_m2": 2.2, "v_east_mps": 47.0, "v_north_mps": 5.0,
      "maneuvers": [
        {"start_cycle": 200, "v_east_mps": 47.0, "v_north_mps": -30.0},
        {"start_cycle": 260, "v_north_mps": 0.0}
      ]
    }]
  })json");

  ASSERT_EQ(scene.targets.size(), 1U);
  ASSERT_EQ(scene.targets[0].maneuvers.size(), 2U);
  EXPECT_EQ(scene.targets[0].maneuvers[0].start_cycle, 200U);
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[0].v_east_mps, 47.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[0].v_north_mps, -30.0);
  EXPECT_EQ(scene.targets[0].maneuvers[1].start_cycle, 260U);
  // v_east 缺省 0（机动条目字段级缺省）。
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[1].v_east_mps, 0.0);
  EXPECT_DOUBLE_EQ(scene.targets[0].maneuvers[1].v_north_mps, 0.0);
}

TEST(SceneDataTest, ManeuverMissingStartCycleFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{
      "id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
      "rcs_m2": 2.2, "maneuvers": [{"v_north_mps": -30.0}]
    }]
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("start_cycle"), std::string::npos);
}

TEST(SceneDataTest, ManeuverStartCycleMustIncrease) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [{
      "id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
      "rcs_m2": 2.2,
      "maneuvers": [
        {"start_cycle": 260, "v_north_mps": 0.0},
        {"start_cycle": 200, "v_north_mps": -30.0}
      ]
    }]
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("increasing"), std::string::npos);
}

TEST(SceneDataTest, MalformedJsonFails) {
  ScopedSceneFile file(R"json({"platform": {"origin_lat_deg": )json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_FALSE(error.empty());
}

TEST(SceneDataTest, AppliesSceneOverrides) {
  const demo::SceneData scene = LoadOk(kBaselineSceneJson);
  demo::ComponentAttachmentConfigs configs;  // 默认构造 = JSON 加载后未覆写状态

  demo::ApplySceneOverrides(scene, &configs);

  // EOS 扫描/帧率覆写（替代原 demo_config 内硬编码）。
  EXPECT_FLOAT_EQ(configs.eos.mission.frame_rate_hz, 10.0f);
  EXPECT_FLOAT_EQ(configs.eos.mission.scan_rate_deg_per_sec, 20.0f);
  EXPECT_FLOAT_EQ(configs.eos.mission.scan_start_az_deg, 50.0f);
  EXPECT_FLOAT_EQ(configs.eos.mission.scan_end_az_deg, 130.0f);
  EXPECT_FLOAT_EQ(configs.eos.mission.scan_center_el_deg, 0.0f);
  EXPECT_FLOAT_EQ(configs.eos.mission.boresight_depression_deg, 0.0f);
  // SAR 任务几何/链路覆写。
  EXPECT_DOUBLE_EQ(configs.sar.hardware.peak_power_w, 1.0e6);
  EXPECT_DOUBLE_EQ(configs.sar.hardware.antenna_gain_db, 40.0);
  EXPECT_DOUBLE_EQ(configs.sar.policy.max_allowed_squint_angle_deg, 10.0);
  EXPECT_DOUBLE_EQ(configs.sar.mission.scene_center_latitude_deg, 30.117117117117118);
  EXPECT_DOUBLE_EQ(configs.sar.mission.scene_center_longitude_deg, 120.06);
  EXPECT_DOUBLE_EQ(configs.sar.mission.scene_center_altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(configs.sar.mission.nominal_slant_range_m, 13000.0);
  EXPECT_DOUBLE_EQ(configs.sar.mission.platform_speed_mps, 50.0);
}

TEST(SceneDataTest, PlansPolygonScanPatrolRoute) {
  // coverage 多边形 + 扫描模式：加载时经 AreaCoveragePlanner 生成巡逻航路。
  // 区域 lat 29.989..30.001（约 1334 m 南北跨度）× lon 120.0..120.03，间距
  // 500 m → 扫描线 v = 250/750/1250（3 条 × 2 端点 = 6 航点），牛耕式交替
  // 方向（线 1 东向 → 线 2 西向 → 线 3 东向）。
  const demo::SceneData scene = LoadOk(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "cruise_altitude_m": 400.0,
        "cruise_speed_mps": 50.0,
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ],
            "scan_heading_deg": 0.0,
            "scan_spacing_m": 500.0,
            "arrival_radius_m": 200.0
        }
    },
    "targets": []

  })json");

  EXPECT_TRUE(scene.coverage.planned);
  EXPECT_EQ(scene.coverage.area.kind, navigation::CoverageAreaKind::kPolygon);
  EXPECT_EQ(scene.coverage.config.mode, navigation::CoverageMode::kScan);
  // 高度/速度缺省回退巡航参数。
  EXPECT_DOUBLE_EQ(scene.coverage.config.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.coverage.config.speed_mps, 50.0);

  ASSERT_EQ(scene.waypoints.size(), 6U);
  // 线 1 东向：同纬度、lon 递增。
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.latitude_deg,
                   scene.waypoints[1].position.latitude_deg);
  EXPECT_LT(scene.waypoints[0].position.longitude_deg,
            scene.waypoints[1].position.longitude_deg);
  // 线 2 西向：北移一条扫描间距、lon 递减。
  EXPECT_GT(scene.waypoints[2].position.latitude_deg,
            scene.waypoints[1].position.latitude_deg);
  EXPECT_GT(scene.waypoints[2].position.longitude_deg,
            scene.waypoints[3].position.longitude_deg);
  // 线 3 东向：北移一条扫描间距。
  EXPECT_GT(scene.waypoints[4].position.latitude_deg,
            scene.waypoints[3].position.latitude_deg);
  EXPECT_LT(scene.waypoints[4].position.longitude_deg,
            scene.waypoints[5].position.longitude_deg);
  // 相邻扫描线纬度差 ≈ 500 m（1° 纬度 ≈ 111190 m），容差覆盖 ENU 回变换误差。
  const double line_delta_deg = scene.waypoints[2].position.latitude_deg -
                                scene.waypoints[0].position.latitude_deg;
  EXPECT_NEAR(line_delta_deg, 500.0 / 111190.0, 1e-4);
  // 航点属性来自规划配置。
  EXPECT_DOUBLE_EQ(scene.waypoints[0].position.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].speed_mps, 50.0);
  EXPECT_DOUBLE_EQ(scene.waypoints[0].radius_m, 200.0);
}

TEST(SceneDataTest, PlansCircleOrbitPatrolRoute) {
  // coverage 圆形 + 盘旋模式：8 段 × 1 环 = 8 航点，均匀分布在半径 5000 m
  // 的圆周上（逆时针，自正北起）。
  const demo::SceneData scene = LoadOk(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "circle",
            "mode": "orbit",
            "center": {
                "lat_deg": 30.0,
                "lon_deg": 120.0
            },
            "radius_m": 5000.0,
            "altitude_m": 400.0,
            "speed_mps": 50.0,
            "arrival_radius_m": 200.0,
            "orbit_segments": 8,
            "orbit_rings": 1
        }
    },
    "targets": []

  })json");

  EXPECT_TRUE(scene.coverage.planned);
  EXPECT_EQ(scene.coverage.area.kind, navigation::CoverageAreaKind::kCircle);
  EXPECT_EQ(scene.coverage.config.mode, navigation::CoverageMode::kOrbit);
  ASSERT_EQ(scene.waypoints.size(), 8U);
  for (const auto& wp : scene.waypoints) {
    // 每个航点都在圆心 5000 m 半径附近：纬度偏移 ≈ 5000/110850 ≈ 0.04511°
    // （WGS84 30°N 子午线弧长），经度偏移 ≈ 5000/(110850·cos30°) ≈ 0.0521°
    // （圆周几何，容差覆盖 ENU 回变换误差）。
    EXPECT_NEAR(wp.position.latitude_deg - 30.0, 0.0, 0.0452);
    EXPECT_NEAR(wp.position.longitude_deg - 120.0, 0.0, 0.0522);
    EXPECT_DOUBLE_EQ(wp.position.altitude_m, 400.0);
    EXPECT_DOUBLE_EQ(wp.speed_mps, 50.0);
  }
}

TEST(SceneDataTest, CoverageAndExplicitWaypointsConflictFails) {
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "waypoints": [
            {
                "lat_deg": 30.0,
                "lon_deg": 120.05
            }
        ],
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ],
            "scan_spacing_m": 500.0
        }
    },
    "targets": []

  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("mutually exclusive"), std::string::npos);
}

TEST(SceneDataTest, InvalidCoverageGeometryFails) {
  // 扫描间距非正 → 规划失败（空计划）→ 加载报错，不允许静默直飞。
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ],
            "scan_spacing_m": 0.0
        }
    },
    "targets": []

  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("planning failed"), std::string::npos);
}

TEST(SceneDataTest, CoverageModeKindMismatchFails) {
  // 盘旋模式 + 多边形区域：模式-区域不匹配 → 规划失败。
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "polygon",
            "mode": "orbit",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.03
                },
                {
                    "lat_deg": 30.001,
                    "lon_deg": 120.0
                }
            ]
        }
    },
    "targets": []

  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("planning failed"), std::string::npos);
}

TEST(SceneDataTest, MissingCoverageVertexFails) {
  ScopedSceneFile file(R"json({

    "platform": {
        "origin_lat_deg": 30.0,
        "origin_lon_deg": 120.0,
        "coverage": {
            "kind": "polygon",
            "mode": "scan",
            "vertices": [
                {
                    "lat_deg": 29.989,
                    "lon_deg": 120.0
                },
                {
                    "lon_deg": 120.03
                }
            ],
            "scan_spacing_m": 500.0
        }
    },
    "targets": []

  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("lat_deg"), std::string::npos);
}

TEST(SceneDataTest, ParsesWingmanPlatforms) {
  // platforms[] 数组：每条目同 platform 块（原点/航向/巡航 + waypoints 或
  // coverage 区域巡逻）；巡航参数缺省回退主平台值。
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {
      "origin_lat_deg": 30.0,
      "origin_lon_deg": 120.0,
      "cruise_altitude_m": 400.0,
      "cruise_speed_mps": 50.0
    },
    "targets": [],
    "platforms": [
      {
        "name": "wing_a",
        "origin_lat_deg": 30.01,
        "origin_lon_deg": 120.01,
        "initial_heading_deg": 270.0,
        "waypoints": [
          {"lat_deg": 30.01, "lon_deg": 120.02, "speed_mps": 45.0}
        ]
      },
      {
        "name": "wing_b",
        "origin_lat_deg": 30.02,
        "origin_lon_deg": 120.02,
        "coverage": {
          "kind": "circle",
          "mode": "orbit",
          "center": {"lat_deg": 30.02, "lon_deg": 120.05},
          "radius_m": 2000.0
        }
      }
    ]
  })json");

  ASSERT_EQ(scene.platforms.size(), 2U);
  // 从机 A：显式航路 + 名称/航向/巡航缺省回退主平台。
  EXPECT_EQ(scene.platforms[0].name, "wing_a");
  EXPECT_DOUBLE_EQ(scene.platforms[0].origin.latitude_deg, 30.01);
  EXPECT_DOUBLE_EQ(scene.platforms[0].initial_heading_deg, 270.0);
  ASSERT_EQ(scene.platforms[0].waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(scene.platforms[0].waypoints[0].speed_mps, 45.0);
  EXPECT_FALSE(scene.platforms[0].coverage.planned);
  // 从机 B：coverage 圆形盘旋 → 规划航路（8 段 × 1 环）。
  EXPECT_EQ(scene.platforms[1].name, "wing_b");
  EXPECT_TRUE(scene.platforms[1].coverage.planned);
  EXPECT_EQ(scene.platforms[1].coverage.area.kind, navigation::CoverageAreaKind::kCircle);
  EXPECT_EQ(scene.platforms[1].coverage.config.mode, navigation::CoverageMode::kOrbit);
  ASSERT_EQ(scene.platforms[1].waypoints.size(), 8U);
  // 巡航缺省回退主平台（400/50），而非 ScenePlatform 默认值。
  EXPECT_DOUBLE_EQ(scene.platforms[1].coverage.config.altitude_m, 400.0);
  EXPECT_DOUBLE_EQ(scene.platforms[1].coverage.config.speed_mps, 50.0);
  // 主平台不受 platforms[] 影响（单平台语义保留）。
  EXPECT_DOUBLE_EQ(scene.platform_origin.latitude_deg, 30.0);
}

TEST(SceneDataTest, WingmanWithoutNameDefaultsToWingmanIndex) {
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "platforms": [
      {"origin_lat_deg": 30.01, "origin_lon_deg": 120.01},
      {"origin_lat_deg": 30.02, "origin_lon_deg": 120.02}
    ]
  })json");
  ASSERT_EQ(scene.platforms.size(), 2U);
  EXPECT_EQ(scene.platforms[0].name, "wingman_1");
  EXPECT_EQ(scene.platforms[1].name, "wingman_2");
}

TEST(SceneDataTest, PlatformsNotArrayFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "platforms": {"origin_lat_deg": 30.01, "origin_lon_deg": 120.01}
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("platforms"), std::string::npos);
}

TEST(SceneDataTest, WingmanCoverageAndWaypointsConflictFails) {
  // 从机同主平台：waypoints 与 coverage 互斥报错（航路来源歧义）。
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [],
    "platforms": [
      {
        "origin_lat_deg": 30.01,
        "origin_lon_deg": 120.01,
        "waypoints": [{"lat_deg": 30.01, "lon_deg": 120.02}],
        "coverage": {
          "kind": "polygon",
          "mode": "scan",
          "vertices": [
            {"lat_deg": 29.989, "lon_deg": 120.0},
            {"lat_deg": 29.989, "lon_deg": 120.03},
            {"lat_deg": 30.001, "lon_deg": 120.03}
          ],
          "scan_spacing_m": 500.0
        }
      }
    ]
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("mutually exclusive"), std::string::npos);
}

TEST(SceneDataTest, ParsesTargetType) {
  // targets[].type：缺省 air；ground 显式（地面目标 = 静止近地运动学点）。
  const demo::SceneData scene = LoadOk(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [
      {"id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
       "rcs_m2": 2.2},
      {"id": 1002, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 0.0,
       "rcs_m2": 1.0, "type": "ground"}
    ]
  })json");
  ASSERT_EQ(scene.targets.size(), 2U);
  EXPECT_EQ(scene.targets[0].type, "air");
  EXPECT_EQ(scene.targets[1].type, "ground");
}

TEST(SceneDataTest, InvalidTargetTypeFails) {
  ScopedSceneFile file(R"json({
    "platform": {"origin_lat_deg": 30.0, "origin_lon_deg": 120.0},
    "targets": [
      {"id": 1001, "azimuth_deg": 0.0, "range_m": 12000.0, "altitude_m": 400.0,
       "rcs_m2": 2.2, "type": "sea"}
    ]
  })json");
  demo::SceneData scene;
  std::string error;
  EXPECT_FALSE(demo::LoadSceneData(file.path().c_str(), &scene, &error));
  EXPECT_NE(error.find("type"), std::string::npos);
}
