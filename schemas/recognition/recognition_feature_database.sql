-- 机载雷达识别特征数据库权威 schema（airborne_radar 识别特征库，v1.1）。
--
-- 唯一事实源：C++ 加载器（src/airborne_radar/recognition/RecognitionFeatureDatabase.cpp）、
-- C++ 测试（tests/CMakeLists.txt configure_file 生成头）、建库工具
-- （tools/recognition_db_builder.py）均以本文件为准，禁止在别处维护第二份 DDL。
--
-- schema_version 由库内 meta 表声明（本文件对应 "1.1"）；版本策略与变更流程见
-- docs/airborne_radar/boundaries.md（major 破坏 / minor 增量）与
-- docs/review/recognition_database_v11_design_plan_2026-08-04.md。

-- 自描述元数据：键值表承载字符串元数据。v1.1 必填键：schema_version（'1.1'）、
-- database_id、version、created_utc、polarization_channels（逗号分隔，如 'H,V'）、
-- polarization_energy_reference。
CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);

-- 量纲声明：7 个已知量必填（rcs/speed/altitude/acceleration/turn_radius/polarization/
-- range），值非空；rcs 必须为 'dBsm'（匹配数学是 dBsm 域，加载器强校验）。
CREATE TABLE units(
  quantity TEXT PRIMARY KEY,
  unit TEXT NOT NULL);

CREATE TABLE categories(
  category_id TEXT PRIMARY KEY,
  display_name TEXT,                -- 可空：显示名为文档性字段
  prior REAL NOT NULL CHECK(prior > 0));

CREATE TABLE models(
  model_id TEXT PRIMARY KEY,
  category_id TEXT NOT NULL REFERENCES categories(category_id),
  display_name TEXT,                -- 可空
  prior REAL NOT NULL CHECK(prior > 0));

-- 适用条件：min_snr_db 可空（不约束）；max_range_resolution_m 可空/NULL（不限）；
-- aspect 四列可空 = 全范围（方位 [-180,180]、俯仰 [-90,90]），均非空时加载校验 min<=max。
CREATE TABLE profiles(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL REFERENCES models(model_id),
  min_snr_db REAL,
  max_range_resolution_m REAL,
  aspect_az_min_deg REAL, aspect_az_max_deg REAL,
  aspect_el_min_deg REAL, aspect_el_max_deg REAL,
  PRIMARY KEY (model_id, profile_id));

-- 模板组表：行存在 = 模板组存在；组内子模板 std 必填且 > 0（CHECK 兜底 +
-- 加载器显式校验），mean 可空（缺省 0.0f）。复合外键挂 profiles。
CREATE TABLE rcs_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  mean_dbsm REAL,
  std_db REAL NOT NULL CHECK(std_db > 0),   -- 列名保持 std_db（错误信息/测试对齐）
  azimuth_variation_db REAL,
  elevation_variation_db REAL,
  minimum_aspect_coverage_deg REAL,
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));

CREATE TABLE motion_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  speed_mean REAL, speed_std REAL NOT NULL CHECK(speed_std > 0),
  altitude_mean REAL, altitude_std REAL NOT NULL CHECK(altitude_std > 0),
  acceleration_mean REAL, acceleration_std REAL NOT NULL CHECK(acceleration_std > 0),
  turn_radius_mean_log10 REAL, turn_radius_std_log10 REAL NOT NULL CHECK(turn_radius_std_log10 > 0),
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));

CREATE TABLE polarization_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  energy_difference_mean REAL, energy_difference_std REAL NOT NULL CHECK(energy_difference_std > 0),
  relative_difference_mean REAL, relative_difference_std REAL NOT NULL CHECK(relative_difference_std > 0),
  energy_sum_mean REAL, energy_sum_std REAL NOT NULL CHECK(energy_sum_std > 0),
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));

CREATE TABLE range_profile_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  length_mean REAL, length_std REAL NOT NULL CHECK(length_std > 0),
  peak_count_mean REAL, peak_count_std REAL NOT NULL CHECK(peak_count_std > 0),
  peak_energy_concentration_mean REAL,
  peak_energy_concentration_std REAL NOT NULL CHECK(peak_energy_concentration_std > 0),
  minimum_bandwidth_hz REAL,
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));
