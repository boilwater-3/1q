-- 机载雷达识别特征数据库权威 schema（airborne_radar 识别特征库）。
--
-- 唯一事实源：C++ 加载器（src/airborne_radar/recognition/RecognitionFeatureDatabase.cpp）、
-- C++ 测试（tests/CMakeLists.txt configure_file 生成头）、建库工具
-- （tools/recognition_db_builder.py）均以本文件为准，禁止在别处维护第二份 DDL。
--
-- schema_version 由库内 meta 表声明（本文件对应 "1.0"）；任何结构变更需走
-- 识别数据库 freeze 流程（docs/review/recognition_database_v11_design_plan_2026-08-04.md）。

CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);

CREATE TABLE categories(category_id TEXT PRIMARY KEY, prior REAL);

CREATE TABLE models(model_id TEXT PRIMARY KEY,
                    category_id TEXT NOT NULL REFERENCES categories(category_id),
                    prior REAL);

CREATE TABLE profiles(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL REFERENCES models(model_id),
  min_snr_db REAL, max_range_resolution_m REAL,
  rcs_mean_dbsm REAL, rcs_std_db REAL,
  rcs_azimuth_variation_db REAL, rcs_elevation_variation_db REAL,
  rcs_minimum_aspect_coverage_deg REAL,
  motion_speed_mean REAL, motion_speed_std REAL,
  motion_altitude_mean REAL, motion_altitude_std REAL,
  motion_acceleration_mean REAL, motion_acceleration_std REAL,
  motion_turn_radius_mean_log10 REAL, motion_turn_radius_std_log10 REAL,
  polarization_energy_difference_mean REAL, polarization_energy_difference_std REAL,
  polarization_relative_difference_mean REAL, polarization_relative_difference_std REAL,
  polarization_energy_sum_mean REAL, polarization_energy_sum_std REAL,
  range_profile_length_mean REAL, range_profile_length_std REAL,
  range_profile_peak_count_mean REAL, range_profile_peak_count_std REAL,
  range_profile_peak_energy_concentration_mean REAL,
  range_profile_peak_energy_concentration_std REAL,
  range_profile_minimum_bandwidth_hz REAL,
  PRIMARY KEY (model_id, profile_id));
