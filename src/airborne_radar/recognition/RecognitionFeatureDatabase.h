/**
 * @file RecognitionFeatureDatabase.h
 * @brief 目标特征数据库（JSON 加载 + 结构校验，库内部）。
 *
 * 每个数据库文件是完整、只读的识别基线；加载为全量原子替换：
 * 新文件通过模式、单位、数值与交叉引用校验后才生效，加载失败时
 * 保持原库不变（调用方持有旧实例）。
 */

#ifndef AIRBORNE_RADAR_RECOGNITION_RECOGNITION_FEATURE_DATABASE_H_
#define AIRBORNE_RADAR_RECOGNITION_RECOGNITION_FEATURE_DATABASE_H_

#include <string>
#include <vector>

namespace airborne_radar {
namespace recognition {

/** @brief 单连续特征模板：均值 + 标准差（截断高斯相似度）。 */
struct RecognitionFeatureTemplate {
  float mean{0.0f};
  float std{1.0f};
};

/** @brief 型号 profile 的 RCS 模板（dBsm 域）。 */
struct RecognitionRcsTemplate {
  float mean_dbsm{0.0f};
  float std_db{1.0f};
  float azimuth_variation_db{0.0f};
  float elevation_variation_db{0.0f};
  float minimum_aspect_coverage_deg{0.0f};
};

/** @brief 型号 profile 的运动模板。 */
struct RecognitionMotionTemplate {
  RecognitionFeatureTemplate speed_mps{};
  RecognitionFeatureTemplate altitude_m{};
  RecognitionFeatureTemplate acceleration_mps2{};
  RecognitionFeatureTemplate turn_radius_log10{};
};

/** @brief 型号 profile 的极化模板（dB 域）。 */
struct RecognitionPolarizationTemplate {
  RecognitionFeatureTemplate energy_difference_db{};
  RecognitionFeatureTemplate relative_difference_db{};
  RecognitionFeatureTemplate energy_sum_db{};
};

/** @brief 型号 profile 的距离像模板。 */
struct RecognitionRangeProfileTemplate {
  RecognitionFeatureTemplate length_m{};
  RecognitionFeatureTemplate peak_count{};
  RecognitionFeatureTemplate peak_energy_concentration{};
  float minimum_bandwidth_hz{0.0f};
};

/** @brief 型号 profile：模板 + 适用条件。 */
struct RecognitionModelProfile {
  std::string profile_id{};
  float min_snr_db{0.0f};
  float max_range_resolution_m{0.0f}; /**< 0 表示不限。 */
  RecognitionRcsTemplate rcs{};
  RecognitionMotionTemplate motion{};
  RecognitionPolarizationTemplate polarization{};
  RecognitionRangeProfileTemplate range_profile{};
};

/** @brief 可匹配型号：模板集合 + 先验。 */
struct RecognitionModel {
  std::string model_id{};
  std::string category_id{};
  float prior{1.0f};
  std::vector<RecognitionModelProfile> profiles{};
};

/** @brief 标准目标大类及先验。 */
struct RecognitionCategoryEntry {
  std::string category_id{};
  float prior{1.0f};
};

/**
 * @brief RecognitionFeatureDatabase 已加载且校验通过的目标特征数据库。
 *
 * Load 成功后才返回非空数据库；失败时 error 含具体路径与字段。
 * 数据库保存的是从观测提取出的特征统计模板，不保存场景侧真值列表。
 */
class RecognitionFeatureDatabase {
 public:
  /**
   * @brief 从 JSON 文件加载并校验数据库。
   * @param[in] path JSON 文件路径。
   * @param[out] error 失败原因（含路径与字段）；成功时为空。
   * @return 校验通过返回 true 且 IsLoaded()==true；失败返回 false，
   *         返回对象保持未加载状态（不 fallback 到默认库）。
   */
  static bool Load(const std::string& path, RecognitionFeatureDatabase* database,
                   std::string* error);

  bool IsLoaded() const { return loaded_; }
  const std::string& database_id() const { return database_id_; }
  const std::string& version() const { return version_; }
  const std::vector<RecognitionCategoryEntry>& categories() const { return categories_; }
  const std::vector<RecognitionModel>& models() const { return models_; }

 private:
  bool loaded_{false};
  std::string database_id_{};
  std::string version_{};
  std::vector<RecognitionCategoryEntry> categories_{};
  std::vector<RecognitionModel> models_{};
};

}  // namespace recognition
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_RECOGNITION_RECOGNITION_FEATURE_DATABASE_H_
