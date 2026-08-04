/**
 * @file ArRecognitionConfig.h
 * @brief 远程目标识别策略配置。
 *
 * 识别配置（权重、门限、窗口、数据库路径）属于判决规则和策略参数，
 * 按四域所有权模型归入 policy 域（`ArPolicyConfig` 第七子域）。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_RECOGNITION_CONFIG_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_RECOGNITION_CONFIG_H_

#include <cstdint>
#include <string>

#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArRecognitionFeatureWeights 四类特征的基础权重。
 *
 * 各分量 ∈ [0, 1]，四者之和必须为 1.0（校验层断言）。
 */
struct ONEQ_API ArRecognitionFeatureWeights {
  float rcs_weight{0.25f};           /**< RCS 特征权重。 */
  float motion_weight{0.25f};        /**< 运动特征权重。 */
  float polarization_weight{0.25f};  /**< 极化特征权重。 */
  float range_profile_weight{0.25f}; /**< 距离像特征权重。 */
};

/**
 * @brief ArRecognitionConfig 远程目标识别策略配置。
 *
 * 默认关闭（enabled=false），需显式启用。运行期可经
 * `ArRuntimeConfigPatch::has_policy` 整域覆盖提交修改。
 */
struct ONEQ_API ArRecognitionConfig {
  bool enabled{false}; /**< 识别能力总开关（默认关闭）。 */

  /** 积累与确认 */
  std::uint32_t min_confirmed_hits{5U}; /**< 允许正式识别所需最小确认命中数（≥1）。 */
  float accumulation_window_sec{10.0f}; /**< 单航迹特征滑动积累窗口（s），必须 ≥ dt_sec。 */
  std::uint32_t min_observation_count{3U}; /**< 允许输出型号所需最小有效观测数（≥1）。 */

  /** 判定门限 */
  float acceptance_score{0.70f}; /**< 型号确认的最低综合得分，[0, 1]。 */
  float minimum_margin{0.10f};   /**< 第一、第二候选的最低得分差，[0, 1]。 */

  /** 时间约束 */
  float result_hold_sec{30.0f}; /**< 退出模式或短时特征缺失后的结论保持时间（s），≥0。 */

  /** 作用范围 */
  float max_range_m{300000.0f}; /**< 识别任务最大作用距离（m），>0。 */

  /** 驻留 */
  float recognition_dwell_sec{0.05f}; /**< 单次识别驻留时间（s），>0。 */

  /** 权重 */
  ArRecognitionFeatureWeights feature_weights{};

  /** 数据库 */
  std::string database_path{}; /**< 特征数据库 JSON 路径；空串表示未配置。 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_RECOGNITION_CONFIG_H_
