/**
 * @file RirPolicyConfig.h
 * @brief 远程识别雷达策略域主配置类型。
 *
 * 自持检测/关联/跟踪/生命周期与识别策略的主头文件。
 * 识别字段集合自阶段 1 平移；阶段 2-S 增加检测门控、最近邻关联、单目标 KF
 * 与计数生命周期策略。max_range_m / recognition_dwell_sec 四域归位（任务域）
 * 列为阶段 3 评估项。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_POLICY_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_POLICY_CONFIG_H_

#include <cstdint>
#include <string>

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace config {

namespace detection {

/** @brief 检测门控模式：检测器判决驱动，或 6 dB SNR 回退门控。 */
enum class ONEQ_API RirDetectionGateMode {
  kDetectorGate = 0, /**< 统计级 CFAR 检测判决驱动目标进入关联/滤波。 */
  kSnrFallback = 1   /**< 回退模式：SNR ≥ 6 dB 即进入关联/滤波（旧识别门控口径）。 */
};

/**
 * @brief RIR 自持检测策略（阶段 2-S S2）。
 */
struct ONEQ_API RirDetectionPolicyConfig {
  float cfar_pfa{1e-6f};                /**< 统计级 CFAR 虚警概率。 */
  float min_snr_db{-10.0f};             /**< SNR 硬截断下限。 */
  float min_detection_margin_db{-2.0f}; /**< 检测可靠性裕量门限。 */
  int pulse_count{10};                  /**< 默认检测积累脉冲数。 */
  std::uint32_t random_seed{42U};       /**< 检测/量测误差随机种子（replay 状态）。 */
  RirDetectionGateMode gate_mode{RirDetectionGateMode::kDetectorGate}; /**< 目标进入门控模式。 */
};

}  // namespace detection

namespace association {

/** @brief 最近邻关联策略（阶段 2-S S2）。 */
struct ONEQ_API RirAssociationPolicyConfig {
  /** @brief 归一化马氏距离波门 sigma 倍数；内部映射为平方门限。 */
  float distance_gate_sigma{3.0f};
};

}  // namespace association

namespace tracking {

/** @brief 单目标 KF 跟踪策略（阶段 2-S S2）。 */
struct ONEQ_API RirTrackingPolicyConfig {
  float kalman_noise_diff_coeff{1.0f};       /**< 连续白噪声加速度扩散系数 q（m/s²）。 */
  float kalman_measurement_noise_std{10.0f}; /**< 缺省量测噪声标准差（m）。 */
};

}  // namespace tracking

namespace lifecycle {

/** @brief 轻量航迹生命周期策略（阶段 2-S S2）。 */
struct ONEQ_API RirLifecyclePolicyConfig {
  std::uint32_t confirm_hits{3U};         /**< tentative 转 confirmed 所需累计命中数。 */
  std::uint32_t max_miss_before_lost{2U}; /**< tentative/confirmed 转 lost 连续失配阈值。 */
  std::uint32_t max_lost_cycles{5U};      /**< lost 保留周期数，超出即回收。 */
};

}  // namespace lifecycle

using association::RirAssociationPolicyConfig;
using detection::RirDetectionGateMode;
using detection::RirDetectionPolicyConfig;
using lifecycle::RirLifecyclePolicyConfig;
using tracking::RirTrackingPolicyConfig;

/**
 * @brief RirRecognitionFeatureWeights 四类特征的基础权重。
 *
 * 各分量 ∈ [0, 1]，四者之和必须为 1.0（校验层断言）。
 */
struct ONEQ_API RirRecognitionFeatureWeights {
  float rcs_weight{0.25f};           /**< RCS 特征权重。 */
  float motion_weight{0.25f};        /**< 运动特征权重。 */
  float polarization_weight{0.25f};  /**< 极化特征权重。 */
  float range_profile_weight{0.25f}; /**< 距离像特征权重。 */
};

/**
 * @brief RirRecognitionPolicy 远程目标识别策略配置。
 *
 * 默认关闭（enabled=false），需显式启用。运行期可经
 * `RirRuntimeConfigPatch::has_policy` 整域覆盖提交修改。
 */
struct ONEQ_API RirRecognitionPolicy {
  bool enabled{false}; /**< 识别能力总开关（默认关闭）。 */

  /** 积累与确认 */
  std::uint32_t min_confirmed_hits{5U};    /**< 允许正式识别所需最小确认命中数（≥1）。 */
  float accumulation_window_sec{10.0f};    /**< 单航迹特征滑动积累窗口（s），必须 ≥ dt_sec。 */
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
  RirRecognitionFeatureWeights feature_weights{};

  /** 数据库 */
  std::string database_path{}; /**< 特征数据库 SQLite 文件路径；空串表示未配置。 */
};

/**
 * @brief RirPolicyConfig 远程识别雷达策略域配置。
 */
struct ONEQ_API RirPolicyConfig {
  RirDetectionPolicyConfig detection{};
  RirAssociationPolicyConfig association{};
  RirTrackingPolicyConfig tracking{};
  RirLifecyclePolicyConfig lifecycle{};
  RirRecognitionPolicy recognition{};
};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_POLICY_CONFIG_H_
