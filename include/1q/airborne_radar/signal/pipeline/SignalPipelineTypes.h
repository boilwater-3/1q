/**
 * @file SignalPipelineTypes.h
 * @brief 定义信号流水线公共配置与结果类型。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/model/DecisionInputFrame.h"
#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/config/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/signal/detection/DetectionTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 关联质量观测指标（Pipeline 对外公开版本）。
 */
struct AssociationQualityMetrics {
  std::size_t prior_track_count{0};  /**< 进入关联阶段的历史先验轨迹数 */
  std::size_t detection_count{0};    /**< 本周期探测成功并参与关联的量测数 */
  std::size_t matched_count{0};      /**< 命中已有轨迹的关联数 */
  std::size_t new_track_count{0};    /**< 触发新建轨迹键的量测数 */
  std::size_t missed_track_count{0}; /**< 未命中任何量测的历史轨迹数 */
  float match_rate{0.0f};            /**< 命中率（matched_count / detection_count） */
  float new_track_rate{0.0f};        /**< 新生率（new_track_count / detection_count） */
  float missed_track_rate{0.0f};     /**< 漏失率（missed_track_count / prior_track_count） */
  float mean_match_cost{0.0f};       /**< 命中关联代价均值（仅统计 matches） */
  float p95_match_cost{0.0f};        /**< 命中关联代价 P95（仅统计 matches） */
  common::utils::JammingSemantic dominant_jamming_semantic{
      common::utils::JammingSemantic::kNone}; /**< 当前周期关联质量对应的主导干扰摘要类型 */
  float jamming_severity{0.0f};        /**< 当前周期关联质量对应的残余干扰强度摘要，范围 [0, 1] */
  float association_stress{0.0f};      /**< 当前周期的归一化关联压力，范围 [0, 1] */
};

/**
 * @brief TrackPoolThreadSafetyMode 定义对象池线程安全策略。
 */
enum class TrackPoolThreadSafetyMode {
  kSingleThreadNoLock = 0, /**< 默认无锁模式，适用于当前单线程生命周期更新 */
  kMultiThreadGlobalLock   /**< 使用全局互斥保护对象池接口 */
};

/**
 * @brief ImmActivationPolicy 定义 IMM 多模型路径的激活策略。
 */
enum class ImmActivationPolicy {
  kAllTracks = 0,      /**< 所有轨迹都按当前 IMM 语义创建和使用多模型路径 */
  kConfirmedTracksOnly /**< 仅已确认轨迹在再次命中时懒创建并启用 IMM */
};

/**
 * @brief LifecycleConfig 定义轨迹状态机阈值配置。
 */
struct LifecycleConfig {
  std::uint32_t confirm_hits{3};         /**< 候选轨迹转已确认所需最小命中次数 */
  std::uint32_t max_miss_before_lost{2}; /**< 已确认轨迹转丢失前允许的最大连续失配次数 */
  std::uint32_t max_lost_cycles{5};      /**< 丢失轨迹可保留的最大周期数，超出则回收 */
  ImmActivationPolicy imm_activation_policy{
      ImmActivationPolicy::kConfirmedTracksOnly}; /**< IMM 激活策略 */
  TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      TrackPoolThreadSafetyMode::kSingleThreadNoLock}; /**< 对象池线程安全策略 */
};

/**
 * @brief SignalDetectionConfig 描述信号探测域配置。
 */
struct SignalDetectionConfig {
  bool enable_physics_detection{false};        /**< 是否启用物理层检测 */
  detection::RadarSystemConfig radar_system{}; /**< 雷达系统配置 */
  float min_detection_margin_db{-2.0f};        /**< 最小检测裕量（dB） */
  int pulse_count{10};                         /**< 脉冲数 */
  bool coherent_integration{true};             /**< 是否启用相干积累 */
};

/**
 * @brief SignalBeamControlConfig 描述波束控制域配置。
 */
struct SignalBeamControlConfig {
  common::config::RadarOrientationConfig radar_orientation{};  /**< 雷达方向配置 */
  common::config::PlatformAttitudeDeg platform_attitude_deg{}; /**< 平台姿态角（单位：度） */
};

/**
 * @brief SignalAssociationConfig 描述信号层关联域配置。
 */
struct SignalAssociationConfig {
  float unassigned_cost{9.0f}; /**< 未分配代价 */
};

/**
 * @brief SignalTrackingConfig 描述信号层跟踪域配置。
 */
struct SignalTrackingConfig {
  bool enable_kalman_filter{true};           /**< 是否启用卡尔曼滤波器 */
  float kalman_noise_diff_coeff{1.0f};       /**< 卡尔曼噪声扩散系数 */
  float kalman_measurement_noise_std{10.0f}; /**< 卡尔曼测量噪声标准差 */
  float speed_decay_ratio_on_loss{0.90f};    /**< 丢失时速度衰减比例 */
  float rcs_decay_ratio_on_loss{0.85f};      /**< 丢失时 RCS 衰减比例 */
};

/**
 * @brief SignalLifecycleConfig 描述生命周期域配置。
 */
struct SignalLifecycleConfig {
  bool enable_auto_lifecycle_manager{false};          /**< 是否启用自动生命周期管理 */
  LifecycleConfig lifecycle_config{};                 /**< 生命周期配置 */
  bool enable_imm_lifecycle{false};                   /**< 是否启用 IMM 生命周期管理 */
  std::vector<float> imm_model_noise_diff_coeffs;     /**< IMM 模型噪声扩散系数列表 */
  std::vector<float> imm_initial_weights;             /**< IMM 初始权重列表 */
  std::vector<float> imm_transition_probability;      /**< IMM 状态转移概率列表 */
  std::size_t lifecycle_track_pool_initial_chunk{64}; /**< 轨迹池初始块大小 */
  std::size_t lifecycle_track_pool_max_chunks{256};   /**< 轨迹池最大块数 */
};

/**
 * @brief JammingEffectsConfig 描述干扰效应建模的可调参数。
 *
 * @details 这些系数控制干扰环境对探测惩罚、量测协方差和关联/跟踪噪声
 * 的影响程度。默认值来自经验整定，可按业务场景微调。
 * 内部物理 ECCM 抑制系数（如旁瓣对消残余因子）不在此暴露，
 * 属于雷达对抗物理模型，不应在运行时修改。
 */
struct JammingEffectsConfig {
  // ---------- D. 置信度截断 ----------
  float confidence_weight_min{0.25f}; /**< 干扰源置信度归一化下界，低于此值按此值计算权重 */

  // ---------- B. 经验 SNR 惩罚权重 ----------
  float heuristic_base_penalty_db{0.8f};          /**< 单源基础惩罚量（dB） */
  float heuristic_power_penalty_slope{0.18f};     /**< 功率贡献斜率（dB/dB，功率范围 [0,20] dB） */
  float heuristic_noise_sidelobe_penalty{0.9f};   /**< 噪声压制旁瓣命中时的额外惩罚（dB） */
  float heuristic_noise_frontlobe_ratio{0.4f};    /**< 噪声压制主瓣时旁瓣惩罚的折减比（无量纲） */
  float heuristic_noise_js_slope{0.4f};           /**< 噪声压制 J/S 比贡献斜率（dB，范围 [0,12]） */
  float heuristic_deception_freq_penalty{1.8f};   /**< 欺骗干扰频率重叠比惩罚权重（dB） */
  float heuristic_deception_prf_penalty{1.2f};    /**< 欺骗干扰 PRF 锁定风险惩罚权重（dB） */
  float heuristic_repeater_prf_penalty{1.0f};     /**< 转发干扰 PRF 锁定风险惩罚权重（dB） */
  float heuristic_repeater_freq_penalty{0.8f};    /**< 转发干扰频率重叠比惩罚权重（dB） */
  float heuristic_unknown_freq_penalty{1.1f};     /**< 未知干扰频率重叠比惩罚权重（dB） */
  float heuristic_unknown_prf_penalty{0.9f};      /**< 未知干扰 PRF 锁定风险惩罚权重（dB） */
  float heuristic_unknown_sidelobe_penalty{0.6f}; /**< 未知干扰旁瓣命中时额外惩罚（dB） */
  float heuristic_unknown_frontlobe_penalty{0.2f};/**< 未知干扰主瓣时额外惩罚（dB） */

  // ---------- C. 关联/跟踪噪声缩放上限 ----------
  float association_scale_max{2.5f};       /**< 关联未分配代价动态放大上限 */
  float tracking_noise_scale_max{2.0f};    /**< 跟踪过程噪声动态放大上限 */
  float measurement_noise_scale_max{1.8f}; /**< 量测噪声标准差动态放大上限 */

  // ---------- C. 各干扰类型对关联/跟踪的缩放步长 ----------
  float deception_association_step{0.18f};    /**< 欺骗干扰对关联代价的放大步长 */
  float deception_tracking_step{0.12f};       /**< 欺骗干扰对跟踪过程噪声的放大步长 */
  float deception_measurement_step{0.10f};    /**< 欺骗干扰对量测噪声的放大步长 */
  float repeater_association_step{0.12f};     /**< 转发干扰对关联代价的放大步长 */
  float repeater_tracking_step{0.15f};        /**< 转发干扰对跟踪过程噪声的放大步长 */
  float repeater_measurement_step{0.08f};     /**< 转发干扰对量测噪声的放大步长 */
  float noise_measurement_step{0.06f};        /**< 噪声压制干扰对量测噪声的放大步长 */
  float unknown_association_step{0.08f};      /**< 未知干扰对关联代价的放大步长 */
  float unknown_tracking_step{0.08f};         /**< 未知干扰对跟踪过程噪声的放大步长 */
  float unknown_measurement_step{0.05f};      /**< 未知干扰对量测噪声的放大步长 */

  // ---------- C. 量测协方差膨胀步长上限 ----------
  float covariance_inflation_max{2.5f};             /**< 量测协方差总膨胀因子上限 */
  float covariance_deception_inflation_step{0.20f}; /**< 欺骗干扰协方差膨胀步长 */
  float covariance_repeater_inflation_step{0.16f};  /**< 转发干扰协方差膨胀步长 */
  float covariance_noise_inflation_step{0.08f};     /**< 噪声压制干扰协方差膨胀步长 */
  float covariance_unknown_inflation_step{0.10f};   /**< 未知干扰协方差膨胀步长 */
};

/**
 * @brief ControlProfileEffectsConfig 描述控制轮廓对天线物理特性和生命周期策略的可调参数。
 *
 * @details 仅包含随平台或任务场景变化的物理量与策略量，例如天线增益、波束宽度、
 * 衰减修正量。Kalman/关联算法内部权重系数为经验整定常量，不在此暴露。
 * 设计与 JammingEffectsConfig 对称，嵌入 SignalPipelineConfig。
 */
struct ControlProfileEffectsConfig {
  // ---------- 旁瓣对消天线物理特性 ----------
  float sidelobe_level_reduction_db{6.0f}; /**< 旁瓣对消启用时天线旁瓣电平降低量（dB）*/

  // ---------- 自适应波束天线物理特性 ----------
  float adaptive_beam_gain_boost_db{2.0f}; /**< 自适应波束成形主波束增益提升量（dB）*/
  float adaptive_beamwidth_scale{0.60f};   /**< 自适应波束成形启用时波束宽度缩放系数 */

  // ---------- LPI 波束成形天线物理特性 ----------
  float lpi_beamwidth_scale{0.75f};    /**< LPI 波束成形启用时波束宽度缩放系数 */
  float lpi_beam_signal_gain_db{1.0f}; /**< LPI 波束成形启用时启发式信号调整量（dB）*/

  // ---------- 自适应波束启发式信号增益 ----------
  float adaptive_beam_signal_gain_db{1.5f}; /**< 自适应波束成形启用时启发式信号调整量（dB）*/

  // ---------- ECCM 激活对生命周期衰减系数的策略修正量 ----------
  float eccm_speed_decay_bonus{0.05f}; /**< ECCM 激活时速度衰减系数增量 */
  float eccm_rcs_decay_bonus{0.08f};   /**< ECCM 激活时 RCS 衰减系数增量 */
};

/**
 * @brief SignalPipelineConfig 描述信号处理流水线顶层配置。
 */
struct SignalPipelineConfig {
  SignalDetectionConfig detection{};                          /**< 探测配置 */
  SignalBeamControlConfig beam_control{};                     /**< 波束控制配置 */
  SignalAssociationConfig association{};                      /**< 关联配置 */
  SignalTrackingConfig tracking{};                            /**< 跟踪配置 */
  SignalLifecycleConfig lifecycle{};                          /**< 生命周期配置 */
  JammingEffectsConfig jamming_effects{};                     /**< 干扰效应建模参数 */
  ControlProfileEffectsConfig control_profile_effects{};     /**< 控制轮廓效应建模参数 */
};

/**
 * @brief SignalCycleResult 描述信号流水线单周期的稳定输出。
 */
struct SignalCycleResult {
  common::model::TargetFeatureList updated_features{};            /**< 当前周期更新后的目标特征列表 */
  common::model::DecisionInputFrame decision_frame{};             /**< 当前周期决策输入帧 */
  AssociationQualityMetrics association_quality_metrics{}; /**< 当前周期关联质量观测指标 */
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_TYPES_H_
