/**
 * @file InterceptPipelineTypes.h
 * @brief 定义电子侦察流水线公共配置与周期结果类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_INTERCEPT_PIPELINE_TYPES_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_INTERCEPT_PIPELINE_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/model/EmitterObservation.h"

namespace electronic_surveillance_radar {
namespace extension {

/**
 * @brief InterceptDetectionConfig 描述截获判定配置。
 */
struct ONEQ_API InterceptDetectionConfig {
  float receiver_noise_floor_w{1.0e-12f};   /**< 接收机等效噪声底（单位：W） */
  float minimum_snr_db{6.0f};            /**< 最小截获信噪比门限（单位：dB） */
  float max_detect_range_m{450000.0f};      /**< 最大截获距离（单位：m） */
  float min_dynamic_range_margin_db{-3.0f}; /**< 动态范围最小裕量（单位：dB） */
  float boundary_resolution_m{50.0f};       /**< 边界搜索分辨率（单位：m） */
  int boundary_max_iterations{32};          /**< 边界搜索最大迭代次数 */
};

/**
 * @brief InterceptIntegrationMode 表示统计检测中的积累模式。
 */
enum class InterceptIntegrationMode {
  kNonCoherent = 0, /**< 非相参积累 */
  kCoherent         /**< 相参积累 */
};

/**
 * @brief InterceptStatisticalDetectionConfig 描述统计检测与门限映射参数。
 */
struct ONEQ_API InterceptStatisticalDetectionConfig {
  float pfa{1.0e-6f};            /**< 期望虚警概率 Pfa，范围 (0, 1) */
  float min_snr_db{6.0f};        /**< 动态门限下限（单位：dB） */
  std::uint32_t pulse_count{8U}; /**< 脉冲积累数量 */
  InterceptIntegrationMode integration_mode{
      InterceptIntegrationMode::kNonCoherent}; /**< 积累模式 */
  float threshold_scale{1.0f};                 /**< 门限缩放系数 */
  bool enable_statistical_detection{true};     /**< 是否启用统计检测模型 */
};

/**
 * @brief InterceptScanConfig 描述扫描调度配置。
 */
struct ONEQ_API InterceptScanConfig {
  float scan_start_az_deg{-60.0f}; /**< 扫描起始方位（单位：deg） */
  float scan_end_az_deg{60.0f};    /**< 扫描结束方位（单位：deg） */
  float scan_start_el_deg{-10.0f}; /**< 扫描起始俯仰（单位：deg） */
  float scan_end_el_deg{10.0f};    /**< 扫描结束俯仰（单位：deg） */
  float az_step_deg{5.0f};         /**< 方位步进（单位：deg） */
  float el_step_deg{5.0f};         /**< 俯仰步进（单位：deg） */
  int scan_start_pos{0};           /**< 扫描起点编号（0: 左上, 1: 右上） */
  int scan_sequence{0};            /**< 扫描顺序（0: 先方位, 1: 先俯仰） */
};

/**
 * @brief InterceptAlgorithmConfig 描述算法辅助配置。
 */
struct ONEQ_API InterceptAlgorithmConfig {
  unsigned int random_seed{20260323U};  /**< 随机种子 */
  float angle_error_coefficient{0.51f}; /**< 测角误差系数 */
};

/**
 * @brief InterceptPreprocessConfig 描述观测预处理配置。
 */
struct ONEQ_API InterceptPreprocessConfig {
  float dedup_time_window_sec{5.0e-6f}; /**< 去重时间窗口（单位：s） */
  double dedup_rf_window_hz{1.0e6};     /**< 去重载频窗口（单位：Hz） */
  double dedup_pw_window_sec{3.0e-7};   /**< 去重脉宽窗口（单位：s） */
  float dedup_az_window_deg{1.0f};      /**< 去重方位窗口（单位：deg） */
  float dedup_el_window_deg{1.0f};      /**< 去重俯仰窗口（单位：deg） */
  bool normalize_quality{true};         /**< 是否根据观测质量规则重标定 quality 字段 */
};

/**
 * @brief InterceptClusterConfig 描述聚类配置与特征尺度。
 */
struct ONEQ_API InterceptClusterConfig {
  float radius{1.0f};           /**< 特征空间半径门限 */
  std::uint32_t min_points{1U}; /**< 成簇最小样本数 */
  float rf_scale_hz{5.0e6f};    /**< 载频尺度（单位：Hz） */
  float pw_scale_sec{1.0e-6f};  /**< 脉宽尺度（单位：s） */
  float az_scale_deg{2.0f};     /**< 方位尺度（单位：deg） */
  float el_scale_deg{2.0f};     /**< 俯仰尺度（单位：deg） */
  float snr_scale_db{8.0f};     /**< 信噪比尺度（单位：dB） */
};

/**
 * @brief InterceptSpectralAnalysisConfig 描述频谱特征分析与标签配置。
 */
struct ONEQ_API InterceptSpectralAnalysisConfig {
  bool enable{true};                             /**< 是否启用簇级频谱分析 */
  std::uint32_t min_sequence_length{4U};         /**< 最小样本序列长度 */
  std::uint32_t fft_length{16U};                 /**< 频谱分析 FFT 点数 */
  float broadband_occupancy_threshold{0.45f};    /**< 宽带占用阈值 */
  float agile_stability_threshold_hz{1.5e6f};    /**< 敏捷判定稳定度阈值（单位：Hz） */
  float agile_peak_sparsity_threshold{0.45f};    /**< 敏捷判定谱峰稀疏度阈值 */
  float occupancy_peak_floor_ratio{0.20f};       /**< 占用计算的谱峰底噪比例 */
};

/**
 * @brief InterceptAssociationConfig 描述假设关联与状态管理配置。
 */
struct ONEQ_API InterceptAssociationConfig {
  float gate_distance{1.2f};           /**< 簇到假设的关联门限距离 */
  std::uint32_t confirm_hits{3U};      /**< 假设确认所需连续命中周期数 */
  std::uint32_t max_missed_cycles{5U}; /**< 假设回收前允许的连续失配周期数 */
  float confidence_alpha{0.3f};        /**< 置信度更新系数 */
  bool output_tentative{true};         /**< 是否输出未确认假设 */
};

/**
 * @brief InterceptSuppressionModelConfig 描述压制分量建模参数。
 */
struct ONEQ_API InterceptSuppressionModelConfig {
  float suppression_noise_scale{1.0f};          /**< 压制功率映射到等效噪声功率的比例系数 */
  float suppression_mark_threshold_w{1.0e-12f}; /**< 压制生效判定阈值（单位：W） */
};

/**
 * @brief InterceptDeceptionModelConfig 描述欺骗分量建模参数。
 */
struct ONEQ_API InterceptDeceptionModelConfig {
  float false_alarm_probability_scale{1.0f};            /**< 欺骗虚警注入概率缩放系数 */
  float confusion_probability_scale{0.6f};              /**< 欺骗错分选触发概率缩放系数 */
  std::uint32_t max_false_observations_per_emitter{1U}; /**< 每辐射源每周期最大伪观测注入数 */
  float aoa_confusion_std_deg{4.0f};                    /**< 错分选方位扰动标准差（单位：deg） */
  float rf_confusion_ratio{0.02f};                      /**< 错分选载频扰动比例上限 */
  float pw_confusion_ratio{0.35f};                      /**< 错分选脉宽扰动比例上限 */
  float cluster_confidence_penalty_scale{0.55f};        /**< 簇级置信度欺骗惩罚缩放系数 */
};

/**
 * @brief InterceptRuntimeConfig 描述会话层注入的运行态参数。
 */
struct ONEQ_API InterceptRuntimeConfig {
  bool sensor_enabled{true};              /**< 设备是否开启 */
  bool use_fixed_receiver_window{false};  /**< 是否启用固定接收频段窗口 */
  double receiver_lower_hz{0.0};          /**< 固定接收频段下限（单位：Hz） */
  double receiver_upper_hz{0.0};          /**< 固定接收频段上限（单位：Hz） */
  float integrated_receive_loss_db{0.0f}; /**< 系统综合接收损耗（单位：dB） */
  float antenna_mount_az_deg{0.0f};       /**< 天线方位安装角（单位：deg） */
  float antenna_mount_el_deg{0.0f};       /**< 天线俯仰安装角（单位：deg） */
  float scan_rate_hz{1.0f};               /**< 扫描数据率（单位：Hz） */
};

/**
 * @brief InterceptPipelineConfig 描述电子侦察流水线顶层配置。
 */
struct ONEQ_API InterceptPipelineConfig {
  InterceptDetectionConfig detection{};                        /**< 截获判定配置 */
  InterceptStatisticalDetectionConfig statistical_detection{}; /**< 统计检测配置 */
  InterceptScanConfig scan{};                                  /**< 扫描调度配置 */
  InterceptAlgorithmConfig algorithm{};                        /**< 算法辅助配置 */
  InterceptPreprocessConfig preprocess{};                      /**< 观测预处理配置 */
  InterceptClusterConfig cluster{};                            /**< 聚类配置 */
  InterceptSpectralAnalysisConfig spectral_analysis{};         /**< 频谱分析配置 */
  InterceptAssociationConfig association{};                    /**< 假设关联配置 */
  InterceptSuppressionModelConfig suppression_model{};         /**< 压制分量建模配置 */
  InterceptDeceptionModelConfig deception_model{};             /**< 欺骗分量建模配置 */
};

/**
 * @brief TruthAssociationRecord 表示观测记录与真值辐射源的评估关联。
 */
struct ONEQ_API TruthAssociationRecord {
  std::uint64_t observation_id{0U};
  std::uint64_t truth_emitter_id{0U};
  bool matched{false};
  float confidence{0.0f};
};

/** @brief TruthAssociationRecordList 表示评估关联记录列表。 */
using TruthAssociationRecordList = std::vector<TruthAssociationRecord>;

/**
 * @brief ObservationOutputFrame 表示观测输出通道。
 */
struct ONEQ_API ObservationOutputFrame {
  std::size_t raw_observation_count{0U};
  std::size_t cluster_count{0U};
  model::EmitterObservationList observations{};
};

/**
 * @brief EmitterOutputFrame 表示侦察输出通道。
 */
struct ONEQ_API EmitterOutputFrame {
  model::EmitterHypothesisList hypotheses{};
};

/**
 * @brief TruthEvaluationFrame 表示真值评估输出通道。
 */
struct ONEQ_API TruthEvaluationFrame {
  TruthAssociationRecordList associations{};
};

/**
 * @brief InterceptPipelineResult 表示电子侦察流水线单周期执行结果（内部类型）。
 */
struct ONEQ_API InterceptPipelineResult {
  ObservationOutputFrame observation_output{};
  EmitterOutputFrame emitter_output{};
  TruthEvaluationFrame truth_evaluation_output{};
};

/**
 * @brief InterceptPipelineRuntimeState 描述 ESR 流水线运行态快照。
 */
struct ONEQ_API InterceptPipelineRuntimeState {
  const void* owner_identity{nullptr};    /**< 快照归属标识 */
  std::uint32_t schema_version{0U};      /**< 快照结构版本号 */
  std::shared_ptr<const void> snapshot;  /**< 不透明状态句柄 */
};

/**
 * @brief EsrPipelineAbortReason 表示单周期核心管线流产原因。
 */
enum class EsrPipelineAbortReason {
  kNone = 0,                    /**< 正常执行完成，未中断 */
  kValidationRejected,          /**< 因输入级严重校验问题（Error）而主动放弃计算 */
  kRuntimeStateRestoreRejected, /**< 因运行时状态回滚失败引发阻断 */
  kOutputContractViolation      /**< 下游计算返回的契约非法或状态错乱 */
};

}  // namespace extension
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_EXTENSION_INTERCEPT_PIPELINE_TYPES_H_
