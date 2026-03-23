/**
 * @file InterceptPipelineTypes.h
 * @brief 定义电子侦察流水线公共配置与周期结果类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_INTERCEPT_PIPELINE_TYPES_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_INTERCEPT_PIPELINE_TYPES_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief InterceptDetectionConfig 描述截获判定配置。
 */
struct ONEQ_API InterceptDetectionConfig {
  float receiver_noise_floor_w{1.0e-12f}; /**< 接收机等效噪声底（单位：W） */
  float min_detect_snr_db{6.0f}; /**< 最小截获信噪比门限（单位：dB） */
  float max_detect_range_m{450000.0f}; /**< 最大截获距离（单位：m） */
  float min_dynamic_range_margin_db{-3.0f}; /**< 动态范围最小裕量（单位：dB） */
  float boundary_resolution_m{50.0f}; /**< 边界搜索分辨率（单位：m） */
  int boundary_max_iterations{32}; /**< 边界搜索最大迭代次数 */
};

/**
 * @brief InterceptScanConfig 描述扫描调度配置。
 */
struct ONEQ_API InterceptScanConfig {
  float scan_start_az_deg{-60.0f}; /**< 扫描起始方位（单位：deg） */
  float scan_end_az_deg{60.0f}; /**< 扫描结束方位（单位：deg） */
  float scan_start_el_deg{-10.0f}; /**< 扫描起始俯仰（单位：deg） */
  float scan_end_el_deg{10.0f}; /**< 扫描结束俯仰（单位：deg） */
  float az_step_deg{5.0f}; /**< 方位步进（单位：deg） */
  float el_step_deg{5.0f}; /**< 俯仰步进（单位：deg） */
  int scan_start_pos{0}; /**< 扫描起点编号（0: 左上, 1: 右上） */
  int scan_sequence{0}; /**< 扫描顺序（0: 先方位, 1: 先俯仰） */
};

/**
 * @brief InterceptAlgorithmConfig 描述算法辅助配置。
 */
struct ONEQ_API InterceptAlgorithmConfig {
  unsigned int random_seed{20260323U}; /**< 随机种子 */
  float angle_error_coefficient{0.51f}; /**< 测角误差系数 */
};

/**
 * @brief InterceptPipelineConfig 描述电子侦察流水线顶层配置。
 */
struct ONEQ_API InterceptPipelineConfig {
  InterceptDetectionConfig detection{}; /**< 截获判定配置 */
  InterceptScanConfig scan{}; /**< 扫描调度配置 */
  InterceptAlgorithmConfig algorithm{}; /**< 算法辅助配置 */
};

/**
 * @brief InterceptCycleResult 描述单周期流水线输出。
 */
struct ONEQ_API InterceptCycleResult {
  common::EmitterObservationList observations{}; /**< 当前周期观测记录 */
  common::EmitterHypothesisList emitter_hypotheses{}; /**< 当前周期辐射源假设 */
  common::TruthAssociationRecordList truth_associations{}; /**< 当前周期真值评估关联记录 */
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_PIPELINE_INTERCEPT_PIPELINE_TYPES_H_
