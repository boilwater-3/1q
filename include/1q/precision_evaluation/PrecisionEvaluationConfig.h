/**
 * @file PrecisionEvaluationConfig.h
 * @brief 定义精度评估会话的聚合配置（双星 SBIRS + 融合 + 推演 + AHP 指标体系）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_CONFIG_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_CONFIG_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/fusion/FusionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "1q/target_inference/TargetInferenceConfig.h"

namespace precision_evaluation {

/**
 * @brief 精度评估会话配置。
 * @details 聚合被评估链路（双星 SBIRS 传感器 + fusion 估计 + target_inference 推演）
 *          与指标体系（AHP 判断矩阵 + 归一化参考误差）。评估会话内部强制
 *          `fusion.enable_track_filtering = true`（速度/位置误差样本依赖逐航迹滤波；
 *          默认关的融合无运动学估计），调用方无需自行设置。
 * @note 参考误差默认值为演示口径（角度 0.05°、双星 10 km、速度 100 m/s、落点 10 km、
 *       发射点 20 km）：score = ref/(ref+rmse)，rmse=ref 时得 0.5 分。正式验收标定
 *       前按装备指标替换（见 docs/precision_evaluation/algorithms.md）。
 */
struct ONEQ_API PrecisionEvaluationConfig {
  sbirs_sensor::config::SbirsSessionConfig satellite_a{}; /**< 主星 SBIRS 会话配置 */
  sbirs_sensor::config::SbirsSessionConfig satellite_b{}; /**< 辅星 SBIRS 会话配置 */
  fusion::FusionConfig fusion{}; /**< 融合配置（评估内部强制开逐航迹滤波） */
  target_inference::TargetInferenceConfig inference{}; /**< 推演配置（落点/发射点预测） */
  AhpJudgmentMatrix ahp{}; /**< AHP 判断矩阵（默认全 1 = 五指标等权） */

  double reference_error_angular_deg{0.05};      /**< 角度误差参考值，deg */
  double reference_error_dual_sat_fix_m{10000.0}; /**< 双星交会位置误差参考值，m */
  double reference_error_velocity_m_per_s{100.0}; /**< 速度误差参考值，m/s */
  double reference_error_impact_m{10000.0};       /**< 落点预测误差参考值，m */
  double reference_error_launch_m{20000.0};       /**< 发射点预测误差参考值，m */

  std::uint32_t inference_interval_cycles{1U}; /**< 关键点推演间隔（周期数；≥1） */

  // 双星进融合的源通道标识：A 星沿用 SensorAdapters 的 SBIRS 源 id（4），B 星用
  // 独立 id（104）保证逐源通道统计；两值不得相等（构造时校验，相等则 B 星 +100）。
  std::uint32_t satellite_a_source_id{4U};
  std::uint32_t satellite_b_source_id{104U};
};

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_CONFIG_H_
