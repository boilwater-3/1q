/**
 * @file TimingRegimeModel.h
 * @brief 定义库内共享的周期级时序体制与统计检测计算模型。
 */

#ifndef COMMON_TIMING_TIMING_REGIME_MODEL_H_
#define COMMON_TIMING_TIMING_REGIME_MODEL_H_

#include <cstdint>

namespace oneq {
namespace common {
namespace timing {

/**
 * @brief IntegrationMode 表示统计检测中的积累模式。
 */
enum class IntegrationMode {
  kNonCoherent = 0, /**< 非相参积累 */
  kCoherent         /**< 相参积累 */
};

/**
 * @brief StatisticalDetectionParams 描述统计检测与门限映射参数。
 */
struct StatisticalDetectionParams {
  float pfa{1.0e-6f};            /**< 期望虚警概率 Pfa，范围 (0, 1) */
  float min_snr_db{6.0f};        /**< 动态门限下限（单位：dB） */
  std::uint32_t pulse_count{8U}; /**< 脉冲积累数量 */
  IntegrationMode integration_mode{IntegrationMode::kNonCoherent}; /**< 积累模式 */
  float threshold_scale{1.0f};                                     /**< 门限缩放系数 */
  bool enable_statistical_detection{true};                         /**< 是否启用统计检测模型 */
};

/**
 * @brief CycleTimingControlParams 描述周期级时序控制输入参数。
 */
struct CycleTimingControlParams {
  int base_pulse_count{1};     /**< 基线脉冲数 */
  float dwell_scale{1.0f};     /**< 驻留缩放因子 */
  float base_prf_hz{0.0f};     /**< 基线 PRF（单位：Hz） */
  bool enable_rejitter{false}; /**< 是否启用重频抖动 */
};

/**
 * @brief CycleTimingBaseParams 描述周期级时序的原始基线输入。
 */
struct CycleTimingBaseParams {
  int base_pulse_count{1};                                         /**< 基线脉冲数 */
  float base_prf_hz{0.0f};                                         /**< 基线 PRF（单位：Hz） */
  float cycle_dt_sec{0.0f};                                        /**< 周期步长（单位：s） */
  double pri_s{0.0};                                               /**< 脉冲重复间隔（单位：s） */
  IntegrationMode integration_mode{IntegrationMode::kNonCoherent}; /**< 积累模式 */
};

/**
 * @brief CycleTimingControlAdjustments 描述作用在周期级时序上的控制调节量。
 */
struct CycleTimingControlAdjustments {
  float dwell_scale{1.0f};     /**< 驻留缩放因子 */
  bool enable_rejitter{false}; /**< 是否启用重频抖动 */
  float prf_scale{1.0f};       /**< PRF 额外缩放因子，为后续模块扩展预留 */
};

/**
 * @brief ResolvedCycleTimingState 描述供检测/截获链路消费的周期级体制状态。
 */
struct ResolvedCycleTimingState {
  std::uint32_t available_pulse_count{1U}; /**< 当前周期可用脉冲数 */
  std::uint32_t effective_pulse_count{1U}; /**< 当前周期有效脉冲数 */
  float effective_prf_hz{0.0f};            /**< 当前周期有效 PRF（单位：Hz） */
  IntegrationMode integration_mode{IntegrationMode::kNonCoherent}; /**< 当前积累模式 */
};

/**
 * @brief 对统计检测配置中的 Pfa 做数值归一化。
 * @param[in] pfa 输入虚警概率。
 * @return 归一化后的 Pfa。
 */
float NormalizePfa(float pfa);

/**
 * @brief 根据周期窗口解析当前周期可用脉冲数。
 * @param[in] params 周期级时序原始基线输入。
 * @return 当前周期可用脉冲数。
 */
std::uint32_t ResolveAvailablePulseCount(const CycleTimingBaseParams& params);

/**
 * @brief 根据驻留缩放映射当前周期脉冲数。
 * @param[in] params 周期级时序控制输入参数。
 * @return 映射后的脉冲数（最小为 1）。
 */
int ResolvePulseCountWithDwell(const CycleTimingControlParams& params);

/**
 * @brief 根据基线与控制调节量解析当前周期有效脉冲数。
 * @param[in] base_params 周期级时序原始基线输入。
 * @param[in] adjustments 周期级时序控制调节量。
 * @return 当前周期有效脉冲数。
 */
std::uint32_t ResolveEffectivePulseCount(const CycleTimingBaseParams& base_params,
                                         const CycleTimingControlAdjustments& adjustments);

/**
 * @brief 根据重频抖动开关映射当前周期 PRF。
 * @param[in] params 周期级时序控制输入参数。
 * @return 映射后的 PRF（单位：Hz）。
 */
float ResolvePrfWithRejitter(const CycleTimingControlParams& params);

/**
 * @brief 根据基线与控制调节量解析当前周期有效 PRF。
 * @param[in] base_params 周期级时序原始基线输入。
 * @param[in] adjustments 周期级时序控制调节量。
 * @return 当前周期有效 PRF（单位：Hz）。
 */
float ResolveEffectivePrfHz(const CycleTimingBaseParams& base_params,
                            const CycleTimingControlAdjustments& adjustments);

/**
 * @brief 解析统一的周期级体制状态。
 * @param[in] base_params 周期级时序原始基线输入。
 * @param[in] adjustments 周期级时序控制调节量。
 * @return 供检测/截获链路消费的统一周期状态。
 */
ResolvedCycleTimingState ResolveCycleTimingState(const CycleTimingBaseParams& base_params,
                                                 const CycleTimingControlAdjustments& adjustments);

/**
 * @brief 计算统计检测对应的积累增益。
 * @param[in] params 统计检测与门限映射参数。
 * @return 增益系数。
 */
double ComputeIntegrationGain(const StatisticalDetectionParams& params);

/**
 * @brief 根据 Pfa 与噪声底计算 CFAR 风格动态门限。
 * @param[in] noise_power_w 等效噪声底（单位：W）。
 * @param[in] params 统计检测与门限映射参数。
 * @return 动态检测门限（单位：dB）。
 */
float ComputeDynamicThresholdSnrDb(double noise_power_w, const StatisticalDetectionParams& params);

/**
 * @brief 计算统计检测概率 Pd。
 * @param[in] snr_db 当前观测信噪比（单位：dB）。
 * @param[in] threshold_snr_db 动态门限（单位：dB）。
 * @param[in] params 统计检测与门限映射参数。
 * @return 检测概率 Pd，范围 [0, 1]。
 */
float ComputeStatisticalDetectionProbability(float snr_db, float threshold_snr_db,
                                             const StatisticalDetectionParams& params);

}  // namespace timing
}  // namespace common
}  // namespace oneq

#endif  // COMMON_TIMING_TIMING_REGIME_MODEL_H_
