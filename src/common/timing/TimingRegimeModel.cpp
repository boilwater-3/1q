#include "common/timing/TimingRegimeModel.h"

#include <algorithm>
#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace oneq {
namespace internal {
namespace timing {

namespace {

/**
 * @brief 数值稳定保护下限。
 * @note 用于避免除零、`log10(0)` 与阈值因子退化。
 */
constexpr double kNumericFloor = 1.0e-18;
/**
 * @brief 默认虚警概率回退值。
 */
constexpr float kDefaultPfa = 1.0e-6f;
/**
 * @brief 虚警概率下界。
 */
constexpr float kMinPfa = 1.0e-9f;
/**
 * @brief 虚警概率上界。
 */
constexpr float kMaxPfa = 1.0e-2f;
/**
 * @brief 驻留缩放下界。
 */
constexpr float kMinDwellScale = 0.25f;
/**
 * @brief 驻留缩放上界。
 */
constexpr float kMaxDwellScale = 4.0f;
/**
 * @brief 非法比例的默认回退值。
 */
constexpr float kScaleFallback = 1.0f;
/**
 * @brief 重频抖动缩放因子。
 */
constexpr float kPrfRejitterScale = 1.10f;

/**
 * @brief 约束控制比例，避免非法值污染运行时配置。
 * @param[in] scale 待校验比例。
 * @param[in] fallback 非法输入时的回退值。
 * @return 合法比例或回退值。
 */
float ClampProfileScale(float scale, float fallback) {
  if (!std::isfinite(scale) || scale <= 0.0f) {
    return fallback;
  }
  return scale;
}

/**
 * @brief 把线性功率比转换为 dB。
 * @param[in] ratio 线性功率比。
 * @return dB 表示值。
 */
float ToDb(double ratio) {
  return static_cast<float>(10.0 * std::log10(std::max(ratio, kNumericFloor)));
}

/**
 * @brief 判断是否提供了显式的周期窗口。
 * @param[in] params 周期级时序原始基线输入。
 * @return 若 `cycle_dt_sec` 与 `pri_s` 均为正且有限，则返回 `true`。
 */
bool HasExplicitPulseWindow(const CycleTimingBaseParams& params) {
  return std::isfinite(params.cycle_dt_sec) && params.cycle_dt_sec > 0.0f &&
         std::isfinite(params.pri_s) && params.pri_s > 0.0;
}

}  // namespace

float NormalizePfa(float pfa) {
  if (!std::isfinite(pfa)) {
    return kDefaultPfa;
  }
  return std::max(kMinPfa, std::min(kMaxPfa, pfa));
}

std::uint32_t ResolveAvailablePulseCount(const CycleTimingBaseParams& params) {
  if (!HasExplicitPulseWindow(params)) {
    return static_cast<std::uint32_t>(std::max(1, params.base_pulse_count));
  }
  const double pulse_window =
      static_cast<double>(params.cycle_dt_sec) / std::max(params.pri_s, kNumericFloor);
  if (!std::isfinite(pulse_window) || pulse_window <= 0.0) {
    return 0U;
  }
  return static_cast<std::uint32_t>(std::floor(pulse_window));
}

int ResolvePulseCountWithDwell(const CycleTimingControlParams& params) {
  const float safe_dwell_scale = ClampProfileScale(params.dwell_scale, kScaleFallback);
  const float dwell_scale = std::max(kMinDwellScale, std::min(kMaxDwellScale, safe_dwell_scale));
  const double scaled_pulse_count =
      static_cast<double>(params.base_pulse_count) * static_cast<double>(dwell_scale);
  return std::max(1, static_cast<int>(std::lround(scaled_pulse_count)));
}

std::uint32_t ResolveEffectivePulseCount(const CycleTimingBaseParams& base_params,
                                         const CycleTimingControlAdjustments& adjustments) {
  CycleTimingControlParams dwell_params;
  dwell_params.base_pulse_count = base_params.base_pulse_count;
  dwell_params.dwell_scale = adjustments.dwell_scale;
  const std::uint32_t dwell_adjusted_pulse_count =
      static_cast<std::uint32_t>(std::max(1, ResolvePulseCountWithDwell(dwell_params)));
  if (!HasExplicitPulseWindow(base_params)) {
    return dwell_adjusted_pulse_count;
  }
  return std::min(dwell_adjusted_pulse_count, ResolveAvailablePulseCount(base_params));
}

float ResolvePrfWithRejitter(const CycleTimingControlParams& params) {
  CycleTimingBaseParams base_params;
  base_params.base_prf_hz = params.base_prf_hz;
  CycleTimingControlAdjustments adjustments;
  adjustments.enable_rejitter = params.enable_rejitter;
  return ResolveEffectivePrfHz(base_params, adjustments);
}

float ResolveEffectivePrfHz(const CycleTimingBaseParams& base_params,
                            const CycleTimingControlAdjustments& adjustments) {
  const float safe_prf_scale = ClampProfileScale(adjustments.prf_scale, kScaleFallback);
  float effective_prf_hz = base_params.base_prf_hz * safe_prf_scale;
  if (adjustments.enable_rejitter) {
    effective_prf_hz *= kPrfRejitterScale;
  }
  return effective_prf_hz;
}

ResolvedCycleTimingState ResolveCycleTimingState(const CycleTimingBaseParams& base_params,
                                                 const CycleTimingControlAdjustments& adjustments) {
  ResolvedCycleTimingState state;
  state.available_pulse_count = ResolveAvailablePulseCount(base_params);
  state.effective_pulse_count = ResolveEffectivePulseCount(base_params, adjustments);
  state.effective_prf_hz = ResolveEffectivePrfHz(base_params, adjustments);
  state.integration_mode = base_params.integration_mode;
  return state;
}

double ComputeIntegrationGain(const StatisticalDetectionParams& params) {
  const std::uint32_t pulse_count = std::max(1U, params.pulse_count);
  if (params.integration_mode == IntegrationMode::kCoherent) {
    return static_cast<double>(pulse_count);
  }
  return std::sqrt(static_cast<double>(pulse_count));
}

float ComputeDynamicThresholdSnrDb(double noise_power_w, const StatisticalDetectionParams& params) {
  const double safe_noise_power_w = std::max(noise_power_w, kNumericFloor);
  const float pfa = NormalizePfa(params.pfa);
  const std::uint32_t pulse_count = std::max(1U, params.pulse_count);
  const double pulse_count_double = static_cast<double>(pulse_count);
  const double threshold_scale =
      std::max(static_cast<double>(params.threshold_scale), static_cast<double>(0.1f));

  double threshold_factor = 1.0;
  if (params.integration_mode == IntegrationMode::kCoherent) {
    threshold_factor = -std::log(static_cast<double>(pfa)) / pulse_count_double;
  } else {
    threshold_factor = pulse_count_double *
                       (std::pow(1.0 / static_cast<double>(pfa), 1.0 / pulse_count_double) - 1.0);
  }
  threshold_factor = std::max(threshold_factor, kNumericFloor) * threshold_scale;
  const double threshold_power_w = safe_noise_power_w * threshold_factor;
  const float threshold_db = ToDb(threshold_power_w / safe_noise_power_w);
  return std::max(params.min_snr_db, threshold_db);
}

float ComputeStatisticalDetectionProbability(float snr_db, float threshold_snr_db,
                                             const StatisticalDetectionParams& params) {
  if (!std::isfinite(snr_db) || !std::isfinite(threshold_snr_db)) {
    return 0.0f;
  }
  const double snr_linear = std::pow(10.0, static_cast<double>(snr_db) / 10.0);
  const double threshold_linear = std::pow(10.0, static_cast<double>(threshold_snr_db) / 10.0);
  const double normalized_metric =
      (snr_linear / std::max(threshold_linear, kNumericFloor)) * ComputeIntegrationGain(params);
  const double pd = 1.0 - std::exp(-std::max(normalized_metric, 0.0));
  return oneq::internal::numerics::Clamp01(static_cast<float>(pd));
}

}  // namespace timing
}  // namespace internal
}  // namespace oneq
