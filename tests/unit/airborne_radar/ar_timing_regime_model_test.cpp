/**
 * @file timing_regime_model_test.cpp
 * @brief 验证共享周期级时序体制与统计检测模型行为。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "common/timing/TimingRegimeModel.h"

namespace oneq {
namespace common {
namespace timing {
namespace {

TEST(TimingRegimeModelTest, ResolvePulseCountWithDwellHandlesBoundaryAndFallback) {
  CycleTimingControlParams params;
  params.base_pulse_count = 10;
  params.dwell_scale = 0.10f;
  EXPECT_EQ(ResolvePulseCountWithDwell(params), 3);

  params.dwell_scale = 6.0f;
  EXPECT_EQ(ResolvePulseCountWithDwell(params), 40);

  params.dwell_scale = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(ResolvePulseCountWithDwell(params), 10);

  params.dwell_scale = 0.0f;
  EXPECT_EQ(ResolvePulseCountWithDwell(params), 10);
}

TEST(TimingRegimeModelTest, ResolvePrfWithRejitterFollowsSwitch) {
  CycleTimingControlParams params;
  params.base_prf_hz = 300.0f;
  params.enable_rejitter = false;
  EXPECT_FLOAT_EQ(ResolvePrfWithRejitter(params), 300.0f);

  params.enable_rejitter = true;
  EXPECT_FLOAT_EQ(ResolvePrfWithRejitter(params), 330.0f);
}

TEST(TimingRegimeModelTest, ResolveAvailablePulseCountSupportsPriWindowAndFallback) {
  CycleTimingBaseParams params;
  params.base_pulse_count = 8;
  params.cycle_dt_sec = 1.0f;
  params.pri_s = 0.2;
  EXPECT_EQ(ResolveAvailablePulseCount(params), 5U);

  params.pri_s = 2.0;
  EXPECT_EQ(ResolveAvailablePulseCount(params), 0U);

  params.cycle_dt_sec = 0.0f;
  params.pri_s = 0.0;
  EXPECT_EQ(ResolveAvailablePulseCount(params), 8U);
}

TEST(TimingRegimeModelTest, ResolveCycleTimingStateCombinesAvailabilityAndControlMapping) {
  CycleTimingBaseParams base_params;
  base_params.base_pulse_count = 10;
  base_params.base_prf_hz = 300.0f;
  base_params.cycle_dt_sec = 1.0f;
  base_params.pri_s = 0.2;
  base_params.integration_mode = IntegrationMode::kCoherent;

  CycleTimingControlAdjustments adjustments;
  adjustments.dwell_scale = 2.0f;
  adjustments.enable_rejitter = true;

  const ResolvedCycleTimingState state = ResolveCycleTimingState(base_params, adjustments);
  EXPECT_EQ(state.available_pulse_count, 5U);
  EXPECT_EQ(state.effective_pulse_count, 5U);
  EXPECT_FLOAT_EQ(state.effective_prf_hz, 330.0f);
  EXPECT_EQ(state.integration_mode, IntegrationMode::kCoherent);
}

TEST(TimingRegimeModelTest, ResolveEffectivePulseCountMonotonicallyTracksAvailabilityClamp) {
  CycleTimingBaseParams base_params;
  base_params.base_pulse_count = 12;
  base_params.cycle_dt_sec = 0.35f;
  base_params.pri_s = 0.1;

  CycleTimingControlAdjustments low_adjustments;
  low_adjustments.dwell_scale = 0.5f;
  const std::uint32_t low_effective_pulse_count =
      ResolveEffectivePulseCount(base_params, low_adjustments);

  CycleTimingControlAdjustments high_adjustments;
  high_adjustments.dwell_scale = 4.0f;
  const std::uint32_t high_effective_pulse_count =
      ResolveEffectivePulseCount(base_params, high_adjustments);

  EXPECT_LE(low_effective_pulse_count, high_effective_pulse_count);
  EXPECT_EQ(high_effective_pulse_count, ResolveAvailablePulseCount(base_params));
}

TEST(TimingRegimeModelTest, NormalizePfaClampsAndFallbacks) {
  EXPECT_FLOAT_EQ(NormalizePfa(std::numeric_limits<float>::quiet_NaN()), 1.0e-6f);
  EXPECT_FLOAT_EQ(NormalizePfa(1.0e-12f), 1.0e-9f);
  EXPECT_FLOAT_EQ(NormalizePfa(1.0e-1f), 1.0e-2f);
  EXPECT_FLOAT_EQ(NormalizePfa(1.0e-6f), 1.0e-6f);
}

TEST(TimingRegimeModelTest, DynamicThresholdAndPdKeepMonotonicBehavior) {
  StatisticalDetectionParams params;
  params.pfa = 1.0e-6f;
  params.min_snr_db = 6.0f;
  params.pulse_count = 8U;
  params.integration_mode = IntegrationMode::kNonCoherent;
  params.threshold_scale = 1.0f;

  const float base_threshold_db = ComputeDynamicThresholdSnrDb(1.0e-12, params);
  params.threshold_scale = 1.5f;
  const float scaled_threshold_db = ComputeDynamicThresholdSnrDb(1.0e-12, params);
  EXPECT_GT(scaled_threshold_db, base_threshold_db);

  params.threshold_scale = 1.0f;
  params.pfa = 1.0e-8f;
  const float strict_pfa_threshold_db = ComputeDynamicThresholdSnrDb(1.0e-12, params);
  EXPECT_GT(strict_pfa_threshold_db, base_threshold_db);

  params.pfa = 1.0e-6f;
  const float pd_low =
      ComputeStatisticalDetectionProbability(base_threshold_db - 3.0f, base_threshold_db, params);
  const float pd_high =
      ComputeStatisticalDetectionProbability(base_threshold_db + 6.0f, base_threshold_db, params);
  EXPECT_LT(pd_low, pd_high);

  StatisticalDetectionParams coherent_params = params;
  coherent_params.pulse_count = 16U;
  coherent_params.integration_mode = IntegrationMode::kCoherent;
  StatisticalDetectionParams noncoherent_params = coherent_params;
  noncoherent_params.integration_mode = IntegrationMode::kNonCoherent;
  // 积分增益只计入一次：门限已按 N/√N 下调，概率函数本身不再乘增益。
  // 相干/非相干的 Pd 差异通过各自门限体现——同一 per-pulse SNR 下，相干门限
  // 更低（10·log10 N vs 5·log10 N 收益），因此相干 Pd 更高。
  const float coherent_threshold_db = ComputeDynamicThresholdSnrDb(1.0e-12, coherent_params);
  const float noncoherent_threshold_db =
      ComputeDynamicThresholdSnrDb(1.0e-12, noncoherent_params);
  EXPECT_LT(coherent_threshold_db, noncoherent_threshold_db);
  const float fixed_snr_db = noncoherent_threshold_db;
  const float pd_coherent = ComputeStatisticalDetectionProbability(
      fixed_snr_db, coherent_threshold_db, coherent_params);
  const float pd_noncoherent = ComputeStatisticalDetectionProbability(
      fixed_snr_db, noncoherent_threshold_db, noncoherent_params);
  EXPECT_GT(pd_coherent, pd_noncoherent);
  // 门限交点处 Pd = 1−exp(−1) ≈ 0.632（指数律），不再是双重计入下的 ≈1。
  EXPECT_NEAR(ComputeStatisticalDetectionProbability(base_threshold_db, base_threshold_db,
                                                     params),
              0.6321206f, 1.0e-4f);

  EXPECT_FLOAT_EQ(ComputeStatisticalDetectionProbability(std::numeric_limits<float>::quiet_NaN(),
                                                         base_threshold_db, params),
                  0.0f);
}

}  // namespace
}  // namespace timing
}  // namespace common
}  // namespace oneq
