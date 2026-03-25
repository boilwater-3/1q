/**
 * @file timing_regime_model_test.cpp
 * @brief 验证共享周期级时序体制与统计检测模型行为。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "common/timing/TimingRegimeModel.h"

namespace oneq {
namespace internal {
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
  const float pd_coherent = ComputeStatisticalDetectionProbability(
      base_threshold_db + 2.0f, base_threshold_db, coherent_params);
  const float pd_noncoherent = ComputeStatisticalDetectionProbability(
      base_threshold_db + 2.0f, base_threshold_db, noncoherent_params);
  EXPECT_GT(pd_coherent, pd_noncoherent);

  EXPECT_FLOAT_EQ(ComputeStatisticalDetectionProbability(std::numeric_limits<float>::quiet_NaN(),
                                                         base_threshold_db, params),
                  0.0f);
}

}  // namespace
}  // namespace timing
}  // namespace internal
}  // namespace oneq
