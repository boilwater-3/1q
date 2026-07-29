/**
 * @file ar_deception_measurement_generator_test.cpp
 * @brief 验证 DeceptionMeasurementGenerator 从欺骗干扰观测合成假目标量测的正确性。
 *
 * 覆盖 P1 修复的核心断言：假目标鉴别处理的应是合成假目标量测，而非真实场景目标。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/environment/EnvironmentTypes.h"
#include "airborne_radar/signal/pipeline/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace {

using ExecutionConfig = config::execution::InternalExecutionConfig;

// 构造一个 kLikelyFalseTarget 干扰观测，携带局部系方位/视距/径向速度与相干发射数。
session::ArInterferenceObservation MakeFalseTargetObservation(std::uint64_t observation_id,
                                                              std::uint32_t coherent_count) {
  session::ArInterferenceObservation obs;
  obs.observation_id = observation_id;
  obs.deception_class = session::DeceptionClass::kLikelyFalseTarget;
  obs.coherent_emission_count = coherent_count;
  obs.estimated_bearing_azimuth_local_deg = 45.0;
  obs.estimated_bearing_elevation_local_deg = 10.0;
  obs.estimated_slant_range_m = 20000.0;
  obs.estimated_range_rate_mps = -300.0;  // 接近视距，负径向速度
  obs.bearing_standard_deviation_deg = 2.0;
  obs.estimated_waveform_kind = oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain;
  return obs;
}

// 构造最小周期上下文：空场景目标列表 + 指定干扰观测。
CycleExecutionContext MakeContext(const session::ArInterferenceObservationList& observations,
                                  const session::ArSceneTargetList& input) {
  static const session::EnvironmentSnapshot kEmptyEnvironment;
  ExecutionConfig config;  // 默认；flag 由调用方在 pass 前设置
  config.enable_anti_false_target_discrimination = true;
  return CycleExecutionContext(input, kEmptyEnvironment, /*cycle_index=*/1U, /*batch_id=*/1U,
                              config, /*platform_altitude_m=*/0.0f, /*rf_v2=*/nullptr,
                              &observations);
}

TEST(DeceptionMeasurementGeneratorTest, SynthesizesOneMeasurementPerCoherentEmission) {
  session::ArInterferenceObservationList observations = {MakeFalseTargetObservation(1U, 3U)};
  session::ArSceneTargetList empty_input;
  CycleExecutionContext context = MakeContext(observations, empty_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(empty_input, scratch);
  // 预置一个真实量测，验证合成量测是追加而非覆盖。
  scratch.track_measurements.push_back(tracking::TrackMeasurement{});

  InjectDeceptionMeasurementsPass(context, context.runtime_config, scratch);

  // 1 个真实量测 + 3 个合成假目标量测。
  ASSERT_EQ(scratch.track_measurements.size(), 4U);
  for (std::size_t i = 1; i < scratch.track_measurements.size(); ++i) {
    const auto& m = scratch.track_measurements[i];
    EXPECT_TRUE(m.raw_measurement.classified_as_false_target);
    EXPECT_EQ(m.raw_measurement.source_index,
              static_cast<std::size_t>(-1));  // sentinel，不索引 per-target 数组
    EXPECT_EQ(m.raw_measurement.target_name, "deception");
    EXPECT_EQ(m.raw_measurement.external_target_id, 0U);
  }
}

TEST(DeceptionMeasurementGeneratorTest, PositionFromLocalBearingAndSlantRange) {
  session::ArInterferenceObservationList observations = {MakeFalseTargetObservation(1U, 1U)};
  session::ArSceneTargetList empty_input;
  CycleExecutionContext context = MakeContext(observations, empty_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(empty_input, scratch);
  InjectDeceptionMeasurementsPass(context, context.runtime_config, scratch);

  ASSERT_EQ(scratch.track_measurements.size(), 1U);
  const auto& m = scratch.track_measurements.front();
  // 局部系方位 45°、俯仰 10°、视距 20000m 的笛卡尔位置（i=0 不抖动）。
  const float az = 45.0f * static_cast<float>(M_PI) / 180.0f;
  const float el = 10.0f * static_cast<float>(M_PI) / 180.0f;
  const float cos_el = std::cos(el);
  const Eigen::Vector3f expected(20000.0f * cos_el * std::cos(az),
                                 20000.0f * cos_el * std::sin(az),
                                 20000.0f * std::sin(el));
  EXPECT_TRUE(m.raw_measurement.position.isApprox(expected, 1.0f));
  // 距离应接近视距。
  EXPECT_NEAR(m.raw_measurement.position.norm(), 20000.0f, 1.0f);
}

TEST(DeceptionMeasurementGeneratorTest, AssociationKeyStableAcrossCyclesForSameObservation) {
  session::ArInterferenceObservationList observations = {MakeFalseTargetObservation(1U, 1U)};
  session::ArSceneTargetList empty_input;
  CycleExecutionContext context_a = MakeContext(observations, empty_input);
  CycleExecutionContext context_b = MakeContext(observations, empty_input);

  CycleExecutionScratch scratch_a;
  ResetCycleExecutionScratch(empty_input, scratch_a);
  InjectDeceptionMeasurementsPass(context_a, context_a.runtime_config, scratch_a);

  CycleExecutionScratch scratch_b;
  ResetCycleExecutionScratch(empty_input, scratch_b);
  InjectDeceptionMeasurementsPass(context_b, context_b.runtime_config, scratch_b);

  ASSERT_EQ(scratch_a.track_measurements.size(), 1U);
  ASSERT_EQ(scratch_b.track_measurements.size(), 1U);
  // 同 observation_id 的假目标跨周期映射到同一 association_key，使 lifecycle 聚到同一假航迹。
  EXPECT_EQ(scratch_a.track_measurements.front().raw_measurement.association_key,
            scratch_b.track_measurements.front().raw_measurement.association_key);
  // key 应落入 deception key 段（高位标志位置 1）。
  EXPECT_NE(scratch_a.track_measurements.front().raw_measurement.association_key & 0x8000'0000'0000'0000ULL,
            0ULL);
}

TEST(DeceptionMeasurementGeneratorTest, SkipsWhenDiscriminationDisabled) {
  session::ArInterferenceObservationList observations = {MakeFalseTargetObservation(1U, 3U)};
  session::ArSceneTargetList empty_input;
  static const session::EnvironmentSnapshot kEmptyEnvironment;
  ExecutionConfig config;  // enable_anti_false_target_discrimination 默认 false
  CycleExecutionContext context(empty_input, kEmptyEnvironment, 1U, 1U, config, 0.0f, nullptr,
                                &observations);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(empty_input, scratch);
  InjectDeceptionMeasurementsPass(context, context.runtime_config, scratch);

  EXPECT_TRUE(scratch.track_measurements.empty());
}

TEST(DeceptionMeasurementGeneratorTest, SkipsNonFalseTargetObservations) {
  session::ArInterferenceObservation obs = MakeFalseTargetObservation(1U, 3U);
  obs.deception_class = session::DeceptionClass::kNone;  // 非假目标
  session::ArInterferenceObservationList observations = {obs};
  session::ArSceneTargetList empty_input;
  CycleExecutionContext context = MakeContext(observations, empty_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(empty_input, scratch);
  InjectDeceptionMeasurementsPass(context, context.runtime_config, scratch);

  EXPECT_TRUE(scratch.track_measurements.empty());
}

}  // namespace
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
