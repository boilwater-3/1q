/**
 * @file ar_deception_measurement_generator_test.cpp
 * @brief 验证 DeceptionMeasurementGenerator 从欺骗候选量测合成假目标量测的正确性。
 *
 * 覆盖 P1 修复的核心断言：假目标鉴别处理的应是合成假目标量测，而非真实场景目标。
 * 候选量测由 resolver 生成（带物理 provenance），关联键由关联引擎分配。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/config/InternalExecutionConfig.h"
#include "airborne_radar/environment/EnvironmentTypes.h"
#include "airborne_radar/signal/detection/ArDeceptionMeasurementCandidate.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/DeceptionMeasurementGenerator.h"
#include "airborne_radar/signal/pipeline/SignalCycleInput.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace {

using ExecutionConfig = config::execution::InternalExecutionConfig;

// 构造带物理 provenance 的欺骗候选量测。
detection::ArDeceptionMeasurementCandidate MakeCandidate(std::uint64_t obs_id,
                                                         std::uint64_t emission_id = 1U) {
  detection::ArDeceptionMeasurementCandidate c;
  c.source_observation_id = obs_id;
  c.source_emission_identity.emission_id = emission_id;
  c.estimated_first_pulse_delay_s = 0.0;
  c.estimated_carrier_offset_hz = 0.0;
  c.apparent_slant_range_m = 20000.0;
  c.apparent_range_rate_mps = -300.0;
  c.jammer_to_noise_db = 30.0;
  c.used_local_bearings = true;
  // 方位 45°、俯仰 10°、视距 20000m 的笛卡尔位置。
  const double az_rad = 45.0 * M_PI / 180.0;
  const double el_rad = 10.0 * M_PI / 180.0;
  const double cos_el = std::cos(el_rad);
  const double range_m = 20000.0;
  c.position = Eigen::Vector3f(
      static_cast<float>(range_m * cos_el * std::cos(az_rad)),
      static_cast<float>(range_m * cos_el * std::sin(az_rad)),
      static_cast<float>(range_m * std::sin(el_rad)));
  c.velocity = Eigen::Vector3f(
      static_cast<float>(-300.0 * cos_el * std::cos(az_rad)),
      static_cast<float>(-300.0 * cos_el * std::sin(az_rad)),
      static_cast<float>(-300.0 * std::sin(el_rad)));
  c.measurement_covariance = Eigen::Matrix3f(static_cast<Eigen::Matrix3f>(
      Eigen::DiagonalMatrix<float, 3>(1e4f, 1e4f, 1e4f)));
  return c;
}

// 构造最小周期上下文：空场景目标列表 + 指定候选量测与关联键。
// annotations 由调用方拥有，生命周期覆盖返回的 CycleExecutionContext。
CycleExecutionContext MakeContext(const detection::ArDeceptionMeasurementCandidateList& candidates,
                                  const std::vector<std::uint64_t>& keys,
                                  SignalCycleInput& cycle_input) {
  static const session::EnvironmentSnapshot kEmptyEnvironment;
  ExecutionConfig config;
  config.enable_anti_false_target_discrimination = true;
  cycle_input.deception_measurement_candidates = candidates;
  return CycleExecutionContext(cycle_input, kEmptyEnvironment,
                               /*cycle_index=*/1U, /*batch_id=*/1U, config,
                               /*platform_altitude_m=*/0.0f);
}

TEST(DeceptionMeasurementGeneratorTest, SynthesizesOneMeasurementPerCandidate) {
  detection::ArDeceptionMeasurementCandidateList candidates = {MakeCandidate(1U)};
  std::vector<std::uint64_t> keys = {1001U};
  SignalCycleInput cycle_input;
  CycleExecutionContext context = MakeContext(candidates, keys, cycle_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  // 预置一个真实量测，验证合成量测是追加而非覆盖。
  scratch.deception_candidate_keys = keys;
  scratch.track_measurements.push_back(tracking::TrackMeasurement{});

  InjectDeceptionMeasurementsPass(context, scratch);

  // 1 个真实量测 + 1 个合成假目标量测。
  ASSERT_EQ(scratch.track_measurements.size(), 2U);
  const auto& m = scratch.track_measurements.back();
  EXPECT_TRUE(m.raw_measurement.classified_as_false_target);
  EXPECT_EQ(m.raw_measurement.association_key, 1001U);
  EXPECT_EQ(m.raw_measurement.source_index, static_cast<std::size_t>(-1));
  EXPECT_EQ(m.raw_measurement.target_name, "deception");
}

TEST(DeceptionMeasurementGeneratorTest, PositionPreservedFromCandidate) {
  detection::ArDeceptionMeasurementCandidateList candidates = {MakeCandidate(1U)};
  std::vector<std::uint64_t> keys = {1001U};
  SignalCycleInput cycle_input;
  CycleExecutionContext context = MakeContext(candidates, keys, cycle_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  scratch.deception_candidate_keys = keys;
  InjectDeceptionMeasurementsPass(context, scratch);

  ASSERT_FALSE(scratch.track_measurements.empty());
  const auto& m = scratch.track_measurements.front();
  // 候选的 position 应直接传递给量测（不重新计算）。
  EXPECT_TRUE(m.raw_measurement.position.isApprox(candidates[0].position, 1e-6f));
  EXPECT_NEAR(m.raw_measurement.position.norm(), 20000.0f, 1.0f);
}

TEST(DeceptionMeasurementGeneratorTest, AssociationKeyFromEngine) {
  detection::ArDeceptionMeasurementCandidateList candidates = {MakeCandidate(1U), MakeCandidate(2U)};
  std::vector<std::uint64_t> keys = {1001U, 1002U};
  SignalCycleInput cycle_input;
  CycleExecutionContext context = MakeContext(candidates, keys, cycle_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  scratch.deception_candidate_keys = keys;
  InjectDeceptionMeasurementsPass(context, scratch);

  ASSERT_EQ(scratch.track_measurements.size(), 2U);
  // key 来自关联引擎，非预烘焙 seed。
  EXPECT_EQ(scratch.track_measurements[0].raw_measurement.association_key, 1001U);
  EXPECT_EQ(scratch.track_measurements[1].raw_measurement.association_key, 1002U);
}

TEST(DeceptionMeasurementGeneratorTest, GeneratesFalseTargetMeasurementsRegardlessOfSwitch) {
  // 反制开关不得反向控制攻击现象：candidate 独立于开关生成。
  detection::ArDeceptionMeasurementCandidateList candidates = {MakeCandidate(1U)};
  std::vector<std::uint64_t> keys = {1001U};
  ExecutionConfig config;
  SignalCycleInput cycle_input;
  cycle_input.deception_measurement_candidates = candidates;
  static const session::EnvironmentSnapshot kEmptyEnvironment;
  CycleExecutionContext context(cycle_input, kEmptyEnvironment, 1U, 1U, config, 0.0f);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  scratch.deception_candidate_keys = keys;
  InjectDeceptionMeasurementsPass(context, scratch);

  ASSERT_EQ(scratch.track_measurements.size(), 1U);
  EXPECT_TRUE(scratch.track_measurements.front().raw_measurement.classified_as_false_target);
}

TEST(DeceptionMeasurementGeneratorTest, SkipsUnassociatedCandidates) {
  // key=0 的 candidate 应被跳过（未关联）。
  detection::ArDeceptionMeasurementCandidateList candidates = {MakeCandidate(1U), MakeCandidate(2U)};
  std::vector<std::uint64_t> keys = {1001U, 0U};  // 第二个未关联
  SignalCycleInput cycle_input;
  CycleExecutionContext context = MakeContext(candidates, keys, cycle_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  scratch.deception_candidate_keys = keys;
  InjectDeceptionMeasurementsPass(context, scratch);

  ASSERT_EQ(scratch.track_measurements.size(), 1U);
  EXPECT_EQ(scratch.track_measurements.front().raw_measurement.association_key, 1001U);
}

TEST(DeceptionMeasurementGeneratorTest, SkipsZeroPositionCandidate) {
  // position 为零向量的 candidate 应被跳过。
  detection::ArDeceptionMeasurementCandidate candidate;
  candidate.position = Eigen::Vector3f::Zero();
  candidate.velocity = Eigen::Vector3f::Zero();
  candidate.measurement_covariance = Eigen::Matrix3f::Zero();
  detection::ArDeceptionMeasurementCandidateList candidates = {candidate};
  std::vector<std::uint64_t> keys = {1001U};
  SignalCycleInput cycle_input;
  CycleExecutionContext context = MakeContext(candidates, keys, cycle_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  scratch.deception_candidate_keys = keys;
  InjectDeceptionMeasurementsPass(context, scratch);

  EXPECT_TRUE(scratch.track_measurements.empty());
}

TEST(DeceptionMeasurementGeneratorTest, CandidateCountMatchesInputCount) {
  // 每个有效 candidate 且 key != 0 生成一个量测。
  detection::ArDeceptionMeasurementCandidateList candidates = {
      MakeCandidate(1U), MakeCandidate(2U), MakeCandidate(3U)};
  std::vector<std::uint64_t> keys = {1001U, 1002U, 1003U};
  SignalCycleInput cycle_input;
  CycleExecutionContext context = MakeContext(candidates, keys, cycle_input);

  CycleExecutionScratch scratch;
  ResetCycleExecutionScratch(session::ArSceneTargetList{}, scratch);
  scratch.deception_candidate_keys = keys;
  InjectDeceptionMeasurementsPass(context, scratch);

  ASSERT_EQ(scratch.track_measurements.size(), 3U);
  for (const auto& m : scratch.track_measurements) {
    EXPECT_TRUE(m.raw_measurement.classified_as_false_target);
  }
}

}  // namespace
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
