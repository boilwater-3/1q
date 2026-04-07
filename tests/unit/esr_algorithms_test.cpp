/**
 * @file esr_algorithms_test.cpp
 * @brief 验证 ESR 首版核心算法组件行为。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "electronic_surveillance_radar/intercept/AngleErrorModel.h"
#include "electronic_surveillance_radar/intercept/BandClassifier.h"
#include "electronic_surveillance_radar/intercept/BoundarySearchSolver.h"
#include "electronic_surveillance_radar/intercept/InterceptGate.h"
#include "electronic_surveillance_radar/intercept/JammingAggregator.h"
#include "electronic_surveillance_radar/intercept/ScanPatternGenerator.h"

namespace electronic_surveillance_radar {
namespace intercept {
namespace {

TEST(EsrAlgorithmsTest, BandClassifierMapsTypicalBoundaryValues) {
  EXPECT_EQ(BandClassifier::Classify(-1.0), RadarBand::kInvalid);
  EXPECT_EQ(BandClassifier::Classify(std::numeric_limits<double>::infinity()), RadarBand::kInvalid);
  EXPECT_EQ(BandClassifier::Classify(0.10e9), RadarBand::kBelowP);
  EXPECT_EQ(BandClassifier::Classify(0.23e9), RadarBand::kP);
  EXPECT_EQ(BandClassifier::Classify(0.50e9), RadarBand::kP);
  EXPECT_EQ(BandClassifier::Classify(1.20e9), RadarBand::kL);
  EXPECT_EQ(BandClassifier::Classify(3.00e9), RadarBand::kS);
  EXPECT_EQ(BandClassifier::Classify(6.00e9), RadarBand::kC);
  EXPECT_EQ(BandClassifier::Classify(10.00e9), RadarBand::kX);
  EXPECT_EQ(BandClassifier::Classify(35.00e9), RadarBand::kKa);
}

TEST(EsrAlgorithmsTest, BandClassifierSupportsCustomBandTable) {
  BandClassifier::BandTable custom_table = BandClassifier::DefaultBandTable();
  custom_table[0] = {0.10, 1.00, RadarBand::kL};
  custom_table[1] = {1.00, 2.00, RadarBand::kS};
  custom_table[2] = {2.00, 4.00, RadarBand::kC};
  custom_table[3] = {4.00, 8.00, RadarBand::kX};
  custom_table[4] = {8.00, 12.00, RadarBand::kKu};
  custom_table[5] = {12.00, 18.00, RadarBand::kK};
  custom_table[6] = {18.00, 27.00, RadarBand::kKa};
  custom_table[7] = {27.00, 40.00, RadarBand::kU};
  custom_table[8] = {40.00, 60.00, RadarBand::kV};
  custom_table[9] = {60.00, 80.00, RadarBand::kW};
  custom_table[10] = {80.00, 100.00, RadarBand::kP};

  EXPECT_EQ(BandClassifier::Classify(0.20e9, custom_table), RadarBand::kL);
  EXPECT_EQ(BandClassifier::Classify(0.20e9), RadarBand::kBelowP);
}

TEST(EsrAlgorithmsTest, ScanPatternGeneratorRespectsStartPositionAndSequence) {
  ScanPatternConfig config;
  config.start_az_deg = -10.0f;
  config.end_az_deg = 10.0f;
  config.start_el_deg = -5.0f;
  config.end_el_deg = 5.0f;
  config.az_step_deg = 10.0f;
  config.el_step_deg = 5.0f;
  config.start_pos = 0;
  config.sequence = 0;

  const std::vector<BeamPointingDeg> pattern = ScanPatternGenerator::Generate(config);
  ASSERT_EQ(pattern.size(), 9U);
  EXPECT_NEAR(pattern.front().az_deg, -10.0f, 1.0e-6f);
  EXPECT_NEAR(pattern.front().el_deg, 5.0f, 1.0e-6f);
  EXPECT_NEAR(pattern[3].az_deg, 10.0f, 1.0e-6f);
  EXPECT_NEAR(pattern[3].el_deg, 0.0f, 1.0e-6f);
  EXPECT_NEAR(pattern.back().az_deg, 10.0f, 1.0e-6f);
  EXPECT_NEAR(pattern.back().el_deg, -5.0f, 1.0e-6f);
}

TEST(EsrAlgorithmsTest, InterceptGateEvaluatesJointConstraints) {
  InterceptGateInput gate_input;
  gate_input.line_of_sight = true;
  gate_input.target_az_deg = 4.0f;
  gate_input.target_el_deg = 2.0f;
  gate_input.beam_az_deg = 0.0f;
  gate_input.beam_el_deg = 0.0f;
  gate_input.beam_az_width_deg = 20.0f;
  gate_input.beam_el_width_deg = 20.0f;
  gate_input.receiver_lower_hz = 9.0e9;
  gate_input.receiver_upper_hz = 11.0e9;
  gate_input.signal_center_hz = 10.0e9;
  gate_input.signal_bandwidth_hz = 1.0e9;
  gate_input.range_m = 120.0f;
  gate_input.max_range_m = 300.0f;
  gate_input.dynamic_range_margin_db = 2.0f;
  gate_input.min_dynamic_range_margin_db = -3.0f;

  const InterceptGateDecision pass_decision = InterceptGate::Evaluate(gate_input);
  EXPECT_TRUE(pass_decision.passed);
  EXPECT_TRUE(pass_decision.frequency_covered);
  EXPECT_TRUE(pass_decision.in_beam);
  EXPECT_TRUE(pass_decision.in_range);
  EXPECT_TRUE(pass_decision.dynamic_range_ok);

  gate_input.dynamic_range_margin_db = -8.0f;
  const InterceptGateDecision fail_decision = InterceptGate::Evaluate(gate_input);
  EXPECT_FALSE(fail_decision.passed);
  EXPECT_FALSE(fail_decision.dynamic_range_ok);

  gate_input = InterceptGateInput();
  gate_input.line_of_sight = true;
  gate_input.target_az_deg = 4.0f;
  gate_input.target_el_deg = 2.0f;
  gate_input.beam_az_deg = 0.0f;
  gate_input.beam_el_deg = 0.0f;
  gate_input.beam_az_width_deg = 8.0f;
  gate_input.beam_el_width_deg = 6.0f;
  gate_input.receiver_lower_hz = 9.5e9;
  gate_input.receiver_upper_hz = 10.5e9;
  gate_input.signal_center_hz = 10.4e9;
  gate_input.signal_bandwidth_hz = 2.0e9;
  gate_input.range_m = 50.0f;
  gate_input.max_range_m = 100.0f;
  gate_input.dynamic_range_margin_db = 1.0f;
  gate_input.min_dynamic_range_margin_db = -3.0f;
  gate_input.min_frequency_overlap_ratio = 0.6f;
  gate_input.beam_guard_factor = 0.9f;
  const InterceptGateDecision strict_decision = InterceptGate::Evaluate(gate_input);
  EXPECT_FALSE(strict_decision.passed);
  EXPECT_FALSE(strict_decision.frequency_covered);
}

TEST(EsrAlgorithmsTest, InterceptGateAcceptsBoundaryValuesWithinEpsilon) {
  InterceptGateInput gate_input;
  gate_input.line_of_sight = true;
  gate_input.target_az_deg = 0.0f;
  gate_input.target_el_deg = 0.0f;
  gate_input.beam_az_deg = 0.0f;
  gate_input.beam_el_deg = 0.0f;
  gate_input.beam_az_width_deg = 0.0f;
  gate_input.beam_el_width_deg = 0.0f;
  gate_input.receiver_lower_hz = 9.5e9;
  gate_input.receiver_upper_hz = 10.5e9;
  gate_input.signal_center_hz = 10.0e9;
  gate_input.signal_bandwidth_hz = 2.0e9;
  gate_input.range_m = 100.0f + 4.0e-7f;
  gate_input.max_range_m = 100.0f;
  gate_input.dynamic_range_margin_db = -3.0f - 4.0e-7f;
  gate_input.min_dynamic_range_margin_db = -3.0f;
  gate_input.min_frequency_overlap_ratio = 0.5f + 4.0e-7f;
  gate_input.beam_guard_factor = 0.0f;

  const InterceptGateDecision decision = InterceptGate::Evaluate(gate_input);
  EXPECT_TRUE(decision.frequency_covered);
  EXPECT_TRUE(decision.in_beam);
  EXPECT_TRUE(decision.in_range);
  EXPECT_TRUE(decision.dynamic_range_ok);
  EXPECT_TRUE(decision.passed);
}

TEST(EsrAlgorithmsTest, JammingAggregatorSeparatesSuppressionChannel) {
  environment::EsrJammerSourceList sources;
  environment::EsrJammerSource source_a;
  source_a.technique = environment::EsrJammingTechnique::kNoiseSuppression;
  source_a.active = true;
  source_a.center_hz = 10.0e9;
  source_a.bandwidth_hz = 2.0e9;
  source_a.power_w = 10.0f;
  source_a.confidence = 1.0f;
  source_a.deception_risk = 0.9f;
  sources.push_back(source_a);

  environment::EsrJammerSource source_b;
  source_b.technique = environment::EsrJammingTechnique::kNoiseSuppression;
  source_b.active = true;
  source_b.center_hz = 11.0e9;
  source_b.bandwidth_hz = 4.0e9;
  source_b.power_w = 4.0f;
  source_b.confidence = 0.5f;
  sources.push_back(source_b);

  const JammingAggregateResult result = JammingAggregator::Aggregate(sources, 10.0e9, 2.0e9);

  EXPECT_NEAR(result.suppression_power_w, 11.0f, 1.0e-5f);
  EXPECT_NEAR(result.suppression_weighted_overlap_ratio, 0.9545454f, 1.0e-6f);
  EXPECT_NEAR(result.deception_risk, 0.0f, 1.0e-6f);
  EXPECT_EQ(result.suppression_source_count, 2U);
  EXPECT_EQ(result.deception_source_count, 0U);
  EXPECT_EQ(result.active_source_count, 2U);
}

TEST(EsrAlgorithmsTest, JammingAggregatorSeparatesDeceptionChannel) {
  environment::EsrJammerSource source;
  source.technique = environment::EsrJammingTechnique::kDeception;
  source.active = true;
  source.center_hz = 10.0e9;
  source.bandwidth_hz = 2.0e9;
  source.power_w = 20.0f;
  source.deception_risk = 0.8f;
  source.confidence = 1.0f;

  const JammingAggregateResult result = JammingAggregator::Aggregate({source}, 10.0e9, 2.0e9);

  EXPECT_NEAR(result.suppression_power_w, 0.0f, 1.0e-6f);
  EXPECT_NEAR(result.deception_risk, 0.8f, 1.0e-6f);
  EXPECT_NEAR(result.deception_weighted_overlap_ratio, 1.0f, 1.0e-6f);
  EXPECT_EQ(result.suppression_source_count, 0U);
  EXPECT_EQ(result.deception_source_count, 1U);
  EXPECT_EQ(result.active_source_count, 1U);
}

TEST(EsrAlgorithmsTest, JammingAggregatorUnknownTechniqueFallsBackToMixed) {
  environment::EsrJammerSource source_mixed;
  source_mixed.technique = environment::EsrJammingTechnique::kMixed;
  source_mixed.active = true;
  source_mixed.center_hz = 12.0e9;
  source_mixed.bandwidth_hz = 4.0e9;
  source_mixed.power_w = 8.0f;
  source_mixed.deception_risk = 0.5f;
  source_mixed.confidence = 1.0f;

  environment::EsrJammerSource source_unknown;
  source_unknown.technique = environment::EsrJammingTechnique::kUnknown;
  source_unknown.active = true;
  source_unknown.center_hz = 10.0e9;
  source_unknown.bandwidth_hz = 2.0e9;
  source_unknown.power_w = 6.0f;
  source_unknown.deception_risk = 0.2f;
  source_unknown.confidence = 0.5f;

  const JammingAggregateResult result =
      JammingAggregator::Aggregate({source_mixed, source_unknown}, 10.0e9, 2.0e9);

  EXPECT_NEAR(result.suppression_power_w, 5.0f, 1.0e-5f);
  EXPECT_NEAR(result.suppression_weighted_overlap_ratio, 0.7f, 1.0e-5f);
  EXPECT_NEAR(result.deception_risk, 0.2125f, 1.0e-6f);
  EXPECT_NEAR(result.deception_weighted_overlap_ratio, 0.5833333f, 1.0e-5f);
  EXPECT_EQ(result.suppression_source_count, 2U);
  EXPECT_EQ(result.deception_source_count, 2U);
  EXPECT_EQ(result.active_source_count, 2U);
}

TEST(EsrAlgorithmsTest, AngleErrorModelSamplingIsStableWithFixedSeed) {
  AngleErrorModelConfig config;
  config.coefficient = 0.5f;
  config.min_std_deg = 0.01f;
  config.max_std_deg = 20.0f;

  std::mt19937 rng_a(42U);
  std::mt19937 rng_b(42U);
  for (int i = 0; i < 10; ++i) {
    const float sample_a = AngleErrorModel::SampleErrorDeg(18.0f, 8.0f, &rng_a, config);
    const float sample_b = AngleErrorModel::SampleErrorDeg(18.0f, 8.0f, &rng_b, config);
    EXPECT_FLOAT_EQ(sample_a, sample_b);
  }

  const float low_snr_std = AngleErrorModel::ComputeStdDevDeg(-3.0f, 8.0f, config);
  const float high_snr_std = AngleErrorModel::ComputeStdDevDeg(20.0f, 8.0f, config);
  EXPECT_GT(low_snr_std, high_snr_std);
}

TEST(EsrAlgorithmsTest, AngleErrorModelClampsExtremeSnrDbInput) {
  AngleErrorModelConfig config;
  config.coefficient = 0.5f;
  config.min_std_deg = 0.01f;
  config.max_std_deg = 20.0f;

  const float std_at_max_bound = AngleErrorModel::ComputeStdDevDeg(100.0f, 8.0f, config);
  const float std_at_far_high = AngleErrorModel::ComputeStdDevDeg(1000.0f, 8.0f, config);
  const float std_at_min_bound = AngleErrorModel::ComputeStdDevDeg(-100.0f, 8.0f, config);
  const float std_at_far_low = AngleErrorModel::ComputeStdDevDeg(-1000.0f, 8.0f, config);

  EXPECT_TRUE(std::isfinite(std_at_far_high));
  EXPECT_TRUE(std::isfinite(std_at_far_low));
  EXPECT_NEAR(std_at_far_high, std_at_max_bound, 1.0e-6f);
  EXPECT_NEAR(std_at_far_low, std_at_min_bound, 1.0e-6f);
}

TEST(EsrAlgorithmsTest, BoundarySearchSolverConvergesForMonotonicPredicate) {
  const float expected_boundary = 1234.0f;
  const BoundarySearchResult result = BoundarySearchSolver::Solve(
      0.0f, 5000.0f, 1.0f, 40, [&](float range_m) { return range_m <= expected_boundary; });

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.iterations, 0);
  EXPECT_NEAR(result.boundary_range_m, expected_boundary, 1.5f);

  const BoundarySearchResult all_true =
      BoundarySearchSolver::Solve(0.0f, 100.0f, 0.1f, 20, [](float) { return true; });
  EXPECT_TRUE(all_true.converged);
  EXPECT_NEAR(all_true.boundary_range_m, 100.0f, 1.0e-6f);

  // 谓词全假：截获条件在全范围内均不满足，应返回 converged=false 以告知调用方截获不可行。
  const BoundarySearchResult all_false =
      BoundarySearchSolver::Solve(0.0f, 100.0f, 0.1f, 20, [](float) { return false; });
  EXPECT_FALSE(all_false.converged);
  EXPECT_NEAR(all_false.boundary_range_m, 0.0f, 1.0e-6f);
}

}  // namespace
}  // namespace intercept
}  // namespace electronic_surveillance_radar
