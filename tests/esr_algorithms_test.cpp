/**
 * @file esr_algorithms_test.cpp
 * @brief 验证 ESR 首版核心算法组件行为。
 */

#include <gtest/gtest.h>

#include <cmath>
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
  EXPECT_EQ(BandClassifier::Classify(0.10e9), RadarBand::kBelowP);
  EXPECT_EQ(BandClassifier::Classify(0.50e9), RadarBand::kP);
  EXPECT_EQ(BandClassifier::Classify(1.20e9), RadarBand::kL);
  EXPECT_EQ(BandClassifier::Classify(3.00e9), RadarBand::kS);
  EXPECT_EQ(BandClassifier::Classify(6.00e9), RadarBand::kC);
  EXPECT_EQ(BandClassifier::Classify(10.00e9), RadarBand::kX);
  EXPECT_EQ(BandClassifier::Classify(35.00e9), RadarBand::kKa);
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

  const std::vector<BeamPointingDeg> pattern =
      ScanPatternGenerator::Generate(config);
  ASSERT_EQ(pattern.size(), 9U);
  EXPECT_NEAR(pattern.front().az_deg, -10.0f, 1.0e-6f);
  EXPECT_NEAR(pattern.front().el_deg, 5.0f, 1.0e-6f);
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
}

TEST(EsrAlgorithmsTest, JammingAggregatorAccumulatesOverlappingPower) {
  environment::EsrJammerSourceList sources;
  environment::EsrJammerSource source_a;
  source_a.active = true;
  source_a.center_hz = 10.0e9;
  source_a.bandwidth_hz = 2.0e9;
  source_a.power_w = 10.0f;
  source_a.confidence = 1.0f;
  source_a.deception_risk = 0.2f;
  sources.push_back(source_a);

  environment::EsrJammerSource source_b;
  source_b.active = true;
  source_b.center_hz = 11.0e9;
  source_b.bandwidth_hz = 4.0e9;
  source_b.power_w = 4.0f;
  source_b.confidence = 0.5f;
  source_b.deception_risk = 0.8f;
  sources.push_back(source_b);

  environment::EsrJammerSource source_c;
  source_c.active = true;
  source_c.center_hz = 20.0e9;
  source_c.bandwidth_hz = 1.0e9;
  source_c.power_w = 100.0f;
  source_c.confidence = 1.0f;
  sources.push_back(source_c);

  const JammingAggregateResult result =
      JammingAggregator::Aggregate(sources, 10.0e9, 2.0e9);

  EXPECT_NEAR(result.jammer_power_w, 11.0f, 1.0e-5f);
  EXPECT_NEAR(result.weighted_overlap_ratio, 0.8333333f, 1.0e-5f);
  EXPECT_NEAR(result.deception_risk, 0.8f, 1.0e-6f);
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
    const float sample_a =
        AngleErrorModel::SampleErrorDeg(18.0f, 8.0f, &rng_a, config);
    const float sample_b =
        AngleErrorModel::SampleErrorDeg(18.0f, 8.0f, &rng_b, config);
    EXPECT_FLOAT_EQ(sample_a, sample_b);
  }
}

TEST(EsrAlgorithmsTest, BoundarySearchSolverConvergesForMonotonicPredicate) {
  const float expected_boundary = 1234.0f;
  const BoundarySearchResult result = BoundarySearchSolver::Solve(
      0.0f, 5000.0f, 1.0f, 40, [&](float range_m) {
        return range_m <= expected_boundary;
      });

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.iterations, 0);
  EXPECT_NEAR(result.boundary_range_m, expected_boundary, 1.5f);
}

}  // namespace
}  // namespace intercept
}  // namespace electronic_surveillance_radar
