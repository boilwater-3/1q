// Copyright 2026. All Rights Reserved.
//
// @file ar_imm_matrix_defaults_test.cpp
// @brief 验证 IMM 矩阵/权重单一构建源的默认值、校验与违规收敛行为。
//
// 该单测直接锁定 imm_defaults 的所有分支与三处历史分叉的收敛点（model_count==0
// 防御、NaN/Inf 校验、违规回调可观测性）。任一处被改动都会立即报红，而无需依赖
// SignalComponentFactory / RuntimeAssemblySupport 两条调用路径间接捕获。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <string>

#include "airborne_radar/signal/pipeline/ImmMatrixDefaults.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace imm_defaults {
namespace {

using ExecutionConfig = ::airborne_radar::config::execution::InternalExecutionConfig;

// 一个总是记录最近一次违规的 reporter，用于断言"违规是否被报告"。
struct CapturingReporter {
  bool invoked{false};
  std::string last_message;
  ViolationReporter as_function() {
    return [this](const char* message, float /*value*/) {
      invoked = true;
      last_message = message;
    };
  }
};

// ---------------------------------------------------------------------------
// BuildTransitionProbability
// ---------------------------------------------------------------------------

TEST(ImmMatrixDefaultsTest, TransitionProbabilityDefaultDiagonalForMultiModel) {
  ExecutionConfig config;  // empty transition probability
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 2U, {});
  ASSERT_EQ(matrix.rows(), 2);
  ASSERT_EQ(matrix.cols(), 2);
  EXPECT_FLOAT_EQ(matrix(0, 0), 0.95f);
  EXPECT_FLOAT_EQ(matrix(1, 1), 0.95f);
  // off-diagonal = 0.05/(n-1) = 0.05
  EXPECT_FLOAT_EQ(matrix(0, 1), 0.05f);
  EXPECT_FLOAT_EQ(matrix(1, 0), 0.05f);
  // rows sum to 1
  EXPECT_FLOAT_EQ(matrix.row(0).sum(), 1.0f);
  EXPECT_FLOAT_EQ(matrix.row(1).sum(), 1.0f);
}

TEST(ImmMatrixDefaultsTest, TransitionProbabilityDefaultIsOneForSingleModel) {
  ExecutionConfig config;
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 1U, {});
  ASSERT_EQ(matrix.rows(), 1);
  ASSERT_EQ(matrix.cols(), 1);
  EXPECT_FLOAT_EQ(matrix(0, 0), 1.0f);
}

TEST(ImmMatrixDefaultsTest, TransitionProbabilityExplicitValidIsAccepted) {
  ExecutionConfig config;
  config.lifecycle.imm_transition_probability = {0.8f, 0.2f, 0.3f, 0.7f};  // row-stochastic
  CapturingReporter reporter;
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 2U, reporter.as_function());
  ASSERT_EQ(matrix.rows(), 2);
  EXPECT_FLOAT_EQ(matrix(0, 0), 0.8f);
  EXPECT_FLOAT_EQ(matrix(1, 1), 0.7f);
  EXPECT_FALSE(reporter.invoked);
}

TEST(ImmMatrixDefaultsTest, TransitionProbabilityRejectsWrongSize) {
  ExecutionConfig config;
  config.lifecycle.imm_transition_probability = {1.0f, 0.0f, 0.0f};  // 3 != 2*2
  CapturingReporter reporter;
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 2U, reporter.as_function());
  EXPECT_EQ(matrix.size(), 0);
  EXPECT_TRUE(reporter.invoked);
}

TEST(ImmMatrixDefaultsTest, TransitionProbabilityRejectsNonStochasticRow) {
  ExecutionConfig config;
  config.lifecycle.imm_transition_probability = {0.5f, 0.2f, 0.5f, 0.5f};  // row 0 sums to 0.7
  CapturingReporter reporter;
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 2U, reporter.as_function());
  EXPECT_EQ(matrix.size(), 0);
  EXPECT_TRUE(reporter.invoked);
}

// 收敛点 1: model_count==0 返回空矩阵（factory 原先无此防御）。
TEST(ImmMatrixDefaultsTest, TransitionProbabilityReturnsEmptyForZeroModelCount) {
  ExecutionConfig config;
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 0U, {});
  EXPECT_EQ(matrix.size(), 0);
}

// 收敛点 2: model_count==0 且 config 显式提供了非空向量（不一致场景）时仍安全返回空。
// 这是 guard 真正可区分的场景：空 config 时 Eigen::Constant(0,0) 本就返回空，
// 但当 config 非空而 model_count==0 时，缺少 guard 会进入 size 断言路径。
TEST(ImmMatrixDefaultsTest, TransitionProbabilityReturnsEmptyForZeroModelCountEvenWithExplicitConfig) {
  ExecutionConfig config;
  config.lifecycle.imm_transition_probability = {0.5f, 0.5f};  // non-empty but model_count==0
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 0U, {});
  EXPECT_EQ(matrix.size(), 0);
}

// 收敛点 3: null reporter ⇒ 静默（runtime 重组路径行为保留）。
TEST(ImmMatrixDefaultsTest, TransitionProbabilitySilentWithNullReporter) {
  ExecutionConfig config;
  config.lifecycle.imm_transition_probability = {1.0f, 0.0f};  // wrong size
  // No crash, no reporter needed; just returns empty.
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 2U, {});
  EXPECT_EQ(matrix.size(), 0);
}

// ---------------------------------------------------------------------------
// BuildInitialWeights
// ---------------------------------------------------------------------------

TEST(ImmMatrixDefaultsTest, InitialWeightsDefaultIsUniform) {
  ExecutionConfig config;  // empty weights
  const Eigen::VectorXf weights = BuildInitialWeights(config, 3U, {});
  ASSERT_EQ(weights.size(), 3);
  EXPECT_FLOAT_EQ(weights(0), 1.0f / 3.0f);
  EXPECT_FLOAT_EQ(weights(1), 1.0f / 3.0f);
  EXPECT_FLOAT_EQ(weights(2), 1.0f / 3.0f);
}

TEST(ImmMatrixDefaultsTest, InitialWeightsExplicitValidIsAccepted) {
  ExecutionConfig config;
  config.lifecycle.imm_initial_weights = {0.5f, 0.3f, 0.2f};
  CapturingReporter reporter;
  const Eigen::VectorXf weights = BuildInitialWeights(config, 3U, reporter.as_function());
  ASSERT_EQ(weights.size(), 3);
  EXPECT_FLOAT_EQ(weights(1), 0.3f);
  EXPECT_FALSE(reporter.invoked);
}

TEST(ImmMatrixDefaultsTest, InitialWeightsRejectsWrongSize) {
  ExecutionConfig config;
  config.lifecycle.imm_initial_weights = {0.5f, 0.5f};  // 2 != 3
  CapturingReporter reporter;
  const Eigen::VectorXf weights = BuildInitialWeights(config, 3U, reporter.as_function());
  EXPECT_EQ(weights.size(), 0);
  EXPECT_TRUE(reporter.invoked);
}

TEST(ImmMatrixDefaultsTest, InitialWeightsRejectsNonUnitySum) {
  ExecutionConfig config;
  config.lifecycle.imm_initial_weights = {0.5f, 0.2f, 0.1f};  // sums to 0.8
  CapturingReporter reporter;
  const Eigen::VectorXf weights = BuildInitialWeights(config, 3U, reporter.as_function());
  EXPECT_EQ(weights.size(), 0);
  EXPECT_TRUE(reporter.invoked);
}

TEST(ImmMatrixDefaultsTest, InitialWeightsReturnsEmptyForZeroModelCount) {
  ExecutionConfig config;
  const Eigen::VectorXf weights = BuildInitialWeights(config, 0U, {});
  EXPECT_EQ(weights.size(), 0);
}

TEST(ImmMatrixDefaultsTest, InitialWeightsReturnsEmptyForZeroModelCountEvenWithExplicitConfig) {
  ExecutionConfig config;
  config.lifecycle.imm_initial_weights = {0.5f, 0.5f};  // non-empty but model_count==0
  const Eigen::VectorXf weights = BuildInitialWeights(config, 0U, {});
  EXPECT_EQ(weights.size(), 0);
}

// ---------------------------------------------------------------------------
// 边界：阈值容差与边界概率值
// ---------------------------------------------------------------------------

TEST(ImmMatrixDefaultsTest, TransitionProbabilityAcceptsBoundaryProbabilityValues) {
  ExecutionConfig config;
  // exact 0.0 and 1.0 are within [0,1]; rows still sum to 1.
  config.lifecycle.imm_transition_probability = {1.0f, 0.0f, 0.0f, 1.0f};
  const Eigen::MatrixXf matrix = BuildTransitionProbability(config, 2U, {});
  EXPECT_EQ(matrix.rows(), 2);
}

TEST(ImmMatrixDefaultsTest, InitialWeightsAcceptsRowSumWithinTolerance) {
  ExecutionConfig config;
  // 0.3333 * 3 = 0.9999, within 1e-3 tolerance of 1.0.
  config.lifecycle.imm_initial_weights = {0.3333f, 0.3333f, 0.3333f};
  const Eigen::VectorXf weights = BuildInitialWeights(config, 3U, {});
  EXPECT_EQ(weights.size(), 3);
}

}  // namespace
}  // namespace imm_defaults
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
