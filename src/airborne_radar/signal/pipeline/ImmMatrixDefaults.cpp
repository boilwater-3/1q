/**
 * @file ImmMatrixDefaults.cpp
 * @brief IMM 矩阵/权重单一构建源（见 ImmMatrixDefaults.h）。
 */

#include "airborne_radar/signal/pipeline/ImmMatrixDefaults.h"

#include <cmath>

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace imm_defaults {

namespace {

// 行和/权和 ≈1 的容差（沿用历史值）。
constexpr float kUnityTolerance = 1.0e-3f;

bool IsProbabilityValue(float value) {
  return std::isfinite(value) != 0 && value >= 0.0f && value <= 1.0f;
}

bool IsNearOne(float value) { return std::fabs(value - 1.0f) <= kUnityTolerance; }

void Report(const ViolationReporter& report_violation, const char* message, float value) {
  if (report_violation) {
    report_violation(message, value);
  }
}

}  // namespace

Eigen::MatrixXf BuildTransitionProbability(const ExecutionConfig& config, std::size_t model_count,
                                           const ViolationReporter& report_violation) {
  if (model_count == 0U) {
    return Eigen::MatrixXf();
  }
  if (config.lifecycle.imm_transition_probability.empty()) {
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
        static_cast<Eigen::Index>(model_count), static_cast<Eigen::Index>(model_count),
        model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U) : 1.0f);
    matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
    return matrix;
  }
  if (config.lifecycle.imm_transition_probability.size() != model_count * model_count) {
    Report(report_violation, "lifecycle.imm_transition_probability size must equal model_count*model_count",
           static_cast<float>(config.lifecycle.imm_transition_probability.size()));
    return Eigen::MatrixXf();
  }
  Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                         static_cast<Eigen::Index>(model_count));
  for (std::size_t r = 0; r < model_count; ++r) {
    float row_sum = 0.0f;
    for (std::size_t c = 0; c < model_count; ++c) {
      const float value = config.lifecycle.imm_transition_probability[r * model_count + c];
      if (!IsProbabilityValue(value)) {
        Report(report_violation, "IMM transition probability must be finite and in [0,1]", value);
        return Eigen::MatrixXf();
      }
      matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = value;
      row_sum += value;
    }
    if (!IsNearOne(row_sum)) {
      Report(report_violation, "each IMM transition matrix row must sum to 1", row_sum);
      return Eigen::MatrixXf();
    }
  }
  return matrix;
}

Eigen::VectorXf BuildInitialWeights(const ExecutionConfig& config, std::size_t model_count,
                                    const ViolationReporter& report_violation) {
  if (model_count == 0U) {
    return Eigen::VectorXf();
  }
  if (config.lifecycle.imm_initial_weights.empty()) {
    return Eigen::VectorXf::Constant(static_cast<Eigen::Index>(model_count),
                                     1.0f / static_cast<float>(model_count));
  }
  if (config.lifecycle.imm_initial_weights.size() != model_count) {
    Report(report_violation, "lifecycle.imm_initial_weights size must equal model_count",
           static_cast<float>(config.lifecycle.imm_initial_weights.size()));
    return Eigen::VectorXf();
  }
  Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
  float sum = 0.0f;
  for (std::size_t i = 0; i < model_count; ++i) {
    const float value = config.lifecycle.imm_initial_weights[i];
    if (!IsProbabilityValue(value)) {
      Report(report_violation, "IMM initial weight must be finite and in [0,1]", value);
      return Eigen::VectorXf();
    }
    weights(static_cast<Eigen::Index>(i)) = value;
    sum += value;
  }
  if (!IsNearOne(sum)) {
    Report(report_violation, "IMM initial weights must sum to 1", sum);
    return Eigen::VectorXf();
  }
  return weights;
}

}  // namespace imm_defaults
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
