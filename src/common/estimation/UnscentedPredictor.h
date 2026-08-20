/**
 * @file UnscentedPredictor.h
 * @brief 定义无迹（Unscented）Kalman 预测器（维度模板化）。
 *
 * 参考 Stone Soup UnscentedKalmanPredictor：均值与协方差均由 sigma 点经非线性转移
 * 函数传播后加权重建，不使用 Jacobian。与 EkfPredictor 共享 ITransitionModel 注入形态，
 * 但只消费 Function（实现 Jacobian 仍为接口要求，本预测器不解其值）。
 */

#ifndef COMMON_ESTIMATION_UNSCENTED_PREDICTOR_H_
#define COMMON_ESTIMATION_UNSCENTED_PREDICTOR_H_

#include "common/estimation/EkfFilter.h"
#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/estimation/UnscentedTransform.h"
#include "common/logging/ProjectLog.h"

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief 无迹预测器配置。
 */
struct UnscentedPredictorConfig {
  float noise_diff_coeff{1.0f};      /**< 过程噪声扩散系数 q（同 KalmanPredictorConfig）。 */
  UnscentedTransformConfig transform{}; /**< 无迹变换参数。 */
};

/**
 * @brief 无迹 Kalman 预测器。
 * @details 时间更新：sigma 点经 f(x,dt) 传播后按 W^m/W^c 重建均值与协方差，再加 Q：
 *          x̂ = Σ W^m_i·f(X_i)；P̂ = Σ W^c_i·(f(X_i)−x̂)(f(X_i)−x̂)ᵀ + Q。
 *          线性极限下与 KalmanPredictor 数值一致（线性函数对均值/协方差精确传播）。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度（预测器不直接使用，与更新器/状态对齐）。
 */
template <int kStateDim, int kMeasurementDim>
class UnscentedPredictor final : public IKalmanPredictor<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using TransitionModel = ITransitionModel<kStateDim>;
  using StateVector = typename GaussianStateT::StateVector;
  using ProcessNoiseCovariance = typename GaussianStateT::ProcessNoiseCovariance;
  using Transform = UnscentedTransform<kStateDim>;
  using PointSet = typename Transform::PointSet;

  /**
   * @brief 构造函数。
   * @param[in] model 转移模型（非拥有指针）。
   * @param[in] config 预测器配置。
   */
  UnscentedPredictor(const TransitionModel* model, UnscentedPredictorConfig config = {})
      : model_(model), config_(config) {}

  GaussianStateT Predict(const GaussianStateT& prior, float dt) const override {
    if (model_ == nullptr) {
      // 中译：转移模型未注入，本次预测被跳过（返回先验原样）。
      // 标识：注入校验——空模型指针属装配缺陷；fail-safe 返回先验，
      //       不解引用空指针。
      PROJECT_LOG_ERROR(
          "[UnscentedPredictor] Transition model is null; prediction is skipped (prior passed through).");
      return prior;
    }

    PointSet point_set;
    if (!Transform::GenerateSigmaPoints(prior.mean, prior.covariance, config_.transform,
                                        &point_set)) {
      // 中译：sigma 点生成失败（协方差 LLT 分解失败），本次预测被跳过。
      // 标识：数值保护——协方差非正定时 fail-safe 返回先验，防止数值发散。
      PROJECT_LOG_ERROR(
          "[UnscentedPredictor] Sigma point generation failed; prediction is skipped (prior passed "
          "through).");
      return prior;
    }

    /* sigma 点经非线性转移函数传播 */
    typename Transform::PointArray propagated{};
    for (std::size_t i = 0U; i < Transform::kCount; ++i) {
      propagated[i] = model_->Function(point_set.points[i], dt);
    }

    /* 加权重建均值与协方差 + 过程噪声（复用 KalmanPredictor 的 CV 模型噪声结构） */
    GaussianStateT predicted;
    predicted.mean =
        UnscentedWeightedMean<kStateDim, Transform::kCount>::Compute(propagated,
                                                                     point_set.mean_weights);
    const ProcessNoiseCovariance Q =
        KalmanPredictor<kStateDim, kMeasurementDim>::BuildProcessNoise(dt, config_.noise_diff_coeff);
    const ProcessNoiseCovariance propagated_covariance =
        UnscentedCrossCovariance<kStateDim, kStateDim, Transform::kCount>::Compute(
            propagated, predicted.mean, propagated, predicted.mean, point_set.covariance_weights);
    predicted.covariance = Transform::SymmetrizeAndFloor(propagated_covariance + Q);
    return predicted;
  }

 private:
  const TransitionModel* model_{nullptr}; /**< 转移模型 */
  UnscentedPredictorConfig config_{};     /**< 配置参数 */
};

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_UNSCENTED_PREDICTOR_H_
