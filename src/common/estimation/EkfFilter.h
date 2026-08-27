/**
 * @file EkfFilter.h
 * @brief 定义扩展 Kalman 滤波器（EKF）的预测器和更新器（维度模板化）。
 *
 * 本头从 airborne_radar 的 EkfFilter 迁移并模板化。EKF 通过 ITransitionModel / IMeasurementModel
 * 注入非线性函数及其 Jacobian，状态/量测维度由模板参数表达。
 */

#ifndef COMMON_ESTIMATION_EKF_FILTER_H_
#define COMMON_ESTIMATION_EKF_FILTER_H_

#include <Eigen/Cholesky>

#include "common/estimation/GaussianState.h"
#include "common/estimation/IKalmanPredictor.h"
#include "common/estimation/IKalmanUpdater.h"
#include "common/estimation/KalmanPredictor.h"
#include "common/logging/ProjectLog.h"

namespace oneq {
namespace common {
namespace estimation {

/**
 * @brief 非线性转移模型抽象接口。
 * @details 对应 Stone Soup 的 TransitionModel + jacobian() 方法。
 * @tparam kStateDim 状态空间维度。
 */
template <int kStateDim>
class ITransitionModel {
 public:
  using StateVector = Eigen::Matrix<float, kStateDim, 1>;
  using TransitionMatrix = Eigen::Matrix<float, kStateDim, kStateDim>;

  virtual ~ITransitionModel() = default;
  /**
   * @brief 非线性状态转移函数 f(x, dt)。
   * @param[in] state 当前状态向量。
   * @param[in] dt 时间步长。
   * @return 预测状态向量。
   */
  virtual StateVector Function(const StateVector& state, float dt) const = 0;
  /**
   * @brief 转移 Jacobian F(x, dt) = ∂f/∂x。
   * @param[in] state 线性化点的状态向量。
   * @param[in] dt 时间步长。
   * @return Jacobian 矩阵。
   */
  virtual TransitionMatrix Jacobian(const StateVector& state, float dt) const = 0;
};

/**
 * @brief 非线性量测模型抽象接口。
 * @details 对应 Stone Soup 的 MeasurementModel + jacobian() 方法。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class IMeasurementModel {
 public:
  using StateVector = Eigen::Matrix<float, kStateDim, 1>;
  using MeasurementVector = Eigen::Matrix<float, kMeasurementDim, 1>;
  using MeasurementMatrix = Eigen::Matrix<float, kMeasurementDim, kStateDim>;

  virtual ~IMeasurementModel() = default;
  /**
   * @brief 非线性量测函数 h(x)。
   * @param[in] state 状态向量。
   * @return 量测向量。
   */
  virtual MeasurementVector Function(const StateVector& state) const = 0;
  /**
   * @brief 量测 Jacobian H(x) = ∂h/∂x。
   * @param[in] state 线性化点的状态向量。
   * @return Jacobian 矩阵。
   */
  virtual MeasurementMatrix Jacobian(const StateVector& state) const = 0;
};

/**
 * @brief 线性恒速转移模型（默认实现）。
 * @details f(x,dt) = F·x，Jacobian = F（恒速矩阵）。仅在 kStateDim == 6 时提供 CV 结构。
 * @tparam kStateDim 状态空间维度。
 */
template <int kStateDim>
class LinearCvTransitionModel final : public ITransitionModel<kStateDim> {
 public:
  using StateVector = typename ITransitionModel<kStateDim>::StateVector;
  using TransitionMatrix = typename ITransitionModel<kStateDim>::TransitionMatrix;

  StateVector Function(const StateVector& state, float dt) const override {
    return KalmanPredictor<kStateDim, 1>::BuildTransitionMatrix(dt) * state;
  }
  TransitionMatrix Jacobian(const StateVector& /*state*/, float dt) const override {
    return KalmanPredictor<kStateDim, 1>::BuildTransitionMatrix(dt);
  }
};

/**
 * @brief 线性位置提取量测模型（默认实现）。
 * @details h(x) = H·x，Jacobian = H（位置提取矩阵）。仅在 6 维 CV 状态 / 3 维位置量测时有效。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class LinearPositionMeasurementModel final : public IMeasurementModel<kStateDim, kMeasurementDim> {
 public:
  using StateVector = typename IMeasurementModel<kStateDim, kMeasurementDim>::StateVector;
  using MeasurementVector = typename IMeasurementModel<kStateDim, kMeasurementDim>::MeasurementVector;
  using MeasurementMatrix = typename IMeasurementModel<kStateDim, kMeasurementDim>::MeasurementMatrix;

  MeasurementVector Function(const StateVector& state) const override {
    MeasurementVector z = MeasurementVector::Zero();
#if __cplusplus >= 201703L
    if constexpr (kStateDim == 6 && kMeasurementDim == 3) {
      z(0) = state(0);
      z(1) = state(2);
      z(2) = state(4);
    }
#else
    ApplyPositionEntries6x3(z, state, std::integral_constant<bool, kStateDim == 6 && kMeasurementDim == 3>{});
#endif
    return z;
  }
  MeasurementMatrix Jacobian(const StateVector& /*state*/) const override {
    return IKalmanUpdater<kStateDim, kMeasurementDim>::BuildPositionMeasurementMatrix();
  }

 private:
  // C++17 以下兼容：tag dispatch 消除维度不匹配时越界的向量访问分支。
  static void ApplyPositionEntries6x3(MeasurementVector& z, const StateVector& state, std::true_type) {
    z(0) = state(0);
    z(1) = state(2);
    z(2) = state(4);
  }
  static void ApplyPositionEntries6x3(MeasurementVector&, const StateVector&, std::false_type) {}
};

/**
 * @brief EKF 预测器配置。
 */
struct EkfPredictorConfig {
  float noise_diff_coeff{1.0f}; /**< 过程噪声扩散系数 q。 */
};

/**
 * @brief 扩展 Kalman 预测器。
 * @details 参考 Stone Soup ExtendedKalmanPredictor：
 *          - 均值预测使用非线性函数：x̂ = f(x, dt)
 *          - 协方差使用 Jacobian 线性化：P̂ = F·P·Fᵀ + Q
 *          其中 F = ∂f/∂x 在当前状态处的 Jacobian。
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class EkfPredictor final : public IKalmanPredictor<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using TransitionModel = ITransitionModel<kStateDim>;
  using StateVector = typename GaussianStateT::StateVector;
  using TransitionMatrix = typename GaussianStateT::TransitionMatrix;
  using ProcessNoiseCovariance = typename GaussianStateT::ProcessNoiseCovariance;

  /**
   * @brief 构造函数。
   * @param[in] model 转移模型（非拥有指针）。
   * @param[in] config 预测器配置。
   */
  EkfPredictor(const TransitionModel* model, EkfPredictorConfig config = {})
      : model_(model), config_(config) {}

  GaussianStateT Predict(const GaussianStateT& prior, float dt) const override {
    if (model_ == nullptr) {
      // 中译：转移模型未注入，本次预测被跳过（返回先验原样）。
      // 标识：注入校验——空模型指针属装配缺陷；fail-safe 返回先验，
      //       不解引用空指针。
      PROJECT_LOG_ERROR(
          "[EkfPredictor] Transition model is null; prediction is skipped (prior passed through).");
      return prior;
    }
    /* 非线性均值预测 */
    const StateVector x_pred = model_->Function(prior.mean, dt);

    /* Jacobian 线性化 */
    const TransitionMatrix F = model_->Jacobian(prior.mean, dt);

    /* 过程噪声（复用 KalmanPredictor 的 CV 模型噪声结构） */
    const ProcessNoiseCovariance Q =
        KalmanPredictor<kStateDim, kMeasurementDim>::BuildProcessNoise(dt, config_.noise_diff_coeff);

    GaussianStateT predicted;
    predicted.mean = x_pred;
    predicted.covariance = F * prior.covariance * F.transpose() + Q;
    return predicted;
  }

 private:
  const TransitionModel* model_{nullptr}; /**< 转移模型 */
  EkfPredictorConfig config_{};            /**< 配置参数 */
};

/**
 * @brief EKF 更新器配置。
 */
struct EkfUpdaterConfig {
  float measurement_noise_std{10.0f}; /**< 量测噪声标准差。 */
};

/**
 * @brief 扩展 Kalman 更新器。
 * @details 参考 Stone Soup ExtendedKalmanUpdater：
 *          - 新息使用非线性函数：y = z - h(x̂)
 *          - Kalman 增益使用 Jacobian 线性化的量测矩阵
 *          - 后验协方差使用 Joseph 形式
 * @tparam kStateDim 状态空间维度。
 * @tparam kMeasurementDim 量测空间维度。
 */
template <int kStateDim, int kMeasurementDim>
class EkfUpdater final : public IKalmanUpdater<kStateDim, kMeasurementDim> {
 public:
  using GaussianStateT = GaussianState<kStateDim, kMeasurementDim>;
  using MeasurementModel = IMeasurementModel<kStateDim, kMeasurementDim>;
  using StateCovariance = typename GaussianStateT::StateCovariance;
  using MeasurementVector = typename GaussianStateT::MeasurementVector;
  using MeasurementCovariance = typename GaussianStateT::MeasurementCovariance;
  using MeasurementMatrix = typename GaussianStateT::MeasurementMatrix;
  using KalmanGainMatrix = typename GaussianStateT::KalmanGainMatrix;
  using Result = KalmanUpdateResult<kStateDim, kMeasurementDim>;

  /**
   * @brief 构造函数。
   * @param[in] model 量测模型（非拥有指针）。
   * @param[in] config 更新器配置。
   */
  EkfUpdater(const MeasurementModel* model, EkfUpdaterConfig config = {})
      : model_(model),
        config_(config),
        R_(MeasurementCovariance::Identity() * (config.measurement_noise_std *
                                                config.measurement_noise_std)) {}

  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement) const override {
    return Update(predicted, measurement, R_);
  }
  Result Update(const GaussianStateT& predicted, const MeasurementVector& measurement,
                const MeasurementCovariance& dynamic_R) const override {
    Result result;
    if (model_ == nullptr) {
      // 中译：量测模型未注入，本次更新被跳过（后验=预测）。
      // 标识：注入校验——空模型指针属装配缺陷；fail-safe 跳过更新，
      //       不解引用空指针。
      PROJECT_LOG_ERROR(
          "[EkfUpdater] Measurement model is null; update is skipped (posterior = predicted).");
      result.posterior = predicted;
      return result;
    }

    /* 非线性量测预测 */
    const MeasurementVector z_pred = model_->Function(predicted.mean);

    /* Jacobian */
    const MeasurementMatrix H = model_->Jacobian(predicted.mean);

    /* Innovation */
    result.innovation = measurement - z_pred;

    /* Innovation covariance */
    result.innovation_covariance = H * predicted.covariance * H.transpose() + dynamic_R;

    /* Kalman gain：K = (S⁻¹·H·P̂)ᵀ，避免计算显式逆 */
    const Eigen::LLT<MeasurementCovariance> llt(result.innovation_covariance);
    if (llt.info() != Eigen::Success) {
      // 中译：创新协方差 LLT 分解失败，本次更新被跳过。
      // 标识：数值保护——协方差非正定时跳过更新（后验=预测），
      //       防止数值发散。
      PROJECT_LOG_ERROR(
          "[EkfUpdater] Innovation covariance LLT decomposition failed; update is skipped.");
      result.posterior = predicted;
      return result;
    }
    const KalmanGainMatrix K = llt.solve(H * predicted.covariance).transpose();

    /* Posterior mean */
    result.posterior.mean = predicted.mean + K * result.innovation;

    /* Posterior covariance (Joseph 形式) */
    const StateCovariance I_KH = StateCovariance::Identity() - K * H;
    result.posterior.covariance =
        I_KH * predicted.covariance * I_KH.transpose() + K * dynamic_R * K.transpose();

    return result;
  }

 private:
  const MeasurementModel* model_{nullptr}; /**< 量测模型 */
  EkfUpdaterConfig config_{};               /**< 配置参数 */
  MeasurementCovariance R_;                 /**< 静态量测噪声协方差 */
};

extern template class EkfPredictor<6, 3>;
extern template class EkfPredictor<6, 2>;
extern template class EkfUpdater<6, 3>;
extern template class EkfUpdater<6, 2>;

}  // namespace estimation
}  // namespace common
}  // namespace oneq

#endif  // COMMON_ESTIMATION_EKF_FILTER_H_
