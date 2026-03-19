/**
 * @file EkfFilter.h
 * @brief 定义扩展 Kalman 滤波器（EKF）的预测器和更新器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_EKF_FILTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_EKF_FILTER_H_

#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {
/**
 * @brief 非线性转移模型抽象接口。
 * @details 对应 Stone Soup 的 TransitionModel + jacobian() 方法。
 */
class ITransitionModel {
 public:
  virtual ~ITransitionModel() = default;
/**
 * @brief 非线性状态转移函数 f(x, dt)。
 * @param state 当前状态向量。
 * @param dt 时间步长。
 * @return 预测状态向量。
 */
  virtual StateVector Function(const StateVector &state, float dt) const = 0;
/**
 * @brief 转移 Jacobian F(x, dt) = ∂f/∂x。
 * @param state 线性化点的状态向量。
 * @param dt 时间步长。
 * @return 6×6 Jacobian 矩阵。
 */
  virtual TransitionMatrix Jacobian(const StateVector &state,
                                    float dt) const = 0;
};
/**
 * @brief 非线性量测模型抽象接口。
 * @details 对应 Stone Soup 的 MeasurementModel + jacobian() 方法。
 */
class IMeasurementModel {
 public:
  virtual ~IMeasurementModel() = default;
/**
 * @brief 非线性量测函数 h(x)。
 * @param state 状态向量。
 * @return 量测向量。
 */
  virtual MeasurementVector Function(const StateVector &state) const = 0;
/**
 * @brief 量测 Jacobian H(x) = ∂h/∂x。
 * @param state 线性化点的状态向量。
 * @return 3×6 Jacobian 矩阵。
 */
  virtual MeasurementMatrix Jacobian(const StateVector &state) const = 0;
};
/**
 * @brief 线性恒速转移模型（默认实现）。
 * @details f(x,dt) = F·x，Jacobian = F（恒速矩阵）。
 */
class LinearCvTransitionModel final : public ITransitionModel {
 public:
/**
 * @brief 计算恒速状态转移。
 * @param state 当前状态 [x, vx, y, vy, z, vz]。
 * @param dt 时间步长（秒）。
 * @return 线性外推后的状态。
 */
  StateVector Function(const StateVector &state, float dt) const override {
    return KalmanPredictor::BuildTransitionMatrix(dt) * state;
  }
/**
 * @brief 获取恒速转移 Jacobian（即 F 矩阵）。
 * @param state 当前状态点。
 * @param dt 时间步长（秒）。
 * @return 状态转移 Jacobian 矩阵。
 */
  TransitionMatrix Jacobian(const StateVector &, float dt) const override {
    return KalmanPredictor::BuildTransitionMatrix(dt);
  }
};
/**
 * @brief 线性位置提取量测模型（默认实现）。
 * @details h(x) = H·x，Jacobian = H（位置提取矩阵）。
 */
class LinearPositionMeasurementModel final : public IMeasurementModel {
 public:
/**
 * @brief 从状态中提取位置分量。
 * @param state 6 维高斯状态。
 * @return 3 维位置量测。
 */
  MeasurementVector Function(const StateVector &state) const override {
    MeasurementVector z;
    z(0) = state(0);
    z(1) = state(2);
    z(2) = state(4);
    return z;
  }
/**
 * @brief 获取量测 Jacobian（即 H 矩阵）。
 * @param state 当前状态点。
 * @return 量测 Jacobian 矩阵。
 */
  MeasurementMatrix Jacobian(const StateVector &) const override {
    MeasurementMatrix H = MeasurementMatrix::Zero();
    H(0, 0) = 1.0f;
    H(1, 2) = 1.0f;
    H(2, 4) = 1.0f;
    return H;
  }
};
/**
 * @brief EKF 预测器配置。
 */
struct EkfPredictorConfig {
/**
 * @brief 过程噪声扩散系数 q。
 */
  float noise_diff_coeff{1.0f};
};
/**
 * @brief 扩展 Kalman 预测器。
 * @details 参考 Stone Soup ExtendedKalmanPredictor：
 *          - 均值预测使用非线性函数：x̂ = f(x, dt)
 *          - 协方差使用 Jacobian 线性化：P̂ = F·P·Fᵀ + Q
 *          其中 F = ∂f/∂x 在当前状态处的 Jacobian。
 */
class EkfPredictor final : public IKalmanPredictor {
 public:
/**
 * @brief 构造函数。
 * @param model 转移模型（非拥有指针）。
 * @param config 预测器配置。
 */
  EkfPredictor(const ITransitionModel *model,
               EkfPredictorConfig config = {});
/**
 * @brief 使用非线性模型执行预测。
 * @param prior 先验高斯状态。
 * @param dt 时间步长。
 * @return 预测后的高斯状态。
 */
  GaussianTrackState Predict(const GaussianTrackState &prior,
                             float dt) const override;

 private:
  const ITransitionModel *model_{nullptr};   /**< 转移模型 */
  EkfPredictorConfig config_{};              /**< 配置参数 */
};
/**
 * @brief EKF 更新器配置。
 */
struct EkfUpdaterConfig {
/**
 * @brief 量测噪声标准差。
 */
  float measurement_noise_std{10.0f};
};
/**
 * @brief 扩展 Kalman 更新器。
 * @details 参考 Stone Soup ExtendedKalmanUpdater：
 *          - 新息使用非线性函数：y = z - h(x̂)
 *          - Kalman 增益使用 Jacobian 线性化的量测矩阵
 *          - 后验协方差使用 Joseph 形式
 */
class EkfUpdater final : public IKalmanUpdater {
 public:
/**
 * @brief 构造函数。
 * @param model 量测模型（非拥有指针）。
 * @param config 更新器配置。
 */
  EkfUpdater(const IMeasurementModel *model,
             EkfUpdaterConfig config = {});
/**
 * @brief 使用非线性量测模型执行更新。
 * @param predicted 预测后的高斯状态。
 * @param measurement 量测向量 [x, y, z]。
 * @return 更新结果。
 */
  KalmanUpdateResult Update(const GaussianTrackState &predicted,
                            const MeasurementVector &measurement) const override;
/**
 * @brief 使用非线性量测模型执行更新（使用动态观测协方差）。
 * @param predicted 预测后的高斯状态。
 * @param measurement 量测向量 [x, y, z]。
 * @param dynamic_R 动态计算的笛卡尔系量测噪声协方差矩阵 R。
 * @return 更新结果。
 */
  KalmanUpdateResult Update(const GaussianTrackState &predicted,
                            const MeasurementVector &measurement,
                            const MeasurementCovariance &dynamic_R) const override;

 private:
/**
 * @brief 构建量测噪声协方差矩阵 R。
 * @param std_dev 标准差。
 * @return 协方差矩阵。
 */
  static MeasurementCovariance BuildMeasurementNoise(float std_dev);

  const IMeasurementModel *model_{nullptr};  /**< 量测模型 */
  EkfUpdaterConfig config_{};                /**< 配置参数 */
  MeasurementCovariance R_;                  /**< 静态量测噪声协方差 */
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_EKF_FILTER_H_
