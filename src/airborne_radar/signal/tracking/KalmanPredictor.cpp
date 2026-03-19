#include "airborne_radar/signal/tracking/KalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

KalmanPredictor::KalmanPredictor(KalmanPredictorConfig config)
    : config_(config) {}

GaussianTrackState KalmanPredictor::Predict(const GaussianTrackState &prior,
                                            float dt) const {
  const TransitionMatrix F = BuildTransitionMatrix(dt);
  const ProcessNoiseCovariance Q = BuildProcessNoise(dt, config_.noise_diff_coeff);

  GaussianTrackState predicted;
  predicted.mean = F * prior.mean;
  predicted.covariance = F * prior.covariance * F.transpose() + Q;
  return predicted;
}

void KalmanPredictor::UpdateConfig(KalmanPredictorConfig config) {
  config_ = config;
}

TransitionMatrix KalmanPredictor::BuildTransitionMatrix(float dt) {
  // 直接构造块对角矩阵，避免运行时 block_diag 调用
  TransitionMatrix F = TransitionMatrix::Identity();

  // X 轴：F(0,1) = dt
  F(0, 1) = dt;
  // Y 轴：F(2,3) = dt
  F(2, 3) = dt;
  // Z 轴：F(4,5) = dt
  F(4, 5) = dt;

  return F;
}

ProcessNoiseCovariance KalmanPredictor::BuildProcessNoise(float dt, float q) {
  const float dt2 = dt * dt;
  const float dt3 = dt2 * dt;

  ProcessNoiseCovariance Q = ProcessNoiseCovariance::Zero();

  // 对 X、Y、Z 三个轴分别填充 2×2 子块
  for (int axis = 0; axis < 3; ++axis) {
    const int base = axis * 2;
    Q(base, base) = dt3 / 3.0f;       // 位置-位置
    Q(base, base + 1) = dt2 / 2.0f;   // 位置-速度
    Q(base + 1, base) = dt2 / 2.0f;   // 速度-位置
    Q(base + 1, base + 1) = dt;       // 速度-速度
  }

  Q *= q;
  return Q;
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
