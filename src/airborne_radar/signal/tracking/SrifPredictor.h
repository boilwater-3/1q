/**
 * @file SrifPredictor.h
 * @brief 定义 SRIF 预测器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_PREDICTOR_H_

#include "airborne_radar/signal/tracking/IKalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief SRIF（Square-Root Information Filter）预测器。
 * @details 参考 REOS `estimation_lib/srif.f90` 的 time-update 思路：
 *          对预测协方差构造信息矩阵并以平方根信息形式稳定回写。
 */
class SrifPredictor final : public IKalmanPredictor {
 public:
  explicit SrifPredictor(KalmanPredictorConfig config = {});

  GaussianTrackState Predict(const GaussianTrackState& prior, float dt) const override;

  void UpdateConfig(KalmanPredictorConfig config) override;

 private:
  static StateCovariance StabilizeWithInformationForm(const StateCovariance& covariance);

  KalmanPredictorConfig config_{};
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_PREDICTOR_H_
