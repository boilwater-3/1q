/**
 * @file UdkfPredictor.h
 * @brief 定义 UD 稳定化 Kalman 预测器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_PREDICTOR_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_PREDICTOR_H_

#include "airborne_radar/signal/tracking/IKalmanPredictor.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief UD 分解稳定化预测器。
 * @details 参考 REOS `estimation_lib/udkf.f90` 的 time-update 思路：
 *          对时间推进后的协方差执行 UD 结构化重建，抑制数值退化。
 */
class UdkfPredictor final : public IKalmanPredictor {
 public:
  /**
   * @brief 构造函数。
   * @param config 预测器配置。
   */
  explicit UdkfPredictor(KalmanPredictorConfig config = {});

  /**
   * @brief 对先验状态执行 UD 稳定化时间更新预测。
   * @param prior 先验高斯状态。
   * @param dt 预测时间步长（秒）。
   * @return 预测后的高斯状态。
   */
  GaussianTrackState Predict(const GaussianTrackState& prior, float dt) const override;

  /**
   * @brief 更新预测器配置。
   * @param config 新的预测器配置。
   */
  void UpdateConfig(KalmanPredictorConfig config) override;

 private:
  static bool Cov2Ud(const StateCovariance& covariance, StateCovariance* upper_u,
                     StateVector* diagonal_d);
  static StateCovariance Ud2Cov(const StateCovariance& upper_u, const StateVector& diagonal_d);
  static StateCovariance StabilizeCovariance(const StateCovariance& covariance);

  KalmanPredictorConfig config_{};
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_PREDICTOR_H_
