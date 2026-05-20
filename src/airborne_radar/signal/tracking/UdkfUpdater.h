/**
 * @file UdkfUpdater.h
 * @brief 定义 UD 稳定化 Kalman 更新器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_UPDATER_H_

#include "airborne_radar/signal/tracking/IKalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief UD 分解稳定化更新器。
 */
class UdkfUpdater final : public IKalmanUpdater {
 public:
  explicit UdkfUpdater(KalmanUpdaterConfig config = {});

  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement) const override;
  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement,
                            const MeasurementCovariance& dynamic_R) const override;
  void UpdateConfig(KalmanUpdaterConfig config) override;

 private:
  static MeasurementMatrix BuildMeasurementMatrix();
  static MeasurementCovariance BuildMeasurementNoise(float std_dev);
  static StateCovariance StabilizeCovarianceWithUd(const StateCovariance& covariance);

  KalmanUpdaterConfig config_{};
  MeasurementMatrix H_;
  MeasurementCovariance R_;
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_UDKF_UPDATER_H_
