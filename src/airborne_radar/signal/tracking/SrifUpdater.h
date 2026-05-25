/**
 * @file SrifUpdater.h
 * @brief 定义 SRIF 量测更新器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_

#include "airborne_radar/signal/tracking/IKalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/**
 * @brief SRIF（Square-Root Information Filter）更新器。
 */
class SrifUpdater final : public IKalmanUpdater {
 public:
  explicit SrifUpdater(KalmanUpdaterConfig config = {});

  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement) const override;
  KalmanUpdateResult Update(const GaussianTrackState& predicted,
                            const MeasurementVector& measurement,
                            const MeasurementCovariance& dynamic_R) const override;
  void UpdateConfig(KalmanUpdaterConfig config) override;

 private:
  KalmanUpdaterConfig config_{};
  MeasurementMatrix H_;
  MeasurementCovariance R_;
};

}  // namespace tracking
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_TRACKING_SRIF_UPDATER_H_
