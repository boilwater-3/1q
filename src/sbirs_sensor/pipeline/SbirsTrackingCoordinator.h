/**
 * @file SbirsTrackingCoordinator.h
 * @brief Internal owner of SBIRS estimated-tracking runtime state.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_TRACKING_COORDINATOR_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_TRACKING_COORDINATOR_H_

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"
#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/tracking/SbirsTrackingTypes.h"

namespace sbirs_sensor {
namespace pipeline {

/** @brief Persisted tracking state mapped directly to SbirsPipelineSnapshot fields. */
struct SbirsTrackingRuntimeState {
  std::map<std::uint64_t, tracking::SbirsGaussianState> filter_states{};
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts{};
  bool imm_active{false};
  std::map<std::uint64_t, tracking::SbirsImmSnapshot> imm_snapshots{};
};

/** @brief Result of one estimated-tracking update. */
struct SbirsTrackingUpdateResult {
  float output_azimuth_deg{0.0f};
  float output_elevation_deg{0.0f};
  bool has_estimation_nis{false};
  float estimation_nis{0.0f};
  bool estimation_nis_gate_exceeded{false};
  bool lost_due_to_estimation_nis{false};
};

/**
 * @brief Coordinates EKF/IMM update state for targets already locked by SbirsPipeline.
 * @note Scheduling, target-state transitions, output construction, and snapshot schema remain pipeline-owned.
 */
class SbirsTrackingCoordinator {
 public:
  void InitializeTarget(std::uint64_t target_id, const session::SbirsSceneTarget& target,
                        const config::SbirsTrackingConfig& tracking);
  SbirsTrackingUpdateResult Update(std::uint64_t target_id,
                                   const config::SbirsPolicyConfig& policy,
                                   foundation::SbirsRandomSource* random_source,
                                   float azimuth_deg, float elevation_deg, double range_m,
                                   float angular_rate_deg_per_sec, float dt_sec,
                                   const session::SbirsVector3M& satellite_position_ecef_m);
  void ReleaseTarget(std::uint64_t target_id);
  void ClearForStandby();
  SbirsTrackingRuntimeState CaptureRuntimeState() const;
  void RestoreRuntimeState(const SbirsTrackingRuntimeState& state);

 private:
  void InitializeImmComponents(const config::SbirsTrackingConfig& tracking);
  tracking::SbirsImmFilter* CreateImmFilter(
      std::uint64_t target_id, const tracking::SbirsGaussianState& initial_state);

  tracking::SbirsCvTransitionModel cv_transition_model_{};
  tracking::SbirsAngleMeasurementModel angle_measurement_model_{};
  std::map<std::uint64_t, tracking::SbirsGaussianState> filter_states_{};
  std::map<std::uint64_t, unsigned int> nis_gate_exceeded_counts_{};
  bool imm_initialized_{false};
  std::vector<std::unique_ptr<tracking::SbirsAngleMeasurementModel>> imm_measurement_models_{};
  std::vector<std::unique_ptr<tracking::SbirsEkfPredictor>> imm_predictors_owned_{};
  std::vector<::oneq::common::estimation::IKalmanPredictor<6, 2>*> imm_predictors_{};
  std::vector<std::unique_ptr<tracking::SbirsEkfUpdater>> imm_updaters_owned_{};
  std::vector<::oneq::common::estimation::IKalmanUpdater<6, 2>*> imm_updaters_{};
  // Predictors and updaters are immutable model plumbing shared by all targets;
  // each target owns its mutable IMM model state.
  std::map<std::uint64_t, std::unique_ptr<tracking::SbirsImmFilter>> imm_filters_by_target_{};
  // Restored snapshots wait here until the target next receives an update.
  std::map<std::uint64_t, tracking::SbirsImmSnapshot> imm_snapshots_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_TRACKING_COORDINATOR_H_
