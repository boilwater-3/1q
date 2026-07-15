/**
 * @file SbirsPointingCoordinator.h
 * @brief Internal per-channel NFOV pointing runtime coordinator.
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_COORDINATOR_H_
#define ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_COORDINATOR_H_

#include <cstdint>
#include <vector>

#include "sbirs_sensor/pipeline/SbirsPointingActuator.h"

namespace sbirs_sensor {
namespace pipeline {

struct SbirsPointingChannelSnapshot {
  int channel_id{-1};
  bool has_bound_target{false};
  std::uint64_t target_id{0U};
  double elapsed_wait_sec{0.0};
  unsigned int tracking_gate_failure_count{0U};
  SbirsPointingActuatorSnapshot actuator{};
};

struct SbirsPointingCoordinatorSnapshot {
  std::vector<SbirsPointingChannelSnapshot> channels{};
};

enum class SbirsPointingAdvanceStatus { kRejected = 0, kSlewing, kSettled, kTimedOut };

struct SbirsPointingAdvanceResult {
  SbirsPointingAdvanceStatus status{SbirsPointingAdvanceStatus::kRejected};
  session::SbirsVector3M current_los{};
  double remaining_angle_deg{0.0};
  double elapsed_wait_sec{0.0};
};

class SbirsPointingCoordinator {
 public:
  explicit SbirsPointingCoordinator(int channel_count);

  bool Reserve(int channel_id, std::uint64_t target_id, const session::SbirsVector3M& initial_los);
  SbirsPointingAdvanceResult Advance(int channel_id, std::uint64_t target_id,
                                     const session::SbirsVector3M& command_los, double dt_sec,
                                     const SbirsPointingActuatorConfig& config);
  bool PromoteToTracking(std::uint64_t target_id);
  SbirsPointingAdvanceResult AdvanceTracking(
      int channel_id, std::uint64_t target_id, const session::SbirsVector3M& command_los,
      double dt_sec, const SbirsPointingActuatorConfig& config);
  unsigned int RecordTrackingGateResult(std::uint64_t target_id, bool gate_passed);
  bool ReleaseTarget(std::uint64_t target_id);
  void Clear();

  bool IsTargetBound(std::uint64_t target_id) const;
  int ChannelOf(std::uint64_t target_id) const;
  SbirsPointingCoordinatorSnapshot Capture() const;
  bool Restore(const SbirsPointingCoordinatorSnapshot& snapshot);

 private:
  struct ChannelRuntime {
    bool has_bound_target{false};
    std::uint64_t target_id{0U};
    double elapsed_wait_sec{0.0};
    unsigned int tracking_gate_failure_count{0U};
    SbirsPointingActuator actuator{};
  };

  bool IsValidChannel(int channel_id) const;
  std::vector<ChannelRuntime> channels_{};
};

}  // namespace pipeline
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_PIPELINE_SBIRS_POINTING_COORDINATOR_H_
