#include "sbirs_sensor/pipeline/SbirsPointingCoordinator.h"

#include <cmath>
#include <set>

namespace sbirs_sensor {
namespace pipeline {

SbirsPointingCoordinator::SbirsPointingCoordinator(int channel_count)
    : channels_(static_cast<std::size_t>(channel_count < 1 ? 1 : channel_count)) {}

bool SbirsPointingCoordinator::IsValidChannel(int channel_id) const {
  return channel_id >= 0 && static_cast<std::size_t>(channel_id) < channels_.size();
}

bool SbirsPointingCoordinator::Reserve(int channel_id, std::uint64_t target_id,
                                       const session::SbirsVector3M& initial_los) {
  if (!IsValidChannel(channel_id)) {
    return false;
  }
  const int existing_channel = ChannelOf(target_id);
  if (existing_channel >= 0) {
    return existing_channel == channel_id;
  }
  ChannelRuntime& channel = channels_[static_cast<std::size_t>(channel_id)];
  if (channel.has_bound_target) {
    return false;
  }
  const SbirsPointingActuatorSnapshot actuator_state = channel.actuator.Capture();
  if (!actuator_state.initialized && !channel.actuator.Initialize(initial_los)) {
    return false;
  }
  channel.has_bound_target = true;
  channel.target_id = target_id;
  channel.elapsed_wait_sec = 0.0;
  return true;
}

SbirsPointingAdvanceResult SbirsPointingCoordinator::Advance(
    int channel_id, std::uint64_t target_id, const session::SbirsVector3M& command_los,
    double dt_sec, const SbirsPointingActuatorConfig& config) {
  SbirsPointingAdvanceResult result;
  if (!IsValidChannel(channel_id)) {
    return result;
  }
  ChannelRuntime& channel = channels_[static_cast<std::size_t>(channel_id)];
  if (!channel.has_bound_target || channel.target_id != target_id) {
    return result;
  }
  SbirsPointingActuatorResult actuator_result;
  if (!channel.actuator.Step(command_los, dt_sec, config, &actuator_result)) {
    return result;
  }

  const double next_elapsed = channel.elapsed_wait_sec + dt_sec;
  channel.elapsed_wait_sec = next_elapsed;
  result.current_los = actuator_result.current_los;
  result.remaining_angle_deg = actuator_result.remaining_angle_deg;
  result.elapsed_wait_sec = next_elapsed;
  if (actuator_result.settled) {
    result.status = SbirsPointingAdvanceStatus::kSettled;
    return result;
  }

  const double timeout_sec = 180.0 / config.max_slew_rate_deg_per_sec;
  if (next_elapsed >= timeout_sec) {
    channel.has_bound_target = false;
    channel.target_id = 0U;
    channel.elapsed_wait_sec = 0.0;
    result.status = SbirsPointingAdvanceStatus::kTimedOut;
    return result;
  }
  result.status = SbirsPointingAdvanceStatus::kSlewing;
  return result;
}

bool SbirsPointingCoordinator::ReleaseTarget(std::uint64_t target_id) {
  const int channel_id = ChannelOf(target_id);
  if (channel_id < 0) {
    return false;
  }
  ChannelRuntime& channel = channels_[static_cast<std::size_t>(channel_id)];
  channel.has_bound_target = false;
  channel.target_id = 0U;
  channel.elapsed_wait_sec = 0.0;
  return true;
}

void SbirsPointingCoordinator::Clear() { channels_.assign(channels_.size(), ChannelRuntime{}); }

bool SbirsPointingCoordinator::IsTargetBound(std::uint64_t target_id) const {
  return ChannelOf(target_id) >= 0;
}

int SbirsPointingCoordinator::ChannelOf(std::uint64_t target_id) const {
  for (std::size_t i = 0; i < channels_.size(); ++i) {
    if (channels_[i].has_bound_target && channels_[i].target_id == target_id) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

SbirsPointingCoordinatorSnapshot SbirsPointingCoordinator::Capture() const {
  SbirsPointingCoordinatorSnapshot snapshot;
  snapshot.channels.reserve(channels_.size());
  for (std::size_t i = 0; i < channels_.size(); ++i) {
    SbirsPointingChannelSnapshot channel;
    channel.channel_id = static_cast<int>(i);
    channel.has_bound_target = channels_[i].has_bound_target;
    channel.target_id = channels_[i].target_id;
    channel.elapsed_wait_sec = channels_[i].elapsed_wait_sec;
    channel.actuator = channels_[i].actuator.Capture();
    snapshot.channels.push_back(channel);
  }
  return snapshot;
}

bool SbirsPointingCoordinator::Restore(const SbirsPointingCoordinatorSnapshot& snapshot) {
  if (snapshot.channels.size() != channels_.size()) {
    return false;
  }
  std::vector<ChannelRuntime> restored(channels_.size());
  std::set<std::uint64_t> bound_targets;
  for (std::size_t i = 0; i < snapshot.channels.size(); ++i) {
    const SbirsPointingChannelSnapshot& source = snapshot.channels[i];
    if (source.channel_id != static_cast<int>(i) || !std::isfinite(source.elapsed_wait_sec) ||
        source.elapsed_wait_sec < 0.0 ||
        (!source.has_bound_target && source.elapsed_wait_sec != 0.0) ||
        (source.has_bound_target && !source.actuator.initialized) ||
        (source.has_bound_target && !bound_targets.insert(source.target_id).second)) {
      return false;
    }
    ChannelRuntime& destination = restored[i];
    if (!destination.actuator.Restore(source.actuator)) {
      return false;
    }
    destination.has_bound_target = source.has_bound_target;
    destination.target_id = source.has_bound_target ? source.target_id : 0U;
    destination.elapsed_wait_sec = source.elapsed_wait_sec;
  }
  channels_ = restored;
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
