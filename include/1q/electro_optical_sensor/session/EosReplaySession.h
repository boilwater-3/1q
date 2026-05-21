/**
 * @file EosReplaySession.h
 * @brief Provides an EOS replay entry point backed by replay trace events.
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_SESSION_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace electro_optical_sensor {
namespace session {

struct ONEQ_API EosReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
  std::string first_error{};
};

ONEQ_API EosReplaySessionResult ReplayEosTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_REPLAY_SESSION_H_
