/**
 * @file FlightDynamicSession.cpp
 * @brief FlightDynamicSession 公共接口实现（PIMPL 转发）。
 */

#include "1q/flight_dynamic/session/FlightDynamicSession.h"
#include "flight_dynamic/session/FlightDynamicSessionImpl.h"

namespace flight_dynamic {
namespace session {

// ---- 构造/析构 ----

FlightDynamicSession::FlightDynamicSession() = default;

FlightDynamicSession::~FlightDynamicSession() = default;

FlightDynamicSession::FlightDynamicSession(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

FlightDynamicSession::FlightDynamicSession(FlightDynamicSession&&) noexcept = default;
FlightDynamicSession& FlightDynamicSession::operator=(FlightDynamicSession&&) noexcept = default;

// ---- 公共接口（转发到 Impl）----

model::FlightDynamicOutput FlightDynamicSession::Step(
    const model::FlightDynamicInput& input) {
  if (!impl_) {
    return model::FlightDynamicOutput{};
  }
  const bool ok = impl_->adapter.Run(input);
  if (ok) {
    impl_->last_output = impl_->mapper.Map(
        impl_->adapter.GetPropagate(),
        impl_->adapter.GetAccelerations(),
        impl_->adapter.GetFdmExec());
    impl_->last_output.ok = true;
  } else {
    impl_->last_output.ok = false;
  }
  return impl_->last_output;
}

void FlightDynamicSession::Reset(
    const oneq::coordinate::ExternalKinematics& kinematics) {
  if (!impl_) {
    return;
  }
  impl_->adapter.Reset(kinematics);
  impl_->last_output = impl_->mapper.Map(
      impl_->adapter.GetPropagate(),
      impl_->adapter.GetAccelerations(),
      impl_->adapter.GetFdmExec());
  impl_->last_output.ok = true;
}

model::FlightDynamicOutput FlightDynamicSession::GetCurrentState() const {
  if (!impl_) {
    return model::FlightDynamicOutput{};
  }
  return impl_->last_output;
}

}  // namespace session
}  // namespace flight_dynamic
