/**
 * @file FlightDynamicSessionFactory.cpp
 * @brief FlightDynamicSessionFactory 实现。
 */

#include "1q/flight_dynamic/session/FlightDynamicSessionFactory.h"
#include "flight_dynamic/session/FlightDynamicSessionImpl.h"

namespace flight_dynamic {
namespace session {

// ---- Factory 实现 ----
FlightDynamicSession FlightDynamicSessionFactory::Create(
    const config::FlightDynamicConfig& config) {
  auto impl = std::unique_ptr<FlightDynamicSession::Impl>(
      new FlightDynamicSession::Impl(config));
  return FlightDynamicSession(std::move(impl));
}

}  // namespace session
}  // namespace flight_dynamic
