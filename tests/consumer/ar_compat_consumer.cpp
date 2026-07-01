/**
 * @file ar_compat_consumer.cpp
 * @brief 验证 AR 迁移期旧 Radar* public API 仍可被外部工程编译链接。
 */

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarSession.h"

int main() {
  airborne_radar::session::RadarSession session =
      airborne_radar::session::RadarSession::Create(airborne_radar::config::RadarSessionConfig{});
  airborne_radar::session::RadarCycleInput input;
  const airborne_radar::session::RadarCycleResult result = session.StepWithResult(input);
  return result.has_validation_error ? 1 : 0;
}
