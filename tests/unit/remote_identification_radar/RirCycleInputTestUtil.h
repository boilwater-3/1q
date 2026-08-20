// Copyright 2026. All Rights Reserved.
//
// @file RirCycleInputTestUtil.h
// @brief RIR 周期输入测试辅助（平台 ECEF 等）。

#ifndef TESTS_UNIT_REMOTE_IDENTIFICATION_RADAR_RIR_CYCLE_INPUT_TEST_UTIL_H_
#define TESTS_UNIT_REMOTE_IDENTIFICATION_RADAR_RIR_CYCLE_INPUT_TEST_UTIL_H_

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"

namespace remote_identification_radar {
namespace tests {

/** @brief 填入测试缺省平台 ECEF（30°N, 120°E, 1000 m）。 */
inline void SetDefaultTestPlatformEcef(session::RirCycleInput* input) {
  if (input == nullptr) {
    return;
  }
  oneq::coordinate::LlaPositionDegM lla{};
  lla.latitude_deg = 30.0;
  lla.longitude_deg = 120.0;
  lla.altitude_m = 1000.0;
  oneq::coordinate::TryLlaToEcef(lla, &input->platform_position);
}

}  // namespace tests
}  // namespace remote_identification_radar

#endif  // TESTS_UNIT_REMOTE_IDENTIFICATION_RADAR_RIR_CYCLE_INPUT_TEST_UTIL_H_
