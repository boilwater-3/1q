/**
 * @file rir_earth_occultation_test.cpp
 * @brief 验证 RIR 地球遮挡硬门：穿地目标不入检测，近距可见目标不受影响。
 *
 * 覆盖：
 *   1) 近距切平面目标照常检出；
 *   2) 对侧（地球背面）目标无航迹归属，带 rir.target_earth_occulted；
 *   3) 遮挡优先于体积门（同时出角域也只报遮挡）。
 */

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirSceneTarget;
using session::RirSession;

config::RirSessionConfig MakeIdentifyConfig() {
  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  // 主瓣覆盖门放宽：本文件聚焦地球遮挡几何，不测波束覆盖门（门限=半功率宽）。
  config.hardware.antenna.nominal_az_beamwidth_deg = 160.0f;
  config.hardware.antenna.nominal_el_beamwidth_deg = 160.0f;
  return config;
}

RirSceneTarget MakeNearTarget(std::uint64_t id) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = "near-visible";
  target.position_x = 10000.0f;
  target.position_z = 2000.0f;
  target.rcs = 0.5f;
  target.range_m = std::sqrt(10000.0f * 10000.0f + 2000.0f * 2000.0f);
  return target;
}

RirSceneTarget MakeAntipodeTarget(std::uint64_t id) {
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 30.0;
  platform_lla.longitude_deg = 120.0;
  platform_lla.altitude_m = 1000.0;
  oneq::coordinate::LlaPositionDegM far_lla;
  far_lla.latitude_deg = -30.0;
  far_lla.longitude_deg = -60.0;
  far_lla.altitude_m = 20000.0;
  oneq::coordinate::EcefPositionM far_ecef;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(far_lla, &far_ecef));
  oneq::coordinate::EnuPositionM enu;
  EXPECT_TRUE(oneq::coordinate::TryEcefToEnu(far_ecef, platform_lla, &enu));
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = "antipode";
  target.position_x = static_cast<float>(enu.east_m);
  target.position_y = static_cast<float>(enu.north_m);
  target.position_z = static_cast<float>(enu.up_m);
  target.rcs = 10.0f;
  const float range = std::sqrt(target.position_x * target.position_x +
                                target.position_y * target.position_y +
                                target.position_z * target.position_z);
  target.range_m = range;
  return target;
}

RirCycleInput MakeInput(std::uint32_t cycle, const std::vector<RirSceneTarget>& targets) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets = targets;
  return input;
}

bool HasAttributionFor(const RirCycleResult& result, std::uint64_t target_id) {
  for (const auto& attribution : result.track_attributions) {
    if (attribution.external_target_id == target_id) {
      return true;
    }
  }
  return false;
}

const session::RirIssue* FindIssue(const RirCycleResult& result, const char* code,
                                   std::uint64_t target_id) {
  const std::string id_text = "target_id=" + std::to_string(target_id);
  for (const session::RirIssue& issue : result.issues) {
    if (issue.code == code && issue.message.find(id_text) != std::string::npos) {
      return &issue;
    }
  }
  return nullptr;
}

TEST(RirEarthOccultationTest, NearTargetIsDetected) {
  const RirSceneTarget target = MakeNearTarget(9101U);
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  bool seen = false;
  for (std::uint32_t cycle = 1U; cycle <= 6U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    seen = seen || HasAttributionFor(result, target.external_target_id);
    EXPECT_EQ(FindIssue(result, session::codes::kTargetEarthOcculted, target.external_target_id),
              nullptr);
  }
  EXPECT_TRUE(seen) << "近距可见目标应检出并形成航迹归属";
}

TEST(RirEarthOccultationTest, AntipodeTargetIsExcludedWithOccultedIssue) {
  const RirSceneTarget target = MakeAntipodeTarget(9102U);
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.orientation.steerable_volume_deg.el_min_deg = -90.0f;
  config.orientation.steerable_volume_deg.el_max_deg = 90.0f;
  RirSession session = RirSession::Create(config);
  for (std::uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_FALSE(HasAttributionFor(result, target.external_target_id))
        << "对侧穿地目标不应入检测候选";
    const session::RirIssue* excluded =
        FindIssue(result, session::codes::kTargetEarthOcculted, target.external_target_id);
    ASSERT_NE(excluded, nullptr) << "穿地目标应携带地球遮挡排除诊断";
    EXPECT_EQ(excluded->cause, session::RirIssueCause::kNone);
    EXPECT_EQ(excluded->location.entity_index, 0U);
    EXPECT_NE(excluded->message.find("occultation_margin_m="), std::string::npos);
  }
}

TEST(RirEarthOccultationTest, OccultationPrecedesVolumeGate) {
  const RirSceneTarget occulted = MakeAntipodeTarget(9103U);
  const RirSceneTarget near_visible = MakeNearTarget(9104U);
  config::RirSessionConfig config = MakeIdentifyConfig();
  // 默认体积 el ∈ [-30, 30]：对侧目标俯仰远低于 -30，若先走体积门会报出界。
  RirSession session = RirSession::Create(config);
  bool seen_near = false;
  for (std::uint32_t cycle = 1U; cycle <= 6U; ++cycle) {
    const RirCycleResult result =
        session.StepWithResult(MakeInput(cycle, {occulted, near_visible}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    const session::RirIssue* occulted_issue =
        FindIssue(result, session::codes::kTargetEarthOcculted, occulted.external_target_id);
    ASSERT_NE(occulted_issue, nullptr);
    EXPECT_EQ(FindIssue(result, session::codes::kTargetOutsideSearchVolume,
                        occulted.external_target_id),
              nullptr)
        << "遮挡应优先于体积门，不得再报出界";
    seen_near = seen_near || HasAttributionFor(result, near_visible.external_target_id);
  }
  EXPECT_TRUE(seen_near) << "同帧近距可见目标仍应检出";
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
