// Copyright 2026. All Rights Reserved.
//
// @file rir_search_sector_test.cpp
// @brief 验证指定空域搜索的候选集角域裁剪（2026-08-22 甲方批注「设定方位俯仰进行扫描」）。
//
// 覆盖：
//   1) 可扫描体积内目标照常检出建航迹；
//   2) 俯仰出体积目标不入检测候选集（无航迹归属）；
//   3) 方位出体积（相对 scan_center）目标无航迹归属；
//   4) 运行期改 scan_center（设定方位俯仰）后窗口随之移动：原出界目标入界检出；
//   5) 出界目标携带 kTargetOutsideSearchVolume 排除诊断（规则 13b），体积内目标不带。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
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
  // 6 dB 回退门控：目标易被准入，聚焦角域裁剪行为（与指定任务测试同口径）。
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  return config;
}

/** @brief 东向 10 km / 高 2 km 目标：az=0°、el≈11.3°（默认体积 ±60/[-30,30] 内）。 */
RirSceneTarget MakeTarget(std::uint64_t id) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = "sector-target";
  target.position_x = 10000.0f;
  target.position_z = 2000.0f;
  target.rcs = 0.5f;
  target.range_m = std::sqrt(10000.0f * 10000.0f + 2000.0f * 2000.0f);
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

/// @brief 按代码与消息中的 target_id 查找排除诊断（与投影会话测试同口径）。
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

// ---------------------------------------------------------------------------
// 1) 体积内目标照常检出
// ---------------------------------------------------------------------------

TEST(RirSearchSectorTest, InVolumeTargetIsDetected) {
  const RirSceneTarget target = MakeTarget(9001U);
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  bool seen = false;
  for (std::uint32_t cycle = 1U; cycle <= 6U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    seen = seen || HasAttributionFor(result, target.external_target_id);
  }
  EXPECT_TRUE(seen) << "体积内目标应照常检出并形成航迹归属";
}

// ---------------------------------------------------------------------------
// 2) 俯仰出体积：目标不入候选集
// ---------------------------------------------------------------------------

TEST(RirSearchSectorTest, ElevationOutsideVolumeIsExcluded) {
  const RirSceneTarget target = MakeTarget(9002U);  // el≈11.3°
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.orientation.steerable_volume_deg.el_max_deg = 5.0f;  // 目标俯仰出界
  RirSession session = RirSession::Create(config);
  for (std::uint32_t cycle = 1U; cycle <= 6U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_FALSE(HasAttributionFor(result, target.external_target_id))
        << "俯仰出体积目标不应入检测候选集";
  }
}

// ---------------------------------------------------------------------------
// 3) 方位出体积（相对 scan_center）：目标无航迹归属
// ---------------------------------------------------------------------------

TEST(RirSearchSectorTest, AzimuthOutsideVolumeIsExcluded) {
  const RirSceneTarget target = MakeTarget(9003U);  // az=0°
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.orientation.steerable_volume_deg.az_min_deg = 30.0f;  // 窗口 [30,60]：目标出界
  config.orientation.steerable_volume_deg.az_max_deg = 60.0f;
  RirSession session = RirSession::Create(config);
  for (std::uint32_t cycle = 1U; cycle <= 6U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_FALSE(HasAttributionFor(result, target.external_target_id))
        << "方位出体积目标不应入检测候选集";
  }
}

// ---------------------------------------------------------------------------
// 4) 运行期改 scan_center：窗口随之移动（原出界目标入界检出）
// ---------------------------------------------------------------------------

TEST(RirSearchSectorTest, RuntimeScanCenterMovesWindow) {
  const RirSceneTarget target = MakeTarget(9004U);  // az=0°
  config::RirSessionConfig config = MakeIdentifyConfig();
  config.orientation.steerable_volume_deg.az_min_deg = -10.0f;
  config.orientation.steerable_volume_deg.az_max_deg = 10.0f;
  RirSession session = RirSession::Create(config);

  // 初始 scan_center az=90：目标相对方位 -90，出界。
  config::RirRuntimeConfigPatch steer;
  steer.has_scan_center = true;
  steer.scan_center_deg.az_deg = 90.0f;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(steer));
  for (std::uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    EXPECT_FALSE(HasAttributionFor(result, target.external_target_id))
        << "指向 90° 时 az=0 目标应在窗口外";
  }

  // 运行期设定方位俯仰回 0°：目标入界检出。
  steer.scan_center_deg.az_deg = 0.0f;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(steer));
  bool seen = false;
  for (std::uint32_t cycle = 5U; cycle <= 10U; ++cycle) {
    const RirCycleResult result = session.StepWithResult(MakeInput(cycle, {target}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    seen = seen || HasAttributionFor(result, target.external_target_id);
  }
  EXPECT_TRUE(seen) << "指向回 0° 后目标应入界检出";
}

// ---------------------------------------------------------------------------
// 5) 出界目标带排除诊断码，体积内目标不带（规则 13b：消失可归因，不再静默）
// ---------------------------------------------------------------------------

TEST(RirSearchSectorTest, OutsideVolumeTargetCarriesExclusionIssue) {
  RirSceneTarget in_volume = MakeTarget(9005U);  // az=0°、el≈11.3°，默认体积内
  RirSceneTarget out_volume = MakeTarget(9006U);  // el=45°，出默认体积 el 上界 30°
  out_volume.position_z = 10000.0f;
  out_volume.range_m = std::sqrt(10000.0f * 10000.0f + 10000.0f * 10000.0f);
  config::RirSessionConfig config = MakeIdentifyConfig();  // 默认体积 ±60/[-30,30]
  RirSession session = RirSession::Create(config);
  for (std::uint32_t cycle = 1U; cycle <= 3U; ++cycle) {
    const RirCycleResult result =
        session.StepWithResult(MakeInput(cycle, {in_volume, out_volume}));
    ASSERT_EQ(result.status, session::RirCycleStatus::kCompleted);
    const session::RirIssue* excluded =
        FindIssue(result, session::codes::kTargetOutsideSearchVolume, out_volume.external_target_id);
    ASSERT_NE(excluded, nullptr) << "出界目标应携带角域裁剪排除诊断";
    EXPECT_EQ(excluded->cause, session::RirIssueCause::kNone);
    EXPECT_EQ(excluded->location.entity_index, 1U) << "出界目标为场景下标 1";
    EXPECT_EQ(FindIssue(result, session::codes::kTargetOutsideSearchVolume,
                        in_volume.external_target_id),
              nullptr)
        << "体积内目标不应携带角域裁剪排除诊断";
  }
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
