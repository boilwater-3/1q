/**
 * @file sbirs_session_test.cpp
 * @brief 验证 SBIRS Session 端到端闭环：状态机交接、多周期跟踪、运行时热切换、校验回退。
 *
 * 对齐 EOS/ESR 集成测试分层：单周期 smoke、WFOV→NFOV 首次捕获、EKF/真值辅助跟踪跨周期持续、
 * 扫描相位推进、runtime patch 立即生效、校验失败复用上一有效输出。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsOutputDebugView.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

namespace sbirs_sensor {
namespace session {
namespace {
namespace output = ::sbirs_sensor::output;
namespace attribution = ::sbirs_sensor::attribution;
namespace sbirs_config = ::sbirs_sensor::config;

SbirsVector3M Vector(double x, double y, double z) {
  SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

// 高温强信号目标，确保 WFOV SNR 远超门限。
SbirsSceneTarget MakeTarget(std::uint64_t id, double offset_y = 0.0) {
  SbirsSceneTarget target;
  target.target_id = id;
  target.target_name = "booster";
  target.position_ecef_m = Vector(8000000.0, offset_y, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e8;
  return target;
}

config::SbirsSessionConfig MakeSessionConfig() {
  config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.hardware.integration_time_sec = 1.0f;
  config.mission.scan_start_az_deg = 359.0f;  // ECI 方位 [0,360)：-1° 等价折入 359°
  config.mission.scan_span_deg = 11.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.mission.wide_field_fov_az_deg = 20.0f;
  config.mission.wide_field_fov_el_deg = 20.0f;
  config.mission.narrow_field_fov_az_deg = 5.0f;
  config.mission.narrow_field_fov_el_deg = 5.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  // 关闭随机误差，使扫描/捕获判定在集成测试中确定可复现。
  config.policy.error_model.range_fraction_sigma = 0.0f;
  return config;
}

SbirsCycleInput MakeBaseInput(std::uint32_t cycle_index = 1U) {
  return SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
      .AddTarget(MakeTarget(1U))
      .Build();
}

// 经 attribution 层中转：先按 target_id 找 detection_id，再在 output_frame 找记录。
const output::SbirsDetectionRecord* FindDetectionByTargetId(const SbirsCycleResult& result,
                                                            std::uint64_t target_id) {
  const attribution::SbirsDetectionAttributionRecord* attr = nullptr;
  for (std::size_t i = 0; i < result.detection_attributions.size(); ++i) {
    if (result.detection_attributions[i].target_id == target_id) {
      attr = &result.detection_attributions[i];
      break;
    }
  }
  if (attr == nullptr) {
    return nullptr;
  }
  for (std::size_t i = 0; i < result.output_frame.detections.size(); ++i) {
    if (result.output_frame.detections[i].detection_id == attr->detection_id) {
      return &result.output_frame.detections[i];
    }
  }
  return nullptr;
}

TEST(SbirsSessionIntegrationTest, StepWithResultProducesDetectionOutput) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  const SbirsCycleResult result = session.StepWithResult(MakeBaseInput());
  EXPECT_FALSE(HasValidationError(result.issues));
  EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
  EXPECT_EQ(result.output_frame.cycle_index, 1U);
  EXPECT_FALSE(result.output_frame.detections.empty());
}

TEST(SbirsSessionIntegrationTest, WfovCandidateTransitionsToNfovAcquisition) {
  // 首周期：WFOV 发现 → 调度选中 → NFOV 首次捕获成功 → 输出 NarrowFieldAcquisition。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  const SbirsCycleResult result = session.StepWithResult(MakeBaseInput());
  ASSERT_FALSE(result.output_frame.detections.empty());
  EXPECT_EQ(result.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_EQ(result.detection_attributions.front().tracking_source,
            attribution::SbirsTrackingSource::kEstimated);
}

TEST(SbirsSessionIntegrationTest, CapturedTargetContinuesEstimatedTrackNextCycle) {
  // 第二周期：默认启用 EKF 估计跟踪（design 2.5a），输出 NarrowFieldTrack。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  session.StepWithResult(MakeBaseInput(1U));
  const SbirsCycleResult result = session.StepWithResult(MakeBaseInput(2U));
  ASSERT_FALSE(result.output_frame.detections.empty());
  EXPECT_EQ(result.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_EQ(result.detection_attributions.front().tracking_source,
            attribution::SbirsTrackingSource::kEstimated);
}

TEST(SbirsSessionIntegrationTest, TruthModesRetagInPlaceAndEstimatedTransitionReacquires) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.policy.tracking.tracking_mode =
      config::SbirsTrackingMode::kStrictTruthAssisted;
  config.policy.error_model.attitude_sigma_deg = 0.2f;
  config.policy.error_model.range_fraction_sigma = 0.01f;
  SbirsSession session = SbirsSession::Create(config);

  const SbirsCycleResult strict = session.StepWithResult(MakeBaseInput(1U));
  ASSERT_EQ(strict.output_frame.detections.size(), 1U);
  ASSERT_EQ(strict.detection_attributions.size(), 1U);
  EXPECT_EQ(strict.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_EQ(strict.detection_attributions.front().tracking_source,
            attribution::SbirsTrackingSource::kStrictTruthAssisted);
  // GMST≈0 残余 2.35e-9 rad。
  EXPECT_NEAR(strict.output_frame.detections.front().azimuth_rad, 0.0f, 1.0e-6f);

  config.policy.tracking.tracking_mode =
      config::SbirsTrackingMode::kSensorLikeTruthAssisted;
  {
    config::SbirsRuntimeConfigPatch policy_patch;
    policy_patch.has_policy = true;
    policy_patch.policy = config.policy;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(policy_patch));

  }
  const SbirsCycleResult sensor_like = session.StepWithResult(MakeBaseInput(2U));
  ASSERT_EQ(sensor_like.output_frame.detections.size(), 1U);
  EXPECT_EQ(sensor_like.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldTrack);
  EXPECT_EQ(sensor_like.detection_attributions.front().tracking_source,
            attribution::SbirsTrackingSource::kSensorLikeTruthAssisted);
  EXPECT_NE(sensor_like.output_frame.detections.front().azimuth_rad, 0.0f);

  config.policy.tracking.tracking_mode = config::SbirsTrackingMode::kEstimated;
  {
    config::SbirsRuntimeConfigPatch policy_patch;
    policy_patch.has_policy = true;
    policy_patch.policy = config.policy;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(policy_patch));

  }
  const SbirsCycleResult estimated = session.StepWithResult(MakeBaseInput(3U));
  ASSERT_EQ(estimated.output_frame.detections.size(), 1U);
  EXPECT_EQ(estimated.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldAcquisition);
  EXPECT_EQ(estimated.detection_attributions.front().tracking_source,
            attribution::SbirsTrackingSource::kEstimated);
}

TEST(SbirsSessionIntegrationTest, MultiCycleScanAdvancesAzimuth) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  const SbirsOutputFrame frame_1 = session.Step(MakeBaseInput(1U));
  const SbirsOutputFrame frame_2 = session.Step(MakeBaseInput(2U));
  EXPECT_NE(frame_1.scan_azimuth_rad, frame_2.scan_azimuth_rad);
}

TEST(SbirsSessionIntegrationTest, StandbyModeProducesNoDetections) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::SbirsWorkMode::kStandby;
  SbirsSession session = SbirsSession::Create(config);
  const SbirsCycleResult result = session.StepWithResult(MakeBaseInput());
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(SbirsSessionIntegrationTest, SensorDisabledProducesNoDetections) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.sensor_enabled = false;
  SbirsSession session = SbirsSession::Create(config);
  const SbirsCycleResult result = session.StepWithResult(MakeBaseInput());
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(SbirsSessionIntegrationTest, EarthOccultedTargetNotDetected) {
  // 视线穿过地球的目标不可观测（design 2.7）。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput();
  input.scene.front().position_ecef_m = Vector(-8000000.0, 0.0, 0.0);
  const SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(SbirsSessionIntegrationTest, RuntimeWorkModeSwitchTakesEffectImmediately) {
  // 运行期从 SearchAndStare 切到 Standby，立即生效（design 1.5 立即提交策略）。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  const SbirsOutputFrame active_frame = session.Step(MakeBaseInput(1U));
  ASSERT_FALSE(active_frame.detections.empty());
  config::SbirsRuntimeConfigPatch patch;
  patch.has_work_mode = true;
  patch.work_mode = config::SbirsWorkMode::kStandby;
  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));

  const SbirsOutputFrame standby_frame = session.Step(MakeBaseInput(2U));
  EXPECT_TRUE(standby_frame.detections.empty());
}

TEST(SbirsSessionIntegrationTest, WideSearchIsWfovOnlyAndRoundTripResumesScheduling) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  ASSERT_FALSE(session.StepWithResult(MakeBaseInput(1U)).output_frame.detections.empty());
  {
    config::SbirsRuntimeConfigPatch wide_patch;
    wide_patch.has_work_mode = true;
    wide_patch.work_mode = config::SbirsWorkMode::kWideSearch;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(wide_patch));

  }

  const SbirsCycleResult wide = session.StepWithResult(MakeBaseInput(2U));
  ASSERT_EQ(wide.output_frame.detections.size(), 1U);
  EXPECT_EQ(wide.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kWideFieldSearch);
  ASSERT_EQ(wide.detection_attributions.size(), 1U);
  EXPECT_EQ(wide.detection_attributions.front().nfov_channel_id, -1);

  {
    config::SbirsRuntimeConfigPatch stare_patch;
    stare_patch.has_work_mode = true;
    stare_patch.work_mode = config::SbirsWorkMode::kSearchAndStare;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(stare_patch));


  }
  const SbirsCycleResult resumed = session.StepWithResult(MakeBaseInput(3U));
  ASSERT_EQ(resumed.output_frame.detections.size(), 1U);
  EXPECT_EQ(resumed.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldAcquisition);
}

TEST(SbirsSessionIntegrationTest, RuntimeScanSectorKeepsPointingInsideAndResetsOutside) {
  // ECI 方位约定（2026-08）：scan_start ∈ [0, 360)；输出弧度 [0, 2π)。
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.scan_start_az_deg = 350.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  SbirsSession session = SbirsSession::Create(config);
  const SbirsOutputFrame initial = session.Step(MakeBaseInput(1U));
  // 350°+5° = 355° → 6.1959 rad。
  ASSERT_FLOAT_EQ(initial.scan_azimuth_rad, 6.1959188f);

  config.mission.scan_start_az_deg = 340.0f;
  config.mission.scan_span_deg = 30.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  {
    config::SbirsRuntimeConfigPatch mission_patch;
    mission_patch.has_mission = true;
    mission_patch.mission = config.mission;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(mission_patch));

  }
  EXPECT_FLOAT_EQ(session.Step(MakeBaseInput(2U)).scan_azimuth_rad, 6.1959188f);

  config.mission.scan_start_az_deg = 30.0f;
  config.mission.scan_span_deg = 20.0f;
  {
    config::SbirsRuntimeConfigPatch mission_patch;
    mission_patch.has_mission = true;
    mission_patch.mission = config.mission;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(mission_patch));

  }
  // 30° → 0.5236 rad。
  EXPECT_FLOAT_EQ(session.Step(MakeBaseInput(3U)).scan_azimuth_rad, 0.5235988f);
}

TEST(SbirsSessionIntegrationTest, StandbyFreezesScanPhaseUntilSearchResumes) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.scan_start_az_deg = 350.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  SbirsSession session = SbirsSession::Create(config);
  // 355° → 6.1959 rad。
  EXPECT_FLOAT_EQ(session.Step(MakeBaseInput(1U)).scan_azimuth_rad, 6.1959188f);

  {
    config::SbirsRuntimeConfigPatch standby_patch;
    standby_patch.has_work_mode = true;
    standby_patch.work_mode = config::SbirsWorkMode::kStandby;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(standby_patch));


  }
  EXPECT_FLOAT_EQ(session.Step(MakeBaseInput(2U)).scan_azimuth_rad, 6.1959188f);
  EXPECT_FLOAT_EQ(session.Step(MakeBaseInput(3U)).scan_azimuth_rad, 6.1959188f);

  {
    config::SbirsRuntimeConfigPatch stare_patch;
    stare_patch.has_work_mode = true;
    stare_patch.work_mode = config::SbirsWorkMode::kSearchAndStare;
    ASSERT_TRUE(session.TryApplyRuntimeConfig(stare_patch));


  }
  // 350°+10° = 360° → 0 rad。
  EXPECT_FLOAT_EQ(session.Step(MakeBaseInput(4U)).scan_azimuth_rad, 0.0f);
}

TEST(SbirsSessionIntegrationTest, RuntimeScanRateChangeUpdatesAdvance) {
  // 用足够大的扫描范围，避免推进后回绕到 scan_start，掩盖速率差异。
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.scan_start_az_deg = 180.0f;
  config.mission.scan_span_deg = 360.0f;

  SbirsSession fast_session = SbirsSession::Create(config);
  const SbirsOutputFrame fast_1 = fast_session.Step(MakeBaseInput(1U));
  config::SbirsRuntimeConfigPatch patch;
  patch.has_scan_rate_deg_per_sec = true;
  patch.scan_rate_deg_per_sec = 97.0f;
  fast_session.TryApplyRuntimeConfig(patch);
  const SbirsOutputFrame fast_2 = fast_session.Step(MakeBaseInput(2U));
  const float delta_fast = std::fabs(fast_2.scan_azimuth_rad - fast_1.scan_azimuth_rad);

  SbirsSession slow_session = SbirsSession::Create(config);
  const SbirsOutputFrame slow_1 = slow_session.Step(MakeBaseInput(1U));
  const SbirsOutputFrame slow_2 = slow_session.Step(MakeBaseInput(2U));
  const float delta_slow = std::fabs(slow_2.scan_azimuth_rad - slow_1.scan_azimuth_rad);
  EXPECT_GT(delta_fast, delta_slow);
}

TEST(SbirsSessionIntegrationTest, RateLimitedPointingReservesChannelUntilSettled) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 2.0f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;
  SbirsSession session = SbirsSession::Create(config);

  for (std::uint32_t cycle = 1U; cycle <= 4U; ++cycle) {
    const SbirsCycleResult result = session.StepWithResult(MakeBaseInput(cycle));
    ASSERT_EQ(result.output_frame.detections.size(), 1U);
    EXPECT_EQ(result.output_frame.detections.front().observation_stage,
              output::SbirsObservationStage::kWideFieldSearch);
    ASSERT_EQ(result.detection_attributions.size(), 1U);
    EXPECT_EQ(result.detection_attributions.front().nfov_channel_id, 0);
  }
  const SbirsCycleResult acquired = session.StepWithResult(MakeBaseInput(5U));
  ASSERT_EQ(acquired.output_frame.detections.size(), 1U);
  EXPECT_EQ(acquired.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldAcquisition);
  const SbirsCycleResult tracked = session.StepWithResult(MakeBaseInput(6U));
  ASSERT_EQ(tracked.output_frame.detections.size(), 1U);
  EXPECT_EQ(tracked.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldTrack);
}

TEST(SbirsSessionIntegrationTest, RuntimeMissionPatchClearsSlewAndUsesNewRate) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.scan_start_az_deg = 350.0f;  // ECI 方位 [0,360)：-10° 等价折入 350°
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 2.0f;
  SbirsSession session = SbirsSession::Create(config);
  const SbirsCycleResult slewing = session.StepWithResult(MakeBaseInput(1U));
  ASSERT_EQ(slewing.output_frame.detections.size(), 1U);
  EXPECT_EQ(slewing.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kWideFieldSearch);

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 20.0f;
  config::SbirsRuntimeConfigPatch patch;
  patch.has_mission = true;
  patch.mission = config.mission;
  ASSERT_TRUE(session.TryApplyRuntimeConfig(patch));
  const SbirsCycleResult acquired = session.StepWithResult(MakeBaseInput(2U));
  ASSERT_EQ(acquired.output_frame.detections.size(), 1U);
  EXPECT_EQ(acquired.output_frame.detections.front().observation_stage,
            output::SbirsObservationStage::kNarrowFieldAcquisition);
}

TEST(SbirsSessionIntegrationTest, DualChannelAssignmentIsIndependentOfInputOrder) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.scan_start_az_deg = 350.0f;  // ECI 方位 [0,360)：-10° 等价折入 350°
  config.mission.scan_span_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 2.0f;
  config.policy.scheduler.max_concurrent_nfov_locks = 2;

  const auto run = [&config](bool reverse) {
    SbirsCycleInputBuilder builder;
    builder.WithCycleIndex(1U)
        .WithDeltaTimeSec(1.0f)
        .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
        .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
        .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{});
    if (reverse) {
      builder.AddTarget(MakeTarget(2U, -1000.0)).AddTarget(MakeTarget(1U, 1000.0));
    } else {
      builder.AddTarget(MakeTarget(1U, 1000.0)).AddTarget(MakeTarget(2U, -1000.0));
    }
    SbirsSession session = SbirsSession::Create(config);
    return session.StepWithResult(builder.Build());
  };
  const SbirsCycleResult forward = run(false);
  const SbirsCycleResult reversed = run(true);
  ASSERT_EQ(forward.detection_attributions.size(), 2U);
  ASSERT_EQ(reversed.detection_attributions.size(), 2U);
  for (std::uint64_t target_id = 1U; target_id <= 2U; ++target_id) {
    int forward_channel = -1;
    int reversed_channel = -1;
    for (const attribution::SbirsDetectionAttributionRecord& value :
         forward.detection_attributions) {
      if (value.target_id == target_id) forward_channel = value.nfov_channel_id;
    }
    for (const attribution::SbirsDetectionAttributionRecord& value :
         reversed.detection_attributions) {
      if (value.target_id == target_id) reversed_channel = value.nfov_channel_id;
    }
    EXPECT_GE(forward_channel, 0);
    EXPECT_EQ(forward_channel, reversed_channel);
  }
}

TEST(SbirsSessionIntegrationTest, InvalidRuntimePatchDoesNotPolluteConfig) {
  // 无效 patch（负扫描速率）被拒绝，配置不变，后续周期正常执行。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  config::SbirsRuntimeConfigPatch invalid;
  invalid.has_scan_rate_deg_per_sec = true;
  invalid.scan_rate_deg_per_sec = -5.0f;
  EXPECT_FALSE(session.TryApplyRuntimeConfig(invalid));
  const SbirsCycleResult result = session.StepWithResult(MakeBaseInput());
  EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
}

TEST(SbirsSessionIntegrationTest, InvalidFirstCycleReturnsEmptyOutput) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput();
  input.dt_sec = -1.0f;
  const SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_NE(result.status, SbirsCycleStatus::kCompleted);
  EXPECT_TRUE(HasValidationError(result.issues));
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(SbirsSessionIntegrationTest, InvalidLaterCycleReturnsEmptyOutputNotReused) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  const SbirsCycleResult valid = session.StepWithResult(MakeBaseInput(1U));
  ASSERT_FALSE(valid.output_frame.detections.empty());

  SbirsCycleInput invalid = MakeBaseInput(2U);
  invalid.dt_sec = 0.0f;
  const SbirsCycleResult result = session.StepWithResult(invalid);
  EXPECT_TRUE(HasValidationError(result.issues));
  EXPECT_NE(result.status, SbirsCycleStatus::kCompleted);
  // 非执行周期返回默认空帧，不复用上一有效输出（统一不复用语义）。
  EXPECT_TRUE(result.output_frame.detections.empty());
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(SbirsSessionIntegrationTest, InactiveTargetProducesNoOutputThatCycle) {
  // 目标标记 inactive 的当周期不输出该目标检测（design 2.5 lost 转移）。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  session.StepWithResult(MakeBaseInput(1U));  // 捕获并锁定

  SbirsCycleInput inactive_input = MakeBaseInput(2U);
  inactive_input.scene.front().active = false;
  const SbirsCycleResult inactive_result = session.StepWithResult(inactive_input);
  // inactive 目标当周期无检测输出。
  EXPECT_EQ(FindDetectionByTargetId(inactive_result, 1U), nullptr);
}

TEST(SbirsSessionIntegrationTest, MultipleWfovCandidatesSingleNfovLock) {
  // 多 WFOV 候选时，单 NFOV 资源只锁定一个（design 2.6 单目标锁定）。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput();
  input.scene.push_back(MakeTarget(2U, 1000.0));
  const SbirsCycleResult result = session.StepWithResult(input);

  std::size_t nfov_acquisitions = 0U;
  for (const attribution::SbirsDetectionAttributionRecord& attr : result.detection_attributions) {
    const output::SbirsDetectionRecord* record = FindDetectionByTargetId(result, attr.target_id);
    if (record != nullptr &&
        record->observation_stage == output::SbirsObservationStage::kNarrowFieldAcquisition &&
        attr.capture_failure_reason == attribution::SbirsCaptureFailureReason::kNone) {
      ++nfov_acquisitions;
      EXPECT_EQ(attr.tracking_source, attribution::SbirsTrackingSource::kEstimated);
    }
  }
  EXPECT_EQ(nfov_acquisitions, 1U);
}

TEST(SbirsSessionIntegrationTest, MultipleWfovCandidatesMultiNfovLock) {
  // design 2.6 多通道：max_concurrent_nfov_locks=2 时，两个 WFOV 候选同时捕获，
  // 各占独立 NFOV 通道，且通道编号互不相同。
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.policy.scheduler.max_concurrent_nfov_locks = 2;
  SbirsSession session = SbirsSession::Create(config);
  SbirsCycleInput input = MakeBaseInput();
  input.scene.push_back(MakeTarget(2U, 1000.0));
  const SbirsCycleResult result = session.StepWithResult(input);

  std::size_t nfov_acquisitions = 0U;
  int channel_of_target_1 = -2;
  int channel_of_target_2 = -2;
  for (const attribution::SbirsDetectionAttributionRecord& attr : result.detection_attributions) {
    const output::SbirsDetectionRecord* record = FindDetectionByTargetId(result, attr.target_id);
    if (record != nullptr &&
        record->observation_stage == output::SbirsObservationStage::kNarrowFieldAcquisition &&
        attr.capture_failure_reason == attribution::SbirsCaptureFailureReason::kNone) {
      ++nfov_acquisitions;
      if (attr.target_id == 1U) channel_of_target_1 = attr.nfov_channel_id;
      if (attr.target_id == 2U) channel_of_target_2 = attr.nfov_channel_id;
    }
  }
  EXPECT_EQ(nfov_acquisitions, 2U);
  EXPECT_GE(channel_of_target_1, 0);
  EXPECT_GE(channel_of_target_2, 0);
  EXPECT_NE(channel_of_target_1, channel_of_target_2);
}

// design cue 延迟外推：横向高速目标在 latency 内移出 NFOV → 首次捕获失败。
// 失败诊断进 result.detection_attributions（capture_failure_reason=kNfovAcquisitionFailed），
// 但 raw output_frame.detections 不含该失败记录（守三层分离边界）。
TEST(SbirsSessionIntegrationTest, CueLatencyFailureAttributionStaysOutOfRawOutput) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.narrow_cue_latency_s = 1.0f;
  config.mission.narrow_field_fov_az_deg = 2.0f;  // 收窄 NFOV 使外推后出界

  SbirsSceneTarget target = MakeTarget(1U);
  target.velocity_ecef_m_per_s = Vector(0.0, 1500000.0, 0.0);  // 横向高速
  target.has_velocity_ecef_m_per_s = true;
  SbirsCycleInput input = SbirsCycleInputBuilder()
                              .WithCycleIndex(1U)
                              .WithDeltaTimeSec(1.0f)
                              .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                              .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                              .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                              .AddTarget(target)
                              .Build();

  SbirsSession session = SbirsSession::Create(config);
  const SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, SbirsPipelineAbortReason::kNone);

  // 失败诊断必须出现在 attribution 层。
  bool found_failure = false;
  for (const attribution::SbirsDetectionAttributionRecord& attr : result.detection_attributions) {
    if (attr.capture_failure_reason ==
        attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed) {
      found_failure = true;
    }
  }
  EXPECT_TRUE(found_failure);

  // raw output 不得包含 detected=false 的记录（边界守卫）。
  for (const output::SbirsDetectionRecord& record : result.output_frame.detections) {
    EXPECT_TRUE(record.detected);
  }
}

TEST(SbirsSessionIntegrationTest, MeasurementCvCueCapturesAfterSecondWfovObservation) {
  config::SbirsSessionConfig config = MakeSessionConfig();
  config.mission.narrow_cue_latency_s = 1.0f;
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;

  SbirsSceneTarget target = MakeTarget(77U);
  target.velocity_ecef_m_per_s = Vector(0.0, 20000.0, 0.0);
  target.has_velocity_ecef_m_per_s = true;
  SbirsSession session = SbirsSession::Create(config);
  const SbirsCycleResult first =
      session.StepWithResult(SbirsCycleInputBuilder()
                                 .WithCycleIndex(1U)
                                 .WithDeltaTimeSec(1.0f)
                                 .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                                 .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                                 .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                                 .AddTarget(target)
                                 .Build());
  EXPECT_EQ(FindDetectionByTargetId(first, 77U), nullptr);

  target.position_ecef_m.y = 20000.0;
  const SbirsCycleResult second =
      session.StepWithResult(SbirsCycleInputBuilder()
                                 .WithCycleIndex(2U)
                                 .WithDeltaTimeSec(1.0f)
                                 .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                                 .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                                 .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                                 .AddTarget(target)
                                 .Build());
  const output::SbirsDetectionRecord* acquired = FindDetectionByTargetId(second, 77U);
  ASSERT_NE(acquired, nullptr);
  EXPECT_TRUE(acquired->detected);
  EXPECT_EQ(acquired->observation_stage, output::SbirsObservationStage::kNarrowFieldAcquisition);
}


TEST(SbirsSessionIntegrationTest, DebugViewRangeBackfillReflectsAttributionOnly) {
  // 2026-08 距离处理结论：被动红外不测距，estimated_range_m 仅为内部诊断字段，
  // 且只在目标获得归属记录（通过全部几何/SNR 门）时回填真值距离；被排除目标
  // （遮挡/距离带/视场/SNR）无归属记录 → 调试行恒为 0。示例层不再展示距离。
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput(1U);
  SbirsSceneTarget occulted = MakeTarget(2U, 0.0);
  occulted.position_ecef_m = Vector(-8000000.0, 0.0, 0.0);  // 卫星另一侧 → 地球遮挡
  input.scene.push_back(occulted);

  const SbirsCycleResult result = session.StepWithResult(input);
  ASSERT_EQ(result.status, SbirsCycleStatus::kCompleted);
  const SbirsOutputDebugView view = SbirsOutputDebugViewBuilder::Build(input, result);
  ASSERT_EQ(view.targets.size(), 2U);
  const SbirsDebugTargetState* detected_row = nullptr;
  const SbirsDebugTargetState* excluded_row = nullptr;
  for (const SbirsDebugTargetState& state : view.targets) {
    if (state.target_id == 1U) detected_row = &state;
    if (state.target_id == 2U) excluded_row = &state;
  }
  ASSERT_NE(detected_row, nullptr);
  ASSERT_NE(excluded_row, nullptr);

  // 检测目标：归属回填真值距离（卫星 (7e6,0,0) → 目标 (8e6,0,0)，range=1e6 m）。
  EXPECT_EQ(detected_row->status, SbirsDebugTargetStatus::kDetected);
  EXPECT_FLOAT_EQ(detected_row->estimated_range_m, 1000000.0f);
  // 遮挡排除目标：无归属 → kNotInOutput，距离恒 0。
  EXPECT_EQ(excluded_row->status, SbirsDebugTargetStatus::kNotInOutput);
  EXPECT_FLOAT_EQ(excluded_row->estimated_range_m, 0.0f);
}

// ===== 合同指标 2：cue 延迟期间卫星位移计入延迟真值 LOS =====
// 镜像 CueLatencyFailureAttributionStaysOutOfRawOutput：目标静止、卫星横向运动，
// 延迟 20s 后真值 LOS 方位 ≈ −9°（卫星 ECI 速度含 ω×r 输运项 ≈8010 m/s），
// 移出 ±1° NFOV → 首次捕获失败；对照组（卫星零速度）捕获成功。
TEST(SbirsSessionIntegrationTest, CueLatencyAccountsForSatelliteDisplacementDuringLatency) {
  const auto run_capture_failure_count = [](const SbirsVector3M& satellite_velocity) {
    config::SbirsSessionConfig config = MakeSessionConfig();
    config.mission.narrow_cue_latency_s = 20.0f;
    config.mission.narrow_field_fov_az_deg = 2.0f;  // 收窄 NFOV 使延迟后出界

    SbirsCycleInput input = SbirsCycleInputBuilder()
                                .WithCycleIndex(1U)
                                .WithDeltaTimeSec(1.0f)
                                .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
                                .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
                                .WithSatelliteVelocity(satellite_velocity).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
                                .AddTarget(MakeTarget(1U))  // 目标静止（无速度）
                                .Build();
    SbirsSession session = SbirsSession::Create(config);
    const SbirsCycleResult result = session.StepWithResult(input);
    EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
    std::size_t failures = 0U;
    for (const attribution::SbirsDetectionAttributionRecord& attr :
         result.detection_attributions) {
      if (attr.capture_failure_reason ==
          attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed) {
        ++failures;
      }
    }
    return failures;
  };

  // 对照组：卫星零速度 → 延迟真值不变 → 捕获成功。
  EXPECT_EQ(run_capture_failure_count(Vector(0.0, 0.0, 0.0)), 0U);
  // 卫星横向运动 → 延迟期间卫星位移使真值 LOS 移出 NFOV → 捕获失败。
  EXPECT_EQ(run_capture_failure_count(Vector(0.0, 7500.0, 0.0)), 1U);
}

// 卫星速度必填（合同指标 2）：缺失即整周期校验拒绝，空帧 + error 级 issue。
TEST(SbirsSessionIntegrationTest, MissingSatelliteVelocityRejectsCycle) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput();
  input.has_satellite_velocity_ecef_m_per_s = false;  // 故意缺失必填速度

  const SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, SbirsCycleStatus::kRejectedInvalidInput);
  EXPECT_EQ(result.abort_reason, SbirsPipelineAbortReason::kValidationRejected);
  EXPECT_TRUE(result.output_frame.detections.empty());
  bool found_code = false;
  for (const SbirsIssue& issue : result.issues) {
    if (issue.code == codes::kInvalidSatelliteVelocity) {
      found_code = true;
      EXPECT_EQ(issue.severity, SbirsIssueSeverity::kError);
    }
  }
  EXPECT_TRUE(found_code);
}

// 卫星姿态必填（阶段 2 指向合成链）：缺失即整周期校验拒绝，空帧 + error 级 issue。
TEST(SbirsSessionIntegrationTest, MissingSatelliteAttitudeRejectsCycle) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput();
  input.has_satellite_attitude = false;  // 故意缺失必填姿态

  const SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, SbirsCycleStatus::kRejectedInvalidInput);
  EXPECT_EQ(result.abort_reason, SbirsPipelineAbortReason::kValidationRejected);
  EXPECT_TRUE(result.output_frame.detections.empty());
  bool found_code = false;
  for (const SbirsIssue& issue : result.issues) {
    if (issue.code == codes::kInvalidSatelliteAttitude) {
      found_code = true;
      EXPECT_EQ(issue.severity, SbirsIssueSeverity::kError);
    }
  }
  EXPECT_TRUE(found_code);
}

// 非零姿态端到端：yaw=30° 体稳定下 WFOV 探测输出 az 保持 ECI 参考（≈0 rad）。
TEST(SbirsSessionIntegrationTest, BodyYawKeepsEciOutputReference) {
  SbirsSession session = SbirsSession::Create(MakeSessionConfig());
  SbirsCycleInput input = MakeBaseInput();
  input.satellite_attitude_eci_body_deg.yaw_deg = 30.0;
  // 目标方位 30° 与传感器系扫描中心（yaw30 平移后 = 30°）对齐，WFOV 内可见。
  input.scene[0] = MakeTarget(1U, 0.0);
  input.scene[0].position_ecef_m =
      Vector(7000000.0 + 1000000.0 * std::cos(30.0 * 3.14159265358979323846 / 180.0),
             1000000.0 * std::sin(30.0 * 3.14159265358979323846 / 180.0), 0.0);

  const SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_EQ(result.status, SbirsCycleStatus::kCompleted);
  const output::SbirsDetectionRecord* record = FindDetectionByTargetId(result, 1U);
  ASSERT_NE(record, nullptr);
  EXPECT_NEAR(record->azimuth_rad, 30.0 * 3.14159265358979323846 / 180.0, 1.0e-3);
}

}  // namespace
}  // namespace session
}  // namespace sbirs_sensor
